#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image


class DepthVFHController(Node):
    def __init__(self) -> None:
        """
        初始化深度VFH控制器节点
        
        负责设置ROS 2参数、订阅者和发布者，配置VFH算法参数
        """
        super().__init__('depth_vfh_controller')

        # 话题配置 - 定义输入输出ROS话题名称
        self.declare_parameter('depth_topic', '/mower/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/mower/camera/camera_info')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # VFH算法和控制参数 - 配置避障和导航行为
        self.declare_parameter('control_rate_hz', 12.0)         # 控制循环频率(Hz)，决定算法执行速度
        self.declare_parameter('sector_angle_deg', 10.0)        # 扇区角度(度)，决定VFH算法的角度分辨率
        self.declare_parameter('view_angle_deg', 75.0)          # 视野角度(度)，机器人前方可感知的角度范围    
        self.declare_parameter('min_depth', 0.45)               # 最小有效深度(m)，过滤过近的噪声数据
        self.declare_parameter('max_depth', 5.0)                # 最大有效深度(m)，限制感知范围，提高计算效率
        self.declare_parameter('inflation_max_range', 1.8)      # 最大膨胀范围(m)，只对近距离障碍物进行膨胀处理
        self.declare_parameter('robot_radius', 0.28)            # 机器人半径(m)，用于障碍物膨胀计算
        self.declare_parameter('safety_margin', 0.08)           # 安全余量(m)，额外的安全距离缓冲
        self.declare_parameter('camera_x_offset', 0.22)         # 相机x偏移(m)，相机到机器人基座的距离
        self.declare_parameter('pixel_step_u', 6)               # 水平像素步长，图像采样间隔，影响计算量
        self.declare_parameter('pixel_step_v', 8)               # 垂直像素步长，图像采样间隔，影响计算量
        self.declare_parameter('scan_v_start_ratio', 0.20)      # 垂直扫描起始比例，忽略图像上部区域
        self.declare_parameter('scan_v_end_ratio', 0.50)        # 垂直扫描结束比例，忽略图像下部区域
        self.declare_parameter('goal_tolerance', 0.25)          # 目标容差(m)，到达目标点的判定距离
        self.declare_parameter('goal_slowdown_dist', 1.0)       # 目标减速距离(m)，接近目标时开始减速
        self.declare_parameter('stop_distance', 0.5)            # 停止距离(m)，与障碍物的最小安全距离
        self.declare_parameter('max_linear_speed', 0.40)        # 最大线速度(m/s)，机器人前进的最大速度       
        self.declare_parameter('min_linear_speed', 0.05)        # 最小线速度(m/s)，确保机器人能够缓慢移动
        self.declare_parameter('max_angular_speed', 0.7)        # 最大角速度(rad/s)，机器人转向的最大速度
        self.declare_parameter('heading_kp', 0.55)              # 航向比例系数，控制转向的响应强度
        self.declare_parameter('turn_in_place_angle_deg', 55.0) # 原地转向角度阈值(度)，大角度转向时限制前进
        self.declare_parameter('cost_goal', 1.5)                # 目标成本权重，控制向目标前进的强度
        self.declare_parameter('cost_smooth', 0.6)              # 平滑成本权重，控制运动方向变化的平滑度    
        self.declare_parameter('cost_clearance', 0.2)           # 安全距离成本权重，控制与障碍物保持距离的强度
        self.declare_parameter('switch_hysteresis', 0.04)       # 切换滞后阈值，防止方向频繁切换的防抖参数
        self.declare_parameter('max_angular_accel', 2.4)        # 最大角加速度(rad/s²)，限制转向加速度确保平滑    
        self.declare_parameter('hold_heading_max_deg', 35.0)    # 保持航向最大角度(度)，滞后切换策略的角度限制

        # 获取参数值并存储到实例变量
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        # 转换和存储控制参数
        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.sector_angle = math.radians(float(self.get_parameter('sector_angle_deg').value))  # 扇区角度(弧度)
        self.view_angle = math.radians(float(self.get_parameter('view_angle_deg').value))  # 视野角度(弧度)
        self.min_depth = float(self.get_parameter('min_depth').value)
        self.max_depth = float(self.get_parameter('max_depth').value)
        self.inflation_max_range = float(self.get_parameter('inflation_max_range').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
        self.camera_x_offset = float(self.get_parameter('camera_x_offset').value)
        self.pixel_step_u = int(self.get_parameter('pixel_step_u').value)
        self.pixel_step_v = int(self.get_parameter('pixel_step_v').value)
        self.scan_v_start_ratio = float(self.get_parameter('scan_v_start_ratio').value)
        self.scan_v_end_ratio = float(self.get_parameter('scan_v_end_ratio').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance').value)
        self.goal_slowdown_dist = float(self.get_parameter('goal_slowdown_dist').value)
        self.stop_distance = float(self.get_parameter('stop_distance').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.heading_kp = float(self.get_parameter('heading_kp').value)
        self.turn_in_place_angle = math.radians(float(self.get_parameter('turn_in_place_angle_deg').value))
        self.cost_goal = float(self.get_parameter('cost_goal').value)
        self.cost_smooth = float(self.get_parameter('cost_smooth').value)
        self.cost_clearance = float(self.get_parameter('cost_clearance').value)
        self.switch_hysteresis = float(self.get_parameter('switch_hysteresis').value)
        self.max_angular_accel = float(self.get_parameter('max_angular_accel').value)
        self.hold_heading_max = math.radians(float(self.get_parameter('hold_heading_max_deg').value))

        # 计算VFH扇区配置
        self.num_sectors = max(5, int(round((2.0 * self.view_angle) / self.sector_angle)))  # 扇区数量(至少5个)
        self.sector_angle = (2.0 * self.view_angle) / float(self.num_sectors)  # 重新计算扇区角度以均匀分布
        self.sector_centers = np.linspace(
            -self.view_angle + 0.5 * self.sector_angle,
            self.view_angle - 0.5 * self.sector_angle,
            self.num_sectors,
            dtype=np.float32,
        )

        # 状态变量初始化
        self.latest_depth: Optional[np.ndarray] = None  # 最新深度图像数据
        self.fx: Optional[float] = None  # 相机焦距x
        self.cx: Optional[float] = None  # 相机主点x

        # 机器人状态
        self.robot_x: Optional[float] = None  # 机器人x坐标
        self.robot_y: Optional[float] = None  # 机器人y坐标
        self.robot_yaw: Optional[float] = None  # 机器人偏航角

        # 目标状态
        self.goal_x: Optional[float] = None  # 目标点x坐标
        self.goal_y: Optional[float] = None  # 目标点y坐标
        self.goal_frame: Optional[str] = None  # 目标坐标系

        # 控制状态
        self.last_selected_heading = 0.0  # 上一次选择的航向角
        self.last_selected_sector_idx: Optional[int] = None  # 上一次选择的扇区索引
        self.last_angular_cmd = 0.0  # 上一次的角速度命令

        # 配置传感器数据的QoS策略(最佳努力，不保证可靠性)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # 创建订阅者
        self.create_subscription(Image, self.depth_topic, self.depth_callback, sensor_qos)  # 深度图像订阅
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, sensor_qos)  # 相机信息订阅
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)  # 里程计数据订阅
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)  # 目标点订阅

        # 创建发布者和定时器
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)  # 速度命令发布者
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)  # 控制循环定时器

        self.get_logger().info(
            f'Depth VFH ready. depth={self.depth_topic}, goal={self.goal_topic}, cmd_vel={self.cmd_vel_topic}'
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """
        相机内参回调函数
        
        从CameraInfo消息中提取相机内参矩阵的关键参数，用于深度图像到3D坐标的转换
        
        Args:
            msg: CameraInfo消息，包含相机内参和外参信息
        """
        # 提取相机内参矩阵K的关键参数
        # K = [fx  0  cx]
        #     [0  fy  cy]
        #     [0   0   1]
        self.fx = float(msg.k[0])  # 焦距x (像素单位)，用于将像素坐标转换为世界坐标
        self.cx = float(msg.k[2])  # 主点x坐标 (像素单位)，图像中心点的x坐标

    def depth_callback(self, msg: Image) -> None:
        """
        深度图像回调函数
        
        接收深度图像消息，将其转换为NumPy数组并存储以供VFH算法使用
        这是VFH避障算法的核心输入数据源
        
        Args:
            msg: Image消息，包含深度图像数据
        """
        # 将ROS Image消息转换为NumPy数组
        # 支持32FC1(32位浮点单通道)等格式
        array = self._depth_image_to_numpy(msg)
        
        # 如果转换成功，更新最新的深度图像数据
        # 该数据将在控制循环中用于障碍物检测和路径规划
        if array is not None:
            self.latest_depth = array

    def odom_callback(self, msg: Odometry) -> None:
        """
        里程计回调函数
        
        从Odometry消息中提取机器人当前位置和姿态信息
        这些信息用于VFH算法中的路径规划和目标导航
        
        Args:
            msg: Odometry消息，包含机器人的位姿和速度信息
        """
        # 提取位置信息 (x, y, z坐标)
        p = msg.pose.pose.position
        # 提取姿态信息 (四元数表示)
        q = msg.pose.pose.orientation
        
        # 更新机器人状态
        self.robot_x = p.x  # 机器人x坐标
        self.robot_y = p.y  # 机器人y坐标
        # 将四元数转换为偏航角(yaw)，用于确定机器人朝向
        self.robot_yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg: PoseStamped) -> None:
        """
        目标点回调函数
        
        接收导航目标点信息，提取目标位置和坐标系信息
        这是VFH导航算法的目标输入，触发机器人向指定位置移动
        
        Args:
            msg: PoseStamped消息，包含目标点的位姿和坐标系信息
        """
        # 提取目标点位置坐标
        self.goal_x = float(msg.pose.position.x)  # 目标点x坐标
        self.goal_y = float(msg.pose.position.y)  # 目标点y坐标
        # 提取目标坐标系信息
        self.goal_frame = msg.header.frame_id  # 目标点所在的坐标系
        
        # 记录接收到的目标点信息，便于调试和监控
        self.get_logger().info(
            f'Received goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, frame={self.goal_frame or "(empty)"}'
        )

    def control_loop(self) -> None:
        """
        VFH控制循环 - 核心导航算法
        
        执行完整的VFH(Vector Field Histogram)避障导航流程：
        1. 状态检查和安全停止
        2. 目标距离和方向计算
        3. 扇区距离和障碍物检测
        4. 最优航向选择
        5. 速度命令生成和发布
        
        该方法在控制定时器触发时周期性执行
        """
        # 1. 状态检查 - 确保所有必要数据可用
        if self.goal_x is None or self.goal_y is None:
            self.publish_stop()  # 无目标点，停止机器人
            return

        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            self.publish_stop()  # 机器人位置未知，停止
            return

        if self.fx is None or self.cx is None or self.latest_depth is None:
            self.publish_stop()  # 相机参数或深度图像缺失，停止
            return

        # 坐标系检查 - 确保目标点在正确的坐标系中
        if self.goal_frame not in (None, '', 'odom'):
            self.get_logger().warn(
                f'Goal frame {self.goal_frame} is not odom. Current node assumes odom frame for goals.',
                throttle_duration_sec=2.0,  # 限流警告，避免日志过多
            )

        # 2. 目标距离和方向计算
        dx = self.goal_x - self.robot_x  # x方向距离差
        dy = self.goal_y - self.robot_y  # y方向距离差
        goal_dist = math.hypot(dx, dy)  # 欧几里得距离

        # 检查是否到达目标点（在容差范围内）
        if goal_dist < self.goal_tolerance:
            self.publish_stop()  # 停止机器人
            self.get_logger().info('Goal reached (within tolerance).')
            self.goal_x = None  # 清除目标点
            self.goal_y = None
            self.goal_frame = None
            return

        # 3. 航向计算和障碍物检测
        goal_heading_world = math.atan2(dy, dx)  # 世界坐标系中的目标方向
        goal_heading_robot = self.normalize_angle(goal_heading_world - self.robot_yaw)  # 机器人坐标系中的目标方向

        # VFH核心算法 - 计算扇区距离和障碍物状态
        sector_dist = self.compute_sector_distance(self.latest_depth)  # 计算每个扇区的最小距离
        blocked = self.compute_blocked_sectors(sector_dist)  # 判断哪些扇区被障碍物阻挡

        # 4. 最优航向选择
        selected_heading, selected_dist, selected_sector_idx = self.select_heading(
            goal_heading_robot,  # 目标方向
            sector_dist,         # 扇区距离
            blocked,             # 障碍物状态
        )
        # 记录选择的航向用于后续控制
        self.last_selected_heading = selected_heading
        self.last_selected_sector_idx = selected_sector_idx

        # 5. 速度命令生成
        cmd = Twist()  # 创建新的速度命令

        # 特殊情况处理 - 所有方向都被阻挡
        if np.all(blocked):
            cmd.linear.x = 0.0  # 停止前进
            # 原地旋转，尝试找到出路
            cmd.angular.z = math.copysign(self.max_angular_speed * 0.7, goal_heading_robot if abs(goal_heading_robot) > 1e-4 else 1.0)
            self.last_angular_cmd = cmd.angular.z
            self.cmd_pub.publish(cmd)
            return

        # 角速度控制 - 使用PID和加速度限制
        target_angular = self.clamp(self.heading_kp * selected_heading, -self.max_angular_speed, self.max_angular_speed)
        dt = max(1e-3, 1.0 / self.control_rate_hz)  # 控制周期
        max_delta = self.max_angular_accel * dt  # 最大角速度变化量
        # 应用加速度限制，确保平滑转向
        cmd.angular.z = self.clamp(target_angular, self.last_angular_cmd - max_delta, self.last_angular_cmd + max_delta)
        self.last_angular_cmd = cmd.angular.z

        # 线速度控制 - 多因素加权
        heading_factor = max(0.0, math.cos(abs(selected_heading)))  # 航向因子：航向偏差越大，速度越小
        clearance_factor = self.clamp((selected_dist - self.stop_distance) / max(1e-3, (self.max_depth - self.stop_distance)), 0.0, 1.0)  # 安全距离因子
        goal_factor = self.clamp(goal_dist / max(1e-3, self.goal_slowdown_dist), 0.0, 1.0)  # 目标距离因子

        # 综合速度计算：最大速度 × 航向因子 × 安全因子 × 目标因子
        cmd.linear.x = self.max_linear_speed * heading_factor * clearance_factor * goal_factor

        # 特殊行为调整
        if abs(selected_heading) > self.turn_in_place_angle:
            cmd.linear.x = min(cmd.linear.x, 0.05)  # 大角度转向时限制前进速度

        # 最小速度保证 - 在安全情况下保持最小移动速度
        if cmd.linear.x > 0.0 and cmd.linear.x < self.min_linear_speed and clearance_factor > 0.3:
            cmd.linear.x = self.min_linear_speed

        # 发布最终速度命令
        self.cmd_pub.publish(cmd)

    def compute_sector_distance(self, depth_img: np.ndarray) -> np.ndarray:
        """
        计算扇区距离 - VFH算法的核心预处理步骤
        
        将深度图像转换为每个扇区的最小障碍物距离
        通过相机几何模型将像素坐标转换为世界坐标，并计算到机器人的距离
        
        Args:
            depth_img: 深度图像数组，每个像素值表示到相机的距离
            
        Returns:
            np.ndarray: 每个扇区的最小障碍物距离，无障碍物时为无穷大
        """
        # 获取图像尺寸
        h, w = depth_img.shape

        # 1. 确定垂直采样范围 - 只关注机器人前方的有效区域
        v_start = max(0, int(h * self.scan_v_start_ratio))  # 垂直采样起始位置
        v_end = min(h, int(h * self.scan_v_end_ratio))      # 垂直采样结束位置
        # 参数有效性检查，确保采样范围合理
        if v_end <= v_start:
            v_start, v_end = int(0.4 * h), int(0.6 * h)  # 使用默认范围

        # 2. 图像采样 - 降低计算量，提高实时性
        sampled = depth_img[v_start:v_end:self.pixel_step_v, ::self.pixel_step_u]  # 按步长采样
        # 采样结果检查
        if sampled.size == 0:
            return np.full(self.num_sectors, np.inf, dtype=np.float32)  # 无有效数据，返回全无穷大

        # 3. 像素坐标归一化 - 准备相机几何转换
        u_coords = np.arange(0, w, self.pixel_step_u, dtype=np.float32)  # 水平像素坐标
        x_norm = (u_coords - self.cx) / self.fx  # 归一化x坐标 (相机坐标系)
        x_norm = np.tile(x_norm, sampled.shape[0])  # 扩展到所有采样行

        # 4. 深度数据预处理和有效性检查
        depths = sampled.reshape(-1).astype(np.float32)  # 展平深度数据
        valid = np.isfinite(depths)                     # 检查有限值（非NaN/Inf）
        valid &= depths > self.min_depth                # 大于最小有效距离
        valid &= depths < self.max_depth                # 小于最大有效距离

        # 检查是否有有效数据点
        if not np.any(valid):
            return np.full(self.num_sectors, np.inf, dtype=np.float32)  # 无有效障碍物数据

        # 5. 相机几何转换 - 像素坐标到世界坐标
        d = depths[valid]                    # 有效深度值
        x = x_norm[valid] * d                # 世界坐标系中的x坐标
        angles = -np.arctan2(x, d)           # 计算障碍物相对于相机的水平角度

        # 6. 视野范围过滤 - 只保留在视野角度内的障碍物
        in_fov = (angles >= -self.view_angle) & (angles <= self.view_angle)  # 视野角度检查
        if not np.any(in_fov):
            return np.full(self.num_sectors, np.inf, dtype=np.float32)  # 无视野内障碍物

        # 提取视野内的有效数据
        d = d[in_fov]        # 视野内的深度值
        x = x[in_fov]        # 视野内的x坐标
        angles = angles[in_fov]  # 视野内的角度值

        # 7. 计算到机器人基座的距离和扇区索引
        dist_from_base = np.hypot(d + self.camera_x_offset, x)  # 考虑相机偏移，计算到机器人基座的距离
        indices = ((angles + self.view_angle) / self.sector_angle).astype(np.int32)  # 计算扇区索引
        indices = np.clip(indices, 0, self.num_sectors - 1)  # 确保索引在有效范围内

        # 8. 计算每个扇区的最小距离
        sector_dist = np.full(self.num_sectors, np.inf, dtype=np.float32)  # 初始化扇区距离为无穷大
        np.minimum.at(sector_dist, indices, dist_from_base)  # 对每个扇区取最小距离
        return sector_dist  # 返回扇区距离数组

    def compute_blocked_sectors(self, sector_dist: np.ndarray) -> np.ndarray:
        """
        计算被阻挡的扇区 - VFH算法的障碍物膨胀处理
        
        根据扇区距离信息，考虑机器人半径和安全余量，判断哪些扇区被障碍物阻挡
        通过几何膨胀算法，将点障碍物扩展为考虑机器人尺寸的区域障碍物
        
        Args:
            sector_dist: 每个扇区的最小障碍物距离数组
            
        Returns:
            np.ndarray: 布尔数组，True表示该扇区被阻挡，False表示可通行
        """
        # 初始化阻挡状态数组，默认所有扇区都可通行
        blocked = np.zeros(self.num_sectors, dtype=bool)
        # 计算膨胀半径：机器人半径 + 安全余量
        inflate_radius = self.robot_radius + self.safety_margin

        # 遍历每个扇区，进行障碍物膨胀处理
        for idx, dist in enumerate(sector_dist):
            # 跳过无效或过远的障碍物：无穷大距离或超出最大膨胀范围
            if not np.isfinite(dist) or dist > self.inflation_max_range:
                continue

            # 计算障碍物膨胀的角度范围
            # 使用反正弦函数计算障碍物需要占据的视角范围
            spread = math.asin(min(0.999, inflate_radius / max(0.05, float(dist))))
            # 将角度范围转换为扇区数量（向上取整确保完全覆盖）
            spread_bins = int(math.ceil(spread / self.sector_angle))

            # 确定膨胀后的阻挡扇区范围
            left = max(0, idx - spread_bins)  # 左边界，确保不越界
            right = min(self.num_sectors - 1, idx + spread_bins)  # 右边界，确保不越界
            # 标记该范围内的所有扇区为阻挡状态
            blocked[left:right + 1] = True

        return blocked  # 返回阻挡状态数组

    def select_heading(
        self,
        goal_heading_robot: float,
        sector_dist: np.ndarray,
        blocked: np.ndarray,
    ) -> Tuple[float, float, int]:
        """
        VFH路径选择算法 - 多目标优化选择最优前进方向
        
        基于目标方向、运动平滑性和安全距离三个因素，通过成本函数选择最佳扇区
        实现目标导向、平滑运动和安全避障的平衡
        
        Args:
            goal_heading_robot: 机器人坐标系中的目标方向(弧度)
            sector_dist: 每个扇区的障碍物距离数组
            blocked: 每个扇区的阻挡状态数组
            
        Returns:
            Tuple[float, float, int]: (选择的航向角, 选择的距离, 选择的扇区索引)
        """
        # 1. 获取可通行扇区索引
        free_idx = np.where(~blocked)[0]  # 找出所有未被阻挡的扇区
        
        # 特殊情况处理：所有扇区都被阻挡
        if free_idx.size == 0:
            # 回退策略：向目标方向旋转，寻找出路
            fallback = math.copysign(self.view_angle, goal_heading_robot if abs(goal_heading_robot) > 1e-4 else 1.0)
            fallback_idx = int(np.argmin(np.abs(self.sector_centers - fallback)))  # 找到最接近的扇区
            return fallback, 0.0, fallback_idx

        # 2. 获取可通行扇区的中心角度
        free_angles = self.sector_centers[free_idx]  # 可通行扇区的角度值

        # 3. 计算三个成本分量
        # 目标成本：与目标方向的偏差，越小越接近目标
        goal_cost = np.abs(np.vectorize(self.normalize_angle)(free_angles - goal_heading_robot))
        # 平滑成本：与上一次选择方向的偏差，越小运动越平滑
        smooth_cost = np.abs(np.vectorize(self.normalize_angle)(free_angles - self.last_selected_heading))

        # 安全距离成本：与障碍物的距离，越大越安全
        clearance = sector_dist[free_idx].copy()  # 复制可通行扇区的距离
        clearance[~np.isfinite(clearance)] = self.max_depth  # 无穷大距离设为最大深度
        clearance = np.clip(clearance / self.max_depth, 0.0, 1.0)  # 归一化到[0,1]
        clearance_cost = 1.0 - clearance  # 转换为成本：距离越小成本越高

        # 4. 综合成本计算和最优选择
        # 加权综合成本：目标成本 + 平滑成本 + 安全成本
        total_cost = self.cost_goal * goal_cost + self.cost_smooth * smooth_cost + self.cost_clearance * clearance_cost
        best_local = int(np.argmin(total_cost))  # 在可通行扇区中找到最小成本索引
        best_idx = int(free_idx[best_local])  # 转换为全局扇区索引

        # 5. 滞后切换策略 - 防止方向频繁切换
        selected_idx = best_idx  # 默认选择当前最优扇区
        
        # 检查是否可以保持上一次的选择（防抖策略）
        if self.last_selected_sector_idx is not None and not blocked[self.last_selected_sector_idx]:
            prev_local = np.where(free_idx == self.last_selected_sector_idx)[0]  # 上一次选择的局部索引
            if prev_local.size > 0:
                prev_heading = float(self.sector_centers[self.last_selected_sector_idx])  # 上一次航向
                best_heading = float(self.sector_centers[best_idx])  # 当前最优航向
                prev_cost = float(total_cost[int(prev_local[0])])  # 上一次选择的成本
                best_cost = float(total_cost[best_local])  # 当前最优成本
                
                # 检查是否在允许保持航向的角度范围内
                can_hold_heading = (
                    abs(prev_heading) <= self.hold_heading_max
                    and abs(best_heading) <= self.hold_heading_max
                )
                
                # 滞后切换条件：当前最优成本 + 滞后阈值 >= 上一次选择的成本
                if can_hold_heading and best_cost + self.switch_hysteresis >= prev_cost:
                    selected_idx = int(self.last_selected_sector_idx)  # 保持上一次选择

        # 6. 返回最终选择结果
        # 获取选择扇区的距离（无效距离使用最大深度）
        selected_dist = float(sector_dist[selected_idx]) if np.isfinite(sector_dist[selected_idx]) else self.max_depth
        # 返回：选择的航向角、距离、扇区索引
        return float(self.sector_centers[selected_idx]), selected_dist, selected_idx

    def publish_stop(self) -> None:
        # 1. 重置角速度控制状态 - 确保停止后重新开始时的平滑性
        self.last_angular_cmd = 0.0
        
        # 2. 重置航向选择状态 - 清除上一次的导航决策
        self.last_selected_heading = 0.0
        self.last_selected_sector_idx = None
        
        # 3. 发布零速度命令 - 停止机器人所有运动
        # Twist()创建空的速度消息，所有速度分量默认为0
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _depth_image_to_numpy(msg: Image) -> Optional[np.ndarray]:
        # 1. 获取图像尺寸并检查有效性
        h = msg.height  # 图像高度（像素）
        w = msg.width   # 图像宽度（像素）
        if h == 0 or w == 0:
            # 5. 不支持其他编码格式
            return None  # 未知或不支持的图像编码格式  # 无效图像尺寸，返回None

        # 2. 获取图像编码格式并转换为大写
        encoding = msg.encoding.upper()  # 统一编码格式为大写，便于比较

        # 3. 处理32FC1格式（32位浮点单通道，单位：米）
        if encoding == '32FC1':
            bpp = 4  # 每个像素4字节（32位浮点数）
            row_elems = msg.step // bpp  # 计算每行的像素数量（考虑步长对齐）
            if row_elems < w:
                return None  # 步长不匹配，数据无效
            # 从消息数据缓冲区创建NumPy数组，并截取有效宽度
            arr = np.ndarray((h, row_elems), dtype=np.float32, buffer=msg.data)[:, :w]
            return np.array(arr, copy=True)  # 返回深度复制的数组，确保数据独立性

        # 4. 处理16UC1格式（16位无符号整数单通道，单位：毫米）
        if encoding == '16UC1':
            bpp = 2  # 每个像素2字节（16位无符号整数）
            row_elems = msg.step // bpp  # 计算每行的像素数量
            if row_elems < w:
                return None  # 步长不匹配，数据无效
            # 从消息数据缓冲区创建NumPy数组，并截取有效宽度
            arr = np.ndarray((h, row_elems), dtype=np.uint16, buffer=msg.data)[:, :w]
            # 转换为浮点数并乘以0.001将毫米转换为米
            return np.array(arr, dtype=np.float32, copy=True) * 0.001

        return None

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        # 计算偏航角的正弦部分：sin(yaw) = 2*(w*z + x*y)
        siny_cosp = 2.0 * (w * z + x * y)
        
        # 计算偏航角的余弦部分：cos(yaw) = 1 - 2*(y² + z²)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        
        # 使用atan2函数计算偏航角，确保正确的象限判断
        # atan2(sin, cos)自动处理所有象限，返回[-π, π]范围内的角度
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        # 参数有效性检查：下限不能大于上限
        if lo > hi:
            raise ValueError(f"clamp参数无效: lo({lo}) > hi({hi})")
        
        # 核心限制逻辑：max(lo, min(hi, value))
        # 1. 先使用min(value, hi)确保不超过上限
        # 2. 再使用max(lo, ...)确保不低于下限
        return max(lo, min(hi, value))

    @staticmethod
    def normalize_angle(angle: float) -> float:
        # 处理角度大于π的情况：通过减去2π循环到[-π, π]范围
        while angle > math.pi:
            angle -= 2.0 * math.pi
            
        # 处理角度小于-π的情况：通过加上2π循环到[-π, π]范围
        # 例如：-3π/2 → -3π/2 + 2π = π/2
        while angle < -math.pi:
            angle += 2.0 * math.pi
            
        return angle  # 返回归一化后的角度


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DepthVFHController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()