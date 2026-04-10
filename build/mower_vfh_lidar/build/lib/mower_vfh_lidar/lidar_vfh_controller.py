#!/usr/bin/env python3
"""
基于激光雷达的VFH（Vector Field Histogram）避障导航控制器

使用激光雷达数据进行实时避障和路径规划，实现机器人的自主导航
基于VFH算法，通过扇区距离直方图选择最优前进方向

作者: [自动生成]
版本: 1.0
"""

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LidarVFHController(Node):
    """
    激光雷达VFH控制器类
    
    基于激光雷达传感器数据实现VFH避障导航算法
    主要功能：
    - 激光雷达数据处理和扇区距离计算
    - 机器人状态跟踪和目标导航
    - 实时避障和路径规划
    - 速度命令生成和发布
    """
    def __init__(self) -> None:
        super().__init__('lidar_vfh_controller')

        # 话题配置 - 定义输入输出ROS话题名称
        self.declare_parameter('scan_topic', '/scan')  # 激光雷达扫描数据话题
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # VFH算法和控制参数 - 配置避障和导航行为
        self.declare_parameter('control_rate_hz', 15.0)           # 控制循环频率(Hz)，激光雷达版本通常有更高频率
        self.declare_parameter('sector_angle_deg', 5.0)           # 扇区角度(度)，激光雷达有更高角度分辨率
        self.declare_parameter('view_angle_deg', 150.0)           # 视野角度(度)，激光雷达通常有更宽视野
        self.declare_parameter('min_range', 0.2)                  # 最小有效距离(m)，过滤过近的噪声数据
        self.declare_parameter('max_range', 6.0)                  # 最大有效距离(m)，激光雷达的典型探测范围
        self.declare_parameter('inflation_max_range', 3.0)        # 最大膨胀范围(m)，只对近距离障碍物进行膨胀处理
        self.declare_parameter('robot_radius', 0.28)              # 机器人半径(m)，用于障碍物膨胀计算
        self.declare_parameter('safety_margin', 0.12)             # 安全余量(m)，额外的安全距离缓冲
        self.declare_parameter('goal_tolerance', 0.25)            # 目标容差(m)，到达目标点的判定距离
        self.declare_parameter('goal_slowdown_dist', 1.0)         # 目标减速距离(m)，接近目标时开始减速
        self.declare_parameter('stop_distance', 0.45)             # 停止距离(m)，与障碍物的最小安全距离
        self.declare_parameter('max_linear_speed', 0.40)          # 最大线速度(m/s)，机器人前进的最大速度
        self.declare_parameter('min_linear_speed', 0.05)          # 最小线速度(m/s)，确保机器人能够缓慢移动
        self.declare_parameter('max_angular_speed', 1.2)          # 最大角速度(rad/s)，激光雷达版本通常允许更快转向
        self.declare_parameter('heading_kp', 1.8)                 # 航向比例系数，激光雷达版本有更强的转向响应
        self.declare_parameter('turn_in_place_angle_deg', 50.0)   # 原地转向角度阈值(度)，大角度转向时限制前进
        self.declare_parameter('cost_goal', 1.0)                  # 目标成本权重，控制向目标前进的强度    
        self.declare_parameter('cost_smooth', 0.35)               # 平滑成本权重，激光雷达版本更注重运动平滑性
        self.declare_parameter('cost_clearance', 0.25)            # 安全距离成本权重，控制与障碍物保持距离的强度

        self.scan_topic = self.get_parameter('scan_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.sector_angle = math.radians(float(self.get_parameter('sector_angle_deg').value))
        self.view_angle = math.radians(float(self.get_parameter('view_angle_deg').value))
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.inflation_max_range = float(self.get_parameter('inflation_max_range').value)
        self.robot_radius = float(self.get_parameter('robot_radius').value)
        self.safety_margin = float(self.get_parameter('safety_margin').value)
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

        self.num_sectors = max(5, int(round((2.0 * self.view_angle) / self.sector_angle)))
        self.sector_angle = (2.0 * self.view_angle) / float(self.num_sectors)
        self.sector_centers = np.linspace(
            -self.view_angle + 0.5 * self.sector_angle,
            self.view_angle - 0.5 * self.sector_angle,
            self.num_sectors,
            dtype=np.float32,
        )

        self.latest_scan_dist: Optional[np.ndarray] = None
        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None
        self.robot_yaw: Optional[float] = None

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_frame: Optional[str] = None

        self.last_selected_heading = 0.0

        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info(
            f'LiDAR VFH ready. scan={self.scan_topic}, goal={self.goal_topic}, cmd_vel={self.cmd_vel_topic}'
        )

    def scan_callback(self, msg: LaserScan) -> None:
        """
        激光雷达扫描回调函数
        
        处理激光雷达数据，将原始距离数据转换为扇区距离信息
        与深度相机版本的主要区别：直接使用角度信息，无需相机几何转换
        
        Args:
            msg: LaserScan消息，包含激光雷达的距离和角度信息
        """
        # 1. 获取距离数据并检查有效性
        ranges = np.asarray(msg.ranges, dtype=np.float32)  # 转换为NumPy数组
        if ranges.size == 0:
            self.latest_scan_dist = None  # 无有效数据
            return

        # 2. 计算每个激光束的角度
        # 激光雷达数据是等角度间隔的，可以直接计算每个点的角度
        angle_array = msg.angle_min + np.arange(ranges.size, dtype=np.float32) * msg.angle_increment

        # 3. 数据有效性检查
        valid = np.isfinite(ranges)  # 检查有限值（非NaN/Inf）
        valid &= ranges > max(self.min_range, float(msg.range_min))  # 大于最小有效距离
        valid &= ranges < min(self.max_range, float(msg.range_max))  # 小于最大有效距离

        # 4. 视野范围过滤 - 只处理VFH算法关注的视野角度
        in_fov = (angle_array >= -self.view_angle) & (angle_array <= self.view_angle)
        valid &= in_fov  # 结合距离和角度有效性

        # 5. 检查是否有有效数据点
        if not np.any(valid):
            self.latest_scan_dist = np.full(self.num_sectors, np.inf, dtype=np.float32)  # 无有效数据，返回全无穷大
            return

        # 6. 提取有效数据
        angles = angle_array[valid]  # 有效角度
        dist = ranges[valid]         # 有效距离

        # 7. 计算扇区索引
        # 将角度映射到扇区索引：索引 = (角度 + 视野角度) / 扇区角度
        indices = ((angles + self.view_angle) / self.sector_angle).astype(np.int32)
        indices = np.clip(indices, 0, self.num_sectors - 1)  # 确保索引在有效范围内

        # 8. 计算每个扇区的最小距离
        sector_dist = np.full(self.num_sectors, np.inf, dtype=np.float32)  # 初始化扇区距离
        np.minimum.at(sector_dist, indices, dist)  # 对每个扇区取最小距离
        self.latest_scan_dist = sector_dist  # 更新最新扇区距离数据

    def odom_callback(self, msg: Odometry) -> None:
        """
        里程计回调函数
        
        从Odometry消息中提取机器人当前位置和姿态信息
        这些信息用于VFH算法中的路径规划和目标导航
        
        Args:
            msg: Odometry消息，包含机器人的位姿和速度信息
        """
        # 提取位置和姿态信息
        p = msg.pose.pose.position  # 位置信息
        q = msg.pose.pose.orientation  # 姿态信息（四元数）
        
        # 更新机器人状态
        self.robot_x = p.x  # 机器人x坐标
        self.robot_y = p.y  # 机器人y坐标
        self.robot_yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)  # 偏航角（从四元数转换）

    def goal_callback(self, msg: PoseStamped) -> None:
        """
        目标点回调函数
        
        接收导航目标点信息，提取目标位置和坐标系信息
        这是VFH导航算法的目标输入，触发机器人向指定位置移动
        
        Args:
            msg: PoseStamped消息，包含目标点的位姿和坐标系信息
        """
        # 提取目标点信息
        self.goal_x = float(msg.pose.position.x)  # 目标点x坐标
        self.goal_y = float(msg.pose.position.y)  # 目标点y坐标
        self.goal_frame = msg.header.frame_id  # 目标坐标系
        
        # 记录目标点接收信息
        self.get_logger().info(
            f'Received goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, frame={self.goal_frame or "(empty)"}'
        )

    def control_loop(self) -> None:
        """
        VFH控制循环 - 核心导航算法
        
        执行完整的VFH避障导航流程
        """
        # 1. 状态检查 - 确保所有必要数据可用
        if self.goal_x is None or self.goal_y is None:
            self.publish_stop()  # 无目标点，停止机器人
            return

        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            self.publish_stop()  # 机器人位置未知，停止
            return

        if self.latest_scan_dist is None:
            self.publish_stop()  # 激光雷达数据缺失，停止
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
            self.goal_x = None  # 清除目标点状态
            self.goal_y = None
            self.goal_frame = None
            return

        # 3. 航向计算和VFH算法执行
        goal_heading_world = math.atan2(dy, dx)  # 世界坐标系中的目标方向
        goal_heading_robot = self.normalize_angle(goal_heading_world - self.robot_yaw)  # 机器人坐标系中的目标方向

        # VFH核心算法
        blocked = self.compute_blocked_sectors(self.latest_scan_dist)  # 计算阻挡扇区
        selected_heading, selected_dist = self.select_heading(goal_heading_robot, self.latest_scan_dist, blocked)  # 选择最优航向
        self.last_selected_heading = selected_heading  # 记录选择的航向

        # 4. 速度命令生成
        cmd = Twist()  # 创建新的速度命令

        # 特殊情况处理 - 所有方向都被阻挡
        if np.all(blocked):
            cmd.linear.x = 0.0  # 停止前进
            # 原地旋转，尝试找到出路
            cmd.angular.z = math.copysign(
                self.max_angular_speed * 0.7,  # 使用70%的最大角速度
                goal_heading_robot if abs(goal_heading_robot) > 1e-4 else 1.0,  # 向目标方向旋转
            )
            self.cmd_pub.publish(cmd)
            return

        # 角速度控制 - 使用比例控制和速度限制
        cmd.angular.z = self.clamp(
            self.heading_kp * selected_heading,  # 比例控制：KP × 航向偏差
            -self.max_angular_speed,  # 下限：负最大角速度
            self.max_angular_speed,   # 上限：正最大角速度
        )

        # 线速度控制 - 多因素加权
        heading_factor = max(0.0, math.cos(abs(selected_heading)))  # 航向因子：航向偏差越大，速度越小
        clearance_factor = self.clamp(
            (selected_dist - self.stop_distance) / max(1e-3, (self.max_range - self.stop_distance)),
            0.0,
            1.0,
        )  # 安全距离因子：距离障碍物越近，速度越小
        goal_factor = self.clamp(goal_dist / max(1e-3, self.goal_slowdown_dist), 0.0, 1.0)  # 目标距离因子：接近目标时减速

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

    def compute_blocked_sectors(self, sector_dist: np.ndarray) -> np.ndarray:
        """
        计算被阻挡的扇区 - VFH算法的障碍物膨胀处理
        
        根据扇区距离信息，考虑机器人半径和安全余量，判断哪些扇区被障碍物阻挡
        通过几何膨胀算法，将点障碍物扩展为考虑机器人尺寸的区域障碍物
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
    ) -> Tuple[float, float]:
        """
        VFH路径选择算法 - 多目标优化选择最优前进方向
        
        基于目标方向、运动平滑性和安全距离三个因素，通过成本函数选择最佳扇区
        实现目标导向、平滑运动和安全避障的平衡
        """
        # 1. 获取可通行扇区索引
        free_idx = np.where(~blocked)[0]  # 找出所有未被阻挡的扇区
        
        # 特殊情况处理：所有扇区都被阻挡
        if free_idx.size == 0:
            # 回退策略：向目标方向旋转，寻找出路
            return math.copysign(self.view_angle, goal_heading_robot if abs(goal_heading_robot) > 1e-4 else 1.0), 0.0

        # 2. 获取可通行扇区的中心角度
        free_angles = self.sector_centers[free_idx]

        # 3. 计算三个成本分量
        goal_cost = np.abs(np.vectorize(self.normalize_angle)(free_angles - goal_heading_robot))  # 目标成本
        smooth_cost = np.abs(np.vectorize(self.normalize_angle)(free_angles - self.last_selected_heading))  # 平滑成本

        # 安全距离成本
        clearance = sector_dist[free_idx].copy()
        clearance[~np.isfinite(clearance)] = self.max_range  # 无效距离设为最大范围
        clearance = np.clip(clearance / self.max_range, 0.0, 1.0)  # 归一化
        clearance_cost = 1.0 - clearance  # 距离越小成本越高

        # 4. 综合成本计算和最优选择
        total_cost = self.cost_goal * goal_cost + self.cost_smooth * smooth_cost + self.cost_clearance * clearance_cost
        best_local = int(np.argmin(total_cost))  # 找到最小成本索引

        # 5. 返回选择结果
        best_idx = free_idx[best_local]
        selected_dist = float(sector_dist[best_idx]) if np.isfinite(sector_dist[best_idx]) else self.max_range
        return float(self.sector_centers[best_idx]), selected_dist

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())  # 发布零速度命令，停止所有运动

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        """
        从四元数中提取偏航角yaw- 机器人朝向计算
        """
        # 计算偏航角的正弦和余弦部分
        siny_cosp = 2.0 * (w * z + x * y)  # sin(yaw) = 2*(w*z + x*y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)  # cos(yaw) = 1 - 2*(y² + z²)
        
        # 使用atan2函数计算偏航角，确保正确的象限判断
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        """
        数值限制函数 - 将数值限制在指定范围内
        """
        return max(lo, min(hi, value))  # 核心限制逻辑：max(lo, min(hi, value))

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        角度归一化函数 - 将角度限制在[-π, π]范围内
        """
        # 处理角度大于π的情况：通过减去2π循环到[-π, π]范围
        while angle > math.pi:
            angle -= 2.0 * math.pi
            
        # 处理角度小于-π的情况：通过加上2π循环到[-π, π]范围
        while angle < -math.pi:
            angle += 2.0 * math.pi
            
        return angle  # 返回归一化后的角度


def main(args=None) -> None:
    rclpy.init(args=args)
    
    # 创建VFH控制器节点实例
    node = LidarVFHController()
    
    try:
        # 启动节点运行 - 进入消息处理循环
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        # 处理键盘中断信号（Ctrl+C）
        pass
        
    finally:
        # 资源清理阶段 - 确保在任何情况下都能正确关闭
        node.publish_stop()  # 发布停止命令
        node.destroy_node()  # 销毁节点
        rclpy.shutdown()     # 关闭ROS 2系统


if __name__ == '__main__':
    main()