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
        super().__init__('depth_vfh_controller')

        # Topics
        self.declare_parameter('depth_topic', '/mower/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/mower/camera/camera_info')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')

        # VFH / control parameters
        self.declare_parameter('control_rate_hz', 12.0)
        self.declare_parameter('sector_angle_deg', 5.0)
        self.declare_parameter('view_angle_deg', 75.0)
        self.declare_parameter('min_depth', 0.25)
        self.declare_parameter('max_depth', 3.5)
        self.declare_parameter('inflation_max_range', 2.5)
        self.declare_parameter('robot_radius', 0.28)
        self.declare_parameter('safety_margin', 0.12)
        self.declare_parameter('camera_x_offset', 0.22)
        self.declare_parameter('pixel_step_u', 4)
        self.declare_parameter('pixel_step_v', 6)
        self.declare_parameter('scan_v_start_ratio', 0.35)
        self.declare_parameter('scan_v_end_ratio', 0.65)
        self.declare_parameter('goal_tolerance', 0.25)
        self.declare_parameter('goal_slowdown_dist', 1.0)
        self.declare_parameter('stop_distance', 0.45)
        self.declare_parameter('max_linear_speed', 0.40)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('max_angular_speed', 1.2)
        self.declare_parameter('heading_kp', 1.8)
        self.declare_parameter('turn_in_place_angle_deg', 50.0)
        self.declare_parameter('cost_goal', 1.0)
        self.declare_parameter('cost_smooth', 0.35)
        self.declare_parameter('cost_clearance', 0.25)

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.goal_topic = self.get_parameter('goal_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value

        self.control_rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.sector_angle = math.radians(float(self.get_parameter('sector_angle_deg').value))
        self.view_angle = math.radians(float(self.get_parameter('view_angle_deg').value))
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

        self.num_sectors = max(5, int(round((2.0 * self.view_angle) / self.sector_angle)))
        self.sector_angle = (2.0 * self.view_angle) / float(self.num_sectors)
        self.sector_centers = np.linspace(
            -self.view_angle + 0.5 * self.sector_angle,
            self.view_angle - 0.5 * self.sector_angle,
            self.num_sectors,
            dtype=np.float32,
        )

        self.latest_depth: Optional[np.ndarray] = None
        self.fx: Optional[float] = None
        self.cx: Optional[float] = None

        self.robot_x: Optional[float] = None
        self.robot_y: Optional[float] = None
        self.robot_yaw: Optional[float] = None

        self.goal_x: Optional[float] = None
        self.goal_y: Optional[float] = None
        self.goal_frame: Optional[str] = None

        self.last_selected_heading = 0.0

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        self.create_subscription(Image, self.depth_topic, self.depth_callback, sensor_qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_callback, sensor_qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(PoseStamped, self.goal_topic, self.goal_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.control_loop)

        self.get_logger().info(
            f'Depth VFH ready. depth={self.depth_topic}, goal={self.goal_topic}, cmd_vel={self.cmd_vel_topic}'
        )

    def camera_info_callback(self, msg: CameraInfo) -> None:
        self.fx = float(msg.k[0])
        self.cx = float(msg.k[2])

    def depth_callback(self, msg: Image) -> None:
        array = self._depth_image_to_numpy(msg)
        if array is not None:
            self.latest_depth = array

    def odom_callback(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.robot_x = p.x
        self.robot_y = p.y
        self.robot_yaw = self._yaw_from_quaternion(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg: PoseStamped) -> None:
        self.goal_x = float(msg.pose.position.x)
        self.goal_y = float(msg.pose.position.y)
        self.goal_frame = msg.header.frame_id
        self.get_logger().info(
            f'Received goal: x={self.goal_x:.2f}, y={self.goal_y:.2f}, frame={self.goal_frame or "(empty)"}'
        )

    def control_loop(self) -> None:
        if self.goal_x is None or self.goal_y is None:
            self.publish_stop()
            return

        if self.robot_x is None or self.robot_y is None or self.robot_yaw is None:
            self.publish_stop()
            return

        if self.fx is None or self.cx is None or self.latest_depth is None:
            self.publish_stop()
            return

        if self.goal_frame not in (None, '', 'odom'):
            self.get_logger().warn(
                f'Goal frame {self.goal_frame} is not odom. Current node assumes odom frame for goals.',
                throttle_duration_sec=2.0,
            )

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y
        goal_dist = math.hypot(dx, dy)

        if goal_dist < self.goal_tolerance:
            self.publish_stop()
            self.get_logger().info('Goal reached (within tolerance).')
            self.goal_x = None
            self.goal_y = None
            self.goal_frame = None
            return

        goal_heading_world = math.atan2(dy, dx)
        goal_heading_robot = self.normalize_angle(goal_heading_world - self.robot_yaw)

        sector_dist = self.compute_sector_distance(self.latest_depth)
        blocked = self.compute_blocked_sectors(sector_dist)

        selected_heading, selected_dist = self.select_heading(
            goal_heading_robot,
            sector_dist,
            blocked,
        )
        self.last_selected_heading = selected_heading

        cmd = Twist()

        if np.all(blocked):
            cmd.linear.x = 0.0
            cmd.angular.z = math.copysign(self.max_angular_speed * 0.7, goal_heading_robot if abs(goal_heading_robot) > 1e-4 else 1.0)
            self.cmd_pub.publish(cmd)
            return

        cmd.angular.z = self.clamp(self.heading_kp * selected_heading, -self.max_angular_speed, self.max_angular_speed)

        heading_factor = max(0.0, math.cos(abs(selected_heading)))
        clearance_factor = self.clamp((selected_dist - self.stop_distance) / max(1e-3, (self.max_depth - self.stop_distance)), 0.0, 1.0)
        goal_factor = self.clamp(goal_dist / max(1e-3, self.goal_slowdown_dist), 0.0, 1.0)

        cmd.linear.x = self.max_linear_speed * heading_factor * clearance_factor * goal_factor

        if abs(selected_heading) > self.turn_in_place_angle:
            cmd.linear.x = min(cmd.linear.x, 0.05)

        if cmd.linear.x > 0.0 and cmd.linear.x < self.min_linear_speed and clearance_factor > 0.3:
            cmd.linear.x = self.min_linear_speed

        self.cmd_pub.publish(cmd)

    def compute_sector_distance(self, depth_img: np.ndarray) -> np.ndarray:
        h, w = depth_img.shape

        v_start = max(0, int(h * self.scan_v_start_ratio))
        v_end = min(h, int(h * self.scan_v_end_ratio))
        if v_end <= v_start:
            v_start, v_end = int(0.4 * h), int(0.6 * h)

        sampled = depth_img[v_start:v_end:self.pixel_step_v, ::self.pixel_step_u]
        if sampled.size == 0:
            return np.full(self.num_sectors, np.inf, dtype=np.float32)

        u_coords = np.arange(0, w, self.pixel_step_u, dtype=np.float32)
        x_norm = (u_coords - self.cx) / self.fx
        x_norm = np.tile(x_norm, sampled.shape[0])

        depths = sampled.reshape(-1).astype(np.float32)
        valid = np.isfinite(depths)
        valid &= depths > self.min_depth
        valid &= depths < self.max_depth

        if not np.any(valid):
            return np.full(self.num_sectors, np.inf, dtype=np.float32)

        d = depths[valid]
        x = x_norm[valid] * d
        angles = -np.arctan2(x, d)

        in_fov = (angles >= -self.view_angle) & (angles <= self.view_angle)
        if not np.any(in_fov):
            return np.full(self.num_sectors, np.inf, dtype=np.float32)

        d = d[in_fov]
        x = x[in_fov]
        angles = angles[in_fov]

        dist_from_base = np.hypot(d + self.camera_x_offset, x)
        indices = ((angles + self.view_angle) / self.sector_angle).astype(np.int32)
        indices = np.clip(indices, 0, self.num_sectors - 1)

        sector_dist = np.full(self.num_sectors, np.inf, dtype=np.float32)
        np.minimum.at(sector_dist, indices, dist_from_base)
        return sector_dist

    def compute_blocked_sectors(self, sector_dist: np.ndarray) -> np.ndarray:
        blocked = np.zeros(self.num_sectors, dtype=bool)
        inflate_radius = self.robot_radius + self.safety_margin

        for idx, dist in enumerate(sector_dist):
            if not np.isfinite(dist) or dist > self.inflation_max_range:
                continue

            spread = math.asin(min(0.999, inflate_radius / max(0.05, float(dist))))
            spread_bins = int(math.ceil(spread / self.sector_angle))

            left = max(0, idx - spread_bins)
            right = min(self.num_sectors - 1, idx + spread_bins)
            blocked[left:right + 1] = True

        return blocked

    def select_heading(
        self,
        goal_heading_robot: float,
        sector_dist: np.ndarray,
        blocked: np.ndarray,
    ) -> Tuple[float, float]:
        free_idx = np.where(~blocked)[0]
        if free_idx.size == 0:
            return math.copysign(self.view_angle, goal_heading_robot if abs(goal_heading_robot) > 1e-4 else 1.0), 0.0

        free_angles = self.sector_centers[free_idx]

        goal_cost = np.abs(np.vectorize(self.normalize_angle)(free_angles - goal_heading_robot))
        smooth_cost = np.abs(np.vectorize(self.normalize_angle)(free_angles - self.last_selected_heading))

        clearance = sector_dist[free_idx].copy()
        clearance[~np.isfinite(clearance)] = self.max_depth
        clearance = np.clip(clearance / self.max_depth, 0.0, 1.0)
        clearance_cost = 1.0 - clearance

        total_cost = self.cost_goal * goal_cost + self.cost_smooth * smooth_cost + self.cost_clearance * clearance_cost
        best_local = int(np.argmin(total_cost))

        best_idx = free_idx[best_local]
        selected_dist = float(sector_dist[best_idx]) if np.isfinite(sector_dist[best_idx]) else self.max_depth
        return float(self.sector_centers[best_idx]), selected_dist

    def publish_stop(self) -> None:
        self.cmd_pub.publish(Twist())

    @staticmethod
    def _depth_image_to_numpy(msg: Image) -> Optional[np.ndarray]:
        h = msg.height
        w = msg.width
        if h == 0 or w == 0:
            return None

        encoding = msg.encoding.upper()

        if encoding == '32FC1':
            bpp = 4
            row_elems = msg.step // bpp
            if row_elems < w:
                return None
            arr = np.ndarray((h, row_elems), dtype=np.float32, buffer=msg.data)[:, :w]
            return np.array(arr, copy=True)

        if encoding == '16UC1':
            bpp = 2
            row_elems = msg.step // bpp
            if row_elems < w:
                return None
            arr = np.ndarray((h, row_elems), dtype=np.uint16, buffer=msg.data)[:, :w]
            return np.array(arr, dtype=np.float32, copy=True) * 0.001

        return None

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def clamp(value: float, lo: float, hi: float) -> float:
        return max(lo, min(hi, value))

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


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
