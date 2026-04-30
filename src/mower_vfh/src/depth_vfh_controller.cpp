/**
 * @file depth_vfh_controller.cpp
 * @brief 基于深度相机的VFH（Vector Field Histogram）避障导航控制器
 * 
 * 该控制器使用深度相机数据构建扇区直方图，计算障碍物分布情况，
 * 并根据目标位置和障碍物信息选择最优导航方向，实现避障导航功能。
 * 
 * 主要功能：
 * 1. 处理深度相机数据，构建扇区距离分布
 * 2. 计算目标方向与障碍物分布的成本函数
 * 3. 选择最优导航方向，生成速度命令
 * 4. 支持实时参数配置
 * 
 * 依赖：
 * - rclcpp: ROS 2 C++客户端库
 * - geometry_msgs: 几何消息类型
 * - nav_msgs: 导航消息类型
 * - sensor_msgs: 传感器消息类型
 * - Eigen: 线性代数库
 */

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <functional>
#include <memory>

using namespace std;
using namespace Eigen;
using namespace std::placeholders;

/**
 * @class DepthVFHController
 * @brief 基于深度相机的VFH避障导航控制器类
 * 
 * 该类继承自rclcpp::Node，实现了基于VFH算法的避障导航功能。
 * 通过处理深度相机数据，构建扇区直方图，计算障碍物分布，
 * 并根据目标位置选择最优导航方向，生成速度命令。
 */
class DepthVFHController : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * 
     * 初始化控制器，声明参数，创建订阅者、发布者和定时器。
     */
    DepthVFHController() : Node("depth_vfh_controller") {
        // 声明参数
        declare_parameter("depth_topic", "/mower/camera/depth/image_raw");
        declare_parameter("camera_info_topic", "/mower/camera/camera_info");
        declare_parameter("odom_topic", "/odom");
        declare_parameter("goal_topic", "/goal_pose");
        declare_parameter("cmd_vel_topic", "/cmd_vel");
        
        // VFH算法和控制参数
        declare_parameter("control_rate_hz", 12.0);
        declare_parameter("sector_angle_deg", 10.0);
        declare_parameter("view_angle_deg", 75.0);
        declare_parameter("min_depth", 0.45);
        declare_parameter("max_depth", 5.0);
        declare_parameter("inflation_max_range", 1.8);
        declare_parameter("robot_radius", 0.28);
        declare_parameter("safety_margin", 0.08);
        declare_parameter("camera_x_offset", 0.22);
        declare_parameter("pixel_step_u", 6);
        declare_parameter("pixel_step_v", 8);
        declare_parameter("scan_v_start_ratio", 0.20);
        declare_parameter("scan_v_end_ratio", 0.50);
        declare_parameter("goal_tolerance", 0.25);
        declare_parameter("goal_slowdown_dist", 1.0);
        declare_parameter("stop_distance", 0.5);
        declare_parameter("max_linear_speed", 0.40);
        declare_parameter("min_linear_speed", 0.05);
        declare_parameter("max_angular_speed", 0.7);
        declare_parameter("heading_kp", 0.55);
        declare_parameter("turn_in_place_angle_deg", 55.0);
        declare_parameter("cost_goal", 1.5);
        declare_parameter("cost_smooth", 0.6);
        declare_parameter("cost_clearance", 0.2);
        declare_parameter("switch_hysteresis", 0.04);
        declare_parameter("max_angular_accel", 2.4);
        declare_parameter("hold_heading_max_deg", 35.0);
        
        // 获取参数
        get_parameter("depth_topic", depth_topic_);
        get_parameter("camera_info_topic", camera_info_topic_);
        get_parameter("odom_topic", odom_topic_);
        get_parameter("goal_topic", goal_topic_);
        get_parameter("cmd_vel_topic", cmd_vel_topic_);
        
        get_parameter("control_rate_hz", control_rate_hz_);
        get_parameter("sector_angle_deg", sector_angle_deg_);
        get_parameter("view_angle_deg", view_angle_deg_);
        get_parameter("min_depth", min_depth_);
        get_parameter("max_depth", max_depth_);
        get_parameter("inflation_max_range", inflation_max_range_);
        get_parameter("robot_radius", robot_radius_);
        get_parameter("safety_margin", safety_margin_);
        get_parameter("camera_x_offset", camera_x_offset_);
        get_parameter("pixel_step_u", pixel_step_u_);
        get_parameter("pixel_step_v", pixel_step_v_);
        get_parameter("scan_v_start_ratio", scan_v_start_ratio_);
        get_parameter("scan_v_end_ratio", scan_v_end_ratio_);
        get_parameter("goal_tolerance", goal_tolerance_);
        get_parameter("goal_slowdown_dist", goal_slowdown_dist_);
        get_parameter("stop_distance", stop_distance_);
        get_parameter("max_linear_speed", max_linear_speed_);
        get_parameter("min_linear_speed", min_linear_speed_);
        get_parameter("max_angular_speed", max_angular_speed_);
        get_parameter("heading_kp", heading_kp_);
        get_parameter("turn_in_place_angle_deg", turn_in_place_angle_deg_);
        get_parameter("cost_goal", cost_goal_);
        get_parameter("cost_smooth", cost_smooth_);
        get_parameter("cost_clearance", cost_clearance_);
        get_parameter("switch_hysteresis", switch_hysteresis_);
        get_parameter("max_angular_accel", max_angular_accel_);
        get_parameter("hold_heading_max_deg", hold_heading_max_deg_);
        
        // 转换单位
        sector_angle_ = M_PI * sector_angle_deg_ / 180.0;
        view_angle_ = M_PI * view_angle_deg_ / 180.0;
        turn_in_place_angle_ = M_PI * turn_in_place_angle_deg_ / 180.0;
        hold_heading_max_ = M_PI * hold_heading_max_deg_ / 180.0;
        
        // 计算扇区数量和角度
        num_sectors_ = max(5, static_cast<int>(round((2.0 * view_angle_) / sector_angle_)));
        sector_angle_ = (2.0 * view_angle_) / static_cast<double>(num_sectors_);
        
        // 初始化扇区中心角度
        sector_centers_.resize(num_sectors_);
        for (int i = 0; i < num_sectors_; i++) {
            sector_centers_[i] = -view_angle_ + 0.5 * sector_angle_ + i * sector_angle_;
        }
        
        // 初始化状态变量
        latest_depth_ = nullptr;
        fx_ = 0.0;
        cx_ = 0.0;
        
        robot_x_ = robot_y_ = robot_yaw_ = numeric_limits<double>::quiet_NaN();
        goal_x_ = goal_y_ = numeric_limits<double>::quiet_NaN();
        goal_frame_ = "";
        
        last_selected_heading_ = 0.0;
        last_selected_sector_idx_ = -1;
        last_angular_cmd_ = 0.0;
        
        // 创建订阅者
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            depth_topic_, 10, std::bind(&DepthVFHController::depth_callback, this, _1));
        
        camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            camera_info_topic_, 10, std::bind(&DepthVFHController::camera_info_callback, this, _1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&DepthVFHController::odom_callback, this, _1));
        
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_, 10, std::bind(&DepthVFHController::goal_callback, this, _1));
        
        // 创建发布者
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        
        // 创建定时器
        timer_ = create_wall_timer(
            chrono::duration<double>(1.0 / control_rate_hz_),
            std::bind(&DepthVFHController::control_loop, this));
        
        RCLCPP_INFO(get_logger(), "Depth VFH ready. depth=%s, goal=%s, cmd_vel=%s", 
                   depth_topic_.c_str(), goal_topic_.c_str(), cmd_vel_topic_.c_str());
    }
    
private:
    // 话题名称
    string depth_topic_;
    string camera_info_topic_;
    string odom_topic_;
    string goal_topic_;
    string cmd_vel_topic_;
    
    // VFH参数
    double control_rate_hz_;
    double sector_angle_deg_;
    double view_angle_deg_;
    double min_depth_;
    double max_depth_;
    double inflation_max_range_;
    double robot_radius_;
    double safety_margin_;
    double camera_x_offset_;
    int pixel_step_u_;
    int pixel_step_v_;
    double scan_v_start_ratio_;
    double scan_v_end_ratio_;
    double goal_tolerance_;
    double goal_slowdown_dist_;
    double stop_distance_;
    double max_linear_speed_;
    double min_linear_speed_;
    double max_angular_speed_;
    double heading_kp_;
    double turn_in_place_angle_deg_;
    double cost_goal_;
    double cost_smooth_;
    double cost_clearance_;
    double switch_hysteresis_;
    double max_angular_accel_;
    double hold_heading_max_deg_;
    
    // 转换后的参数
    double sector_angle_;
    double view_angle_;
    double turn_in_place_angle_;
    double hold_heading_max_;
    int num_sectors_;
    vector<double> sector_centers_;
    
    // 状态变量
    sensor_msgs::msg::Image::SharedPtr latest_depth_;
    double fx_;
    double cx_;
    
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    double goal_x_;
    double goal_y_;
    string goal_frame_;
    double last_selected_heading_;
    int last_selected_sector_idx_;
    double last_angular_cmd_;
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    /**
     * @brief 相机内参回调函数
     * 
     * 从CameraInfo消息中提取相机内参矩阵的关键参数，用于深度图像到3D坐标的转换
     * 
     * @param msg 相机信息消息
     */
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
        fx_ = msg->k[0];  // 焦距x
        cx_ = msg->k[2];  // 主点x
    }
    
    /**
     * @brief 深度图像回调函数
     * 
     * 接收深度图像消息，存储以供VFH算法使用
     * 
     * @param msg 深度图像消息
     */
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        latest_depth_ = msg;
    }
    
    /**
     * @brief 里程计回调函数
     * 
     * 更新机器人的当前位置和姿态（偏航角）
     * 
     * @param msg 里程计消息
     */
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        const auto& p = msg->pose.pose.position;
        const auto& q = msg->pose.pose.orientation;
        
        robot_x_ = p.x;
        robot_y_ = p.y;
        robot_yaw_ = yaw_from_quaternion(q.x, q.y, q.z, q.w);
    }
    
    /**
     * @brief 目标位置回调函数
     * 
     * 更新导航目标位置
     * 
     * @param msg 目标位置消息
     */
    void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_x_ = msg->pose.position.x;
        goal_y_ = msg->pose.position.y;
        goal_frame_ = msg->header.frame_id;
        
        RCLCPP_INFO(get_logger(), "Received goal: x=%.2f, y=%.2f, frame=%s", 
                   goal_x_, goal_y_, goal_frame_.empty() ? "(empty)" : goal_frame_.c_str());
    }
    
    /**
     * @brief 控制循环回调函数
     * 
     * 主要控制逻辑，包括：
     * 1. 状态检查
     * 2. 目标距离和方向计算
     * 3. 扇区距离和障碍物检测
     * 4. 最优航向选择
     * 5. 速度命令生成和发布
     */
    void control_loop() {
        // 状态检查
        if (isnan(goal_x_) || isnan(goal_y_)) {
            publish_stop();
            return;
        }
        
        if (isnan(robot_x_) || isnan(robot_y_) || isnan(robot_yaw_)) {
            publish_stop();
            return;
        }
        
        if (fx_ == 0.0 || cx_ == 0.0 || !latest_depth_) {
            publish_stop();
            return;
        }
        
        // 坐标系检查
        if (!goal_frame_.empty() && goal_frame_ != "odom") {
            RCLCPP_WARN(get_logger(), "Goal frame %s is not odom. Current node assumes odom frame for goals.", 
                       goal_frame_.c_str());
        }
        
        // 计算目标距离和方向
        double dx = goal_x_ - robot_x_;
        double dy = goal_y_ - robot_y_;
        double goal_dist = sqrt(dx * dx + dy * dy);
        
        // 检查是否到达目标
        if (goal_dist < goal_tolerance_) {
            publish_stop();
            RCLCPP_INFO(get_logger(), "Goal reached (within tolerance).");
            goal_x_ = goal_y_ = numeric_limits<double>::quiet_NaN();
            goal_frame_ = "";
            return;
        }
        
        // 计算目标方向
        double goal_heading_world = atan2(dy, dx);
        double goal_heading_robot = normalize_angle(goal_heading_world - robot_yaw_);
        
        // VFH核心算法
        vector<double> sector_dist = compute_sector_distance(latest_depth_);
        vector<bool> blocked = compute_blocked_sectors(sector_dist);
        
        // 选择最优航向
        tuple<double, double, int> selected = select_heading(goal_heading_robot, sector_dist, blocked);
        double selected_heading = get<0>(selected);
        double selected_dist = get<1>(selected);
        int selected_sector_idx = get<2>(selected);
        
        last_selected_heading_ = selected_heading;
        last_selected_sector_idx_ = selected_sector_idx;
        
        // 生成速度命令
        geometry_msgs::msg::Twist cmd;
        
        // 检查是否所有方向都被阻挡
        bool all_blocked = true;
        for (bool b : blocked) {
            if (!b) {
                all_blocked = false;
                break;
            }
        }
        
        if (all_blocked) {
            cmd.linear.x = 0.0;
            double direction = (fabs(goal_heading_robot) > 1e-4) ? goal_heading_robot : 1.0;
            cmd.angular.z = copysign(max_angular_speed_ * 0.7, direction);
            last_angular_cmd_ = cmd.angular.z;
            cmd_pub_->publish(cmd);
            return;
        }
        
        // 角速度控制
        double target_angular = clamp(heading_kp_ * selected_heading, -max_angular_speed_, max_angular_speed_);
        double dt = max(1e-3, 1.0 / control_rate_hz_);
        double max_delta = max_angular_accel_ * dt;
        cmd.angular.z = clamp(target_angular, last_angular_cmd_ - max_delta, last_angular_cmd_ + max_delta);
        last_angular_cmd_ = cmd.angular.z;
        
        // 线速度控制
        double heading_factor = max(0.0, cos(fabs(selected_heading)));
        double clearance_factor = clamp((selected_dist - stop_distance_) / max(1e-3, max_depth_ - stop_distance_), 0.0, 1.0);
        double goal_factor = clamp(goal_dist / max(1e-3, goal_slowdown_dist_), 0.0, 1.0);
        
        cmd.linear.x = max_linear_speed_ * heading_factor * clearance_factor * goal_factor;
        
        // 特殊行为调整
        if (fabs(selected_heading) > turn_in_place_angle_) {
            cmd.linear.x = min(cmd.linear.x, 0.05);
        }
        
        if (cmd.linear.x > 0.0 && cmd.linear.x < min_linear_speed_ && clearance_factor > 0.3) {
            cmd.linear.x = min_linear_speed_;
        }
        
        // 发布速度命令
        cmd_pub_->publish(cmd);
    }
    
    /**
     * @brief 计算扇区距离
     * 
     * 将深度图像转换为每个扇区的最小障碍物距离
     * 
     * @param depth_img 深度图像
     * @return 每个扇区的最小障碍物距离
     */
    vector<double> compute_sector_distance(const sensor_msgs::msg::Image::SharedPtr depth_img) {
        vector<double> sector_dist(num_sectors_, numeric_limits<double>::infinity());
        
        int h = depth_img->height;
        int w = depth_img->width;
        
        if (h == 0 || w == 0) {
            return sector_dist;
        }
        
        // 确定垂直采样范围
        int v_start = max(0, static_cast<int>(h * scan_v_start_ratio_));
        int v_end = min(h, static_cast<int>(h * scan_v_end_ratio_));
        
        if (v_end <= v_start) {
            v_start = static_cast<int>(0.4 * h);
            v_end = static_cast<int>(0.6 * h);
        }
        
        // 处理深度图像
        string encoding = depth_img->encoding;
        if (encoding == "32FC1") {
            process_depth_32fc1(depth_img, v_start, v_end, sector_dist);
        } else if (encoding == "16UC1") {
            process_depth_16uc1(depth_img, v_start, v_end, sector_dist);
        }
        
        return sector_dist;
    }
    
    /**
     * @brief 处理32FC1格式的深度图像
     * 
     * @param depth_img 深度图像
     * @param v_start 垂直起始位置
     * @param v_end 垂直结束位置
     * @param sector_dist 扇区距离数组
     */
    void process_depth_32fc1(const sensor_msgs::msg::Image::SharedPtr depth_img, 
                           int v_start, int v_end, vector<double>& sector_dist) {
        int h = depth_img->height;
        int w = depth_img->width;
        
        for (int v = v_start; v < v_end; v += pixel_step_v_) {
            for (int u = 0; u < w; u += pixel_step_u_) {
                size_t idx = v * depth_img->step + u * 4;
                if (idx + 3 >= depth_img->data.size()) continue;
                
                float depth = *reinterpret_cast<const float*>(&depth_img->data[idx]);
                if (!isfinite(depth) || depth < min_depth_ || depth > max_depth_) continue;
                
                double x_norm = (u - cx_) / fx_;
                double angle = -atan2(x_norm * depth, depth);
                
                if (angle < -view_angle_ || angle > view_angle_) continue;
                
                double dist_from_base = sqrt(pow(depth + camera_x_offset_, 2) + pow(x_norm * depth, 2));
                int sector_idx = static_cast<int>((angle + view_angle_) / sector_angle_);
                sector_idx = max(0, min(num_sectors_ - 1, sector_idx));
                
                if (dist_from_base < sector_dist[sector_idx]) {
                    sector_dist[sector_idx] = dist_from_base;
                }
            }
        }
    }
    
    /**
     * @brief 处理16UC1格式的深度图像
     * 
     * @param depth_img 深度图像
     * @param v_start 垂直起始位置
     * @param v_end 垂直结束位置
     * @param sector_dist 扇区距离数组
     */
    void process_depth_16uc1(const sensor_msgs::msg::Image::SharedPtr depth_img, 
                           int v_start, int v_end, vector<double>& sector_dist) {
        int h = depth_img->height;
        int w = depth_img->width;
        
        for (int v = v_start; v < v_end; v += pixel_step_v_) {
            for (int u = 0; u < w; u += pixel_step_u_) {
                size_t idx = v * depth_img->step + u * 2;
                if (idx + 1 >= depth_img->data.size()) continue;
                
                uint16_t depth_mm = *reinterpret_cast<const uint16_t*>(&depth_img->data[idx]);
                double depth = depth_mm * 0.001;  // 转换为米
                
                if (!isfinite(depth) || depth < min_depth_ || depth > max_depth_) continue;
                
                double x_norm = (u - cx_) / fx_;
                double angle = -atan2(x_norm * depth, depth);
                
                if (angle < -view_angle_ || angle > view_angle_) continue;
                
                double dist_from_base = sqrt(pow(depth + camera_x_offset_, 2) + pow(x_norm * depth, 2));
                int sector_idx = static_cast<int>((angle + view_angle_) / sector_angle_);
                sector_idx = max(0, min(num_sectors_ - 1, sector_idx));
                
                if (dist_from_base < sector_dist[sector_idx]) {
                    sector_dist[sector_idx] = dist_from_base;
                }
            }
        }
    }
    
    /**
     * @brief 计算阻挡扇区
     * 
     * 根据扇区距离信息，考虑机器人半径和安全余量，判断哪些扇区被障碍物阻挡
     * 
     * @param sector_dist 扇区距离数组
     * @return 阻挡扇区标记数组
     */
    vector<bool> compute_blocked_sectors(const vector<double>& sector_dist) {
        vector<bool> blocked(num_sectors_, false);
        double inflate_radius = robot_radius_ + safety_margin_;
        
        for (int i = 0; i < num_sectors_; i++) {
            double dist = sector_dist[i];
            if (!isfinite(dist) || dist > inflation_max_range_) {
                continue;
            }
            
            double spread = asin(min(0.999, inflate_radius / max(0.05, dist)));
            int spread_bins = static_cast<int>(ceil(spread / sector_angle_));
            
            int left = max(0, i - spread_bins);
            int right = min(num_sectors_ - 1, i + spread_bins);
            
            for (int j = left; j <= right; j++) {
                blocked[j] = true;
            }
        }
        
        return blocked;
    }
    
    /**
     * @brief 选择最优航向
     * 
     * 根据目标方向、当前方向和障碍物分布，选择最优的导航方向
     * 
     * @param goal_heading_robot 目标方向（机器人坐标系）
     * @param sector_dist 扇区距离数组
     * @param blocked 阻挡扇区标记数组
     * @return 最优航向、距离和扇区索引
     */
    tuple<double, double, int> select_heading(double goal_heading_robot, 
                                           const vector<double>& sector_dist, 
                                           const vector<bool>& blocked) {
        vector<int> free_idx;
        for (int i = 0; i < num_sectors_; i++) {
            if (!blocked[i]) {
                free_idx.push_back(i);
            }
        }
        
        if (free_idx.empty()) {
            double direction = (fabs(goal_heading_robot) > 1e-4) ? goal_heading_robot : 1.0;
            double fallback = copysign(view_angle_, direction);
            int fallback_idx = 0;
            double min_diff = numeric_limits<double>::max();
            for (int i = 0; i < num_sectors_; i++) {
                double diff = fabs(sector_centers_[i] - fallback);
                if (diff < min_diff) {
                    min_diff = diff;
                    fallback_idx = i;
                }
            }
            return {fallback, 0.0, fallback_idx};
        }
        
        vector<double> free_angles;
        vector<double> free_distances;
        for (int idx : free_idx) {
            free_angles.push_back(sector_centers_[idx]);
            free_distances.push_back(sector_dist[idx]);
        }
        
        vector<double> total_cost(free_idx.size());
        size_t best_idx = 0;
        double min_cost = numeric_limits<double>::max();
        
        for (size_t i = 0; i < free_idx.size(); i++) {
            double angle = free_angles[i];
            double dist = free_distances[i];
            
            double goal_cost = fabs(normalize_angle(angle - goal_heading_robot));
            double smooth_cost = fabs(normalize_angle(angle - last_selected_heading_));
            
            double clearance = isfinite(dist) ? dist : max_depth_;
            clearance = clamp(clearance / max_depth_, 0.0, 1.0);
            double clearance_cost = 1.0 - clearance;
            
            total_cost[i] = cost_goal_ * goal_cost + cost_smooth_ * smooth_cost + cost_clearance_ * clearance_cost;
            
            if (total_cost[i] < min_cost) {
                min_cost = total_cost[i];
                best_idx = i;
            }
        }
        
        int best_sector_idx = free_idx[best_idx];
        
        // 滞后切换策略
        int selected_idx = best_sector_idx;
        if (last_selected_sector_idx_ != -1 && !blocked[last_selected_sector_idx_]) {
            auto it = find(free_idx.begin(), free_idx.end(), last_selected_sector_idx_);
            if (it != free_idx.end()) {
                int prev_local = distance(free_idx.begin(), it);
                double prev_heading = sector_centers_[last_selected_sector_idx_];
                double best_heading = sector_centers_[best_sector_idx];
                double prev_cost = total_cost[prev_local];
                double best_cost = total_cost[best_idx];
                
                bool can_hold_heading = (fabs(prev_heading) <= hold_heading_max_ && 
                                       fabs(best_heading) <= hold_heading_max_);
                
                if (can_hold_heading && best_cost + switch_hysteresis_ >= prev_cost) {
                    selected_idx = last_selected_sector_idx_;
                }
            }
        }
        
        double selected_dist = isfinite(sector_dist[selected_idx]) ? sector_dist[selected_idx] : max_depth_;
        return {sector_centers_[selected_idx], selected_dist, selected_idx};
    }
    
    /**
     * @brief 发布停止命令
     * 
     * 发布零速度命令，使机器人停止运动
     */
    void publish_stop() {
        last_angular_cmd_ = 0.0;
        last_selected_heading_ = 0.0;
        last_selected_sector_idx_ = -1;
        
        geometry_msgs::msg::Twist cmd;
        cmd_pub_->publish(cmd);
    }
    
    /**
     * @brief 从四元数计算偏航角
     * 
     * @param x 四元数x分量
     * @param y 四元数y分量
     * @param z 四元数z分量
     * @param w 四元数w分量
     * @return 偏航角（弧度）
     */
    double yaw_from_quaternion(double x, double y, double z, double w) {
        double siny_cosp = 2.0 * (w * z + x * y);
        double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
        return atan2(siny_cosp, cosy_cosp);
    }
    
    /**
     * @brief 限制值在指定范围内
     * 
     * @param value 输入值
     * @param lo 最小值
     * @param hi 最大值
     * @return 限制后的值
     */
    double clamp(double value, double lo, double hi) {
        return max(lo, min(hi, value));
    }
    
    /**
     * @brief 归一化角度到[-π, π]范围
     * 
     * @param angle 原始角度（弧度）
     * @return 归一化后的角度（弧度）
     */
    double normalize_angle(double angle) {
        while (angle > M_PI) {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2.0 * M_PI;
        }
        return angle;
    }
};

/**
 * @brief 主函数
 * 
 * 初始化ROS 2，创建DepthVFHController节点，并启动执行循环
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数
 * @return 退出码
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<DepthVFHController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}