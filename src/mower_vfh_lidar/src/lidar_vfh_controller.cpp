/**
 * @file lidar_vfh_controller.cpp
 * @brief 基于激光雷达的VFH（Vector Field Histogram）避障导航控制器
 * 
 * 该控制器使用激光雷达数据构建扇区直方图，计算障碍物分布情况，
 * 并根据目标位置和障碍物信息选择最优导航方向，实现避障导航功能。
 * 
 * 主要功能：
 * 1. 处理激光雷达扫描数据，构建扇区距离分布
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
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <string>
#include <limits>
#include <functional>

using namespace std;
using namespace Eigen;
using namespace std::placeholders;

/**
 * @class LidarVFHController
 * @brief 基于激光雷达的VFH避障导航控制器类
 * 
 * 该类继承自rclcpp::Node，实现了基于VFH算法的避障导航功能。
 * 通过处理激光雷达数据，构建扇区直方图，计算障碍物分布，
 * 并根据目标位置选择最优导航方向，生成速度命令。
 */
class LidarVFHController : public rclcpp::Node {
public:
    /**
     * @brief 构造函数
     * 
     * 初始化控制器，声明参数，创建订阅者、发布者和定时器。
     */
    LidarVFHController() : Node("lidar_vfh_controller") {
        // 声明参数
        declare_parameter("scan_topic", "/scan");         // 激光雷达扫描话题
        declare_parameter("odom_topic", "/odom");         // 里程计话题
        declare_parameter("goal_topic", "/goal_pose");     // 目标位置话题
        declare_parameter("cmd_vel_topic", "/cmd_vel");    // 速度命令话题
        
        // VFH算法和控制参数
        declare_parameter("control_rate_hz", 15.0);          // 控制循环频率
        declare_parameter("sector_angle_deg", 5.0);          // 扇区角度（度）
        declare_parameter("view_angle_deg", 150.0);          // 视野角度（度）
        declare_parameter("min_range", 0.2);                 // 最小有效距离
        declare_parameter("max_range", 6.0);                 // 最大有效距离
        declare_parameter("inflation_max_range", 3.0);       // 障碍物膨胀最大距离
        declare_parameter("robot_radius", 0.28);             // 机器人半径
        declare_parameter("safety_margin", 0.12);            // 安全余量
        declare_parameter("goal_tolerance", 0.25);           // 目标位置容差
        declare_parameter("goal_slowdown_dist", 1.0);        // 目标减速距离
        declare_parameter("stop_distance", 0.45);            // 停止距离
        declare_parameter("max_linear_speed", 0.40);         // 最大线速度
        declare_parameter("min_linear_speed", 0.05);         // 最小线速度
        declare_parameter("max_angular_speed", 1.2);         // 最大角速度
        declare_parameter("heading_kp", 1.8);                // 航向控制比例系数
        declare_parameter("turn_in_place_angle_deg", 50.0);  // 原地转向角度阈值
        declare_parameter("cost_goal", 1.0);                 // 目标方向成本权重
        declare_parameter("cost_smooth", 0.35);              // 平滑度成本权重
        declare_parameter("cost_clearance", 0.25);           // 障碍物距离成本权重
        
        // 获取参数
        get_parameter("scan_topic", scan_topic_);
        get_parameter("odom_topic", odom_topic_);
        get_parameter("goal_topic", goal_topic_);
        get_parameter("cmd_vel_topic", cmd_vel_topic_);
        
        get_parameter("control_rate_hz", control_rate_hz_);
        get_parameter("sector_angle_deg", sector_angle_deg_);
        get_parameter("view_angle_deg", view_angle_deg_);
        get_parameter("min_range", min_range_);
        get_parameter("max_range", max_range_);
        get_parameter("inflation_max_range", inflation_max_range_);
        get_parameter("robot_radius", robot_radius_);
        get_parameter("safety_margin", safety_margin_);
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
        
        // 转换单位（度转弧度）
        sector_angle_ = M_PI * sector_angle_deg_ / 180.0;  // 扇区角度（弧度）
        view_angle_ = M_PI * view_angle_deg_ / 180.0;      // 视野角度（弧度）
        turn_in_place_angle_ = M_PI * turn_in_place_angle_deg_ / 180.0;  // 原地转向角度阈值（弧度）
        
        // 计算扇区数量和角度
        num_sectors_ = max(5, static_cast<int>(round((2.0 * view_angle_) / sector_angle_)));
        sector_angle_ = (2.0 * view_angle_) / static_cast<double>(num_sectors_);
        
        // 初始化扇区中心角度
        sector_centers_.resize(num_sectors_);
        for (int i = 0; i < num_sectors_; i++) {
            sector_centers_[i] = -view_angle_ + 0.5 * sector_angle_ + i * sector_angle_;
        }
        
        // 初始化状态变量
        latest_scan_dist_.resize(num_sectors_, numeric_limits<float>::infinity());
        robot_x_ = robot_y_ = robot_yaw_ = numeric_limits<float>::quiet_NaN();  // 机器人位置和姿态
        goal_x_ = goal_y_ = numeric_limits<float>::quiet_NaN();  // 目标位置
        goal_frame_ = "";  // 目标坐标系
        last_selected_heading_ = 0.0;  // 上次选择的航向
        
        // 创建订阅者
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic_, 10, std::bind(&LidarVFHController::scan_callback, this, _1));
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 10, std::bind(&LidarVFHController::odom_callback, this, _1));
        
        goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            goal_topic_, 10, std::bind(&LidarVFHController::goal_callback, this, _1));
        
        // 创建发布者
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);
        
        // 创建定时器
        timer_ = create_wall_timer(
            chrono::duration<double>(1.0 / control_rate_hz_),
            std::bind(&LidarVFHController::control_loop, this));
        
        RCLCPP_INFO(get_logger(), "LiDAR VFH ready. scan=%s, goal=%s, cmd_vel=%s", 
                   scan_topic_.c_str(), goal_topic_.c_str(), cmd_vel_topic_.c_str());
    }
    
private:
    // 话题名称
    string scan_topic_;         // 激光雷达扫描话题
    string odom_topic_;         // 里程计话题
    string goal_topic_;         // 目标位置话题
    string cmd_vel_topic_;      // 速度命令话题
    
    // VFH参数
    double control_rate_hz_;          // 控制循环频率
    double sector_angle_deg_;          // 扇区角度（度）
    double view_angle_deg_;            // 视野角度（度）
    double min_range_;                 // 最小有效距离
    double max_range_;                 // 最大有效距离
    double inflation_max_range_;       // 障碍物膨胀最大距离
    double robot_radius_;              // 机器人半径
    double safety_margin_;             // 安全余量
    double goal_tolerance_;            // 目标位置容差
    double goal_slowdown_dist_;        // 目标减速距离
    double stop_distance_;             // 停止距离
    double max_linear_speed_;          // 最大线速度
    double min_linear_speed_;          // 最小线速度
    double max_angular_speed_;         // 最大角速度
    double heading_kp_;                // 航向控制比例系数
    double turn_in_place_angle_deg_;   // 原地转向角度阈值（度）
    double cost_goal_;                 // 目标方向成本权重
    double cost_smooth_;               // 平滑度成本权重
    double cost_clearance_;            // 障碍物距离成本权重
    
    // 转换后的参数
    double sector_angle_;           // 扇区角度（弧度）
    double view_angle_;             // 视野角度（弧度）
    double turn_in_place_angle_;    // 原地转向角度阈值（弧度）
    int num_sectors_;               // 扇区数量
    vector<double> sector_centers_; // 扇区中心角度
    
    // 状态变量
    vector<float> latest_scan_dist_;   // 最近扫描的扇区距离
    double robot_x_;                   // 机器人x坐标
    double robot_y_;                   // 机器人y坐标
    double robot_yaw_;                 // 机器人偏航角
    double goal_x_;                    // 目标x坐标
    double goal_y_;                    // 目标y坐标
    string goal_frame_;                // 目标坐标系
    double last_selected_heading_;     // 上次选择的航向
    
    // 订阅者和发布者
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;  // 激光雷达订阅者
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;     // 里程计订阅者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;  // 目标订阅者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;       // 速度命令发布者
    rclcpp::TimerBase::SharedPtr timer_;  // 控制循环定时器
    
    /**
     * @brief 激光雷达扫描回调函数
     * 
     * 处理激光雷达扫描数据，计算每个扇区的最小距离，并存储到latest_scan_dist_数组中。
     * 
     * @param msg 激光雷达扫描消息
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        const auto& ranges = msg->ranges;
        int num_points = ranges.size();
        
        if (num_points == 0) {
            fill(latest_scan_dist_.begin(), latest_scan_dist_.end(), numeric_limits<float>::infinity());
            return;
        }
        
        // 计算角度数组
        vector<double> angles(num_points);
        for (int i = 0; i < num_points; i++) {
            angles[i] = msg->angle_min + i * msg->angle_increment;
        }
        
        // 初始化扇区距离
        fill(latest_scan_dist_.begin(), latest_scan_dist_.end(), numeric_limits<float>::infinity());
        
        // 处理每个激光点
        for (int i = 0; i < num_points; i++) {
            double range = ranges[i];
            double angle = angles[i];
            
            // 有效性检查
            if (isnan(range) || isinf(range)) continue;  // 跳过无效数据
            if (range < std::max(min_range_, static_cast<double>(msg->range_min))) continue;  // 跳过过近距离
            if (range > std::min(max_range_, static_cast<double>(msg->range_max))) continue;  // 跳过过远距离
            if (angle < -view_angle_ || angle > view_angle_) continue;  // 跳过视野范围外的数据
            
            // 计算扇区索引
            int idx = static_cast<int>((angle + view_angle_) / sector_angle_);
            idx = max(0, min(num_sectors_ - 1, idx));  // 确保索引在有效范围内
            
            // 更新扇区最小距离
            if (range < latest_scan_dist_[idx]) {
                latest_scan_dist_[idx] = static_cast<float>(range);
            }
        }
    }
    
    /**
     * @brief 里程计回调函数
     * 
     * 更新机器人的当前位置和姿态（偏航角）。
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
     * 更新导航目标位置。
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
     * 1. 状态检查（目标位置、机器人位置、激光数据）
     * 2. 坐标系检查
     * 3. 计算目标距离和方向
     * 4. 检查是否到达目标
     * 5. 计算目标方向（相对于机器人坐标系）
     * 6. VFH核心算法（计算阻挡扇区、选择航向）
     * 7. 生成速度命令
     * 8. 特殊行为调整（原地转向、速度限制）
     * 9. 发布速度命令
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
        
        // 检查是否有有效激光数据
        bool has_valid_data = false;
        for (float dist : latest_scan_dist_) {
            if (isfinite(dist)) {
                has_valid_data = true;
                break;
            }
        }
        if (!has_valid_data) {
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
            goal_x_ = goal_y_ = numeric_limits<float>::quiet_NaN();
            goal_frame_ = "";
            return;
        }
        
        // 计算目标方向（世界坐标系和机器人坐标系）
        double goal_heading_world = atan2(dy, dx);
        double goal_heading_robot = normalize_angle(goal_heading_world - robot_yaw_);
        
        // VFH核心算法
        vector<bool> blocked = compute_blocked_sectors();  // 计算阻挡扇区
        pair<double, double> selected = select_heading(goal_heading_robot, blocked);  // 选择最优航向
        double selected_heading = selected.first;
        double selected_dist = selected.second;
        last_selected_heading_ = selected_heading;
        
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
            // 所有方向都被阻挡，原地转向寻找可通行方向
            cmd.linear.x = 0.0;
            double direction = (fabs(goal_heading_robot) > 1e-4) ? goal_heading_robot : 1.0;
            cmd.angular.z = copysign(max_angular_speed_ * 0.7, direction);
            cmd_pub_->publish(cmd);
            return;
        }
        
        // 角速度控制
        cmd.angular.z = clamp(heading_kp_ * selected_heading, -max_angular_speed_, max_angular_speed_);
        
        // 线速度控制（考虑航向、障碍物距离和目标距离）
        double heading_factor = max(0.0, cos(fabs(selected_heading)));  // 航向因子
        double clearance_factor = clamp((selected_dist - stop_distance_) / max(1e-3, max_range_ - stop_distance_), 0.0, 1.0);  // 障碍物距离因子
        double goal_factor = clamp(goal_dist / max(1e-3, goal_slowdown_dist_), 0.0, 1.0);  // 目标距离因子
        
        cmd.linear.x = max_linear_speed_ * heading_factor * clearance_factor * goal_factor;
        
        // 特殊行为调整
        if (fabs(selected_heading) > turn_in_place_angle_) {
            // 大角度转向时降低线速度
            cmd.linear.x = min(cmd.linear.x, 0.05);
        }
        
        if (cmd.linear.x > 0.0 && cmd.linear.x < min_linear_speed_ && clearance_factor > 0.3) {
            // 确保最小线速度
            cmd.linear.x = min_linear_speed_;
        }
        
        // 发布速度命令
        cmd_pub_->publish(cmd);
    }
    
    /**
     * @brief 计算阻挡扇区
     * 
     * 根据激光雷达数据，计算哪些扇区被障碍物阻挡。
     * 考虑机器人半径和安全余量，对障碍物进行膨胀处理。
     * 
     * @return 阻挡扇区标记数组
     */
    vector<bool> compute_blocked_sectors() {
        vector<bool> blocked(num_sectors_, false);
        double inflate_radius = robot_radius_ + safety_margin_;  // 膨胀半径
        
        for (int i = 0; i < num_sectors_; i++) {
            float dist = latest_scan_dist_[i];
            if (!isfinite(dist) || dist > inflation_max_range_) {
                continue;  // 跳过无效或过远的距离
            }
            
            // 计算障碍物膨胀后的影响范围
            double spread = asin(min(0.999, inflate_radius / max(0.05, static_cast<double>(dist))));
            int spread_bins = static_cast<int>(ceil(spread / sector_angle_));
            
            // 计算影响的扇区范围
            int left = max(0, i - spread_bins);
            int right = min(num_sectors_ - 1, i + spread_bins);
            
            // 标记阻挡扇区
            for (int j = left; j <= right; j++) {
                blocked[j] = true;
            }
        }
        
        return blocked;
    }
    
    /**
     * @brief 选择最优航向
     * 
     * 根据目标方向、当前方向和障碍物分布，选择最优的导航方向。
     * 考虑三个成本因素：
     * 1. 目标方向成本：与目标方向的偏差
     * 2. 平滑度成本：与当前方向的偏差
     * 3. 障碍物距离成本：与障碍物的距离
     * 
     * @param goal_heading_robot 目标方向（机器人坐标系）
     * @param blocked 阻挡扇区标记
     * @return 最优航向和距离
     */
    pair<double, double> select_heading(double goal_heading_robot, const vector<bool>& blocked) {
        // 找到所有可通行的扇区
        vector<int> free_idx;
        for (int i = 0; i < num_sectors_; i++) {
            if (!blocked[i]) {
                free_idx.push_back(i);
            }
        }
        
        if (free_idx.empty()) {
            // 没有可通行扇区，返回最大视野方向
            double direction = (fabs(goal_heading_robot) > 1e-4) ? goal_heading_robot : 1.0;
            return {copysign(view_angle_, direction), 0.0};
        }
        
        // 提取可通行扇区的角度和距离
        vector<double> free_angles;
        vector<float> free_distances;
        for (int idx : free_idx) {
            free_angles.push_back(sector_centers_[idx]);
            free_distances.push_back(latest_scan_dist_[idx]);
        }
        
        // 计算成本
        vector<double> total_cost(free_idx.size());
        size_t best_idx = 0;
        double min_cost = numeric_limits<double>::max();
        
        for (size_t i = 0; i < free_idx.size(); i++) {
            double angle = free_angles[i];
            float dist = free_distances[i];
            
            // 目标方向成本
            double goal_cost = fabs(normalize_angle(angle - goal_heading_robot));
            
            // 平滑度成本（与上次选择方向的差异）
            double smooth_cost = fabs(normalize_angle(angle - last_selected_heading_));
            
            // 障碍物距离成本
            double clearance = isfinite(dist) ? dist : max_range_;
            clearance = clamp(clearance / max_range_, 0.0, 1.0);
            double clearance_cost = 1.0 - clearance;
            
            // 总成本
            total_cost[i] = cost_goal_ * goal_cost + cost_smooth_ * smooth_cost + cost_clearance_ * clearance_cost;
            
            if (total_cost[i] < min_cost) {
                min_cost = total_cost[i];
                best_idx = i;
            }
        }
        
        // 获取最优扇区的角度和距离
        int sector_idx = free_idx[best_idx];
        double selected_dist = isfinite(latest_scan_dist_[sector_idx]) ? latest_scan_dist_[sector_idx] : max_range_;
        
        return {sector_centers_[sector_idx], selected_dist};
    }
    
    /**
     * @brief 发布停止命令
     * 
     * 发布零速度命令，使机器人停止运动。
     */
    void publish_stop() {
        geometry_msgs::msg::Twist cmd;  // 默认值为0
        cmd_pub_->publish(cmd);
    }
    
    /**
     * @brief 从四元数计算偏航角
     * 
     * 将四元数转换为偏航角（yaw）。
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
 * 初始化ROS 2，创建LidarVFHController节点，并启动执行循环。
 * 
 * @param argc 命令行参数数量
 * @param argv 命令行参数
 * @return 退出码
 */
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = make_shared<LidarVFHController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}