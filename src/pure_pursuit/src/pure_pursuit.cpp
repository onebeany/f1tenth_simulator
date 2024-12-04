#include "pure_pursuit.hpp"

#include <math.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include "std_msgs/msg/float32.hpp"

PurePursuit::PurePursuit() : Node("pure_pursuit_node") {
    // initialise parameters
    this->declare_parameter("waypoints_path", "/home/onebean/sim_ws/src/racelines/IV2024.csv");
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("car_refFrame", "ego_racecar/base_link");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("rviz_current_waypoint_topic", "/current_waypoint");
    this->declare_parameter("rviz_lookahead_waypoint_topic", "/lookahead_waypoint");
    this->declare_parameter("global_refFrame", "map");
    this->declare_parameter("min_lookahead", 0.5);
    this->declare_parameter("max_lookahead", 3.0);
    this->declare_parameter("lookahead_ratio", 8.0);
    this->declare_parameter("K_p", 0.5);
    this->declare_parameter("steering_limit", 25.0);
    this->declare_parameter("velocity_percentage", 1.0);
    this->declare_parameter("accel_weight", 0.5);
    this->declare_parameter("accel_mode", false);

    // Default Values
    waypoints_path = this->get_parameter("waypoints_path").as_string();
    odom_topic = this->get_parameter("odom_topic").as_string();
    car_refFrame = this->get_parameter("car_refFrame").as_string();
    drive_topic = this->get_parameter("drive_topic").as_string();
    rviz_current_waypoint_topic = this->get_parameter("rviz_current_waypoint_topic").as_string();
    rviz_lookahead_waypoint_topic = this->get_parameter("rviz_lookahead_waypoint_topic").as_string();
    global_refFrame = this->get_parameter("global_refFrame").as_string();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    K_p = this->get_parameter("K_p").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    accel_weight = this->get_parameter("accel_weight").as_double();
    accel_mode = this->get_parameter("accel_mode").as_bool();

    subscription_odom = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic, 25, std::bind(&PurePursuit::odom_callback, this, _1));
    timer_ = this->create_wall_timer(2000ms, std::bind(&PurePursuit::timer_callback, this));

    lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PurePursuit::lidar_callback, this, _1));

    publisher_drive = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, 25);
    vis_current_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_current_waypoint_topic, 10);
    vis_lookahead_point_pub = this->create_publisher<visualization_msgs::msg::Marker>(rviz_lookahead_waypoint_topic, 10);

    curr_lookahead_pub = this->create_publisher<std_msgs::msg::Float32>("/curr_lookahead", 25);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_INFO(this->get_logger(), "Pure pursuit node has been launched");

    load_waypoints();
}

double PurePursuit::to_radians(double degrees) {
    double radians;
    return radians = degrees * M_PI / 180.0;
}

double PurePursuit::to_degrees(double radians) {
    double degrees;
    return degrees = radians * 180.0 / M_PI;
}

double PurePursuit::p2pdist(double &x1, double &x2, double &y1, double &y2) {
    double dist = sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
    return dist;
}

void PurePursuit::load_waypoints() {
    csvFile_waypoints.open(waypoints_path, std::ios::in);

    if (!csvFile_waypoints.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", waypoints_path);
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "CSV File Opened");
    }

    // std::vector<std::string> row;
    std::string line, word, temp;

    while (!csvFile_waypoints.eof()) {
        std::getline(csvFile_waypoints, line, '\n');
        std::stringstream s(line);

        int j = 0;
        while (getline(s, word, ',')) {
            if (!word.empty()) {
                if (j == 0) {
                    waypoints.X.push_back(std::stod(word));
                } else if (j == 1) {
                    waypoints.Y.push_back(std::stod(word));
                } else if (j == 2) {
                    waypoints.V.push_back(std::stod(word));
                }
            }
            j++;
        }
    }

    csvFile_waypoints.close();
    num_waypoints = waypoints.X.size();
    RCLCPP_INFO(this->get_logger(), "Finished loading %d waypoints from %s", num_waypoints, waypoints_path);

    double average_dist_between_waypoints = 0.0;
    for (int i = 0; i < num_waypoints - 1; i++) {
        average_dist_between_waypoints += p2pdist(waypoints.X[i], waypoints.X[i + 1], waypoints.Y[i], waypoints.Y[i + 1]);
    }
    average_dist_between_waypoints /= num_waypoints;
    RCLCPP_INFO(this->get_logger(), "Average distance between waypoints: %f", average_dist_between_waypoints);
}

void PurePursuit::visualize_lookahead_point(Eigen::Vector3d &point) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.r = 1.0;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.id = 1;
    vis_lookahead_point_pub->publish(marker);
}

void PurePursuit::visualize_current_point(Eigen::Vector3d &point) {
    auto marker = visualization_msgs::msg::Marker();
    marker.header.frame_id = "map";
    marker.header.stamp = rclcpp::Clock().now();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;
    marker.color.a = 1.0;
    marker.color.b = 1.0;

    marker.pose.position.x = point(0);
    marker.pose.position.y = point(1);
    marker.id = 1;
    vis_current_point_pub->publish(marker);
}

bool PurePursuit::isPathClear(double x_start, double y_start, double x_end, double y_end) {
    // 라이다 데이터가 없으면 false 반환
    if (lidar_ranges.empty()) {
        return false;
    }

    // 목표점까지의 거리와 각도 계산
    double dx = x_end - x_start;
    double dy = y_end - y_start;
    double target_distance = std::sqrt(dx*dx + dy*dy);
    double target_angle = std::atan2(dy, dx);

    // LiDAR 특성
    const int LIDAR_POINTS = 1080;  // 전체 포인트 수
    const double LIDAR_FOV = 270.0 * M_PI / 180.0;  // 270도를 라디안으로 변환
    const double ANGLE_INCREMENT = LIDAR_FOV / LIDAR_POINTS;  // 각도 증분
    const double ANGLE_MIN = -135.0 * M_PI / 180.0;  // 시작 각도 (-135도)

    // 목표 방향에 해당하는 라이다 인덱스 계산
    // target_angle을 -135도 ~ 135도 범위로 정규화
    double normalized_angle = target_angle;
    while (normalized_angle < ANGLE_MIN) normalized_angle += 2 * M_PI;
    while (normalized_angle > ANGLE_MIN + LIDAR_FOV) normalized_angle -= 2 * M_PI;

    // 정규화된 각도가 LiDAR FOV를 벗어나면 false 반환
    if (normalized_angle < ANGLE_MIN || normalized_angle > ANGLE_MIN + LIDAR_FOV) {
        RCLCPP_INFO(this->get_logger(), 
            "웨이포인트가 LiDAR FOV를 벗어남: %.2f도", 
            normalized_angle * 180.0 / M_PI);
        return false;
    }

    // 라이다 인덱스 계산
    int index = static_cast<int>((normalized_angle - ANGLE_MIN) / ANGLE_INCREMENT);
    
    // 인덱스가 범위를 벗어나면 false 반환
    if (index < 0 || index >= LIDAR_POINTS) {
        return false;
    }

    // 라이다 거리가 무한대이거나 유효하지 않은 값이면 true 반환
    if (std::isinf(lidar_ranges[index]) || std::isnan(lidar_ranges[index])) {
        return true;
    }

    // 주변 포인트들도 확인 (노이즈 필터링)
    const int CHECK_RANGE = 5;  // 앞뒤로 5개 포인트 확인
    int valid_count = 0;
    double min_distance = std::numeric_limits<double>::max();

    for (int i = -CHECK_RANGE; i <= CHECK_RANGE; i++) {
        int check_index = index + i;
        if (check_index >= 0 && check_index < LIDAR_POINTS) {
            if (!std::isinf(lidar_ranges[check_index]) && 
                !std::isnan(lidar_ranges[check_index])) {
                valid_count++;
                min_distance = std::min(min_distance, 
                    static_cast<double>(lidar_ranges[check_index]));
            }
        }
    }

    // 유효한 포인트가 일정 수 이상이고, 최소 거리가 목표 거리보다 작으면 장애물이 있다고 판단
    const double SAFETY_MARGIN = 0.3;  // 30cm 안전 여유
    if (valid_count >= 3 && min_distance < (target_distance - SAFETY_MARGIN)) {
        RCLCPP_INFO(this->get_logger(), 
            "장애물 감지 - 인덱스: %d, 라이다 거리: %.2fm, 목표 거리: %.2fm", 
            index, min_distance, target_distance);
        return false;
    }

    return true;
}

void PurePursuit::get_waypoint(Eigen::Vector2d &car_heading) {

    // Main logic: Search within the search_range for the next waypoint
    double longest_distance = 0.0;
    int final_i = -1;

    int max_search_range = 300;  // Maximum number of waypoints to search for
    int start_index = waypoints.index;
    // int end_index = waypoints.index + max_search_range;
    int end_index = waypoints.index + max_search_range;

    base_velocity = waypoints.V[waypoints.velocity_index] * velocity_percentage;

    // Lookahead needs to be between the min_lookhead and the max_lookahead
    curr_lookahead = std::min(std::max(min_lookahead, max_lookahead * base_velocity / lookahead_ratio), max_lookahead);

    // curr_lookahead를 publish
    auto lookahead_msg = std_msgs::msg::Float32();
    lookahead_msg.data = curr_lookahead;
    curr_lookahead_pub->publish(lookahead_msg);

    /*
    for (int i = start_index; i < end_index; i++) {
        int index = i % num_waypoints; // 웨이포인트 인덱스가 순환하도록 함
        if (p2pdist(waypoints.X[index], x_car_world, waypoints.Y[index], y_car_world) <= curr_lookahead && p2pdist(waypoints.X[index], x_car_world, waypoints.Y[index], y_car_world) >= longest_distance) {
                longest_distance = p2pdist(waypoints.X[index], x_car_world, waypoints.Y[index], y_car_world);
                final_i = i;
            }
    }
    */

    for (int i = start_index; i < end_index; i++) {
        int index = i % num_waypoints; // 웨이포인트 인덱스가 순환하도록 함

        // 웨이포인트와의 거리 계산
        double dx = waypoints.X[index] - x_car_world;
        double dy = waypoints.Y[index] - y_car_world;

        // 웨이포인트로의 벡터
        Eigen::Vector2d waypoint_vector(dx, dy);

        // 거리 계산
        double distance = waypoint_vector.norm();

        // 진행 방향과의 내적 계산
        double dot_product = car_heading.dot(waypoint_vector.normalized());

        if (dot_product > 0) {
            // 조건에 맞는 웨이포인트 선택
            if (distance <= curr_lookahead && distance >= longest_distance) {
                
                longest_distance = distance;
                final_i = index;
                
                
                // if(isPathClear(x_car_world, y_car_world, waypoints.X[index], waypoints.Y[index])) {
                //    longest_distance = distance;
                //    final_i = index;
                // }
            }
        }
    }

    if (final_i == -1) {  // if we haven't found anything, search from the beginning
        RCLCPP_INFO(this->get_logger(), "No valid waypoint found. Searching from the beginning");
        final_i = 0;
        
        for (int i = 0; i < num_waypoints; i++) {
            double dx = waypoints.X[i] - x_car_world;
            double dy = waypoints.Y[i] - y_car_world;
            Eigen::Vector2d waypoint_vector(dx, dy);
            double distance = waypoint_vector.norm();
            
            // 여기서도 방향 체크를 추가
            double dot_product = car_heading.dot(waypoint_vector.normalized());
            
            if (dot_product > 0 && 
                distance <= curr_lookahead && 
                distance >= longest_distance) {
                longest_distance = distance;
                final_i = i;
            }
        }

        if(final_i == 0) {
            final_i = waypoints.index;
            RCLCPP_INFO(this->get_logger(), 
            "No valid waypoint found. Keeping current index: %d, "
            "Current position: (%.2f, %.2f), Heading: (%.2f, %.2f), "
            "Lookahead: %.2f",
            final_i, x_car_world, y_car_world, 
            car_heading(0), car_heading(1), curr_lookahead);
        }
    }

    // Find the closest point to the car, and use the velocity index for that
    double shortest_distance = p2pdist(waypoints.X[0], x_car_world, waypoints.Y[0], y_car_world);
    int velocity_i = 0;
    for (int i = 0; i < num_waypoints; i++) {
        if (p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world) <= shortest_distance) {
            shortest_distance = p2pdist(waypoints.X[i], x_car_world, waypoints.Y[i], y_car_world);
            velocity_i = i;
        }
    }

    // If a waypoint is not found within our radius, then waypoints.index = 0
    waypoints.index = final_i;
    waypoints.velocity_index = velocity_i;
}

void PurePursuit::quat_to_rot(double q0, double q1, double q2, double q3) {
    double r00 = (double)(2.0 * (q0 * q0 + q1 * q1) - 1.0);
    double r01 = (double)(2.0 * (q1 * q2 - q0 * q3));
    double r02 = (double)(2.0 * (q1 * q3 + q0 * q2));

    double r10 = (double)(2.0 * (q1 * q2 + q0 * q3));
    double r11 = (double)(2.0 * (q0 * q0 + q2 * q2) - 1.0);
    double r12 = (double)(2.0 * (q2 * q3 - q0 * q1));

    double r20 = (double)(2.0 * (q1 * q3 - q0 * q2));
    double r21 = (double)(2.0 * (q2 * q3 + q0 * q1));
    double r22 = (double)(2.0 * (q0 * q0 + q3 * q3) - 1.0);

    rotation_m << r00, r01, r02, r10, r11, r12, r20, r21, r22;
}

void PurePursuit::transformandinterp_waypoint() {  // pass old waypoint here
    // initialise vectors
    waypoints.lookahead_point_world << waypoints.X[waypoints.index], waypoints.Y[waypoints.index], 0.0;
    waypoints.current_point_world << waypoints.X[waypoints.velocity_index], waypoints.Y[waypoints.velocity_index], 0.0;

    visualize_lookahead_point(waypoints.lookahead_point_world);
    visualize_current_point(waypoints.current_point_world);

    // look up transformation at that instant from tf_buffer_
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
        // Get the transform from the base_link reference to world reference frame
        transformStamped = tf_buffer_->lookupTransform(car_refFrame, global_refFrame, tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
        RCLCPP_INFO(this->get_logger(), "Could not transform. Error: %s", ex.what());
    }

    // transform points (rotate first and then translate)
    Eigen::Vector3d translation_v(transformStamped.transform.translation.x, transformStamped.transform.translation.y, transformStamped.transform.translation.z);
    quat_to_rot(transformStamped.transform.rotation.w, transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z);

    waypoints.lookahead_point_car = (rotation_m * waypoints.lookahead_point_world) + translation_v;
}

double PurePursuit::p_controller() {
    double r = waypoints.lookahead_point_car.norm();  // r = sqrt(x^2 + y^2)
    double y = waypoints.lookahead_point_car(1);
    double angle = K_p * 2 * y / pow(r, 2);  // Calculated from https://docs.google.com/presentation/d/1jpnlQ7ysygTPCi8dmyZjooqzxNXWqMgO31ZhcOlKVOE/edit#slide=id.g63d5f5680f_0_33

    return angle;
}

double PurePursuit::get_velocity(double steering_angle) {

    double velocity = 0;
    double threshold_lookahead = (min_lookahead + max_lookahead) / 2.0;

    if(waypoints.V[waypoints.velocity_index]) {

        if(curr_lookahead <= threshold_lookahead) {
            velocity = base_velocity;
        } else {
            double acceleration_factor = 1.0;
            if(accel_mode){
                double normalized_distance = (curr_lookahead - threshold_lookahead) / (max_lookahead - threshold_lookahead);
                normalized_distance = std::min(std::max(normalized_distance, 0.0), 1.0);
                acceleration_factor = 1.0 + accel_weight * pow(normalized_distance, 2.0);
            }
            velocity = base_velocity * acceleration_factor;
        }
    }
    else{
        if (abs(steering_angle) >= to_radians(0.0) && abs(steering_angle) < to_radians(10.0)) {
            velocity = 6.0 * velocity_percentage;
        } else if (abs(steering_angle) >= to_radians(10.0) && abs(steering_angle) <= to_radians(20.0)) {
            velocity = 2.5 * velocity_percentage;
        } else {
            velocity = 2.0 * velocity_percentage;
        }
    }

    return velocity;
}

void PurePursuit::publish_message(double steering_angle) {
    auto drive_msgObj = ackermann_msgs::msg::AckermannDriveStamped();

    if (steering_angle < 0.0) {
        drive_msgObj.drive.steering_angle = std::max(steering_angle, -to_radians(steering_limit));  // ensure steering angle is dynamically capable
    } else {
        drive_msgObj.drive.steering_angle = std::min(steering_angle, to_radians(steering_limit));  // ensure steering angle is dynamically capable
    }
    
    curr_velocity = get_velocity(drive_msgObj.drive.steering_angle);
    drive_msgObj.drive.speed = curr_velocity;

    RCLCPP_INFO(this->get_logger(), "index: %d ... distance: %.2fm ... Speed: %.2fm/s ... Steering Angle: %.2f ... K_p: %.2f ... velocity_percentage: %.2f", waypoints.index, p2pdist(waypoints.X[waypoints.index], x_car_world, waypoints.Y[waypoints.index], y_car_world), drive_msgObj.drive.speed, to_degrees(drive_msgObj.drive.steering_angle), K_p, velocity_percentage);

    publisher_drive->publish(drive_msgObj);
}

// 라이다 콜백 함수
void PurePursuit::lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    lidar_ranges = scan->ranges;
    angle_min = scan->angle_min;
    angle_increment = scan->angle_increment;
}

void PurePursuit::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_submsgObj) {
    x_car_world = odom_submsgObj->pose.pose.position.x;
    y_car_world = odom_submsgObj->pose.pose.position.y;

    tf2::Quaternion q(
        odom_submsgObj->pose.pose.orientation.x,
        odom_submsgObj->pose.pose.orientation.y,
        odom_submsgObj->pose.pose.orientation.z,
        odom_submsgObj->pose.pose.orientation.w
    );
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw); // yaw: heading direction of the car

    Eigen::Vector2d car_heading(std::cos(yaw), std::sin(yaw));

    // interpolate between different way-points
    get_waypoint(car_heading);

    // use tf2 transform the goal point
    transformandinterp_waypoint();

    // Calculate curvature/steering angle
    double steering_angle = p_controller();

    // publish object and message: AckermannDriveStamped on drive topic
    publish_message(steering_angle);
}

void PurePursuit::timer_callback() {
    // Periodically check parameters and update
    K_p = this->get_parameter("K_p").as_double();
    velocity_percentage = this->get_parameter("velocity_percentage").as_double();
    min_lookahead = this->get_parameter("min_lookahead").as_double();
    max_lookahead = this->get_parameter("max_lookahead").as_double();
    lookahead_ratio = this->get_parameter("lookahead_ratio").as_double();
    steering_limit = this->get_parameter("steering_limit").as_double();
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node_ptr = std::make_shared<PurePursuit>();  // initialise node pointer
    rclcpp::spin(node_ptr);
    rclcpp::shutdown();
    return 0;
}
