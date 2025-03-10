#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto node = rclcpp::Node::make_shared("pose_estimate_publisher");
    auto publisher = node->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("initialpose", 10);

    // 메시지 생성
    auto pose_msg = geometry_msgs::msg::PoseWithCovarianceStamped();

    // 헤더 설정
    pose_msg.header.frame_id = "map";  // 기본 프레임은 "map"
    pose_msg.header.stamp = node->get_clock()->now();

    // 위치 설정
    pose_msg.pose.pose.position.x = 0.0;  // 원하는 x 좌표
    pose_msg.pose.pose.position.y = 0.0;  // 원하는 y 좌표
    pose_msg.pose.pose.position.z = 0.0;

    // 방향 설정 (Quaternion)
    pose_msg.pose.pose.orientation.x = 0.0;
    pose_msg.pose.pose.orientation.y = 0.0;
    pose_msg.pose.pose.orientation.z = 0.0;
    pose_msg.pose.pose.orientation.w = 1.0;  // 정방향 (회전 없음)

    // 공분산 행렬 설정
    for (int i = 0; i < 36; i++) {
        pose_msg.pose.covariance[i] = (i % 7 == 0) ? 0.0 : 0.0;  // 대각선 요소에만 0.5 설정
    }

    // 퍼블리시 루프
    rclcpp::Rate loop_rate(1); // 1Hz
    while (rclcpp::ok()) {
        RCLCPP_INFO(node->get_logger(), "Publishing 2D Pose Estimate...");
        publisher->publish(pose_msg);
        rclcpp::spin_some(node);
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}