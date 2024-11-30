#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" // 추가


using std::placeholders::_1;

class RvizClickTo2D : public rclcpp::Node {
public:
    RvizClickTo2D() : Node("rviz_click_to_2d") {
        pub_goal_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_2d", 10);
        pub_initial_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_2d", 10);

        sub_goal_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "move_base_simple/goal", 10, std::bind(&RvizClickTo2D::handle_goal, this, _1));
        sub_initial_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&RvizClickTo2D::handle_initial_pose, this, _1));
    }

private:
    void handle_goal(const geometry_msgs::msg::PoseStamped::SharedPtr goal) {
        auto rpyGoal = geometry_msgs::msg::PoseStamped();
        rpyGoal.header.frame_id = "map";
        rpyGoal.header.stamp = this->get_clock()->now();

        rpyGoal.pose.position.x = goal->pose.position.x;
        rpyGoal.pose.position.y = goal->pose.position.y;
        rpyGoal.pose.position.z = 0;

        tf2::Quaternion q(0, 0, goal->pose.orientation.z, goal->pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion yaw_quat;
        yaw_quat.setRPY(0, 0, yaw);

        // tf2::toMsg 대신
        rpyGoal.pose.orientation = tf2::toMsg(yaw_quat);

        pub_goal_->publish(rpyGoal);
    }

    void handle_initial_pose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr pose) {
        auto rpyPose = geometry_msgs::msg::PoseStamped();
        rpyPose.header.frame_id = "map";
        rpyPose.header.stamp = this->get_clock()->now();

        rpyPose.pose.position.x = pose->pose.pose.position.x;
        rpyPose.pose.position.y = pose->pose.pose.position.y;
        rpyPose.pose.position.z = 0;

        tf2::Quaternion q(0, 0, pose->pose.pose.orientation.z, pose->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        tf2::Quaternion yaw_quat;
        yaw_quat.setRPY(0, 0, yaw);

        // tf2::toMsg 대신
        rpyPose.pose.orientation = tf2::toMsg(yaw_quat);

        pub_initial_->publish(rpyPose);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_goal_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_initial_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initial_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RvizClickTo2D>());
    rclcpp::shutdown();
    return 0;
}