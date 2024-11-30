#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <iostream>

// Create odometry data publishers
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub;
rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat;
nav_msgs::msg::Odometry odomNew;
nav_msgs::msg::Odometry odomOld;

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 620; // For reference purposes.
const double WHEEL_RADIUS = 0.033; // Wheel radius in meters
const double WHEEL_BASE = 0.17; // Center of left tire to center of right tire
const double TICKS_PER_METER = 3100; // Original was 2800

// Distance both wheels have traveled
double distanceLeft = 0;
double distanceRight = 0;

// Flag to see if initial pose has been received
bool initialPoseRecieved = false;

using namespace std;

// Get initial_2d message from either Rviz clicks or a manual pose publisher
void set_initial_2d(const geometry_msgs::msg::PoseStamped::SharedPtr rvizClick) {
  odomOld.pose.pose.position.x = rvizClick->pose.position.x;
  odomOld.pose.pose.position.y = rvizClick->pose.position.y;
  odomOld.pose.pose.orientation.z = rvizClick->pose.orientation.z;
  initialPoseRecieved = true;
  std::cout <<"pose : " << odomOld.pose.pose.position.x << ", " << odomOld.pose.pose.position.y << ", " << odomOld.pose.pose.orientation.z << std::endl;
}

// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const std_msgs::msg::Int16::SharedPtr leftCount) {
  static int lastCountL = 0;
  if (leftCount->data != 0 && lastCountL != 0) {
    int leftTicks = (leftCount->data - lastCountL);

    if (leftTicks > 10000) {
      leftTicks = 0 - (65535 - leftTicks);
    } else if (leftTicks < -10000) {
      leftTicks = 65535 - leftTicks;
    }
    distanceLeft = leftTicks / TICKS_PER_METER;
  }
  lastCountL = leftCount->data;
}

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const std_msgs::msg::Int16::SharedPtr rightCount) {
  static int lastCountR = 0;
  if (rightCount->data != 0 && lastCountR != 0) {
    int rightTicks = rightCount->data - lastCountR;

    if (rightTicks > 10000) {
      distanceRight = (0 - (65535 - distanceRight)) / TICKS_PER_METER;
    } else if (rightTicks < -10000) {
      rightTicks = 65535 - rightTicks;
    }
    distanceRight = rightTicks / TICKS_PER_METER;
  }
  lastCountR = rightCount->data;
}

// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat() {
  tf2::Quaternion q;
  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

  nav_msgs::msg::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

  for (int i = 0; i < 36; i++) {
    if (i == 0 || i == 7 || i == 14) {
      quatOdom.pose.covariance[i] = .01;
    } else if (i == 21 || i == 28 || i == 35) {
      quatOdom.pose.covariance[i] += 0.1;
    } else {
      quatOdom.pose.covariance[i] = 0;
    }
  }

  odom_data_pub_quat->publish(quatOdom);
}

// Update odometry information
void update_odom() {
  // Calculate the average distance
  double cycleDistance = (distanceRight + distanceLeft) / 2;

  // Calculate the number of radians the robot has turned since the last cycle
  double cycleAngle = asin((distanceRight - distanceLeft) / WHEEL_BASE);

  // Average angle during the last cycle
  double avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z;

  if (avgAngle > PI) {
    avgAngle -= 2 * PI;
  } else if (avgAngle < -PI) {
    avgAngle += 2 * PI;
  }

  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance;
  odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y) || isnan(odomNew.pose.pose.position.z)) {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }

  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI) {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  } else if (odomNew.pose.pose.orientation.z < -PI) {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }

  // Compute the velocity
  odomNew.header.stamp = rclcpp::Clock().now();
  odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.nanosec - odomOld.header.stamp.nanosec);
  odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.nanosec - odomOld.header.stamp.nanosec);

  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;

  // Publish the odometry message
  odom_data_pub->publish(odomNew);
}

int main(int argc, char **argv) {
  // Set the data fields of the odometry message
  odomNew.header.frame_id = "odom";
  odomNew.pose.pose.position.z = 0;
  odomNew.pose.pose.orientation.x = 0;
  odomNew.pose.pose.orientation.y = 0;
  odomNew.twist.twist.linear.x = 0;
  odomNew.twist.twist.linear.y = 0;
  odomNew.twist.twist.linear.z = 0;
  odomNew.twist.twist.angular.x = 0;
  odomNew.twist.twist.angular.y = 0;
  odomNew.twist.twist.angular.z = 0;
  odomOld.pose.pose.position.x = initialX;
  odomOld.pose.pose.position.y = initialY;
  odomOld.pose.pose.orientation.z = initialTheta;

  // Launch ROS and create a node
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rclcpp::Node>("ekf_odom_pub", options);

  // Subscribe to ROS topics
  auto subForRightCounts = node->create_subscription<std_msgs::msg::Int16>(
    "right_ticks", 100, Calc_Right);
  auto subForLeftCounts = node->create_subscription<std_msgs::msg::Int16>(
    "left_ticks", 100, Calc_Left);
  auto subForInitialPose = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "initial_2d", 100, set_initial_2d);

  // Initialize the publishers
  odom_data_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 100);
  odom_data_pub_quat = node->create_publisher<nav_msgs::msg::Odometry>("odom_quat", 100);

  rclcpp::Rate loop_rate(50);

  while (rclcpp::ok()) {
    update_odom();
    publish_quat();
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}