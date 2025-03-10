#include <iostream>
#include <fstream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <filesystem> // 디렉토리 확인 및 생성

class OdomDataSaver : public rclcpp::Node {
public:
    OdomDataSaver()
        : Node("odom_data_saver") {
        // 저장할 파일 경로 설정
        std::string directory = "/home/aims_xavier/XAiMs/XAiMS_ws/src/xaims_localization/data";
        std::string file_path = directory + "/odom_data.csv";

        // 디렉토리 없으면 생성
        std::filesystem::create_directories(directory);

        // 파일 열기
        file_.open(file_path, std::ios::out);
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return;
        }

        // CSV 헤더 작성
        file_ << "time_sec,time_nsec,position_x,position_y,position_z,"
              << "orientation_x,orientation_y,orientation_z,orientation_w\n";

        // "odom" 토픽 구독
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&OdomDataSaver::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Saving odometry data to: %s", file_path.c_str());
    }

    ~OdomDataSaver() {
        if (file_.is_open()) {
            file_.close();
        }
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!file_.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "File is not open, cannot save data");
            return;
        }

        // odom 데이터를 CSV 형식으로 저장
        file_ << msg->header.stamp.sec << ","
              << msg->header.stamp.nanosec << ","
              << msg->pose.pose.position.x << ","
              << msg->pose.pose.position.y << ","
              << msg->pose.pose.position.z << ","
              << msg->pose.pose.orientation.x << ","
              << msg->pose.pose.orientation.y << ","
              << msg->pose.pose.orientation.z << ","
              << msg->pose.pose.orientation.w << "\n";

        RCLCPP_INFO(this->get_logger(), "Saved odometry data");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::ofstream file_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomDataSaver>());
    rclcpp::shutdown();
    return 0;
}