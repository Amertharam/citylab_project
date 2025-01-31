#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <robot_patrol_interfaces/srv/get_direction.hpp>

class TestService : public rclcpp::Node {
public:
  TestService() : Node("test_service_node") {
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&TestService::scan_callback, this, std::placeholders::_1));
    client_ = create_client<robot_patrol_interfaces::srv::GetDirection>("direction_service");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto request = std::make_shared<robot_patrol_interfaces::srv::GetDirection::Request>();
    request->laser_data = *msg;

    client_->async_send_request(request,
      [this](rclcpp::Client<robot_patrol_interfaces::srv::GetDirection>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(get_logger(), "Direction: %s", response->direction.c_str());
      });
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Client<robot_patrol_interfaces::srv::GetDirection>::SharedPtr client_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestService>());
  rclcpp::shutdown();
  return 0;
}