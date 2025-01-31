#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_patrol_interfaces/srv/get_direction.hpp>

using namespace std::chrono_literals;

class PatrolWithService : public rclcpp::Node {
public:
  PatrolWithService() : Node("patrol_with_service_node") {
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, std::bind(&PatrolWithService::scan_callback, this, std::placeholders::_1));
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    client_ = create_client<robot_patrol_interfaces::srv::GetDirection>("direction_service");
    timer_ = create_wall_timer(100ms, std::bind(&PatrolWithService::control_loop, this));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    last_scan_ = msg;
  }

  void control_loop() {
    if (!last_scan_) return;

    auto request = std::make_shared<robot_patrol_interfaces::srv::GetDirection::Request>();
    request->laser_data = *last_scan_;

    // Call service asynchronously
    auto result = client_->async_send_request(request,
      [this](rclcpp::Client<robot_patrol_interfaces::srv::GetDirection>::SharedFuture future) {
        auto response = future.get();
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.1;

        if (response->direction == "left") {
          cmd.angular.z = 0.5;
        } else if (response->direction == "right") {
          cmd.angular.z = -0.5;
        } else {
          cmd.angular.z = 0.0;
        }

        cmd_pub_->publish(cmd);
      });
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Client<robot_patrol_interfaces::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PatrolWithService>());
  rclcpp::shutdown();
  return 0;
}