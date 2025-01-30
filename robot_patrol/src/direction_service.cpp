#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <robot_patrol_interfaces/srv/get_direction.hpp>
#include <cmath>

using namespace std::chrono_literals;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    srv_ = create_service<robot_patrol_interfaces::srv::GetDirection>(
      "direction_service",
      std::bind(&DirectionService::handle_request, this, std::placeholders::_1, std::placeholders::_2)
    );
  }

private:
  void handle_request(
    const robot_patrol_interfaces::srv::GetDirection::Request::SharedPtr request,
    robot_patrol_interfaces::srv::GetDirection::Response::SharedPtr response
  ) {
    const auto& scan = request->laser_data;
    double angle_min = scan.angle_min;
    double angle_increment = scan.angle_increment;

    float sum_left = 0.0, sum_front = 0.0, sum_right = 0.0;

    // Divide laser data into 3 sections (60º each)
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      double angle = angle_min + i * angle_increment; // Current angle in radians

      // Left section (60º to 120º from front)
      if (angle > M_PI/3 && angle <= 2*M_PI/3) {
        sum_left += scan.ranges[i];
      }
      // Front section (-60º to 60º)
      else if (angle >= -M_PI/3 && angle <= M_PI/3) {
        sum_front += scan.ranges[i];
      }
      // Right section (-120º to -60º)
      else if (angle >= -2*M_PI/3 && angle < -M_PI/3) {
        sum_right += scan.ranges[i];
      }
    }

    // Determine safest direction
    if (sum_front > sum_left && sum_front > sum_right) {
      response->direction = "forward";
    } else if (sum_left > sum_front && sum_left > sum_right) {
      response->direction = "left";
    } else {
      response->direction = "right";
    }
  }

  rclcpp::Service<robot_patrol_interfaces::srv::GetDirection>::SharedPtr srv_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}