#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&Patrol::scan_callback, this, std::placeholders::_1));
    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 100);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::control_loop, this));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    scan_ = msg;
  }

  void control_loop() 
  {
   if(scan_ == nullptr){ return; }

   geometry_msgs::msg::Twist cmd;
   cmd.linear.x = FORWARD_SPEED;

   float safest_angle = find_safest_angle();

   if (std::isfinite(safest_angle)) 
   { 
    cmd.angular.z = -safest_angle * ANGULAR_SPEED_GAIN; 
   } 
   else 
   {
    cmd.angular.z = 0.0; 
   }
   cmd_pub_->publish(cmd);
 }

  float find_safest_angle() {
    float max_distance = 0.0;
    float safest_angle = std::numeric_limits<float>::quiet_NaN(); 

    for (int i = -SCAN_ANGLE_RANGE / 2; i <= SCAN_ANGLE_RANGE / 2; ++i) {
      int index = (i + 360) % 360; 
      float distance = scan_->ranges[index];

      if (std::isfinite(distance) && distance > max_distance) {
        max_distance = distance;
        safest_angle = i * scan_->angle_increment; 
      }
    }

    return safest_angle;
  }

  const float FORWARD_SPEED = 0.1;
  const float ANGULAR_SPEED_GAIN = 0.25;
  const int SCAN_ANGLE_RANGE = 180; 
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Patrol>());
  rclcpp::shutdown();
  return 0;
}