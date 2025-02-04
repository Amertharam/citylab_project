#include <rclcpp/rclcpp.hpp>
#include "rclcpp/logging.hpp"
#include "rclcpp/subscription_options.hpp"
#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <algorithm>
#include <limits>
#include <chrono>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    patrol_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    rclcpp::SubscriptionOptions patrol_opt;
    patrol_opt.callback_group = patrol_group;
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    laser_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&Patrol::laserCallback, this, std::placeholders::_1), patrol_opt);

    control_timer = this->create_wall_timer(100ms, std::bind(&Patrol::controlLoop, this), patrol_group);
    msg_timer     = this->create_wall_timer(500ms, std::bind(&Patrol::infoMsg, this), patrol_group);

    cmd.linear.x = 0.0f;
    cmd.angular.z = 0.0f;
  }
  ~Patrol() {
    RCLCPP_INFO(this->get_logger(), "Destructor Called Patrol Node Destroyed");
  }

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Front sector detection (center ±50 samples)
        size_t center = msg->ranges.size() / 2;
        auto front_sector = std::vector<float>(
            msg->ranges.begin() + center - 50,
            msg->ranges.begin() + center + 50
        );
        
        // Obstacle detection
        obstacle_detected_ = std::any_of(front_sector.begin(), front_sector.end(),
            [](float dist){ return std::isfinite(dist) && dist < 0.35; });

        // Full 180° processing (-90° to +90°)
        auto front_180 = std::vector<float>(
            msg->ranges.begin() + msg->ranges.size()/4,
            msg->ranges.begin() + 3*msg->ranges.size()/4
        );

        // Find safest direction
        float max_dist = 0;
        size_t max_index = 0;
        for(size_t i = 0; i < front_180.size(); ++i) {
            if(front_180[i] > max_dist && std::isfinite(front_180[i])) {
                max_dist = front_180[i];
                max_index = i;
            }
        }
        direction_ = (static_cast<float>(max_index)/front_180.size() - 0.5) * M_PI;
    }

    void controlLoop() {       
        cmd.linear.x = 0.1; // To always maintain 0.1 m/s forward velocity. I fixed it as a constant value.            
        cmd.angular.z = obstacle_detected_ ? (direction_ / 2) : 0.0;    // Apply angular velocity only when obstacle detected        
        cmd_vel_pub_->publish(cmd);
    }
    void infoMsg() {
      RCLCPP_INFO(this->get_logger(), "Linear Vel:  %f | Angular Vel : %f", cmd.linear.x, cmd.angular.z);
    }

    geometry_msgs::msg::Twist cmd;
    rclcpp::CallbackGroup::SharedPtr patrol_group;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::TimerBase::SharedPtr control_timer;
    rclcpp::TimerBase::SharedPtr msg_timer;
    float direction_ = 0.0;
    bool obstacle_detected_ = false;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<Patrol> patrol_node= std::make_shared<Patrol>();
  rclcpp::executors::MultiThreadedExecutor execute;
  execute.add_node(patrol_node);
  execute.spin();
  rclcpp::shutdown();
  return 0;
}