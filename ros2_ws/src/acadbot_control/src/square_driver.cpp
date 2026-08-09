// square_driver.cpp
// ---------------------------------------------------------------------------
// Session 1 — "Odometry drift" demo.
//
// An open-loop C++ controller: it drives the robot around a square by
// alternating "drive straight" and "turn 90°" phases, purely on a timer with
// NO feedback from odometry. Because it never corrects itself, the robot's
// odometry estimate visibly drifts away from the true pose in RViz2 — which is
// exactly the motivation for SLAM in the rest of the track.
//
// This is also the students' first real ROS 2 C++ node: a publisher + a timer,
// parameters loaded from YAML, and a small state machine.
// ---------------------------------------------------------------------------
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class SquareDriver : public rclcpp::Node
{
public:
  SquareDriver() : Node("square_driver")
  {
    // ---- Parameters (overridable from YAML / CLI) ----
    side_length_   = declare_parameter<double>("side_length", 2.0);     // m
    linear_speed_  = declare_parameter<double>("linear_speed", 0.25);   // m/s
    angular_speed_ = declare_parameter<double>("angular_speed", 0.6);   // rad/s
    laps_          = declare_parameter<int>("laps", 0);   // 0 = drive forever (old behaviour)

    // Time to cover one side, and time to turn 90 degrees, at the set speeds.
    drive_time_ = side_length_ / linear_speed_;
    turn_time_  = (M_PI / 2.0) / angular_speed_;

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // 20 Hz control loop.
    timer_ = create_wall_timer(50ms, std::bind(&SquareDriver::on_timer, this));
    phase_start_ = now();

    RCLCPP_INFO(get_logger(),
      "square_driver: side=%.2fm  v=%.2fm/s  w=%.2frad/s  (drive %.1fs / turn %.1fs)",
      side_length_, linear_speed_, angular_speed_, drive_time_, turn_time_);
  }

private:
  enum class Phase { DRIVE, TURN };

  void on_timer()
  {
    const double elapsed = (now() - phase_start_).seconds();
    geometry_msgs::msg::Twist cmd;

    if (phase_ == Phase::DRIVE) {
      cmd.linear.x = linear_speed_;
      if (elapsed >= drive_time_) {
        switch_phase(Phase::TURN, "turning");
      }
    } else {  // TURN
      cmd.angular.z = angular_speed_;
      if (elapsed >= turn_time_) {
        sides_done_++;
        switch_phase(Phase::DRIVE, "driving");
        if (sides_done_ % 4 == 0) {
          const int laps_done = sides_done_ / 4;
          RCLCPP_INFO(get_logger(),
            "Completed a full loop (%d sides). Watch the odometry drift in RViz!",
            sides_done_);
          if (laps_ > 0 && laps_done >= laps_) {
            RCLCPP_INFO(get_logger(),
              "Ran %d lap(s) as requested — stopping.", laps_done);
            cmd_pub_->publish(geometry_msgs::msg::Twist());  // zero Twist: actually stop the robot
            rclcpp::shutdown();
            return;
          }
        }
      }
    }
    cmd_pub_->publish(cmd);
  }

  void switch_phase(Phase next, const char * what)
  {
    phase_ = next;
    phase_start_ = now();
    RCLCPP_DEBUG(get_logger(), "phase -> %s", what);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  double side_length_, linear_speed_, angular_speed_;
  double drive_time_, turn_time_;
  int laps_;
  Phase phase_{Phase::DRIVE};
  rclcpp::Time phase_start_;
  int sides_done_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SquareDriver>());
  rclcpp::shutdown();
  return 0;
}
