#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tuple>

using namespace std::chrono_literals;

class TurnController : public rclcpp::Node {
public:
  TurnController() : Node("turn_controller") {
    odom_subscription = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&TurnController::odomCallback, this, std::placeholders::_1));

    velocity_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer = this->create_wall_timer(
        20ms, std::bind(&TurnController::timerCallback, this));

    lastExecutionTime = std::chrono::high_resolution_clock::now();
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    currentPosition = msg->pose.pose;
  }

  void timerCallback() {

    float deltaTime =
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::high_resolution_clock::now() - lastExecutionTime)
            .count() /
        1000000000.0;

    float errorDistanceX =
        (currentPosition.position.x + std::get<0>(goals[goalCounter])) -
        currentPosition.position.x;
    float errorDistanceY =
        (currentPosition.position.y + std::get<1>(goals[goalCounter])) -
        currentPosition.position.y;

    float theta = atan2(errorDistanceY, errorDistanceX);

    float newErrorAngle = theta - calculateYaw();
    newErrorAngle = atan2(sin(newErrorAngle), cos(newErrorAngle));

    RCLCPP_INFO(this->get_logger(), "%f", newErrorAngle);

    // Error for the proportional term
    float errorProportional = newErrorAngle;

    // Error for the integral term.
    errorIntegral = errorIntegral + newErrorAngle * deltaTime;

    // Error for the derivative term.
    float errorDerivative = (newErrorAngle - errorAngle) / deltaTime;

    float speedFactor =
        Kp * errorProportional + Ki * errorIntegral + Kd * errorDerivative;

    cmd_vel_msg.angular.z =
        (abs(newErrorAngle) > 0.05) ? speedFactor * 1.0 : 0.0;

    velocity_publisher->publish(cmd_vel_msg);

    if (abs(newErrorAngle) < 0.05) {

      goalCounter++;

      rclcpp::sleep_for(std::chrono::seconds(1));

      if (goalCounter == goals.size()) {
        this->timer->cancel();
        rclcpp::shutdown();
        return;
      }
    }

    errorAngle = newErrorAngle;
    lastExecutionTime = std::chrono::high_resolution_clock::now();
  }

  float calculateYaw() {
    tf2::Quaternion q(
        currentPosition.orientation.x, currentPosition.orientation.y,
        currentPosition.orientation.z, currentPosition.orientation.w);

    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return yaw;
  }

  // PID gains
  float Kp = 2.5;
  float Ki = 0.01;
  float Kd = 0.5;

  geometry_msgs::msg::Pose currentPosition;

  std::chrono::time_point<std::chrono::system_clock> lastExecutionTime;

  size_t goalCounter = 0;
  std::vector<std::tuple<float, float>> goals = {
      {0.6, -1.3}, {1.5, -0.3}, {0.7, 0.6}};

  float errorIntegral;
  float errorAngle;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher;
  rclcpp::TimerBase::SharedPtr timer;

  geometry_msgs::msg::Twist cmd_vel_msg;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TurnController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
