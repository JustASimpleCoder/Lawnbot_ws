#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <std_msgs/msg/float32.hpp>
#include <iostream>

class ManualControl : public rclcpp::Node {
public:
    ManualControl() : Node("manual_control") {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        temp_sub_ = this->create_subscription<sensor_msgs::msg::Temperature>(
            "temperature", 10, 
            [this](const sensor_msgs::msg::Temperature::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "Temperature: %.2f °C", msg->temperature);
            });
        pid_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "pid_info", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), "PID Value: %.2f", msg->data);
            });

        RCLCPP_INFO(this->get_logger(), "Use keys: 'w/a/s/d' for movement, '+'/'-' to adjust speed.");
        speed_ = 0.5;
        std::thread(&ManualControl::keyListener, this).detach();
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temp_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pid_sub_;
    double speed_;

    void keyListener() {
        char input;
        while (true) {
            std::cin >> input;
            geometry_msgs::msg::Twist cmd_msg;

            switch (input) {
                case 'w': cmd_msg.linear.x = speed_; break;
                case 's': cmd_msg.linear.x = -speed_; break;
                case 'a': cmd_msg.angular.z = speed_; break;
                case 'd': cmd_msg.angular.z = -speed_; break;
                case '+': speed_ += 0.1; RCLCPP_INFO(this->get_logger(), "Speed: %.2f", speed_); continue;
                case '-': speed_ -= 0.1; RCLCPP_INFO(this->get_logger(), "Speed: %.2f", speed_); continue;
                default: continue;
            }
            cmd_pub_->publish(cmd_msg);
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManualControl>());
    rclcpp::shutdown();
    return 0;
}
