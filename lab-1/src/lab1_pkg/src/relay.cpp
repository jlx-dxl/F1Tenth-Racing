#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class RelayNode : public rclcpp::Node {
public:
    RelayNode() : Node("relay") {
        subscription_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive",
            10,
            std::bind(&RelayNode::listener_callback, this, std::placeholders::_1)
        );

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);
    }

private:
    void listener_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        auto new_msg = ackermann_msgs::msg::AckermannDriveStamped();
        new_msg.header.stamp = this->get_clock()->now();
        new_msg.drive.speed = msg->drive.speed * 3;
        new_msg.drive.steering_angle = msg->drive.steering_angle * 3;

        publisher_->publish(new_msg);
        RCLCPP_INFO(this->get_logger(), "Publishing to /drive_relay: Speed: %f, Steering Angle: %f",
                    new_msg.drive.speed, new_msg.drive.steering_angle);
    }

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RelayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}