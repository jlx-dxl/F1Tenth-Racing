#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class TalkerNode : public rclcpp::Node {
public:
    TalkerNode() : Node("talker") {
        this->declare_parameter<double>("v", 0.0);
        this->declare_parameter<double>("d", 0.0);

        publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(0),
            std::bind(&TalkerNode::timer_callback, this)
        );
    }

private:
    void timer_callback() {
        double v = this->get_parameter("v").as_double();
        double d = this->get_parameter("d").as_double();

        auto message = ackermann_msgs::msg::AckermannDriveStamped();
        message.drive.speed = v;
        message.drive.steering_angle = d;

        publisher_->publish(message);
		// RCLCPP_INFO(this->get_logger(), "Publishing to /drive: Speed: %f, Steering Angle: %f", v, d);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TalkerNode>());
    rclcpp::shutdown();
    return 0;
}