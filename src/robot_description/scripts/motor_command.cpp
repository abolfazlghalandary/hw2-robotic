#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64.hpp>

class MotorCommandNode : public rclcpp::Node
{
public:
    MotorCommandNode() : Node("motor_command_node")
    {
        this->declare_parameter("wheel_radius", 0.1);
        this->declare_parameter("wheel_separation", 0.46);
        
        wheel_radius_ = this->get_parameter("wheel_radius").as_double();
        wheel_separation_ = this->get_parameter("wheel_separation").as_double();
        
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MotorCommandNode::cmdVelCallback, this, std::placeholders::_1));
        
        left_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_motor_rpm", 10);
        right_motor_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_motor_rpm", 10);
        
        RCLCPP_INFO(this->get_logger(), "Motor Command Node started");
        RCLCPP_INFO(this->get_logger(), "Wheel radius: %.3f m, Wheel separation: %.3f m", 
                    wheel_radius_, wheel_separation_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double v = msg->linear.x;
        double w = msg->angular.z;

        double v_left = v - (w * wheel_separation_ / 2.0);
        double v_right = v + (w * wheel_separation_ / 2.0);
        
        double omega_left = v_left / wheel_radius_;
        double omega_right = v_right / wheel_radius_;
        
        std_msgs::msg::Float64 left_msg, right_msg;
        left_msg.data = omega_left;
        right_msg.data = omega_right;
        
        left_motor_pub_->publish(left_msg);
        right_motor_pub_->publish(right_msg);
        
        RCLCPP_INFO(this->get_logger(), 
                    "cmd_vel: v=%.3f m/s, w=%.3f rad/s -> Left=%.3f rad/s, Right=%.3f rad/s", 
                    v, w, omega_left, omega_right);
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_motor_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_motor_pub_;
    
    double wheel_radius_;
    double wheel_separation_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotorCommandNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
