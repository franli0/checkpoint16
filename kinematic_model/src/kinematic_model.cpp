#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <cmath>

class KinematicModel : public rclcpp::Node
{
public:
    KinematicModel() : Node("kinematic_model")
    {
        // Robot parameters for ROSBot XL with mecanum wheels
        wheel_radius_ = 0.05;        // 5 cm wheel radius (mecanum)
        wheelbase_x_ = 0.170;        // 17 cm between front and rear wheels
        wheelbase_y_ = 0.270;        // 27 cm between left and right wheels
        
        // Create subscriber for wheel speeds
        wheel_speed_subscriber_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_speed", 10,
            std::bind(&KinematicModel::wheelSpeedCallback, this, std::placeholders::_1));
        
        // Create publisher for cmd_vel
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        RCLCPP_INFO(this->get_logger(), "Kinematic model node initialized");
    }

private:
    void wheelSpeedCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4) {
            RCLCPP_WARN(this->get_logger(), "Expected 4 wheel speeds, got %zu", msg->data.size());
            return;
        }

        // Extract wheel speeds (order: front_left, front_right, rear_left, rear_right)
        double w_fl = msg->data[0];  // front left
        double w_fr = msg->data[1];  // front right
        double w_rl = msg->data[2];  // rear left
        double w_rr = msg->data[3];  // rear right

        // Apply mecanum wheel kinematic model
        // Forward kinematics: wheel speeds -> robot velocities
        // For mecanum wheels:
        // vx = (w_fl + w_fr + w_rl + w_rr) * r / 4
        // vy = (-w_fl + w_fr + w_rl - w_rr) * r / 4
        // wz = (-w_fl + w_fr - w_rl + w_rr) * r / (4 * (lx + ly))
        
        double lx = wheelbase_x_ / 2.0;  // half wheelbase in x direction
        double ly = wheelbase_y_ / 2.0;  // half wheelbase in y direction
        
        // Calculate robot velocities using mecanum kinematics
        double vx = (w_fl + w_fr + w_rl + w_rr) * wheel_radius_ / 4.0;
        double vy = (-w_fl + w_fr + w_rl - w_rr) * wheel_radius_ / 4.0;
        double wz = (-w_fl + w_fr - w_rl + w_rr) * wheel_radius_ / (4.0 * (lx + ly));

        // Create and publish Twist message
        geometry_msgs::msg::Twist twist_msg;
        twist_msg.linear.x = vx;
        twist_msg.linear.y = vy;
        twist_msg.linear.z = 0.0;
        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = wz;

        cmd_vel_publisher_->publish(twist_msg);
        
        // Log the transformation for debugging
        RCLCPP_DEBUG(this->get_logger(), 
            "Wheel speeds: [%.2f, %.2f, %.2f, %.2f] -> Twist: vx=%.2f, vy=%.2f, wz=%.2f",
            w_fl, w_fr, w_rl, w_rr, vx, vy, wz);
    }

    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    
    // Robot parameters
    double wheel_radius_;
    double wheelbase_x_;
    double wheelbase_y_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<KinematicModel>());
    rclcpp::shutdown();
    return 0;
}