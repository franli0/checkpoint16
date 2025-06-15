#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <vector>

class EightTrajectorySimple : public rclcpp::Node
{
public:
    EightTrajectorySimple() : Node("eight_trajectory"), current_waypoint_(0), trajectory_started_(false)
    {
        // Create subscriber for odometry
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_xl_base_controller/odom", 10,
            std::bind(&EightTrajectorySimple::odomCallback, this, std::placeholders::_1));
        
        // Create publisher for wheel speeds
        wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
        
        // Robot parameters
        wheel_radius_ = 0.05;
        wheelbase_x_ = 0.170;
        wheelbase_y_ = 0.270;
        
        // Waypoints [dphi, dx, dy] - EXACTLY as specified in task
        waypoints_ = {
            {0.0,     1.0, -1.0},  // w1
            {0.0,     1.0,  1.0},  // w2  
            {0.0,     1.0,  1.0},  // w3
            {-1.5708, 1.0, -1.0},  // w4
            {-1.5708, -1.0, -1.0}, // w5
            {0.0,     -1.0,  1.0}, // w6
            {0.0,     -1.0,  1.0}, // w7
            {0.0,     -1.0, -1.0}  // w8
        };
        
        RCLCPP_INFO(this->get_logger(), "Eight trajectory simple controller initialized");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        // Update current pose
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        current_theta_ = tf2::getYaw(quat);
        
        if (!trajectory_started_) {
            // Store starting position for this waypoint
            start_x_ = current_x_;
            start_y_ = current_y_;
            start_theta_ = current_theta_;
            
            // Calculate target for first waypoint
            auto& wp = waypoints_[current_waypoint_];
            target_x_ = start_x_ + wp[1];
            target_y_ = start_y_ + wp[2];
            target_theta_ = start_theta_ + wp[0];
            
            trajectory_started_ = true;
            
            RCLCPP_INFO(this->get_logger(), "Starting from: (%.3f, %.3f, %.1f°)", 
                       start_x_, start_y_, start_theta_ * 180.0 / M_PI);
            RCLCPP_INFO(this->get_logger(), "Moving to waypoint 1: (%.3f, %.3f, %.1f°)", 
                       target_x_, target_y_, target_theta_ * 180.0 / M_PI);
            return;
        }
        
        if (current_waypoint_ < waypoints_.size()) {
            moveToCurrentTarget();
        } else {
            // All waypoints completed
            publishWheelSpeeds(0.0, 0.0, 0.0);
            static bool completed = false;
            if (!completed) {
                completed = true;
                RCLCPP_INFO(this->get_logger(), "Figure-eight trajectory completed!");
                double error = sqrt(current_x_*current_x_ + current_y_*current_y_);
                RCLCPP_INFO(this->get_logger(), "Final positioning error: %.3f m", error);
                rclcpp::shutdown();
            }
        }
    }
    
    void moveToCurrentTarget()
    {
        // Calculate errors
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance = sqrt(dx*dx + dy*dy);
        
        double theta_error = target_theta_ - current_theta_;
        while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
        while (theta_error < -M_PI) theta_error += 2.0 * M_PI;
        
        // Debug every 25 calls
        static int debug_count = 0;
        if (++debug_count % 25 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "WP%zu: Pos(%.3f,%.3f,%.1f°) Target(%.3f,%.3f,%.1f°) Dist=%.3f AngleErr=%.1f°", 
                       current_waypoint_+1, current_x_, current_y_, current_theta_*180/M_PI,
                       target_x_, target_y_, target_theta_*180/M_PI, distance, fabs(theta_error)*180/M_PI);
        }
        
        // Check if reached target
        if (distance < 0.1 && fabs(theta_error) < 0.2) {
            RCLCPP_INFO(this->get_logger(), "Reached waypoint %zu!", current_waypoint_ + 1);
            
            current_waypoint_++;
            if (current_waypoint_ < waypoints_.size()) {
                // Set up next waypoint
                start_x_ = current_x_;
                start_y_ = current_y_;
                start_theta_ = current_theta_;
                
                auto& wp = waypoints_[current_waypoint_];
                target_x_ = start_x_ + wp[1];
                target_y_ = start_y_ + wp[2];
                target_theta_ = start_theta_ + wp[0];
                
                RCLCPP_INFO(this->get_logger(), "Now moving to waypoint %zu: (%.3f, %.3f, %.1f°)", 
                           current_waypoint_+1, target_x_, target_y_, target_theta_ * 180.0 / M_PI);
            }
            return;
        }
        
        // Simple proportional control - move straight toward target
        double vx = dx * 0.6;  // Proportional gain
        double vy = dy * 0.6;
        double wz = theta_error * 0.8;
        
        // Limit velocities
        vx = std::max(-0.15, std::min(0.15, vx));
        vy = std::max(-0.15, std::min(0.15, vy));
        wz = std::max(-0.3, std::min(0.3, wz));
        
        publishWheelSpeeds(vx, vy, wz);
    }
    
    void publishWheelSpeeds(double vx, double vy, double wz)
    {
        // Inverse kinematics for mecanum wheels
        double lx = wheelbase_x_ / 2.0;
        double ly = wheelbase_y_ / 2.0;
        double wheel_base_sum = lx + ly;
        
        double w_fl = (vx - vy - wz * wheel_base_sum) / wheel_radius_;
        double w_fr = (vx + vy + wz * wheel_base_sum) / wheel_radius_;
        double w_rl = (vx + vy - wz * wheel_base_sum) / wheel_radius_;
        double w_rr = (vx - vy + wz * wheel_base_sum) / wheel_radius_;
        
        auto msg = std_msgs::msg::Float32MultiArray();
        msg.data = {static_cast<float>(w_fl), static_cast<float>(w_fr), 
                    static_cast<float>(w_rl), static_cast<float>(w_rr)};
        
        wheel_speed_publisher_->publish(msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_publisher_;
    
    std::vector<std::vector<double>> waypoints_;  // [dphi, dx, dy]
    size_t current_waypoint_;
    bool trajectory_started_;
    
    // Current pose
    double current_x_, current_y_, current_theta_;
    
    // Target for current waypoint 
    double target_x_, target_y_, target_theta_;
    
    // Starting position for current waypoint
    double start_x_, start_y_, start_theta_;
    
    // Robot parameters
    double wheel_radius_;
    double wheelbase_x_;
    double wheelbase_y_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EightTrajectorySimple>());
    rclcpp::shutdown();
    return 0;
}