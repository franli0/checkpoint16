#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <vector>
#include <chrono>

class EightTrajectory : public rclcpp::Node
{
public:
    EightTrajectory() : Node("eight_trajectory"), current_waypoint_(0), trajectory_started_(false)
    {
        // Create subscriber for odometry
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/rosbot_xl_base_controller/odom", 10,
            std::bind(&EightTrajectory::odomCallback, this, std::placeholders::_1));
        
        // Create publisher for wheel speeds
        wheel_speed_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
        
        // Robot parameters
        wheel_radius_ = 0.05;        
        wheelbase_x_ = 0.170;        
        wheelbase_y_ = 0.270;        
        
        // TRUE FIGURE-EIGHT WAYPOINTS
        // Robot starts at W8, follows: W8→W1→W2→W3→W4→W5→W6→W7→W8
        // W2 and W6 are the same position
        waypoints_ = {
            {0.0,     1.0, -1.0},  // W8→W1: up-right diagonal "/"
            {0.0,     1.0,  1.0},  // W1→W2: up-left diagonal "\"  
            {0.0,     1.0,  1.0},  // W2→W3: up-left diagonal "\"
            {-1.5708, 1.0, -1.0},  // W3→W4: up-right diagonal "/" + rotate -90°
            {-1.5708, -1.0, -1.0}, // W4→W5: down-right diagonal "\" + rotate -90°
            {0.0,     -1.0,  1.0}, // W5→W6: down-left diagonal "/"
            {0.0,     -1.0,  1.0}, // W6→W7: down-left diagonal "/"
            {0.0,     -1.0, -1.0}  // W7→W8: down-right diagonal "\" (back to start)
        };
        
        // Control parameters
        position_tolerance_ = 0.15;   
        angle_tolerance_ = 0.15;     
        base_speed_ = 0.12;          
        
        RCLCPP_INFO(this->get_logger(), "TRUE FIGURE-EIGHT TRAJECTORY");
        RCLCPP_INFO(this->get_logger(), "Pattern: W8(start)->W1->W2->W3->W4->W5->W6->W7->W8");
        RCLCPP_INFO(this->get_logger(), "W2 and W6 are the crossing point of the figure-eight");
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        
        tf2::Quaternion quat;
        tf2::fromMsg(msg->pose.pose.orientation, quat);
        current_theta_ = tf2::getYaw(quat);
        
        if (!trajectory_started_) {
            initializeTrajectory();
            return;
        }
        
        if (current_waypoint_ < waypoints_.size()) {
            executeCurrentWaypoint();
        } else {
            publishWheelSpeeds(0.0, 0.0, 0.0);
            static bool completed_logged = false;
            if (!completed_logged) {
                RCLCPP_INFO(this->get_logger(), "FIGURE-EIGHT COMPLETED! Back at W8!");
                completed_logged = true;
            }
        }
    }
    
    void initializeTrajectory()
    {
        start_x_ = current_x_;
        start_y_ = current_y_;
        start_theta_ = current_theta_;
        
        calculateTarget();
        trajectory_started_ = true;
        
        RCLCPP_INFO(this->get_logger(), "Starting figure-eight at W8: (%.2f, %.2f, %.0f°)", 
                   start_x_, start_y_, start_theta_ * 180.0 / M_PI);
        RCLCPP_INFO(this->get_logger(), "First movement: W8 -> W1");
    }
    
    void calculateTarget()
    {
        if (current_waypoint_ >= waypoints_.size()) return;
        
        auto& wp = waypoints_[current_waypoint_];
        double dphi = wp[0];
        double dx = wp[1];
        double dy = wp[2];
        
        // Calculate target position
        target_x_ = current_x_ + dx;
        target_y_ = current_y_ + dy;
        
        // Calculate target orientation (shortest path)
        double desired_theta = current_theta_ + dphi;
        while (desired_theta > M_PI) desired_theta -= 2.0 * M_PI;
        while (desired_theta < -M_PI) desired_theta += 2.0 * M_PI;
        
        double angle_diff = desired_theta - current_theta_;
        while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
        while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;
        
        target_theta_ = current_theta_ + angle_diff;
        while (target_theta_ > M_PI) target_theta_ -= 2.0 * M_PI;
        while (target_theta_ < -M_PI) target_theta_ += 2.0 * M_PI;
        
        waypoint_start_time_ = this->now();
        
        // Describe each movement in the figure-eight sequence
        std::vector<std::string> movement_descriptions = {
            "W8→W1: up-right '/' diagonal",
            "W1→W2: up-left '\\' diagonal (to crossing point)", 
            "W2→W3: up-left '\\' diagonal",
            "W3→W4: up-right '/' diagonal + rotate -90°",
            "W4→W5: down-right '\\' diagonal + rotate -90°",
            "W5→W6: down-left '/' diagonal (to crossing point)",
            "W6→W7: down-left '/' diagonal", 
            "W7→W8: down-right '\\' diagonal (back to start)"
        };
        
        RCLCPP_INFO(this->get_logger(), "Waypoint%zu: %s", 
                   current_waypoint_+1, movement_descriptions[current_waypoint_].c_str());
    }
    
    void executeCurrentWaypoint()
    {
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double distance = sqrt(dx*dx + dy*dy);
        
        double theta_error = target_theta_ - current_theta_;
        while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
        while (theta_error < -M_PI) theta_error += 2.0 * M_PI;
        
        // Timeout check
        auto waypoint_duration = (this->now() - waypoint_start_time_).seconds();
        if (waypoint_duration > 15.0) {
            RCLCPP_WARN(this->get_logger(), "TIMEOUT Waypoint%zu - advancing", current_waypoint_+1);
            current_waypoint_++;
            if (current_waypoint_ < waypoints_.size()) calculateTarget();
            return;
        }
        
        // Progress logging
        static int debug_count = 0;
        if (++debug_count % 25 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                       "WP%zu: (%.2f,%.2f,%.0f°) → (%.2f,%.2f,%.0f°) | %.2fm %.0f° [%.1fs]", 
                       current_waypoint_+1, 
                       current_x_, current_y_, current_theta_*180/M_PI,
                       target_x_, target_y_, target_theta_*180/M_PI, 
                       distance, fabs(theta_error)*180/M_PI, waypoint_duration);
        }
        
        // Check if reached
        if (distance < position_tolerance_ && fabs(theta_error) < angle_tolerance_) {
            RCLCPP_INFO(this->get_logger(), "COMPLETED Waypoint%zu in %.1fs!", 
                       current_waypoint_ + 1, waypoint_duration);
            current_waypoint_++;
            if (current_waypoint_ < waypoints_.size()) {
                calculateTarget();
            }
            return;
        }
        
        // FIGURE-EIGHT DIAGONAL MOVEMENT CONTROL
        double kp_pos = 0.8;
        double kp_ang = 1.0;
        
        double vx = dx * kp_pos;
        double vy = dy * kp_pos;
        double wz = theta_error * kp_ang;
        
        // Apply speed limits
        double max_linear = base_speed_;
        double max_angular = 0.3;
        
        vx = std::max(-max_linear, std::min(max_linear, vx));
        vy = std::max(-max_linear, std::min(max_linear, vy));
        wz = std::max(-max_angular, std::min(max_angular, wz));
        
        if ((current_waypoint_ == 3 || current_waypoint_ == 4) && debug_count % 15 == 0) {
            std::string expected_movement = "";
            if (current_waypoint_ == 3) {
                // W3→W4: should be up-right diagonal + rotation
                expected_movement = "up-right '/' + rotate -90°";
                bool correct_dir = (vx > 0.01 && vy < -0.01);
                bool correct_rot = (wz < -0.01);
                RCLCPP_INFO(this->get_logger(), "WP4: %s | Movement: %s Rotation: %s", 
                           expected_movement.c_str(),
                           correct_dir ? "O" : "X", correct_rot ? "V" : "X");
            } else if (current_waypoint_ == 4) {
                // W4→W5: should be down-right diagonal + rotation  
                expected_movement = "down-right '\\' + rotate -90°";
                bool correct_dir = (vx < -0.01 && vy < -0.01);
                bool correct_rot = (wz < -0.01);
                RCLCPP_INFO(this->get_logger(), "WP5: %s | Movement: %s Rotation: %s", 
                           expected_movement.c_str(),
                           correct_dir ? "V" : "X", correct_rot ? "V" : "X");
            }
            
            RCLCPP_INFO(this->get_logger(), "Commanding: vx=%.3f vy=%.3f wz=%.3f", vx, vy, wz);
        }
        
        publishWheelSpeeds(vx, vy, wz);
    }
    
    void publishWheelSpeeds(double vx, double vy, double wz)
    {
        // Standard mecanum wheel inverse kinematics
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

    // ROS2 components
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speed_publisher_;
    
    // Trajectory data
    std::vector<std::vector<double>> waypoints_;
    size_t current_waypoint_;
    bool trajectory_started_;
    
    // Current and target pose
    double current_x_, current_y_, current_theta_;
    double target_x_, target_y_, target_theta_;
    double start_x_, start_y_, start_theta_;
    
    // Robot parameters
    double wheel_radius_, wheelbase_x_, wheelbase_y_;
    double position_tolerance_, angle_tolerance_;
    double base_speed_;
    
    // Timing
    rclcpp::Time waypoint_start_time_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EightTrajectory>());
    rclcpp::shutdown();
    return 0;
}