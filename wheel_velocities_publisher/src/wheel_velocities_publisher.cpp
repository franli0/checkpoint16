#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

class WheelVelocitiesPublisher : public rclcpp::Node
{
public:
    WheelVelocitiesPublisher() : Node("wheel_velocities_publisher"), motion_index_(0), motion_start_time_(this->now())
    {
        // Create publisher for wheel speeds
        publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/wheel_speed", 10);
        
        // Create timer that runs at 10Hz
        timer_ = this->create_wall_timer(100ms, std::bind(&WheelVelocitiesPublisher::publishWheelSpeeds, this));
        
        // Initialize motion sequences
        initializeMotions();
        
        RCLCPP_INFO(this->get_logger(), "Initialized wheel velocities publisher node");
    }

private:
    void initializeMotions()
    {
        // Define wheel speeds for each motion [fl, fr, rl, rr] in rad/sec
        // Based on mecanum wheel kinematics
        
        // Motion 0: Move forward - all wheels same direction
        motions_.push_back({1.0, 1.0, 1.0, 1.0});
        motion_names_.push_back("Move forward");
        
        // Motion 1: Move backward - all wheels opposite direction  
        motions_.push_back({-1.0, -1.0, -1.0, -1.0});
        motion_names_.push_back("Move backward");
        
        // Motion 2: Move left - mecanum pattern for left movement
        motions_.push_back({-1.0, 1.0, 1.0, -1.0});
        motion_names_.push_back("Move left");
        
        // Motion 3: Move right - mecanum pattern for right movement  
        motions_.push_back({1.0, -1.0, -1.0, 1.0});
        motion_names_.push_back("Move right");
        
        // Motion 4: Turn clockwise - left wheels forward, right wheels backward
        motions_.push_back({-1.0, 1.0, -1.0, 1.0});
        motion_names_.push_back("Turn clockwise");
        
        // Motion 5: Turn counter-clockwise - left wheels backward, right wheels forward
        motions_.push_back({1.0, -1.0, 1.0, -1.0});
        motion_names_.push_back("Turn counter-clockwise");
        
        // Motion 6: Stop
        motions_.push_back({0.0, 0.0, 0.0, 0.0});
        motion_names_.push_back("Stop");
    }

    void publishWheelSpeeds()
    {
        auto current_time = this->now();
        
        // Log the current motion if we haven't already
        if (!logged_current_motion_ && motion_index_ < motions_.size())
        {
            RCLCPP_INFO(this->get_logger(), "%s", motion_names_[motion_index_].c_str());
            logged_current_motion_ = true;
        }
        
        // Check if 3 seconds have passed for current motion
        if ((current_time - motion_start_time_).seconds() >= 3.0)
        {
            // Move to next motion
            motion_index_++;
            motion_start_time_ = current_time;
            logged_current_motion_ = false; // Reset flag for next motion
            
            // Check if we've completed all motions
            if (motion_index_ >= motions_.size())
            {
                RCLCPP_INFO(this->get_logger(), "All motions completed. Shutting down.");
                rclcpp::shutdown();
                return;
            }
        }
        
        // Publish current motion wheel speeds
        if (motion_index_ < motions_.size())
        {
            auto msg = std_msgs::msg::Float32MultiArray();
            msg.data = motions_[motion_index_];
            publisher_->publish(msg);
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    std::vector<std::vector<float>> motions_;
    std::vector<std::string> motion_names_;
    size_t motion_index_;
    rclcpp::Time motion_start_time_;
    bool logged_current_motion_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WheelVelocitiesPublisher>());
    rclcpp::shutdown();
    return 0;
}