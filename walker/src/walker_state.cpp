#include "walker/walker_state.hpp"
#include "walker/walker_node.hpp"
#include <chrono>
#include <thread>

bool WalkerState::is_path_blocked(
    const std::vector<float>& ranges, 
    double min_distance) {
    
    // Check each range value in the provided array
    for (float range : ranges) {
        // If range is less than minimum distance and not infinite
        if (!std::isinf(range) && range < min_distance) {
            return true;  // Path is blocked
        }
    }
    return false;  // Path is clear
}

geometry_msgs::msg::Twist ForwardState::process_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    
    geometry_msgs::msg::Twist velocity;
    
    // Reduced scan arc to ±20°
    size_t start_idx = scan->ranges.size() * 10 / 36;   // -20 degrees
    size_t end_idx = scan->ranges.size() * 10 / 36;     // +20 degrees
    
    float min_range = std::numeric_limits<float>::infinity();
    int valid_readings = 0;
    float sum_ranges = 0.0;
    
    // Add predictive checking
    static float prev_min_range = std::numeric_limits<float>::infinity();
    static float range_rate = 0.0;
    static auto last_state_change = std::chrono::steady_clock::now();
    
    for (size_t i = start_idx; i < end_idx; ++i) {
        if (!std::isinf(scan->ranges[i]) && scan->ranges[i] > 0.1) {
            sum_ranges += scan->ranges[i];
            valid_readings++;
            if (scan->ranges[i] < min_range) {
                min_range = scan->ranges[i];
            }
        }
    }
    
    // Calculate rate of change of distance
    range_rate = (min_range - prev_min_range) / 0.1;
    prev_min_range = min_range;
    
    float predicted_range = min_range + range_rate * 0.5;
    
    double warn_dist = context_->get_warning_distance();
    double crit_dist = context_->get_critical_distance();
    
    // Add minimum time between state changes (2 seconds)
    auto current_time = std::chrono::steady_clock::now();
    auto time_since_last_change = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - last_state_change).count();
    
    if ((predicted_range <= crit_dist || min_range <= crit_dist + 0.3) && 
        time_since_last_change > 2000) {  // 2 second minimum between changes
        
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
        
        RCLCPP_INFO(rclcpp::get_logger("walker_node"), 
            "Obstacle detected at %.2f meters (predicted: %.2f). Stopping...", 
            min_range, predicted_range);
        
        last_state_change = current_time;
        context_->change_state(
            std::make_unique<RotatingState>(
                context_, 
                static_cast<bool>(rand() % 2)
            )
        );
    }
    else if (predicted_range <= warn_dist || min_range <= warn_dist) {
        double slow_factor = std::min(1.0, 
            (min_range - crit_dist) / (warn_dist - crit_dist));
        slow_factor = std::pow(slow_factor, 1.5);  // Less aggressive slowdown
        
        velocity.linear.x = context_->get_linear_velocity() * slow_factor;
        velocity.angular.z = 0.0;
    }
    else {
        velocity.linear.x = context_->get_linear_velocity();
        velocity.angular.z = 0.0;
    }
    
    return velocity;
}

geometry_msgs::msg::Twist RotatingState::process_scan(
    const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    
    geometry_msgs::msg::Twist velocity;
    static auto rotation_start_time = std::chrono::steady_clock::now();
    static int rotation_count = 0;
    
    // Reduced scan arc from ±30° to ±20°
    size_t start_idx = scan->ranges.size() * 10 / 36;   // -20 degrees
    size_t end_idx = scan->ranges.size() * 13 / 36;     // +20 degrees
    
    float min_range = std::numeric_limits<float>::infinity();
    int valid_readings = 0;
    int clear_path_count = 0;
    
    for (size_t i = start_idx; i < end_idx; ++i) {
        if (!std::isinf(scan->ranges[i]) && scan->ranges[i] > 0.1) {
            valid_readings++;
            if (scan->ranges[i] < min_range) {
                min_range = scan->ranges[i];
            }
            if (scan->ranges[i] > context_->get_warning_distance() + 0.5) {  // Add buffer
                clear_path_count++;
            }
        }
    }

    auto current_time = std::chrono::steady_clock::now();
    auto rotation_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        current_time - rotation_start_time).count();

    float clear_path_percentage = valid_readings > 0 ? 
        (float)clear_path_count / valid_readings : 0.0;

    // Require minimum rotation time and higher clear path threshold
    if (clear_path_percentage > 0.8 && rotation_duration > 2000) {  // 80% clear and 2 seconds minimum
        RCLCPP_INFO(rclcpp::get_logger("walker_node"), 
            "Clear path detected (%.2f%% clear). Resuming forward movement.", 
            clear_path_percentage * 100);
        
        rotation_count = 0;  // Reset rotation count
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
        context_->change_state(std::make_unique<ForwardState>(context_));
        velocity.linear.x = 0.0;
        velocity.angular.z = 0.0;
    }
    else {
        velocity.linear.x = 0.0;
        double rotation_speed = context_->get_angular_velocity();
        
        if (min_range < context_->get_warning_distance()) {
            rotation_speed *= 0.5;  // Slower rotation near obstacles
        }
        
        velocity.angular.z = rotate_clockwise_ ? -rotation_speed : rotation_speed;
        
        // Change direction after longer rotation time
        if (rotation_duration > 4000) {  // 4 seconds
            rotate_clockwise_ = !rotate_clockwise_;
            rotation_start_time = current_time;
            rotation_count++;
            
            // If stuck in rotation too long, try to move forward
            if (rotation_count > 3) {
                RCLCPP_INFO(rclcpp::get_logger("walker_node"), 
                    "Multiple rotations detected, attempting forward movement");
                context_->change_state(std::make_unique<ForwardState>(context_));
            }
        }
    }
    
    return velocity;
}