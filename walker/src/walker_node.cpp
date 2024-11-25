#include "walker/walker_node.hpp"

WalkerNode::WalkerNode() 
    : Node("walker_node")
{
    // Declare parameters with default values
    this->declare_parameter("warning_dist", 0.7);
    this->declare_parameter("critical_dist", 0.4);
    this->declare_parameter("emergency_dist", 0.25);
    this->declare_parameter("linear_vel", 0.15);
    this->declare_parameter("angular_vel", 0.2);

    // Get parameters
    warning_distance_ = this->get_parameter("warning_dist").as_double();
    critical_distance_ = this->get_parameter("critical_dist").as_double();
    emergency_distance_ = this->get_parameter("emergency_dist").as_double();
    linear_velocity_ = this->get_parameter("linear_vel").as_double();
    angular_velocity_ = this->get_parameter("angular_vel").as_double();
    
    RCLCPP_INFO(this->get_logger(), 
        "Walker initialized with parameters:\n"
        "Warning distance: %.2f\n"
        "Critical distance: %.2f\n"
        "Emergency distance: %.2f\n"
        "Linear velocity: %.2f\n"
        "Angular velocity: %.2f",
        warning_distance_, critical_distance_, emergency_distance_,
        linear_velocity_, angular_velocity_);

    // Initialize ROS2 communications
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&WalkerNode::laser_callback, this, std::placeholders::_1));

    // Initialize state machine with forward state
    current_state_ = std::make_unique<ForwardState>(this);
    
    RCLCPP_INFO(this->get_logger(), 
        "Walker node initialized in %s state", 
        current_state_->get_state_name().c_str());
}

WalkerNode::WalkerNode(
    const rclcpp::NodeOptions& options,
    double linear_vel,
    double angular_vel,
    double warn_distance,
    double crit_distance,
    double emerg_distance)
    : Node("walker_node", options),
      rotate_clockwise_(true),
      linear_velocity_(linear_vel),
      angular_velocity_(angular_vel),
      min_distance_(crit_distance),        // Use critical distance as min_distance
      warning_distance_(warn_distance),
      critical_distance_(crit_distance),
      emergency_distance_(emerg_distance),
      current_linear_vel_(linear_vel)
{
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan",
        10,
        std::bind(&WalkerNode::laser_callback, this, std::placeholders::_1));
    
    current_state_ = std::make_unique<ForwardState>(this);
    
    RCLCPP_INFO(this->get_logger(), 
        "Walker node initialized with custom parameters");
}

void WalkerNode::change_state(std::unique_ptr<WalkerState> new_state) {
    RCLCPP_INFO(this->get_logger(), 
        "State changing from %s to %s", 
        current_state_->get_state_name().c_str(),
        new_state->get_state_name().c_str());
    
    current_state_ = std::move(new_state);
}

void WalkerNode::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto velocity = current_state_->process_scan(msg);
    last_cmd_ = velocity;
    publisher_->publish(velocity);
}