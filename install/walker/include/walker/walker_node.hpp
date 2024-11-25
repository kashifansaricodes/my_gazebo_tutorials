#ifndef WALKER_WALKER_BOT_HPP
#define WALKER_WALKER_BOT_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

// Forward declaration
class WalkerState;

/**
 * @class Walker
 * @brief A ROS2 node for controlling a walking robot.
 *
 * The Walker class is responsible for managing the state and movement of a walking robot.
 * It sets up necessary publishers and subscribers, handles state transitions, and processes
 * sensor data to navigate the robot safely.
 *
 * @details
 * The Walker node initializes the robot in a forward-moving state and provides methods to
 * change the robot's state, publish velocity commands, check for obstacles, and toggle the
 * rotation direction. It also includes a method to create timers for periodic tasks.
 *
 * @note The robot's rotation direction can be toggled between clockwise and counter-clockwise.
 */
class Walker : public rclcpp::Node
{
public:
    /**
     * @brief Constructor initializing the Walker node.
     *
     * Sets up publishers, subscribers, and initializes the robot in ForwardState.
     */
    Walker();

    /**
     * @brief Changes the current state of the Walker.
     * @param new_state Pointer to the new state to transition to.
     *
     * Deletes the current state and transitions to the provided new state.
     */
    void change_state(WalkerState *new_state);

    /**
     * @brief Publishes velocity commands to the robot.
     * @param linear Linear velocity in meters per second.
     * @param angular Angular velocity in radians per second.
     */
    void publish_velocity(double linear, double angular);

    /**
     * @brief Checks if the path ahead is clear of obstacles.
     * @param scan Shared pointer to the laser scan data.
     * @return true if path is clear, false if obstacle detected.
     */
    bool is_path_clear(const sensor_msgs::msg::LaserScan::SharedPtr scan) const;

    /**
     * @brief Toggles the rotation direction between clockwise and
     * counter-clockwise.
     */
    void toggle_rotation_direction();

    /**
     * @brief Gets the current rotation direction.
     * @return 1.0 for counter-clockwise, -1.0 for clockwise rotation.
     */
    double get_rotation_direction() const { return rotation_direction_; }

    /**
     * @brief Creates a timer with specified period and callback.
     * @param period Duration between timer callbacks.
     * @param callback Function to be called when timer expires.
     * @return Shared pointer to the created timer.
     */
    rclcpp::TimerBase::SharedPtr create_timer(
        const std::chrono::duration<double> &period,
        std::function<void()> callback);

private:
    /**
     * @brief Callback function for processing laser scan messages.
     * @param scan Shared pointer to the received laser scan message.
     */
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan);

    WalkerState *current_state_; ///< Pointer to current state object
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
        vel_publisher_; ///< Velocity command publisher
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
        scan_subscriber_;       ///< Laser scan subscriber
    double rotation_direction_; ///< Current rotation direction (1.0 for CCW,
                               ///< -1.0 for CW)
    const double SAFE_DISTANCE =
        0.8; ///< Minimum safe distance from obstacles in meters
};

/**
 * @class WalkerState
 * @brief Abstract base class for Walker states.
 *
 * Defines the interface for different states of the Walker robot.
 */
class WalkerState
{
public:
    /**
     * @brief Virtual destructor ensuring proper cleanup of derived classes.
     */
    virtual ~WalkerState() = default;

    /**
     * @brief Pure virtual function to handle robot behavior in current state.
     * @param walker Pointer to the Walker object.
     * @param scan Shared pointer to laser scan data.
     */
    virtual void handle(Walker *walker,
                       const sensor_msgs::msg::LaserScan::SharedPtr scan) = 0;
};

/**
 * @class ForwardState
 * @brief State implementing forward movement behavior.
 *
 * Handles robot behavior when moving forward, including obstacle detection
 * and state transitions.
 */
class ForwardState : public WalkerState
{
public:
    /**
     * @brief Handles forward movement and obstacle detection.
     * @param walker Pointer to the Walker object.
     * @param scan Shared pointer to laser scan data.
     */
    void handle(Walker *walker,
               const sensor_msgs::msg::LaserScan::SharedPtr scan) override;
};

/**
 * @class RotationState
 * @brief State implementing rotation behavior.
 *
 * Manages robot behavior during rotation, including timed rotations
 * and path checking.
 */
class RotationState : public WalkerState
{
public:
    /**
     * @brief Constructor initializing rotation state parameters.
     */
    RotationState() : initial_rotation_(true), rotation_timer_(nullptr) {}

    /**
     * @brief Handles rotation behavior and state transitions.
     * @param walker Pointer to the Walker object.
     * @param scan Shared pointer to laser scan data.
     */
    void handle(Walker *walker,
               const sensor_msgs::msg::LaserScan::SharedPtr scan) override;

private:
    bool initial_rotation_; ///< Flag tracking initial rotation period
    rclcpp::TimerBase::SharedPtr
        rotation_timer_; ///< Timer for managing rotation duration
};

#endif