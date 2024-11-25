/**
 * @file main.cpp
 * @brief Main entry point for the Walker node application.
 *
 * This file contains the main function which initializes the ROS 2 framework,
 * creates an instance of the Walker node, and spins it until the application
 * is shut down.
 *
 * @author Kashif Ansari
 * @date 2023
 */
#include "walker/walker_node.hpp"

/**
 * @brief Main function to start the Walker node.
 *
 * Initializes the ROS 2 framework, creates an instance of the Walker node,
 * and spins it until the application is shut down.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit code of the application.
 */
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Walker>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0; 
}