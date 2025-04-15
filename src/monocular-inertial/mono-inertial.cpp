#include <iostream>
#include <string> // Added for std::string
#include <vector> // Added for std::vector if needed, good practice
#include <memory> // Added for std::make_shared

#include "rclcpp/rclcpp.hpp"
// MODIFIED: Include the correct node header
#include "monocular-inertial-slam-node.hpp"

// Include ORB_SLAM3 System header
#include "System.h"

int main(int argc, char **argv)
{
    // MODIFIED: Update argument check and usage message
    // Arguments: 1=program_name, 2=vocab, 3=settings, 4=camera_topic, [5=do_equalize]
    if(argc < 4)
    {
        std::cerr << "\nUsage: ros2 run <your_package> <node_name> path_to_vocabulary path_to_settings camera_topic [do_equalize]" << std::endl;
        std::cerr << "\nExample: ros2 run orbslam_ros2 mono_inertial_node vocab.fbow settings.yaml /camera/image_raw true" << std::endl;
        // rclcpp::shutdown(); // Cannot call shutdown before init
        return 1;
    }

    // Store arguments in variables for clarity and handling default
    std::string vocabulary_file = argv[5];
    std::string settings_file = argv[6];
    // Initialize ROS 2
    rclcpp::init(argc, argv);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // Use shared_ptr or unique_ptr for better memory management if preferred, but raw pointer is often used here.
    // MODIFIED: Change sensor type to IMU_MONOCULAR
    bool visualization = true; // Set visualization on/off (can be loaded from settings)
    ORB_SLAM3::System slam_system(vocabulary_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR, visualization);

    // MODIFIED: Create the MonocularInertialSlamNode
    // Pass arguments: SLAM system pointer, settings file path, do_equalize string, camera topic string
    auto node = std::make_shared<MonocularInertialSlamNode>(&slam_system);

    // Print info
    std::cout << "============================\n"
              << " ORB-SLAM3 Monocular Inertial Node Started.\n"
              << " Vocabulary:    " << vocabulary_file << "\n"
              << " Settings:      " << settings_file << "\n"
              << " Visualization: " << (visualization ? "On" : "Off") << "\n"
              << "============================" << std::endl;
    // Spin the node to process callbacks
    rclcpp::spin(node);
    rclcpp::shutdown();

    std::cout << "ORB-SLAM3 Monocular Inertial Node finished." << std::endl;
    return 0;
}