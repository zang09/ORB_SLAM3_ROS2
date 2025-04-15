// MODIFIED: Updated header guard
#ifndef __MONOCULAR_INERTIAL_SLAM_NODE_HPP__
#define __MONOCULAR_INERTIAL_SLAM_NODE_HPP__

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <cv_bridge/cv_bridge.h> // Needed for GetImage if conversion is done there
#include <opencv2/core/core.hpp> // For cv::Mat and cv::Ptr
#include <opencv2/imgproc.hpp>  // For cv::CLAHE

// C++ Standard Library Includes
#include <string>
#include <vector> // For std::vector (if needed directly)
#include <queue>
#include <mutex>
#include <thread>

// Include ORB_SLAM3 System header (adapt path if necessary)
#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"
// Include utility file (adapt path if necessary)
#include "utility.hpp"

// Aliases for ROS message types
using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;

// MODIFIED: Class name
class MonocularInertialSlamNode : public rclcpp::Node
{
public:
    // MODIFIED: Constructor signature
    MonocularInertialSlamNode(ORB_SLAM3::System *SLAM);
    // MODIFIED: Destructor name
    ~MonocularInertialSlamNode();

private:
    // --- Private methods ---
    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabImage(const ImageMsg::SharedPtr msg);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg); // Declaration unchanged
    void SyncWithImu(); // Declaration unchanged

    // --- ROS 2 Subscribers ---
    rclcpp::Subscription<ImuMsg>::SharedPtr   subImu_;
    rclcpp::Subscription<ImageMsg>::SharedPtr subImg_;

    // --- ORB-SLAM3 and Thread ---
    ORB_SLAM3::System *SLAM_; // Pointer to the SLAM instance
    std::thread *syncThread_; // Pointer to the synchronization thread

    // IMU
    std::queue<ImuMsg::SharedPtr> imuBuf_;
    std::mutex bufMutexImu_;

    // Image (Monocular)
    std::queue<ImageMsg::SharedPtr> imgBuf_;
    std::mutex bufMutexImg_;

};

#endif // __MONOCULAR_INERTIAL_SLAM_NODE_HPP__