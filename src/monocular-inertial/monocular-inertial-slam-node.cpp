// 1. Includes
#include "monocular-inertial-slam-node.hpp" // This brings in the class definition
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include "utility.hpp"
#include <vector>
#include <sstream>
#include <iostream>
#include <chrono>
#include <mutex> // For std::lock_guard, std::adopt_lock etc.

// DO NOT PUT "class MonocularInertialSlamNode { ... };" HERE AGAIN!

using std::placeholders::_1;
// 2. Method Implementations (Definitions)
MonocularInertialSlamNode::MonocularInertialSlamNode(ORB_SLAM3::System *SLAM) :
    Node("ORB_SLAM3_ROS2_Mono_Inertial"), // Initializer list starts here
    SLAM_(SLAM)
    // Initialize other members if needed (though clahe_ is often done in the body)
{

    // Subscriber initialization
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
    subImu_ = this->create_subscription<ImuMsg>("/livox/imu", 100, std::bind(&MonocularInertialSlamNode::GrabImu, this, _1));
    subImg_ = this->create_subscription<ImageMsg>("/camera", 100, std::bind(&MonocularInertialSlamNode::GrabImage, this, _1));;

    RCLCPP_INFO(this->get_logger(), "Subscribing to image topic: /livox/imu");
    RCLCPP_INFO(this->get_logger(), "Subscribing to IMU topic: imu");

    syncThread_ = new std::thread(&MonocularInertialSlamNode::SyncWithImu, this);
}

MonocularInertialSlamNode::~MonocularInertialSlamNode()
{
    // Destructor implementation...
     if(syncThread_) {
        // Consider adding stop flag logic here
        syncThread_->join();
        delete syncThread_;
    }
    // ... rest of destructor code ...
     if (SLAM_) {
        RCLCPP_INFO(this->get_logger(), "Shutting down SLAM system...");
        SLAM_->Shutdown();
        RCLCPP_INFO(this->get_logger(), "SLAM system shutdown complete.");
        // ... save trajectory ...
        SLAM_->SaveKeyFrameTrajectoryTUM("yes.txt");
    }
}

void MonocularInertialSlamNode::GrabImu(const ImuMsg::SharedPtr msg)
{
    // GrabImu implementation...

    bufMutexImu_.lock();

    if (!imgBuf_.empty())
        imuBuf_.pop();
    imuBuf_.push(msg);

    bufMutexImu_.unlock();
}

void MonocularInertialSlamNode::GrabImage(const ImageMsg::SharedPtr msg)
{
    // GrabImage implementation...
    bufMutexImg_.lock();

    if (!imgBuf_.empty())
       imgBuf_.pop();
    imgBuf_.push(msg);
 
    bufMutexImg_.unlock();
 
}

cv::Mat MonocularInertialSlamNode::GetImage(const ImageMsg::SharedPtr msg)
{

    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0)
    {
        return cv_ptr->image.clone();
    }
    else
    {
        std::cerr << "Error image type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void MonocularInertialSlamNode::SyncWithImu()
{
    const double maxTimeDiff = 0.01;

    while (1)
    {
        cv::Mat img;
        double tImg;
        if (!imgBuf_.empty() && !imuBuf_.empty())
        {
            tImg = Utility::StampToSec(imgBuf_.front()->header.stamp);

            bufMutexImg_.lock();
            while ( imgBuf_.size() > 1)
            {
                imgBuf_.pop();
                tImg = Utility::StampToSec(imgBuf_.front()->header.stamp);
            }
            bufMutexImg_.unlock();



            bufMutexImg_.lock();
            img = GetImage(imgBuf_.front());
            imgBuf_.pop();
            bufMutexImg_.unlock();

            if (tImg > Utility::StampToSec(imuBuf_.back()->header.stamp))
            continue;

            vector<ORB_SLAM3::IMU::Point> vImuMeas;
            
            bufMutexImu_.lock();
            if (!imuBuf_.empty())
            {
                // Load imu measurements from buffer
                vImuMeas.clear();
                while (!imuBuf_.empty() && Utility::StampToSec(imuBuf_.front()->header.stamp) <= tImg)
                {
                    double t = Utility::StampToSec(imuBuf_.front()->header.stamp);
                    cv::Point3f acc(imuBuf_.front()->linear_acceleration.x, imuBuf_.front()->linear_acceleration.y, imuBuf_.front()->linear_acceleration.z);
                    cv::Point3f gyr(imuBuf_.front()->angular_velocity.x, imuBuf_.front()->angular_velocity.y, imuBuf_.front()->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
                    imuBuf_.pop();
                }
            }
            bufMutexImu_.unlock();

            SLAM_->TrackMonocular(img, tImg, vImuMeas);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);

    }
}
}

// End of file