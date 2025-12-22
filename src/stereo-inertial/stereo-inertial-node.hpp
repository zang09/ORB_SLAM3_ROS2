#ifndef __STEREO_INERTIAL_NODE_HPP__
#define __STEREO_INERTIAL_NODE_HPP__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
// #include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

#include <cv_bridge/cv_bridge.h>

#include "System.h"
#include "Frame.h"
#include "Map.h"
#include "Tracking.h"

#include "utility.hpp"

using ImuMsg = sensor_msgs::msg::Imu;
using ImageMsg = sensor_msgs::msg::Image;
using PoseMsg = geometry_msgs::msg::PoseStamped;
// using OdomMsg = nav_msgs::msg::Odometry;

class StereoInertialNode : public rclcpp::Node
{
public:
    StereoInertialNode(ORB_SLAM3::System* pSLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual);
    ~StereoInertialNode();

private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> approximate_sync_policy;

    void GrabImu(const ImuMsg::SharedPtr msg);
    void GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight);
    cv::Mat GetImage(const ImageMsg::SharedPtr msg);
    
    // IMU interpolation helper (matches RealSense pattern)
    ORB_SLAM3::IMU::Point InterpolateIMU(double targetTime, 
                                         const ImuMsg::SharedPtr current, 
                                         const ImuMsg::SharedPtr previous);

    // Stereo image synchronization using message_filters
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> left_sub_;
    std::shared_ptr<message_filters::Subscriber<ImageMsg>> right_sub_;
    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate_;

    // IMU subscription
    rclcpp::Subscription<ImuMsg>::SharedPtr subImu_;
    
    // Pose publisher
    rclcpp::Publisher<PoseMsg>::SharedPtr posePub_;
    // rclcpp::Publisher<OdomMsg>::SharedPtr odomPub_;

    ORB_SLAM3::System *SLAM_;

    // IMU buffer with timestamps
    struct ImuData {
        ImuMsg::SharedPtr msg;
        double timestamp;
    };
    queue<ImuData> imuBuf_;
    std::mutex bufMutex_;
    double lastImageTime_;  // Track timestamp of last processed image
    double firstImuTime_;   // Timestamp of first IMU measurement received
    bool bImuPreloaded_;    // Flag indicating sufficient IMU data has been collected
    
    // For IMU interpolation
    ImuMsg::SharedPtr prevImuMsg_;  // Previous IMU message for interpolation
    double prevImuTime_;            // Timestamp of previous IMU

    bool doRectify_;
    bool doEqual_;
    cv::Mat M1l_, M2l_, M1r_, M2r_;

    bool bClahe_;
    cv::Ptr<cv::CLAHE> clahe_ = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif
