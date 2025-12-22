#include "stereo-inertial-node.hpp"

#include <opencv2/core/core.hpp>
#include <iomanip>

using std::placeholders::_1;

StereoInertialNode::StereoInertialNode(ORB_SLAM3::System *SLAM, const string &strSettingsFile, const string &strDoRectify, const string &strDoEqual) :
    Node("ORB_SLAM3_ROS2"),
    SLAM_(SLAM),
    lastImageTime_(0.0),
    firstImuTime_(0.0),
    bImuPreloaded_(false),
    prevImuMsg_(nullptr),
    prevImuTime_(0.0)
{
    stringstream ss_rec(strDoRectify);
    ss_rec >> boolalpha >> doRectify_;

    stringstream ss_eq(strDoEqual);
    ss_eq >> boolalpha >> doEqual_;

    bClahe_ = doEqual_;
    std::cout << "Rectify: " << doRectify_ << std::endl;
    std::cout << "Equal: " << doEqual_ << std::endl;

    if (doRectify_)
    {
        // Load settings related to stereo calibration
        cv::FileStorage fsSettings(strSettingsFile, cv::FileStorage::READ);
        if (!fsSettings.isOpened())
        {
            cerr << "ERROR: Wrong path to settings" << endl;
            assert(0);
        }

        cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
        fsSettings["LEFT.K"] >> K_l;
        fsSettings["RIGHT.K"] >> K_r;

        fsSettings["LEFT.P"] >> P_l;
        fsSettings["RIGHT.P"] >> P_r;

        fsSettings["LEFT.R"] >> R_l;
        fsSettings["RIGHT.R"] >> R_r;

        fsSettings["LEFT.D"] >> D_l;
        fsSettings["RIGHT.D"] >> D_r;

        int rows_l = fsSettings["LEFT.height"];
        int cols_l = fsSettings["LEFT.width"];
        int rows_r = fsSettings["RIGHT.height"];
        int cols_r = fsSettings["RIGHT.width"];

        if (K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l == 0 || rows_r == 0 || cols_l == 0 || cols_r == 0)
        {
            cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
            assert(0);
        }

        cv::initUndistortRectifyMap(K_l, D_l, R_l, P_l.rowRange(0, 3).colRange(0, 3), cv::Size(cols_l, rows_l), CV_32F, M1l_, M2l_);
        cv::initUndistortRectifyMap(K_r, D_r, R_r, P_r.rowRange(0, 3).colRange(0, 3), cv::Size(cols_r, rows_r), CV_32F, M1r_, M2r_);
    }

    // Use sensor data QoS profile with best effort reliability
    /// without qos setting, imu is not subscribed by StereoInertialNode
    rclcpp::QoS imu_qos(10);
    imu_qos.best_effort();
    imu_qos.durability_volatile();
    subImu_ = this->create_subscription<ImuMsg>("/place_detector_cam/zed_node/imu/data", imu_qos, std::bind(&StereoInertialNode::GrabImu, this, _1));
    
    // Setup stereo image synchronization using message_filters
    left_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/place_detector_cam/zed_node/left/image_rect_color", rmw_qos_profile_sensor_data);
    right_sub_ = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "/place_detector_cam/zed_node/right/image_rect_color", rmw_qos_profile_sensor_data);

    syncApproximate_ = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *left_sub_, *right_sub_);
    syncApproximate_->registerCallback(&StereoInertialNode::GrabStereo, this);

    // Create pose and odometry publishers
    posePub_ = this->create_publisher<PoseMsg>("/orb3/pose", 10);
    // odomPub_ = this->create_publisher<OdomMsg>("~/orb3/odom", 10);
}

StereoInertialNode::~StereoInertialNode()
{
    // Stop all threads
    SLAM_->Shutdown();

    // Save camera trajectory
    SLAM_->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
}

void StereoInertialNode::GrabImu(const ImuMsg::SharedPtr msg)
{   
    double t = Utility::StampToSec(msg->header.stamp);
    
    // Filter out invalid timestamps (zero or negative)
    if (t <= 0.0)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                            "Skipping IMU measurement with invalid timestamp: %.6f", t);
        return;
    }
    
    bufMutex_.lock();
    
    // Track first valid IMU timestamp for preloading
    if (firstImuTime_ == 0.0)
    {
        firstImuTime_ = t;
        RCLCPP_INFO(this->get_logger(), "First IMU measurement received at %.6f - starting preload", t);
    }
    
    // Interpolate IMU measurements if time gap is significant
    if (prevImuMsg_ != nullptr && prevImuTime_ > 0.0)
    {
        double dt = t - prevImuTime_;
        
        // If gap > 15ms (normal is ~7ms for 140Hz), add interpolated measurements
        if (dt > 0.015)
        {
            // Calculate number of interpolations needed to maintain ~7ms spacing
            int numInterpolated = std::min(static_cast<int>(dt / 0.007) - 1, 20);
            
            for (int i = 1; i <= numInterpolated; ++i)
            {
                double alpha = static_cast<double>(i) / (numInterpolated + 1);
                double interpTime = prevImuTime_ + alpha * dt;
                
                // Create interpolated IMU message
                auto interpMsg = std::make_shared<ImuMsg>();
                interpMsg->header = msg->header;
                
                // Linear interpolation for accelerometer
                interpMsg->linear_acceleration.x = prevImuMsg_->linear_acceleration.x + 
                    alpha * (msg->linear_acceleration.x - prevImuMsg_->linear_acceleration.x);
                interpMsg->linear_acceleration.y = prevImuMsg_->linear_acceleration.y + 
                    alpha * (msg->linear_acceleration.y - prevImuMsg_->linear_acceleration.y);
                interpMsg->linear_acceleration.z = prevImuMsg_->linear_acceleration.z + 
                    alpha * (msg->linear_acceleration.z - prevImuMsg_->linear_acceleration.z);
                
                // Linear interpolation for gyroscope
                interpMsg->angular_velocity.x = prevImuMsg_->angular_velocity.x + 
                    alpha * (msg->angular_velocity.x - prevImuMsg_->angular_velocity.x);
                interpMsg->angular_velocity.y = prevImuMsg_->angular_velocity.y + 
                    alpha * (msg->angular_velocity.y - prevImuMsg_->angular_velocity.y);
                interpMsg->angular_velocity.z = prevImuMsg_->angular_velocity.z + 
                    alpha * (msg->angular_velocity.z - prevImuMsg_->angular_velocity.z);
                
                // Add to buffer with interpolated timestamp
                imuBuf_.push({interpMsg, interpTime});
            }
        }
    }
    
    // Store current IMU measurement
    imuBuf_.push({msg, t});
    prevImuMsg_ = msg;
    prevImuTime_ = t;
    
    // Check if we have preloaded enough IMU data (at least 1.0 second)
    if (!bImuPreloaded_ && !imuBuf_.empty())
    {
        double imuDuration = t - firstImuTime_;
        if (imuDuration >= 1.0)
        {
            bImuPreloaded_ = true;
            RCLCPP_INFO(this->get_logger(), 
                       "IMU preloading complete: %.2f seconds buffered (%zu measurements, avg rate: %.1f Hz)",
                       imuDuration, imuBuf_.size(), imuBuf_.size() / imuDuration);
        }
    }
    
    bufMutex_.unlock();
}

void StereoInertialNode::GrabStereo(const ImageMsg::SharedPtr msgLeft, const ImageMsg::SharedPtr msgRight)
{
    // Get image timestamp
    double tImCurrent = Utility::StampToSec(msgLeft->header.stamp);
    
    // Convert images to cv::Mat
    cv::Mat imLeft = GetImage(msgLeft);
    cv::Mat imRight = GetImage(msgRight);
    
    bufMutex_.lock();
    
    // Wait for IMU preloading to complete (at least 1 second of IMU data)
    // This ensures ORB-SLAM3 has sufficient historical IMU for initialization
    if (!bImuPreloaded_)
    {
        bufMutex_.unlock();
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Waiting for IMU preload to complete (need 1.0s of data)...");
        return;
    }
    
    // Collect IMU measurements between last image and current image
    // This matches the RealSense example pattern
    vector<ORB_SLAM3::IMU::Point> vImuMeas;
    
    if (!imuBuf_.empty())
    {
        // Wait until we have IMU data covering up to the current image timestamp
        double tImuLatest = imuBuf_.back().timestamp;
        if (tImCurrent > tImuLatest)
        {
            bufMutex_.unlock();
            RCLCPP_DEBUG(this->get_logger(), 
                        "Waiting for IMU data: image %.6f > latest IMU %.6f", 
                        tImCurrent, tImuLatest);
            return;
        }
        
        // For the first frame, we need to initialize lastImageTime_
        // but KEEP the IMU buffer - don't clear it yet
        if (lastImageTime_ == 0.0)
        {
            // Find the oldest IMU timestamp we want to keep
            // Keep IMU data starting from 0.5 seconds before first image
            double tMinKeep = std::max(firstImuTime_, tImCurrent - 0.5);
            
            // First frame: collect ALL available IMU data from buffer start to current image
            vImuMeas.clear();
            queue<ImuData> tempBuf;
            
            while (!imuBuf_.empty())
            {
                double tImu = imuBuf_.front().timestamp;
                auto imuMsg = imuBuf_.front().msg;
                
                // Keep IMU measurements up to current image
                if (tImu <= tImCurrent && tImu >= tMinKeep)
                {
                    cv::Point3f acc(imuMsg->linear_acceleration.x, 
                                  imuMsg->linear_acceleration.y, 
                                  imuMsg->linear_acceleration.z);
                    cv::Point3f gyr(imuMsg->angular_velocity.x, 
                                  imuMsg->angular_velocity.y, 
                                  imuMsg->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tImu));
                }
                
                // Keep future IMU data in buffer
                if (tImu > tImCurrent)
                {
                    tempBuf.push(imuBuf_.front());
                }
                
                imuBuf_.pop();
            }
            
            // Restore future IMU data
            imuBuf_ = tempBuf;
            
            lastImageTime_ = tImCurrent;
            
            bufMutex_.unlock();
            
            RCLCPP_INFO(this->get_logger(), 
                       "First stereo frame at %.6f: collected %zu IMU measurements for initialization",
                       tImCurrent, vImuMeas.size());
            
            // Ensure we have minimum IMU measurements for first frame (at least 10)
            if (vImuMeas.size() < 10)
            {
                RCLCPP_WARN(this->get_logger(), 
                           "Insufficient IMU measurements for first frame (%zu < 10) - skipping",
                           vImuMeas.size());
                return;
            }
        }
        else
        {
            // Subsequent frames: extract IMU measurements in the time range (lastImageTime_, tImCurrent]
            vImuMeas.clear();
            while (!imuBuf_.empty())
            {
                double tImu = imuBuf_.front().timestamp;
                auto imuMsg = imuBuf_.front().msg;
                
                // Stop when we reach IMU data beyond current image timestamp
                if (tImu > tImCurrent)
                    break;
                    
                // Collect IMU measurements after last processed image
                if (tImu > lastImageTime_)
                {
                    cv::Point3f acc(imuMsg->linear_acceleration.x, 
                                  imuMsg->linear_acceleration.y, 
                                  imuMsg->linear_acceleration.z);
                    cv::Point3f gyr(imuMsg->angular_velocity.x, 
                                  imuMsg->angular_velocity.y, 
                                  imuMsg->angular_velocity.z);
                    vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, tImu));
                }
                
                // Remove processed IMU data from buffer
                imuBuf_.pop();
            }
            
            RCLCPP_INFO(this->get_logger(), 
                       "Stereo frame at %.6f: collected %zu IMU measurements (range: %.6f to %.6f)", 
                       tImCurrent, vImuMeas.size(), lastImageTime_, tImCurrent);
            
            // Update last processed image timestamp for next iteration
            lastImageTime_ = tImCurrent;
        }
    }
    else
    {
        bufMutex_.unlock();
        RCLCPP_WARN(this->get_logger(), "No IMU data available - waiting for IMU stream");
        return;
    }
    
    bufMutex_.unlock();
    
    // Apply CLAHE if enabled
    if (bClahe_)
    {
        clahe_->apply(imLeft, imLeft);
        clahe_->apply(imRight, imRight);
    }
    
    // Apply rectification if needed
    if (doRectify_)
    {
        cv::remap(imLeft, imLeft, M1l_, M2l_, cv::INTER_LINEAR);
        cv::remap(imRight, imRight, M1r_, M2r_, cv::INTER_LINEAR);
    }
    
    // Track with SLAM
    Sophus::SE3f Tcw = SLAM_->TrackStereo(imLeft, imRight, tImCurrent, vImuMeas);
    
    // Publish camera pose if tracking is successful
    const auto trackingState = SLAM_->GetTrackingState();
    RCLCPP_INFO(this->get_logger(), "Tracking state: %d", trackingState);
    if (trackingState == ORB_SLAM3::Tracking::OK)
    {
        RCLCPP_INFO(this->get_logger(), "Valid state - publishing pose");
        // Publish pose if tracking was successful
        if (!Tcw.translation().isZero() || !Tcw.so3().matrix().isIdentity())
        {
            RCLCPP_INFO(this->get_logger(), "Publishing pose");
            // Tcw is camera pose (transform from world to camera)
            // We need Twc (transform from camera to world) for publishing
            Sophus::SE3f Twc = Tcw.inverse();
            
            geometry_msgs::msg::PoseStamped pose_msg;
            pose_msg.header.stamp = msgLeft->header.stamp;
            pose_msg.header.frame_id = "map";
            
            // Extract translation
            pose_msg.pose.position.x = Twc.translation().x();
            pose_msg.pose.position.y = Twc.translation().y();
            pose_msg.pose.position.z = Twc.translation().z();
            
            // Extract rotation as quaternion
            Eigen::Quaternionf q(Twc.so3().unit_quaternion());
            pose_msg.pose.orientation.x = q.x();
            pose_msg.pose.orientation.y = q.y();
            pose_msg.pose.orientation.z = q.z();
            pose_msg.pose.orientation.w = q.w();
            
            posePub_->publish(pose_msg);
        }
    }
}

cv::Mat StereoInertialNode::GetImage(const ImageMsg::SharedPtr msg)
{
    // Copy the ros image message to cv::Mat.
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

