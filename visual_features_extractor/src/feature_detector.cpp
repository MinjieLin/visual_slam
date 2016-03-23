#include "visual_features_extractor/feature_detector.h"
#include "visual_features_extractor/Frame.h"
#include "visual_features_extractor/KeyPoint.h"
#include "visual_slam_msgs/TrackingState.h"

#include <pluginlib/class_list_macros.h>

// watch the capitalization carefully
PLUGINLIB_EXPORT_CLASS(visual_features_extractor::FeatureExtractorNodelet, nodelet::Nodelet) 

namespace visual_features_extractor
{

void FeatureExtractorNodelet::onInit(){
    work = new boost::asio::io_service::work(ioService);

    if (ros::names::remap("image") == "image"
            || ros::names::remap("camera_info") == "camera_info") {
        ROS_WARN(
                "Topics 'image' and 'camera_info' have not been remapped!");
    }

    ros::NodeHandle& nh = getPrivateNodeHandle();

    nh.param("publish_image_", publish_image_, false);
    if (publish_image_) {
        img_pub_ = nh.advertise<sensor_msgs::Image>("feature_image", 1);
    }

    // TODO: check with default of 2000
    // TODO: make this a service
    nh.param("num_features", num_features_, 4000);
    ROS_INFO("Detecting %d features", num_features_);
    msg_pub_ = nh.advertise<visual_features_extractor::Frame>("features", 10);

    bool subscribe_state_;
    nh.param("subscribe_to_state", subscribe_state_, false);
    state_sub_ = nh.subscribe("/slam/tracking_state", 1,
        &FeatureExtractorNodelet::tracker_state_callback, this);

    //	ORB_detector_ = new cv::ORB(num_features_,scale_factor_,nlevels_,edge_threshold_);
    //ORB_detector_ = new cv::ORB(num_features_);
    nh.param("undistort_points", undistort_points_, false);
    if(undistort_points_)
        ROS_INFO("Undistorting points");
    current_state = visual_slam_msgs::TrackingState::SYSTEM_NOT_READY;

    int max_threads;
    nh.param("max_threads", max_threads, 4);
    for (int i = 0; i < 4; i ++){
        threadpool.create_thread(
            boost::bind(&boost::asio::io_service::run, &ioService)
        );
    }


    ros::Subscriber sub;
    if (undistort_points_){
        image_filter_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh,
                "image", 10);
        camera_info_filter_sub = new message_filters::Subscriber<
                sensor_msgs::CameraInfo>(nh, "/usb_cam/camera_info", 10);
        msg_sync = new message_filters::TimeSynchronizer<sensor_msgs::Image,
                sensor_msgs::CameraInfo>(*image_filter_sub, *camera_info_filter_sub,
                10);
        msg_sync->registerCallback(boost::bind(
            &FeatureExtractorNodelet::img_and_info_callback, this, _1, _2));
    } else {
        sub = nh.subscribe("/usb_cam/image", 10, 
            &FeatureExtractorNodelet::img_callback, this);
    }

    //ros::spin();
    //ros::AsyncSpinner spinner(4); // Use 4 threads
    //spinner.start();
    //ros::waitForShutdown();;

//    return 0;
}

void FeatureExtractorNodelet::undistort_keypoints(
    std::vector<cv::KeyPoint> &keypoints,
    const sensor_msgs::CameraInfoConstPtr& cam_info,
    std::vector<cv::KeyPoint> &undistorted_keypoints) 
{

    // Fill matrix with points
    cv::Mat mat(keypoints.size(), 2, CV_32F);
    for (int i = 0; i < keypoints.size(); i++) {
        mat.at<float>(i, 0) = keypoints[i].pt.x;
        mat.at<float>(i, 1) = keypoints[i].pt.y;
    }

    cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
    K.at<float>(0, 0) = cam_info->K.at(0);
    K.at<float>(1, 1) = cam_info->K.at(4);
    K.at<float>(0, 2) = cam_info->K.at(2);
    K.at<float>(1, 2) = cam_info->K.at(5);
    K.at<float>(2, 2) = 1;

    cv::Mat DistCoef(4, 1, CV_32F);
    // TODO: check that this is ok
    DistCoef.at<float>(0) = cam_info->D.at(0);
    DistCoef.at<float>(1) = cam_info->D.at(1);
    DistCoef.at<float>(2) = cam_info->D.at(2);
    DistCoef.at<float>(3) = cam_info->D.at(3);


    // Undistort points
    mat = mat.reshape(2);
    cv::undistortPoints(mat, mat, K, DistCoef, cv::Mat(), K);
    mat = mat.reshape(1);

    // Fill undistorted keypoint vector
    undistorted_keypoints.resize(keypoints.size());
    for (int i = 0; i < keypoints.size(); i++) {
        cv::KeyPoint kp = keypoints[i];
        kp.pt.x = mat.at<float>(i, 0);
        kp.pt.y = mat.at<float>(i, 1);
        undistorted_keypoints[i] = kp;
    }
}


void FeatureExtractorNodelet::proc_img(
    const sensor_msgs::ImageConstPtr& img,
    const sensor_msgs::CameraInfoConstPtr& cam_info) 
{
    //ROS_INFO("Started callback");

    cv::ORB ORB_detector_(num_features_);
    // Convert the image into something opencv can handle.
    cv::Mat frame = cv_bridge::toCvShare(img, img->encoding)->image;

    // Convert to gray
    cv::Mat src_gray;
    if (frame.channels() > 1) {
        cv::cvtColor(frame, src_gray, cv::COLOR_RGB2GRAY);
    } else {
        src_gray = frame;
        //cv::cvtColor(src_gray, frame, cv::COLOR_GRAY2BGR);
    }

    // Output structures
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Apply detector
    (ORB_detector_)(src_gray, cv::noArray(), keypoints, descriptors);

    std::vector<cv::KeyPoint> undistorted_keypoints;
    if (undistort_points_){
        undistort_keypoints(keypoints, cam_info, undistorted_keypoints);
    } else {
        undistorted_keypoints = keypoints;
    }

    if (publish_image_) {
        // Draw corners detected
        int r = 4;
        for (size_t i = 0; i < keypoints.size(); i++) {
            cv::circle(frame, keypoints[i].pt, r, cv::Scalar(255, 0, 0));
        }
        // Publish the image.
        sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(img->header,
                "bgr8", frame).toImageMsg();
        img_pub_.publish(out_img);
    }

    // Create msgs
    visual_features_extractor::Frame f;
    for (size_t i = 0; i < undistorted_keypoints.size(); i++) {
        visual_features_extractor::KeyPoint kp;
        kp.x = undistorted_keypoints[i].pt.x;
        kp.y = undistorted_keypoints[i].pt.y;
        kp.size = undistorted_keypoints[i].size;
        kp.angle = undistorted_keypoints[i].angle;
        kp.octave = undistorted_keypoints[i].octave;

        // Pointer to the i-th row
        std::copy(descriptors.ptr<uchar>(i), descriptors.ptr<uchar>(i) + 32,
                kp.descriptor.begin());

        f.keypoints.push_back(kp);
    }
    f.header.stamp = img->header.stamp;
    f.height = img->height;
    f.width = img->width;

    boost::unique_lock<boost::mutex> lock(lock_mtx_);
    msg_pub_.publish(f);
    running_threads--;
    threads_available_cond.notify_all();

    //ROS_INFO("Finished callback");
}

void FeatureExtractorNodelet::img_and_info_callback(
    const sensor_msgs::ImageConstPtr& img,  
    const sensor_msgs::CameraInfoConstPtr cam_info)
{
    boost::unique_lock<boost::mutex> lock(lock_mtx_);
    while (running_threads >= 4)
        threads_available_cond.wait(lock);

    running_threads++;
    ROS_INFO("Running threads: %d", running_threads);

    //new std::thread(proc_img, img, cam_info);
    ioService.post(boost::bind(&FeatureExtractorNodelet::proc_img,
        this, img, cam_info));
}

void FeatureExtractorNodelet::img_callback(
    const sensor_msgs::ImageConstPtr& img) 
{
    const sensor_msgs::CameraInfoConstPtr cam_info;

    boost::unique_lock<boost::mutex> lock(lock_mtx_);
    while (running_threads >= 4)
        threads_available_cond.wait(lock);

    running_threads++;
    ROS_INFO("Running threads: %d", running_threads);

    //new std::thread(proc_img, img, cam_info);
    ioService.post(boost::bind(&FeatureExtractorNodelet::proc_img,
        this, img, cam_info));
}


void FeatureExtractorNodelet::tracker_state_callback(
    const visual_slam_msgs::TrackingState &msg)
{
    if (current_state != msg.state){
        if (msg.state == visual_slam_msgs::TrackingState::NOT_INITIALIZED){
            num_features_ = num_features_param_;
            ROS_INFO("Using %d features to initialize", num_features_);
        } else {
            num_features_ =  num_features_param_/2;
            ROS_INFO("Using %d features after initialization", num_features_);
        }
        current_state = msg.state;
    }
}

} // namespace






