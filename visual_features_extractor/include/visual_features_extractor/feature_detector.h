#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>  

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <nodelet/nodelet.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "visual_features_extractor/FeatureDetectorConfig.h"
#include "visual_slam_msgs/Frame.h"
#include "visual_slam_msgs/KeyPoint.h"
#include "visual_slam_msgs/TrackingState.h"

namespace visual_features_extractor
{
    

    class FeatureExtractorNodelet : public nodelet::Nodelet
    {
        public:
            virtual void onInit();            
        private:
            void undistort_keypoints(std::vector<cv::KeyPoint> &keypoints,
                const sensor_msgs::CameraInfoConstPtr& cam_info,
                std::vector<cv::KeyPoint> &undistorted_keypoints);
            
            void proc_img(const sensor_msgs::ImageConstPtr& img,
                    const sensor_msgs::CameraInfoConstPtr& cam_info);

            void img_and_info_callback(const sensor_msgs::ImageConstPtr& img, 
                const sensor_msgs::CameraInfoConstPtr cam_info);

            void img_callback(const sensor_msgs::ImageConstPtr& img);


            void tracker_state_callback(const visual_slam_msgs::TrackingState &msg);
        
            // Thread pool
            boost::asio::io_service ioService;
            boost::thread_group threadpool;
            boost::asio::io_service::work * work;
            int running_threads = 0;
            boost::mutex mtx_exc_;
            boost::mutex lock_mtx_;
            boost::condition_variable threads_available_cond;

            // Image and features publishers
            ros::Publisher img_pub_;
            ros::Publisher msg_pub_;
            // Tracking state subscriber
            ros::Subscriber state_sub_;
            int current_state;
            int num_features_, num_features_param_;
            // Parameters
            bool undistort_points_;
            bool publish_image_;
    
            message_filters::Subscriber<sensor_msgs::Image> * image_filter_sub;
            message_filters::Subscriber<sensor_msgs::CameraInfo> * camera_info_filter_sub;
            message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> * msg_sync;
    };

}









