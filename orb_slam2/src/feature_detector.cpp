/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2014, Kei Okada.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Kei Okada nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/features2d/features2d.hpp"

#include "visual_features_extractor/FeatureDetectorConfig.h"
#include "visual_features_extractor/Frame.h"
#include "visual_features_extractor/KeyPoint.h"

ros::Publisher img_pub_;
ros::Subscriber img_sub_;
ros::Publisher msg_pub_;
//cv::Ptr<cv::FeatureDetector> ORB_detector_;
cv::ORB * ORB_detector_;

bool publish_image_;

void proc_img(const sensor_msgs::ImageConstPtr& msg)
{
    // Convert the image into something opencv can handle.
    cv::Mat frame = cv_bridge::toCvShare(msg, msg->encoding)->image;

    // Convert to gray
    cv::Mat src_gray;
    if ( frame.channels() > 1 ) {
        cv::cvtColor( frame, src_gray, cv::COLOR_RGB2GRAY );
    } else {
        src_gray = frame;
        cv::cvtColor( src_gray, frame, cv::COLOR_GRAY2BGR );
    }

    // Output structures
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    // Apply detector
    (*ORB_detector_)(src_gray,cv::noArray(), keypoints, descriptors);


    if( publish_image_){
        // Draw corners detected
        int r = 4;
        for( size_t i = 0; i < keypoints.size(); i++ )
        {
            cv::circle( frame, keypoints[i].pt, r, cv::Scalar(255, 0, 0));
        }
        // Publish the image.
        sensor_msgs::Image::Ptr out_img = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        img_pub_.publish(out_img);
    }

    // Create msgs
    visual_features_extractor::Frame f;
    for( size_t i = 0; i< keypoints.size(); i++ ) {
        visual_features_extractor::KeyPoint kp;
        kp.x = keypoints[i].pt.x;
        kp.y = keypoints[i].pt.y;
        kp.size = keypoints[i].size;
        kp.angle = keypoints[i].angle;
        kp.octave = keypoints[i].octave;

        // Pointer to the i-th row
        std::copy(descriptors.ptr<uchar>(i), descriptors.ptr<uchar>(i) + 32, kp.descriptor.begin());

        f.keypoints.push_back(kp);
    }
    f.header.stamp = msg->header.stamp;
    f.height = msg->height;
    f.width = msg->width;

    // Publish keypoints
    msg_pub_.publish(f);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_detector");
    if (ros::names::remap("image") == "image") {
        ROS_WARN("Topic 'image' has not been remapped! Typical command-line usage:\n"
                 "\t$ rosrun image_rotate image_rotate image:=<image topic> [transport]");
    }

    ros::NodeHandle nh;

    nh.param("publish_image_",publish_image_, false);


    int num_features_ = 4000;
//    float scale_factor_ = 1.2f;
//    int nlevels_ = 8;
//    int edge_threshold_ = 31;

    //	ORB_detector_ = new cv::ORB(num_features_,scale_factor_,nlevels_,edge_threshold_);
    ORB_detector_ = new cv::ORB(num_features_);

    if ( publish_image_ ){
        img_pub_ = nh.advertise<sensor_msgs::Image>("image", 1);
    }

    msg_pub_ = nh.advertise<visual_features_extractor::Frame>("features", 1);

    img_sub_ = nh.subscribe("image", 3, &proc_img);

    ros::spin();
    return 0;
}
