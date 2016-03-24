/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <ros/package.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>

#include "include/System.h"

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "visual_slam_msgs/Frame.h"
#include "sensor_msgs/CameraInfo.h"

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

// Globals
ORB_SLAM2::System* mpSLAM;
ros::Subscriber features_sub;
ros::Subscriber cam_info_sub;
message_filters::Subscriber<sensor_msgs::Image > * image_filter_sub;
message_filters::Subscriber<visual_slam_msgs::Frame >  * features_filter_sub;
message_filters::TimeSynchronizer<Image, visual_slam_msgs::Frame > * msg_sync;
bool debug_view;
string vocab_path;

void grabImageAndFeatures(const sensor_msgs::ImageConstPtr& image,
                          const visual_slam_msgs::FrameConstPtr & frame)
{
		//ROS_INFO("Reveived frame with info");
    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mpSLAM->TrackMonocular(cv_ptr->image,cv_ptr->header.stamp.toSec(), *frame);
}

void grabFeatures(const visual_slam_msgs::FrameConstPtr & frame)
{
		//ROS_INFO("Reveived frame with info");
    // If no features, just pass on a blank image. It has heigh rows and width columns
    cv::Mat emptyMat = cv::Mat(frame->height,frame->width, CV_8U);
    // Fill it with black
    emptyMat = cv::Scalar(0,0,0);
    // Send an empty image
    // TODO: use different methods for gui or not
    mpSLAM->TrackMonocular(emptyMat,frame->header.stamp.toSec(), *frame);
}

void grab_cam_info_and_setup(const sensor_msgs::CameraInfoConstPtr & cam_info){
    ROS_INFO("Received the camera parameters. Proceeding to setup SLAM");

    ros::NodeHandle nh("~");

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    mpSLAM = new ORB_SLAM2::System(vocab_path, cam_info,ORB_SLAM2::System::MONOCULAR,true);

    ROS_INFO("SLAM Initialized");



    if (debug_view){
        ROS_INFO("Subscribing to images and features");
        image_filter_sub = new message_filters::Subscriber<sensor_msgs::Image>(nh, "image_raw", 10);
        features_filter_sub = new message_filters::Subscriber<visual_slam_msgs::Frame>(nh, "features", 10);
        msg_sync = new message_filters::TimeSynchronizer<Image, visual_slam_msgs::Frame>(*image_filter_sub, *features_filter_sub, 10);
        msg_sync->registerCallback(boost::bind(&grabImageAndFeatures, _1, _2));
    } else {
        ROS_INFO("Subscribing to features only");
        features_sub = nh.subscribe("features", 10, &grabFeatures);
    }

    cam_info_sub.shutdown();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam");
    ros::start();

    ros::NodeHandle nh("~");

    nh.param("debug_view", debug_view, true);

    if (!nh.getParam("vocabulary_path",vocab_path))
    {
        vocab_path = ros::package::getPath("orb_slam2") + "/Vocabulary/ORBvoc.txt";
        ROS_INFO("Vocabulary Path Not Provided. Using %s", vocab_path.c_str());
    }

    cam_info_sub = nh.subscribe("camera_info", 1, &grab_cam_info_and_setup);

    ROS_INFO("Waiting for camera parameters to initialize SLAM");

    ros::spin();

    // Stop all threads
    mpSLAM->Shutdown();

    // Save camera trajectory
    mpSLAM->SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    ros::shutdown();

    return 0;
}




