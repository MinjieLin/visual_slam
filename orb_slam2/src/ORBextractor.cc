/**
* This file is part of ORB-SLAM2.
* This file is based on the file orb.cpp from the OpenCV library (see BSD license below).
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
/**
* Software License Agreement (BSD License)
*
*  Copyright (c) 2009, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*
*/


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vector>


#include "ORBextractor.h"


using namespace cv;
using namespace std;

namespace ORB_SLAM2
{

const int PATCH_SIZE = 31;
const int HALF_PATCH_SIZE = 15;
const int EDGE_THRESHOLD = 19;


ORBextractor::ORBextractor(int _nfeatures, float _scaleFactor, int _nlevels,
         int _iniThFAST, int _minThFAST) :
    scaleFactor(_scaleFactor), nlevels(_nlevels)
{
    logScaleFactor = log(scaleFactor);
    mvScaleFactor.resize(_nlevels);
    mvLevelSigma2.resize(_nlevels);
    mvScaleFactor[0]=1.0f;
    mvLevelSigma2[0]=1.0f;
    for(int i=1; i<_nlevels; i++)
    {
        mvScaleFactor[i]=mvScaleFactor[i-1]*_scaleFactor;
        mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i];
    }

    mvInvScaleFactor.resize(_nlevels);
    mvInvLevelSigma2.resize(_nlevels);
    for(int i=0; i<_nlevels; i++)
    {
        mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
        mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];
    }

//    mvImagePyramid.resize(_nlevels);

    // Migration
    ORB_detector_ = new cv::ORB(_nfeatures, _scaleFactor, _nlevels, _iniThFAST);
}



void ORBextractor::operator()( InputArray _image, const visual_features_extractor::Frame & f, InputArray _mask, vector<KeyPoint>& _keypoints,
                      OutputArray _descriptors)
{ 
    vector<KeyPoint> keypoints;
    cv::Mat descriptors = cv::Mat(f.keypoints.size(), 32, CV_8UC1);
    int i = 0;
    for (visual_features_extractor::KeyPoint kp : f.keypoints){
        cv::KeyPoint cvKp;
        cvKp.pt.x = kp.x;
        cvKp.pt.y = kp.y;
        cvKp.angle = kp.angle;
        cvKp.octave = kp.octave;
        cvKp.size = kp.size;


        keypoints.push_back(cvKp);

        std::copy(kp.descriptor.begin(), kp.descriptor.end(), descriptors.ptr<uchar>(i));
        i++;
    }

    _descriptors.create(keypoints.size(), 32, CV_8U);
    descriptors.copyTo(_descriptors);
    _keypoints.clear();
    _keypoints.reserve(keypoints.size());
    _keypoints.insert(_keypoints.end(), keypoints.begin(), keypoints.end());
}

} //namespace ORB_SLAM
