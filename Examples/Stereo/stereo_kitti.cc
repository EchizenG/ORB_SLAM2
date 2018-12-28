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
#include <iomanip>
#include <chrono>
#include <stdio.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <time.h>

#include <opencv2/core/core.hpp>
#include "MoveSenseCamera.h"

#include <System.h>

using namespace std;
using namespace movesense;

static unsigned char willExit = 0;
static int handexit = 1;
static int frameCnt = 0;
static double time1=cv::getTickCount(), time2=0, timeT=0;

static __sighandler_t exitHandler();
static bool initCamera(MoveSenseCamera *camera, cv::Mat &left, cv::Mat &right, cv::Mat &disparity);
static void getImage(MoveSenseCamera *camera, cv::Mat &left, cv::Mat &right, cv::Mat &disparity);

int main(int argc, char **argv)
{
  MoveSenseCamera *camera = new MoveSenseCamera(0);
  cv::Mat left,right,disparity;
  if(false == initCamera(camera,left,right,disparity))
  {
    return 0;
  }

  // Create SLAM system. It initializes all system threads and gets ready to process frames.
  ORB_SLAM2::System SLAM("Vocabulary/ORBvoc.bin","Examples/CS480/CS480.yaml",ORB_SLAM2::System::STEREO,true,(bool)atoi(argv[1])); 

  sigset(SIGINT,(__sighandler_t)exitHandler);

  while(handexit)
  {
    //get image frome camera
    getImage(camera,left,right,disparity);

    // Pass the images to the SLAM system
    SLAM.TrackStereo(left,right,0);
  }
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory
  SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

  //close camera
  camera->StopStream();
  camera->Close();
  delete camera;

  return 0;
}

static __sighandler_t exitHandler()
{
  handexit = 0;
  return 0;
}

static bool initCamera(MoveSenseCamera *camera, cv::Mat &left, cv::Mat &right, cv::Mat &disparity)
{
  if (!camera->IsAvailable())
  {
    cout << "No camera connected..." << endl;
    return false;
  }
  else
  {
    cout << "camera connected" << endl;
  }

  int mode = CAMERA_LR;
  int res = camera->SetMode(mode);
  if (res != MS_SUCCESS)
  {
    cout << "The mode is not supported..." << endl;
    return false;
  }
  else
  {
    cout << "The mode is " << mode << endl;
    cout << "0 = CAMERA_LR" << endl
         << "1 = CAMERA_LD" << endl
         << "2 = CAMERA_LRD" << endl
         << "3 = CAMERA_LR_HD" << endl;
  }

  //Open camera
  if (camera->Open() != MS_SUCCESS)
  {
    cout << "Open camera failed!" << endl;
    camera->Close();
    delete camera;
    return false;
  }
  else
  {
    cout << "Open camera successfully" << endl;
  }

  camera->StartStream();
  camera->SetStereoRectify(true);

  //Get camera setting
  Resolution resolution = camera->GetResolution();
  int w = resolution.width;
  int h = resolution.height;
  int bitDepth = camera->GetBitDepth();
  if( bitDepth != 8 && bitDepth != 16 )
  {
    cout << "bitDepth illegal (should be 8 or 16)!" << endl;
    camera->StopStream();
    camera->Close();
    delete camera;
    return false;
  }
  
  left.create(h,w,CV_8UC1);
  right.create(h,w,CV_8UC1);
  disparity.create(h,w,CV_16UC1);

  return true;
}

static void getImage(MoveSenseCamera *camera, cv::Mat &left, cv::Mat &right, cv::Mat &disparity)
{
  int len = camera->GetImageDataLength();
  int res = camera->GetImageData(left.data, right.data, disparity.data, len);

  if(res == MS_SUCCESS)
  {
    frameCnt++;

    if (frameCnt == 50)
    {
      time2 = (double)cv::getTickCount();
      timeT = (time2-time1)/(double)cv::getTickFrequency();
      cout << "fps: " << (frameCnt)*1.0/timeT << endl;

      frameCnt = 0;
      time1 = time2;
    }
  }
  else if (res == MS_TIMEOUT)
  {
    cout << "GetImageData: timeout" << endl;
  }
  else
  {
    cout << "GetImageData: failed" << endl;
  }
}
