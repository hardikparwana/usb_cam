/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Robert Bosch LLC.
*  All rights reserved.
*
*  Modified by Hardik Parwana
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
*   * Neither the name of the Robert Bosch nor the names of its
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
*********************************************************************/

#include <ros/ros.h>
#include <usb_cam/usb_cam.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sstream>
#include <std_srvs/Empty.h>

#include "opencv2/opencv.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <std_msgs/Header.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

using namespace cv;
using namespace std;

namespace usb_cam {

class UsbCamNode
{
public:
  // private ROS node handle
  ros::NodeHandle node_;
  
  // shared image message
  sensor_msgs::Image img_;
  image_transport::CameraPublisher image_pub_;
  image_transport::CameraPublisher left_image_pub_;
  image_transport::CameraPublisher right_image_pub_;

  // parameters
  std::string video_device_name_, io_method_name_, pixel_format_name_, camera_name_, camera_info_url_, left_camera_name_, right_camera_name_, left_camera_info_url_, right_camera_info_url_;
  //std::string start_service_name_, start_service_name_;
  std::string video_dir_, source_;
  bool streaming_status_;
  int image_width_, image_height_, framerate_, exposure_, brightness_, contrast_, saturation_, sharpness_, focus_,
      white_balance_, gain_;
  bool autofocus_, autoexposure_, auto_white_balance_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfo_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> left_cinfo_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> right_cinfo_;

  UsbCam cam_;

  ros::ServiceServer service_start_, service_stop_;

  VideoCapture *cap;

  bool service_start_cap(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.start_capturing();
    return true;
  }


  bool service_stop_cap( std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res )
  {
    cam_.stop_capturing();
    return true;
  }

  UsbCamNode() :
      node_("~")
  {
    // advertise the main image topic
    image_transport::ImageTransport it(node_);
    image_pub_ = it.advertiseCamera("image_raw", 1);
    left_image_pub_ = it.advertiseCamera("left/image_raw", 1);
    right_image_pub_ = it.advertiseCamera("right/image_raw", 1);

    // grab the parameters
    node_.param("video_device", video_device_name_, std::string("/dev/video0"));
    node_.param("brightness", brightness_, -1); //0-255, -1 "leave alone"
    node_.param("contrast", contrast_, -1); //0-255, -1 "leave alone"
    node_.param("saturation", saturation_, -1); //0-255, -1 "leave alone"
    node_.param("sharpness", sharpness_, -1); //0-255, -1 "leave alone"
    // possible values: mmap, read, userptr
    node_.param("io_method", io_method_name_, std::string("mmap"));
    node_.param("image_width", image_width_, 640);
    node_.param("image_height", image_height_, 480);
    node_.param("framerate", framerate_, 30);
    // possible values: yuyv, uyvy, mjpeg, yuvmono10, rgb24
    node_.param("pixel_format", pixel_format_name_, std::string("mjpeg"));
    // enable/disable autofocus
    node_.param("autofocus", autofocus_, false);
    node_.param("focus", focus_, -1); //0-255, -1 "leave alone"
    // enable/disable autoexposure
    node_.param("autoexposure", autoexposure_, true);
    node_.param("exposure", exposure_, 100);
    node_.param("gain", gain_, -1); //0-100?, -1 "leave alone"
    // enable/disable auto white balance temperature
    node_.param("auto_white_balance", auto_white_balance_, true);
    node_.param("white_balance", white_balance_, 4000);

    // load the camera info
    node_.param("camera_frame_id", img_.header.frame_id, std::string("head_camera"));
    node_.param("camera_name", camera_name_, std::string("head_camera"));
    // node_.param("left_camera_name", left_camera_name_, std::string("stereo_left"));
    // node_.param("right_camera_name", right_camera_name_, std::string("stereo_right"));
    node_.param("camera_info_url", camera_info_url_, std::string(""));
    node_.param("left_camera_info_url", left_camera_info_url_, std::string(""));
    node_.param("right_camera_info_url", right_camera_info_url_, std::string(""));
    node_.param("video_dir",video_dir_,std::string(""));
    node_.param("source",source_,std::string(""));

    cout << "info " << left_camera_info_url_ << endl;

    cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_, camera_info_url_));
    left_cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_ + "/left", left_camera_info_url_));
    right_cinfo_.reset(new camera_info_manager::CameraInfoManager(node_, camera_name_+ "/right", right_camera_info_url_));

    // cap = new VideoCapture("/media/worldhaptics/DATA/UbuntuHome/lightdetection1/traffic-light-detection/splice2.mp4"); //splice2.mp4
    // cap = new VideoCapture("/home/interface/softwares/tensorflow/Tensorflow_dataset/videos/video1.mp4"); 

    std::string stereo_cam = "cam";
    std::string video_file = "video";
    if (strcmp(source_.c_str(),video_file.c_str())==0)
    {
      cout << video_dir_ << endl;
      cap = new VideoCapture(video_dir_); 
    }
    
    // cap = new VideoCapture("/media/worldhaptics/DATA/UbuntuHome/lightdetection1/AsiaHaptics/traffic-light-detection/video2_1.mp4"); 
   // Check if camera opened successfully
    if(!cap->isOpened()){
    cout << "Error opening video stream or file" << endl;
    return;
    }

    // create Services
    service_start_ = node_.advertiseService("start_capture", &UsbCamNode::service_start_cap, this);
    service_stop_ = node_.advertiseService("stop_capture", &UsbCamNode::service_stop_cap, this);

    // check for default camera info
    if (!cinfo_->isCalibrated())
    {
      cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_;
      camera_info.height = image_height_;
      cinfo_->setCameraInfo(camera_info);
    }
    if (!left_cinfo_->isCalibrated())
    {
      left_cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_/2;
      camera_info.height = image_height_;
      left_cinfo_->setCameraInfo(camera_info);
    }
    if (!right_cinfo_->isCalibrated())
    {
      right_cinfo_->setCameraName(video_device_name_);
      sensor_msgs::CameraInfo camera_info;
      camera_info.header.frame_id = img_.header.frame_id;
      camera_info.width = image_width_/2;
      camera_info.height = image_height_;
      right_cinfo_->setCameraInfo(camera_info);
    }

    // cout << "h11" << endl;
    ROS_INFO("Starting '%s' (%s) at %dx%d via %s (%s) at %i FPS", camera_name_.c_str(), video_device_name_.c_str(),
        image_width_, image_height_, io_method_name_.c_str(), pixel_format_name_.c_str(), framerate_);
    // cout << "h12" << endl;
    // set the IO method
    UsbCam::io_method io_method = UsbCam::io_method_from_string(io_method_name_);
    if(io_method == UsbCam::IO_METHOD_UNKNOWN)
    {
      ROS_FATAL("Unknown IO method '%s'", io_method_name_.c_str());
      node_.shutdown();
      return;
    }
    // cout << "h13" << endl;
    // set the pixel format
    UsbCam::pixel_format pixel_format = UsbCam::pixel_format_from_string(pixel_format_name_);
    if (pixel_format == UsbCam::PIXEL_FORMAT_UNKNOWN)
    {
      ROS_FATAL("Unknown pixel format '%s'", pixel_format_name_.c_str());
      node_.shutdown();
      return;
    }
    // cout << "h14" << endl;
    // start the camera
    cam_.start(video_device_name_.c_str(), io_method, pixel_format, image_width_,
		     image_height_, framerate_);

    // set camera parameters
    if (brightness_ >= 0)
    {
      cam_.set_v4l_parameter("brightness", brightness_);
    }
    // cout << "h15" << endl;
    if (contrast_ >= 0)
    {
      cam_.set_v4l_parameter("contrast", contrast_);
    }

    if (saturation_ >= 0)
    {
      cam_.set_v4l_parameter("saturation", saturation_);
    }

    if (sharpness_ >= 0)
    {
      cam_.set_v4l_parameter("sharpness", sharpness_);
    }

    if (gain_ >= 0)
    {
      cam_.set_v4l_parameter("gain", gain_);
    }
    // cout << "h16" << endl;
    // check auto white balance
    if (auto_white_balance_)
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("white_balance_temperature_auto", 0);
      cam_.set_v4l_parameter("white_balance_temperature", white_balance_);
    }

    // check auto exposure
    if (!autoexposure_)
    {
      // turn down exposure control (from max of 3)
      cam_.set_v4l_parameter("exposure_auto", 1);
      // change the exposure level
      cam_.set_v4l_parameter("exposure_absolute", exposure_);
    }

    // check auto focus
    if (autofocus_)
    {
      cam_.set_auto_focus(1);
      cam_.set_v4l_parameter("focus_auto", 1);
    }
    else
    {
      cam_.set_v4l_parameter("focus_auto", 0);
      if (focus_ >= 0)
      {
        cam_.set_v4l_parameter("focus_absolute", focus_);
      }
    }
    // cout << "h17" << endl;
  }

  virtual ~UsbCamNode()
  {
    cam_.shutdown();
  }

  bool take_and_send_image()
  {
    // grab the image
    cam_.grab_image(&img_);
    // grab the camera info
    sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo_->getCameraInfo()));
    // ci->header.frame_id = img_.header.frame_id;        ////////////////////////////////////////////
    // ci->header.stamp = img_.header.stamp;           //////////////////////////////////////// 

    ci->header.frame_id = "cam";
    ci->header.stamp = ros::Time::now();

   // publish the image
   // image_pub_.publish(img_, *ci);

    sensor_msgs::ImageConstPtr img_ptr( new sensor_msgs::Image( img_ ) );    ///////////////////////////////////////////
    cv::Mat image_stereo;
    image_stereo = cv_bridge::toCvShare(img_ptr, "bgr8")->image;     ///////////////////////////////////////////
    Mat frame1;
    *cap >> frame1;
    image_stereo = frame1;
    // image_stereo = imread("/home/worldhaptics/catkin_ws/splice2tf1.png");
    // image_stereo = imread("/home/worldhaptics/Pictures/i13.png");  //tf: 19,6 : 24:tf,car (good!)
    int width_stereo   = image_stereo.size().width; //width = col = x;
    int height_stereo    = image_stereo.size().height; //height = row = y;
    int width_single   = width_stereo/2;
    int height_single    = height_stereo;
    // cout << "h3" << endl;
    //cut one image into two
    sensor_msgs::ImagePtr img_full = cv_bridge::CvImage(ci->header, "bgr8", image_stereo).toImageMsg();

    image_pub_.publish(*img_full, *ci);


    cv::Mat image_left1  = image_stereo(Range(0, height_single), Range(0, width_single));
    cv::Mat image_right1 = image_stereo(Range(0, height_single), Range(width_single, width_stereo));

    //change the size if the size of incoming image is diff with the size of the calibration data setting
    // if(is_init_size && (width_single!=width || height_single!=height) )
    // {
    // if(is_first_image)
    //   { 
    //     ROS_WARN("the size of raw image data [%d, %d] is different with the size of param[%d, %d]. resize!", width_single, height_single, width, height); 
    //   }
    //   resize(image_left,image_left,Size(width,height),0,0,CV_INTER_LINEAR);
    //   resize(image_right,image_right,Size(width,height),0,0,CV_INTER_LINEAR);
    // }
    // cout << "h4" << endl;
    //build image msg
    sensor_msgs::ImagePtr img_left_ = cv_bridge::CvImage(ci->header, "bgr8", image_left1).toImageMsg();
    sensor_msgs::ImagePtr img_right_ = cv_bridge::CvImage(ci->header, "bgr8", image_right1).toImageMsg();
    // cout << "h5" << endl;

    //LEFT
    // grab the camera info
    sensor_msgs::CameraInfoPtr cil(new sensor_msgs::CameraInfo(left_cinfo_->getCameraInfo()));
    cil->header.frame_id = img_.header.frame_id;
    cil->header.stamp = img_.header.stamp;    
    // cout << "h6" << endl;
    // publish the image
    left_image_pub_.publish(*img_left_, *cil);

    //RIGHT
    // grab the camera info
    sensor_msgs::CameraInfoPtr cir(new sensor_msgs::CameraInfo(right_cinfo_->getCameraInfo()));
    cir->header.frame_id = img_.header.frame_id;
    cir->header.stamp = img_.header.stamp;
    // cout << "h7" << endl;
    // publish the image
    right_image_pub_.publish(*img_right_, *cir);

    return true;
  }

  bool spin()
  {
    ros::Rate loop_rate(this->framerate_);
    while (node_.ok())
    {
      // cout << "looping" << endl;
      if (cam_.is_capturing()) {
        if (!take_and_send_image()) ROS_WARN("USB camera did not respond in time.");
      }
      // cout << "loopinge" << endl;
      ros::spinOnce();
      loop_rate.sleep();

    }
    return true;
  }






};

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "stereo_cam_node");
  usb_cam::UsbCamNode a;
  // cout << "hstart" << endl;
  a.spin();
  return EXIT_SUCCESS;
}
