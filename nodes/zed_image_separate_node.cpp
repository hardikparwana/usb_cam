#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
// #include <opencv2/nonfree/features2d.hpp> //FeatureDetector µÄÀà¹ØÊý
// #include <opencv2/nonfree/nonfree.hpp>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <cv_bridge/cv_bridge.h>
// #include "find_ros_path.h"

using namespace std;
using namespace cv;

image_transport::Publisher image_left_pub;
image_transport::Publisher image_right_pub;

std_msgs::Header header_left;
std_msgs::Header header_right;

int width = 0;
int height = 0;

bool is_init_size = false;
bool is_first_image = true;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::Mat image_stereo = cv_bridge::toCvShare(msg, "bgr8")->image;
    int width_stereo 	 = image_stereo.size().width; //width = col = x;
    int height_stereo 	 = image_stereo.size().height; //height = row = y;
    int width_single 	 = width_stereo/2;
    int height_single 	 = height_stereo;

    //cut one image into two
    cv::Mat image_left  = image_stereo(Range(0, height_single), Range(0, width_single));
    cv::Mat image_right = image_stereo(Range(0, height_single), Range(width_single, width_stereo));

    //change the size if the size of incoming image is diff with the size of the calibration data setting
    if(is_init_size && (width_single!=width || height_single!=height) )
    {
	if(is_first_image)
	{ ROS_WARN("the size of raw image data [%d, %d] is different with the size of param[%d, %d]. resize!", width_single, height_single, width, height); }
	resize(image_left,image_left,Size(width,height),0,0,CV_INTER_LINEAR);
	resize(image_right,image_right,Size(width,height),0,0,CV_INTER_LINEAR);
    }
    //update msg
    header_left.stamp = ros::Time::now();
    header_right.stamp = ros::Time::now();

    //build image msg
    sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(header_left, "bgr8", image_left).toImageMsg();
    sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(header_right, "bgr8", image_right).toImageMsg();
    image_left_pub.publish(image_left_msg);
    image_right_pub.publish(image_right_msg);

    is_first_image = false;
  }
  catch (cv_bridge::Exception& e)
  { ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str()); }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zed_image_seperate_node");
  ros::NodeHandle nh_;
  ros::NodeHandle pravite_nh_("~");

  //cv::namedWindow("image_left");
  //cv::namedWindow("image_right");
  //cv::startWindowThread();

  image_transport::ImageTransport it(nh_);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

  image_left_pub = it.advertise("image_zed_left_raw", 1);
  image_right_pub = it.advertise("image_zed_right_raw", 1);

  header_left.stamp 	= ros::Time::now();
  header_left.frame_id 	= "camera_left";
  header_left.seq 	= 255;
  header_right.stamp 	= ros::Time::now();
  header_right.frame_id = "camera_right";
  header_right.seq 	= 255;

 
  ros::spin();
  //cv::destroyWindow("view");
}