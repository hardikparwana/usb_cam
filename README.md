# usb_cam
A supplement to the original [usb_cam](https://github.com/ros-drivers/usb_cam) ROS Driver for V4L USB Cameras to include node which can read raw images from stereo cameras such as ZED and output separated images as well as calibration data in *sensor_msgs::CameraInfo* format.

## Additional Dependencies
* cv_bridge
* OpenCV (tested with v3.4.4)

## Usage
The *stereo_cam_node* can be used to get images from stereo cameras like ZED that give concatenated images as raw input OR you can specify the video file from which you would like to stream video. Specify the following params in the **stereo_cam_test.launch** file
* source: *cam* for camera and *video* for streaming video from a file
* video_device: camera port on your device. default: /dev/video0
* video_dir: video file address
* image_width: width of camera input frame
* image_height: height of camera input frame
* left_camera_info_url: address of the yaml calibration file on your device. a sample has been provided in the calibration folder for ZED camera.
* right_camera_info_url: address of the yaml calibration file on your device. a sample has been provided in the calibration folder for ZED camera.

Simply run
> roslaunch usb_cam stereo_cam_test.launch 
