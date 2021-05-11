# Introduction to OpenCV for Drone Applications

This tutorial assumes you have installed the ardupilot-sitl with the ardupilot gazebo plugin. This tutorial will teach you how to create a simple computer vision algorithm to be run on a ROS image stream.

## Pre-Req

Clone the iq_vision ros package
```
git clone https://github.com/Intelligent-Quads/iq_vision.git
```

## Setup 

create the file `canny_edge.cpp` in `iq_vision/src`

Add the following line to the end of the `CMakeLists.txt`
```
add_executable(canny_edge src/canny_edge.cpp)
target_link_libraries(canny_edge ${catkin_LIBRARIES} ${OpenCV_INCLUDE_DIRS})
```

## Setup our ROS node 

the following code contains the includes the needed ros libraries as well as the opencv libraries we will need for the tutorial.

```
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

// Add vision object here

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  ros::spin();
  return 0;
}
```


## Creating an Object to do Vision Processing 

```
class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/webcam/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Run Canny edge detector on image
    cv::Mat src = cv_ptr->image;
    cv::Mat dst;
    Canny( src, dst, 0, 0, 3 );

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, src);
    cv::imshow("canny", dst);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

```