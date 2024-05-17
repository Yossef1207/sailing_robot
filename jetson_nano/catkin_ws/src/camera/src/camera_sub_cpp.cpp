#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
 
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
 
  // Pointer used for the conversion from a ROS message to 
  // an OpenCV-compatible image
  cv_bridge::CvImagePtr cv_ptr;
   
  try
  { 
   
    // Convert the ROS message  
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
     
    // Store the values of the OpenCV-compatible image
    // into the current_frame variable
    cv::Mat current_frame = cv_ptr->image;
     
    cv::Mat testimg, grayimg, colorimg, bwimg;
    // Decimate image by factor of 2
    cv::resize(current_frame, testimg, cv::Size(), 0.5, 0.5);
    
    // Convert to grayscale
    cv::cvtColor(current_frame, grayimg, cv::COLOR_RGB2GRAY);
    cv::cvtColor(grayimg, colorimg, cv::COLOR_GRAY2RGB);

    // Thresholding
    cv::threshold(grayimg, bwimg, 255, 255, cv::THRESH_OTSU);

    // Display the current frame
    cv::imshow("view", bwimg); 
     
    // Display frame for 30 milliseconds
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}
 
int main(int argc, char **argv)
{
  // The name of the node
  ros::init(argc, argv, "frame_listener");
   
  // Default handler for nodes in ROS
  ros::NodeHandle nh;
   
  // Used to publish and subscribe to images
  image_transport::ImageTransport it(nh);
   
  // Subscribe to the /camera topic
  image_transport::Subscriber sub = it.subscribe("camera", 1, imageCallback);
   
  // Make sure we keep reading new video frames by calling the imageCallback function
  ros::spin();
   
  // Close down OpenCV
  cv::destroyWindow("view");
}
