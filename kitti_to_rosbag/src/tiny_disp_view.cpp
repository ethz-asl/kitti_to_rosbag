#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <stereo_msgs/DisparityImage.h>

void disparityCallback(const stereo_msgs::DisparityImageConstPtr& disparity) {
  cv_bridge::CvImageConstPtr cv_img_ptr =
  cv_bridge::toCvShare(disparity->image, disparity);
  cv::Mat image;
  cv_img_ptr->image.convertTo(image, CV_8U, 5.0);
  cv::imshow("Disparity", image);
  cv::waitKey(5);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "tiny_disp_view");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ros::Subscriber disparity_sub =
      nh.subscribe("disparity", 1, &disparityCallback);

  ros::spin();
}
