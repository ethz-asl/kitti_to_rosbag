#include <ros/ros.h>
#include <opencv2/highgui/highgui.hpp>

#include "kitti_to_rosbag/kitti_parser.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  kitti::KittiParser parser(
      "/Users/helen/data/kitti/2011_09_26",
      "/Users/helen/data/kitti/2011_09_26/2011_09_26_drive_0035_sync", true);

  parser.loadCalibration();
  parser.loadTimestampMaps();

  uint64_t timestamp;
  kitti::Transformation pose;
  parser.getPoseAtEntry(0, &timestamp, &pose);

  std::cout << "Timestamp: " << timestamp << " Pose: " << pose << std::endl;

  pcl::PointCloud<pcl::PointXYZI> ptcloud;
  parser.getPointcloudAtEntry(0, &timestamp, &ptcloud);

  std::cout << "Timestamp: " << timestamp << " Num points: " << ptcloud.size()
            << std::endl;

  cv::Mat image;
  parser.getImageAtEntry(3, 0, &timestamp, &image);

  cv::imshow("Display window", image);
  cv::waitKey(0);

  ros::spin();
  return 0;
}
