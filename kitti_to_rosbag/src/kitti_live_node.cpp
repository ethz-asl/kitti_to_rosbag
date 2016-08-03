#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>
#include <tf/transform_broadcaster.h>

#include "kitti_to_rosbag/kitti_parser.h"
#include "kitti_to_rosbag/kitti_ros_conversions.h"

namespace kitti {

class KittiLiveNode {
 public:
  KittiLiveNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                const std::string& calibration_path,
                const std::string& dataset_path);

  bool publishEntry(uint64_t entry);

  void publishTf(const ros::Time& timestamp_ros,
                 const Transformation& imu_pose);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers for the topics.
  ros::Publisher clock_pub_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher transform_pub_;
  tf::TransformBroadcaster tf_broadcaster_;

  image_transport::ImageTransport image_transport_;
  image_transport::CameraPublisher image_pub_;

  kitti::KittiParser parser_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string velodyne_frame_id_;
};

KittiLiveNode::KittiLiveNode(const ros::NodeHandle& nh,
                             const ros::NodeHandle& nh_private,
                             const std::string& calibration_path,
                             const std::string& dataset_path)
    : nh_(nh),
      nh_private_(nh_private),
      image_transport_(nh_),
      parser_(calibration_path, dataset_path, true),
      world_frame_id_("world"),
      imu_frame_id_("imu"),
      cam_frame_id_prefix_("cam"),
      velodyne_frame_id_("velodyne") {
  // Advertise all the publishing topics for ROS live streaming.
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1, false);
  pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "velodyne_points", 10, false);
  pose_pub_ =
      nh_private_.advertise<geometry_msgs::PoseStamped>("pose_imu", 10, false);
  transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "transform_imu", 10, false);

  // TODO(helenol): make a vector of these for all cameras.
  image_pub_ = image_transport_.advertiseCamera("cam0", 1);

  // Load all the timestamp maps and calibration parameters.
  parser_.loadCalibration();
  parser_.loadTimestampMaps();
}

bool KittiLiveNode::publishEntry(uint64_t entry) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;
  rosgraph_msgs::Clock clock_time;

  // Publish poses + TF transforms + clock.
  Transformation pose;
  if (parser_.getPoseAtEntry(entry, &timestamp_ns, &pose)) {
    geometry_msgs::PoseStamped pose_msg;
    geometry_msgs::TransformStamped transform_msg;

    timestampToRos(timestamp_ns, &timestamp_ros);
    pose_msg.header.frame_id = world_frame_id_;
    pose_msg.header.stamp = timestamp_ros;
    transform_msg.header.frame_id = world_frame_id_;
    transform_msg.header.stamp = timestamp_ros;

    poseToRos(pose, &pose_msg);
    transformToRos(pose, &transform_msg);

    pose_pub_.publish(pose_msg);
    transform_pub_.publish(transform_msg);

    clock_time.clock = timestamp_ros;
    clock_pub_.publish(clock_time);

    publishTf(timestamp_ros, pose);
  } else {
    return false;
  }

  // Publish images.
  // TODO(helenol): all cams!
  cv::Mat image;
  if (parser_.getImageAtEntry(entry, 0, &timestamp_ns, &image)) {
    sensor_msgs::Image image_msg;
    imageToRos(image, &image_msg);

    // TODO(helenol): cache this.
    // Get the calibration info for this camera.
    CameraCalibration cam_calib;
    parser_.getCameraCalibration(0, &cam_calib);
    sensor_msgs::CameraInfo cam_info;
    calibrationToRos(0, cam_calib, &cam_info);

    timestampToRos(timestamp_ns, &timestamp_ros);

    image_pub_.publish(image_msg, cam_info, timestamp_ros);
  }

  // Publish pointclouds.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  if (parser_.getPointcloudAtEntry(entry, &timestamp_ns, &pointcloud)) {
    timestampToRos(timestamp_ns, &timestamp_ros);

    pointcloud.header.stamp = timestamp_ns;
    pointcloud_pub_.publish(pointcloud);
  }

  return true;
}

void KittiLiveNode::publishTf(const ros::Time& timestamp_ros,
                              const Transformation& imu_pose) {}

}  // namespace kitti

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_node");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  const std::string calibration_path = "/Users/helen/data/kitti/2011_09_26";
  const std::string dataset_path =
      "/Users/helen/data/kitti/2011_09_26/2011_09_26_drive_0035_sync";

  kitti::KittiParser parser(calibration_path, dataset_path, true);

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
  parser.getImageAtEntry(0, 3, &timestamp, &image);

  cv::imshow("Display window", image);
  cv::waitKey(0);

  ros::spin();
  return 0;
}
