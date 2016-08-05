#include <opencv2/highgui/highgui.hpp>
#include <pcl_ros/point_cloud.h>
#include <rosbag/bag.h>
#include <tf/tfMessage.h>

#include "kitti_to_rosbag/kitti_parser.h"
#include "kitti_to_rosbag/kitti_ros_conversions.h"

namespace kitti {

class KittiBagConverter {
 public:
  KittiBagConverter(const std::string& calibration_path,
                    const std::string& dataset_path,
                    const std::string& output_filename);

  void convertAll();
  bool convertEntry(uint64_t entry);
  void convertTf(uint64_t timestamp_ns, const Transformation& imu_pose);

 private:
  kitti::KittiParser parser_;

  rosbag::Bag bag_;

  std::string world_frame_id_;
  std::string imu_frame_id_;
  std::string cam_frame_id_prefix_;
  std::string velodyne_frame_id_;

  std::string pose_topic_;
  std::string transform_topic_;
  std::string pointcloud_topic_;
};


KittiBagConverter::KittiBagConverter(const std::string& calibration_path,
                                     const std::string& dataset_path,
                                     const std::string& output_filename)
    : parser_(calibration_path, dataset_path, true),
      world_frame_id_("world"),
      imu_frame_id_("imu"),
      cam_frame_id_prefix_("cam"),
      velodyne_frame_id_("velodyne"),
      pose_topic_("pose_imu"),
      transform_topic_("transform_imu"),
      pointcloud_topic_("velodyne_points") {
  // Load all the timestamp maps and calibration parameters.
  parser_.loadCalibration();
  parser_.loadTimestampMaps();

  bag_.open(output_filename, rosbag::bagmode::Write);

  /*
  // Advertise all the publishing topics for ROS live streaming.
  clock_pub_ = nh_.advertise<rosgraph_msgs::Clock>("/clock", 1, false);
  pointcloud_pub_ = nh_private_.advertise<pcl::PointCloud<pcl::PointXYZI> >(
      "velodyne_points", 10, false);
  pose_pub_ =
      nh_private_.advertise<geometry_msgs::PoseStamped>("pose_imu", 10, false);
  transform_pub_ = nh_private_.advertise<geometry_msgs::TransformStamped>(
      "transform_imu", 10, false);

  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    image_pubs_.push_back(
        image_transport_.advertiseCamera(getCameraFrameId(cam_id), 1));
  } */
}

void KittiBagConverter::convertAll() {
  uint64_t entry = 0;
  while (convertEntry(entry)) {
    entry++;
  }
  std::cout << "Converted " << entry << " entries into a rosbag.\n";
}

bool KittiBagConverter::convertEntry(uint64_t entry) {
  ros::Time timestamp_ros;
  uint64_t timestamp_ns;

  // Convert poses + TF transforms.
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

    bag_.write(pose_topic_, timestamp_ros, pose_msg);
    bag_.write(transform_topic_, timestamp_ros, transform_msg);

    convertTf(timestamp_ns, pose);
  } else {
    return false;
  }

  // Convert images.
  cv::Mat image;
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    if (parser_.getImageAtEntry(entry, cam_id, &timestamp_ns, &image)) {
      sensor_msgs::Image image_msg;
      imageToRos(image, &image_msg);

      // TODO(helenol): cache this.
      // Get the calibration info for this camera.
      CameraCalibration cam_calib;
      parser_.getCameraCalibration(cam_id, &cam_calib);
      sensor_msgs::CameraInfo cam_info;
      calibrationToRos(cam_id, cam_calib, &cam_info);

      timestampToRos(timestamp_ns, &timestamp_ros);

      bag_.write(getCameraFrameId(cam_id) + "/image_raw", timestamp_ros,
                 image_msg);
      bag_.write(getCameraFrameId(cam_id) + "/camera_info", timestamp_ros,
                 cam_info);
    }
  }

  // Convert pointclouds.
  pcl::PointCloud<pcl::PointXYZI> pointcloud;
  if (parser_.getPointcloudAtEntry(entry, &timestamp_ns, &pointcloud)) {
    timestampToRos(timestamp_ns, &timestamp_ros);

    // This value is in MICROSECONDS, not nanoseconds.
    pointcloud.header.stamp = timestamp_ns / 1000;
    pointcloud.header.frame_id = velodyne_frame_id_;

    bag_.write(pointcloud_topic_, timestamp_ros, pointcloud);
  }

  return true;
}

void KittiBagConverter::convertTf(uint64_t timestamp_ns,
                                  const Transformation& imu_pose) {
  tf::tfMessage tf_msg;
  ros::Time timestamp_ros;
  timestampToRos(timestamp_ns, &timestamp_ros);

  // Create the full transform chain.
  Transformation T_imu_world = imu_pose;
  Transformation T_vel_imu = parser_.T_vel_imu();
  Transformation T_cam_imu;

  geometry_msgs::TransformStamped tf_imu_world, tf_vel_imu, tf_cam_imu;
  transformToRos(T_imu_world, &tf_imu_world);
  tf_imu_world.header.frame_id = world_frame_id_;
  tf_imu_world.child_frame_id = imu_frame_id_;
  tf_imu_world.header.stamp = timestamp_ros;
  transformToRos(T_vel_imu.inverse(), &tf_vel_imu);
  tf_vel_imu.header.frame_id = imu_frame_id_;
  tf_vel_imu.child_frame_id = velodyne_frame_id_;
  tf_vel_imu.header.stamp = timestamp_ros;

  // Put them into one tf_msg.
  tf_msg.transforms.push_back(tf_imu_world);
  tf_msg.transforms.push_back(tf_vel_imu);

  // Get all of the camera transformations as well.
  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    T_cam_imu = parser_.T_camN_imu(cam_id);
    transformToRos(T_cam_imu.inverse(), &tf_cam_imu);
    tf_cam_imu.header.frame_id = imu_frame_id_;
    tf_cam_imu.child_frame_id = getCameraFrameId(cam_id);
    tf_cam_imu.header.stamp = timestamp_ros;
    tf_msg.transforms.push_back(tf_cam_imu);
  }


  bag_.write("/tf", timestamp_ros, tf_msg);

  /* ros::Time timestamp_ros;
  timestampToRos(timestamp_ns, &timestamp_ros);
  Transformation T_imu_world = imu_pose;
  Transformation T_vel_imu = parser_.T_vel_imu();
  Transformation T_cam_imu;

  tf::Transform tf_imu_world, tf_cam_imu, tf_vel_imu;

  transformToTf(T_imu_world, &tf_imu_world);
  transformToTf(T_vel_imu.inverse(), &tf_vel_imu);

  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_imu_world, timestamp_ros, world_frame_id_, imu_frame_id_));
  tf_broadcaster_.sendTransform(tf::StampedTransform(
      tf_vel_imu, timestamp_ros, imu_frame_id_, velodyne_frame_id_));

  for (size_t cam_id = 0; cam_id < parser_.getNumCameras(); ++cam_id) {
    T_cam_imu = parser_.T_camN_imu(cam_id);
    transformToTf(T_cam_imu.inverse(), &tf_cam_imu);
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        tf_cam_imu, timestamp_ros, imu_frame_id_, getCameraFrameId(cam_id)));
  } */
}

}  // namespace kitti

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();

  const std::string calibration_path = "/Users/helen/data/kitti/2011_09_26";
  const std::string dataset_path =
      "/Users/helen/data/kitti/2011_09_26/2011_09_26_drive_0035_sync";
  const std::string output_path =
      "/Users/helen/data/kitti/2011_09_26/2011_09_26_drive_0035_sync/"
      "testbag.bag";

  kitti::KittiBagConverter converter(calibration_path, dataset_path,
                                     output_path);

  converter.convertAll();

  return 0;
}
