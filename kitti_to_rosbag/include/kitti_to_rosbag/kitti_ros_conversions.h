#ifndef KITTI_TO_ROSBAG_KITTI_ROS_CONVERSIONS_H_
#define KITTI_TO_ROSBAG_KITTI_ROS_CONVERSIONS_H_

#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>

#include "kitti_to_rosbag/kitti_common.h"

namespace kitti {

void calibrationToRos(uint64_t cam_id, const CameraCalibration& cam,
                      sensor_msgs::CameraInfo* cam_msg);
void stereoCalibrationToRos(const CameraCalibration& left_cam,
                            const CameraCalibration& right_cam,
                            sensor_msgs::CameraInfo* left_cam_msg,
                            sensor_msgs::CameraInfo* right_cam_msg);
void imageToRos(const cv::Mat& image, sensor_msgs::Image* image_msg);
void poseToRos(const Transformation& transform,
               geometry_msgs::PoseStamped* pose_msg);
void transformToTf(const Transformation& transform,
                   tf::Transform* tf_transform);
void transformToRos(const Transformation& transform,
                    geometry_msgs::TransformStamped* transform_msg);
void timestampToRos(uint64_t timestamp_ns, ros::Time* time);

}  // namespace kitty

#endif  // KITTI_TO_ROSBAG_KITTI_ROS_CONVERSIONS_H_
