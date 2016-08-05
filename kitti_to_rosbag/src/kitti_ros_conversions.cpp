#include <cv_bridge/cv_bridge.h>
#include <minkindr_conversions/kindr_msg.h>
#include <minkindr_conversions/kindr_tf.h>

#include "kitti_to_rosbag/kitti_ros_conversions.h"

namespace kitti {

std::string getCameraFrameId(int cam_id) {
  char buffer[20];
  sprintf(buffer, "cam%02d", cam_id);
  return std::string(buffer);
}

void calibrationToRos(uint64_t cam_id, const CameraCalibration& cam,
                      sensor_msgs::CameraInfo* cam_msg) {
  cam_msg->header.frame_id = getCameraFrameId(cam_id);

  cam_msg->width = cam.image_size.x();
  cam_msg->height = cam.image_size.y();

  cam_msg->distortion_model = "plumb_bob";

  // D is otherwise empty by default, hopefully this is fine.
  if (cam.distorted) {
    const size_t kNumDistortionParams = 5;
    cam_msg->D.resize(kNumDistortionParams);
    for (size_t i = 0; i < kNumDistortionParams; ++i) {
      cam_msg->D[i] = cam.D(i);
    }
  }

  // Copy over intrinsics.
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      cam_msg->K[3 * i + j] = cam.K(i, j);
    }
  }

  // Rectification/projection matrices.
  for (size_t i = 0; i < 3; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      cam_msg->R[3 * i + j] = cam.rect_mat(i, j);
    }
  }

  for (size_t i = 0; i < 4; ++i) {
    for (size_t j = 0; j < 3; ++j) {
      cam_msg->P[4 * i + j] = cam.projection_mat(i, j);
    }
  }
}

void stereoCalibrationToRos(uint64_t left_cam_id, uint64_t right_cam_id,
                            const CameraCalibration& left_cam,
                            const CameraCalibration& right_cam,
                            sensor_msgs::CameraInfo* left_cam_msg,
                            sensor_msgs::CameraInfo* right_cam_msg) {
  // Fill in the basics for each camera.
  calibrationToRos(left_cam_id, left_cam, left_cam_msg);
  calibrationToRos(right_cam_id, right_cam, right_cam_msg);

  // Since all transforms are given relative to cam0, need to remove the cam0
  // transform from both to get the relative one between the two (cam0 to cam0
  // is ostensibly identity anyway).
  // This is probably not even necessary.
  // Transformation T_left_cam0 = left_cam.T_cam0_cam.inverse();
  // Transformation T_left_right = T_left_cam0 * right_cam.T_cam0_cam;
}

void imageToRos(const cv::Mat& image, sensor_msgs::Image* image_msg) {
  cv_bridge::CvImage image_cv_bridge;
  image_cv_bridge.image = image;
  // TODO!!!
  image_cv_bridge.encoding = "mono8";
  image_cv_bridge.toImageMsg(*image_msg);
}

void poseToRos(const Transformation& transform,
               geometry_msgs::PoseStamped* pose_msg) {
  tf::poseKindrToMsg(transform, &pose_msg->pose);
}

void transformToTf(const Transformation& transform,
                   tf::Transform* tf_transform) {
  tf::transformKindrToTF(transform, tf_transform);
}

void transformToRos(const Transformation& transform,
                    geometry_msgs::TransformStamped* transform_msg) {
  tf::transformKindrToMsg(transform, &transform_msg->transform);
}

void timestampToRos(uint64_t timestamp_ns, ros::Time* time) {
  time->fromNSec(timestamp_ns);
}

}  // namespace kitti
