#ifndef KITTI_TO_ROSBAG_KITTI_COMMON_H_
#define KITTI_TO_ROSBAG_KITTI_COMMON_H_

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <kindr/minimal/quat-transformation.h>

namespace kitti {

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformation Transformation;
typedef kindr::minimal::RotationQuaternion Rotation;

struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Intrinsics.
  Eigen::Vector2d image_size = Eigen::Vector2d::Zero();  // S_xx in calibration.
  Eigen::Matrix3d rect_mat =
      Eigen::Matrix3d::Identity();  // R_rect_xx in calibration.
  Eigen::Matrix<double, 3, 4> projection_mat =
      Eigen::Matrix<double, 3, 4>::Identity();  // P_xx in calibration.

  // Unrectified (raw) intrinsics. Should only be used if rectified set to
  // false.
  Eigen::Matrix3d K =
      Eigen::Matrix3d::Zero();  // Camera intrinsics, K_xx in calibration.
  Eigen::Matrix<double, 1, 5> D =
      Eigen::Matrix<double, 1,
                    5>::Zero();  // Distortion parameters, radtan model.

  // Extrinsics.
  Transformation T_cam0_cam;

  bool distorted = false;
};

typedef std::vector<CameraCalibration,
                    Eigen::aligned_allocator<CameraCalibration> >
    CameraCalibrationVector;

}  // namespace kitti

#endif  // KITTI_TO_ROSBAG_KITTI_COMMON_H_
