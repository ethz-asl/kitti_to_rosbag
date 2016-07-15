#ifndef KITTI_TO_ROSBAG_KITTI_PARSER_H_
#define KITTI_TO_ROSBAG_KITTI_PARSER_H_

#include <memory>
#include <map>

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
  Eigen::Vector2d image_size;                  // S_xx in calibration.
  Eigen::Matrix3d rect_mat;                    // R_xx in calibration.
  Eigen::Matrix<double, 3, 4> projection_mat;  // P_xx in calibration.

  // Unrectified (raw) intrinsics. Should only be used if rectified set to
  // false.
  Eigen::Matrix3d K;              // Camera intrinsics, K_xx in calibration.
  Eigen::Matrix<double, 1, 5> D;  // Distortion parameters, radtan model.

  // Extrinsics.
  Eigen::Matrix3d R;  // Rotation matrix, extrinsics, wrt 0th camera (?)
  Eigen::Vector3d T;  // Translation matrix, extrinsics, wrt 0th camera (?)

  // Accessor for full transform. Derived from 'raw' extrinsics above.
  Transformation getTransformToCam0() const {
    Transformation transform(Rotation::fromApproximateRotationMatrix(R), T);
    return transform;
  }
};

typedef std::vector<CameraCalibration,
                    Eigen::aligned_allocator<CameraCalibration> >
    CameraCalibrationVector;

class KittiParser {
 public:
  // Constants for filenames for calibration files.
  static const std::string kVelToCamCalibrationFilename;
  static const std::string kCamToCamCalibrationFilename;
  static const std::string kImuToVelCalibrationFilename;

  KittiParser(const std::string& calibration_path,
              const std::string& dataset_path, bool rectified);

  // MAIN API: all you should need to use!
  // Loading calibration files.
  bool loadCalibration();

  // Load specific entries (indexed by filename).
  bool getPoseAtEntry();
  bool getGpsAtEntry();
  bool getImuAtEntry();
  bool getPointcloudAtEntry();
  bool getImageAtEntry();

  bool getCameraCalibration();
  bool getCameraIntrinsics();
  bool getCameraExtrinsics();

  Transformation T_camN_vel(int cam_number) const;
  Transformation T_camN_imu(int cam_number) const;

  // Returns the nanosecond timestamp since epoch for a particular entry.
  // Returns -1 if no valid timestamp is found.
  int64_t getTimestampNsAtEntry(int64_t entry) const;

  // Basic accessors.
  Transformation T_cam0_vel() const;
  Transformation T_vel_imu() const;

 private:
  bool loadCamToCamCalibration();
  bool loadVelToCamCalibration();
  bool loadImuToVelCalibration();

  void convertGpsToPose();

  void loadImage();

  bool parseVectorOfDoubles(const std::string& input,
                            std::vector<double>* output);

  // Base paths.
  std::string calibration_path_;
  std::string dataset_path_;
  // Whether this dataset contains raw or rectified images. This determines
  // which calibration is read.
  bool rectified_;

  // Cached calibration parameters -- std::vector of camera calibrations.
  CameraCalibrationVector camera_calibrations_;

  // Transformation chain (cam-to-cam extrinsics stored above in cam calib
  // struct).
  Transformation T_cam0_vel_;
  Transformation T_vel_imu_;

  // Timestamp map from index to nanoseconds.
  std::map<uint64_t, uint64_t> timestamp_map_ns_;
};
}

#endif  // KITTI_TO_ROSBAG_KITTI_PARSER_H_
