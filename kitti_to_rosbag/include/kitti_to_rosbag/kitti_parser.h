#ifndef KITTI_TO_ROSBAG_KITTI_PARSER_H_
#define KITTI_TO_ROSBAG_KITTI_PARSER_H_

#include <memory>
#include <map>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <kindr/minimal/quat-transformation.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>

namespace kitti {

// Transformation type for defining sensor orientation.
typedef kindr::minimal::QuatTransformation Transformation;
typedef kindr::minimal::RotationQuaternion Rotation;

struct CameraCalibration {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Intrinsics.
  Eigen::Vector2d image_size;                  // S_xx in calibration.
  Eigen::Matrix3d rect_mat;                    // R_rect_xx in calibration.
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
  static const std::string kVelodyneFolder;
  static const std::string kCameraFolder;
  static const std::string kPoseFolder;
  static const std::string kTimestampFilename;
  static const std::string kDataFolder;

  KittiParser(const std::string& calibration_path,
              const std::string& dataset_path, bool rectified);

  // MAIN API: all you should need to use!
  // Loading calibration files.
  bool loadCalibration();
  void loadTimestampMaps();

  // Load specific entries (indexed by filename).
  bool getPoseAtEntry(uint64_t entry, uint64_t* timestamp,
                      Transformation* pose);
  bool getGpsAtEntry() { /* TODO! */
    return false;
  }
  bool getImuAtEntry() { /* TODO! */
    return false;
  }
  bool getPointcloudAtEntry(uint64_t entry, uint64_t* timestamp,
                            pcl::PointCloud<pcl::PointXYZI>* ptcloud);
  bool getImageAtEntry();

  bool getCameraCalibration();
  bool getCameraIntrinsics();
  bool getCameraExtrinsics();

  Transformation T_camN_vel(int cam_number) const;
  Transformation T_camN_imu(int cam_number) const;

  // Returns the nanosecond timestamp since epoch for a particular entry.
  // Returns -1 if no valid timestamp is found.
  // int64_t getTimestampNsAtEntry(int64_t entry) const;

  // Basic accessors.
  Transformation T_cam0_vel() const;
  Transformation T_vel_imu() const;

 private:
  bool loadCamToCamCalibration();
  bool loadVelToCamCalibration();
  bool loadImuToVelCalibration();

  bool convertGpsToPose(const std::vector<double>& oxts, Transformation* pose);
  double latToScale(double lat) const;
  void latlonToMercator(double lat, double lon, double scale,
                        Eigen::Vector2d* mercator) const;
  void loadImage();

  bool loadTimestampsIntoVector(const std::string& filename,
                                std::vector<uint64_t>* timestamp_vec) const;

  bool parseVectorOfDoubles(const std::string& input,
                            std::vector<double>* output) const;

  std::string getFolderNameForCamera(int cam_number) const;
  std::string getFilenameForEntry(uint64_t entry) const;

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
  std::vector<uint64_t> timestamps_vel_ns_;
  std::vector<uint64_t> timestamps_pose_ns_;
  // Vector of camera timestamp vectors.
  std::vector<std::vector<uint64_t> > timestamps_cam_ns_;

  // Cached pose information, to correct to odometry frame (instead of absolute
  // world coordinates).
  bool initial_pose_set_;
  Transformation T_initial_pose_;
  double mercator_scale_;
};
}

#endif  // KITTI_TO_ROSBAG_KITTI_PARSER_H_
