#include <iostream>
#include <fstream>
#include <iomanip>

#include "kitti_to_rosbag/kitti_parser.h"

namespace kitti {

const std::string KittiParser::kVelToCamCalibrationFilename =
    "calib_velo_to_cam.txt";
const std::string KittiParser::kCamToCamCalibrationFilename =
    "calib_cam_to_cam.txt";
const std::string KittiParser::kImuToVelCalibrationFilename =
    "calib_imu_to_velo.txt";

const std::string KittiParser::kVelodyneFolder = "velodyne_points";
const std::string KittiParser::kCameraFolder = "image_";
const std::string KittiParser::kPoseFolder = "oxts";

const std::string KittiParser::kTimestampFilename = "timestamps.txt";
const std::string KittiParser::kDataFolder = "data";

KittiParser::KittiParser(const std::string& calibration_path,
                         const std::string& dataset_path, bool rectified)
    : calibration_path_(calibration_path),
      dataset_path_(dataset_path),
      rectified_(rectified) {}

bool KittiParser::loadCalibration() {
  loadVelToCamCalibration();
  loadImuToVelCalibration();
  loadCamToCamCalibration();
  return true;
}

bool KittiParser::loadVelToCamCalibration() {
  std::string filename = calibration_path_ + "/" + kVelToCamCalibrationFilename;
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    // Check what the header is. Each line consists of two parts:
    // a header followed by a ':' followed by space-separated data.
    std::string header;
    std::getline(line_stream, header, ':');
    std::string data;
    std::getline(line_stream, data, ':');

    std::vector<double> parsed_doubles;
    // Compare all header possibilities...
    if (header.compare("R") == 0) {
      // Parse the rotation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Matrix3d R(parsed_doubles.data());
        // All matrices are written row-major but Eigen is column-major
        // (can swap this, but I guess it's anyway easier to just transpose
        // for these small matrices).
        T_cam0_vel_.getRotation() =
            Rotation::fromApproximateRotationMatrix(R.transpose());
      }
    } else if (header.compare("T") == 0) {
      // Parse the translation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Vector3d T(parsed_doubles.data());
        T_cam0_vel_.getPosition() = T;
      }
    }
  }
  std::cout << "T_cam0_vel:\n" << T_cam0_vel_ << std::endl;
  // How do we return false?
  return true;
}

bool KittiParser::loadImuToVelCalibration() {
  std::string filename = calibration_path_ + "/" + kImuToVelCalibrationFilename;
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    // Check what the header is. Each line consists of two parts:
    // a header followed by a ':' followed by space-separated data.
    std::string header;
    std::getline(line_stream, header, ':');
    std::string data;
    std::getline(line_stream, data, ':');

    std::vector<double> parsed_doubles;
    // Compare all header possibilities...
    if (header.compare("R") == 0) {
      // Parse the rotation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Matrix3d R(parsed_doubles.data());
        // All matrices are written row-major but Eigen is column-major
        // (can swap this, but I guess it's anyway easier to just transpose
        // for these small matrices).
        T_vel_imu_.getRotation() =
            Rotation::fromApproximateRotationMatrix(R.transpose());
      }
    } else if (header.compare("T") == 0) {
      // Parse the translation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Vector3d T(parsed_doubles.data());
        T_vel_imu_.getPosition() = T;
      }
    }
  }
  std::cout << "T_vel_imu:\n" << T_vel_imu_ << std::endl;
  return true;
}

bool KittiParser::loadCamToCamCalibration() {
  std::string filename = calibration_path_ + "/" + kCamToCamCalibrationFilename;
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    // Check what the header is. Each line consists of two parts:
    // a header followed by a ':' followed by space-separated data.
    std::string header;
    std::getline(line_stream, header, ':');
    std::string data;
    std::getline(line_stream, data, ':');

    std::vector<double> parsed_doubles;
    // Compare all header possibilities...

    std::cout << "Header: " << header << "\n";

    // Load image size.
    if (header.compare(0, 6, "S_rect") == 0) {
      std::cout << "S_rect header: " << header
                << " substring: " << header.substr(7) << std::endl;
      if (rectified_) {
        // Figure out which number this is.
        int index = std::stoi(header.substr(7));
        std::cout << "Index: " << index << std::endl;
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          camera_calibrations_[index].image_size =
              Eigen::Vector2d(parsed_doubles.data());
          std::cout << "Image size: " << camera_calibrations_[index].image_size
                    << std::endl;
        }
      }
    } else if (!rectified_ && header.compare(0, 2, "S_") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        camera_calibrations_[index].image_size =
            Eigen::Vector2d(parsed_doubles.data());
      }
    }

    // Load rectification matrix.
    if (header.compare(0, 6, "R_rect") == 0) {
      std::cout << "R_rect header: " << header
                << " substring: " << header.substr(7) << std::endl;
      if (rectified_) {
        // Figure out which number this is.
        int index = std::stoi(header.substr(7));
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          camera_calibrations_[index].rect_mat =
              Eigen::Matrix3d(parsed_doubles.data()).transpose();
        }
      }
      continue;
    } /* else if (!rectified_ && header.compare(0, 2, "R_") == 0) {
      int index = std::stoi(header.substr(3));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        camera_calibrations_[index].rect_mat =
            Eigen::Matrix3d(parsed_doubles.data()).transpose();
      }
    } */

    // Projection mat.
    if (header.compare(0, 6, "P_rect") == 0) {
      std::cout << "P_rect header: " << header
                << " substring: " << header.substr(7) << std::endl;
      if (rectified_) {
        // Figure out which number this is.
        int index = std::stoi(header.substr(7));
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          camera_calibrations_[index].projection_mat =
              Eigen::Matrix<double, 4, 3>(parsed_doubles.data()).transpose();
        }
      }
      continue;
    } else if (!rectified_ && header.compare(0, 2, "P_") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        camera_calibrations_[index].projection_mat =
            Eigen::Matrix<double, 4, 3>(parsed_doubles.data()).transpose();
      }
      continue;
    }

    // If not rectified, try to load K and D.
    if (!rectified_) {
      if (header.compare(0, 1, "K") == 0) {
        int index = std::stoi(header.substr(2));
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        // Parse the rotation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Matrix3d K(parsed_doubles.data());
          // All matrices are written row-major but Eigen is column-major
          // (can swap this, but I guess it's anyway easier to just transpose
          // for these small matrices).
          camera_calibrations_[index].K = K.transpose();
        }
        continue;
      } else if (header.compare(0, 1, "D") == 0) {
        int index = std::stoi(header.substr(2));
        if (camera_calibrations_.size() <= index) {
          camera_calibrations_.resize(index + 1);
        }
        // Parse the translation matrix.
        if (parseVectorOfDoubles(data, &parsed_doubles)) {
          Eigen::Matrix<double, 1, 5> D(parsed_doubles.data());
          camera_calibrations_[index].D = D;
        }
        continue;
      }
    }

    if (header.compare(0, 1, "R") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      // Parse the rotation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Matrix3d R(parsed_doubles.data());
        // All matrices are written row-major but Eigen is column-major
        // (can swap this, but I guess it's anyway easier to just transpose
        // for these small matrices).
        camera_calibrations_[index].R = R.transpose();
      }
      continue;
    } else if (header.compare(0, 1, "T") == 0) {
      int index = std::stoi(header.substr(2));
      if (camera_calibrations_.size() <= index) {
        camera_calibrations_.resize(index + 1);
      }
      // Parse the translation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Vector3d T(parsed_doubles.data());
        camera_calibrations_[index].T = T;
      }
      continue;
    }
  }
  // std::cout << "T_vel_imu:\n" << T_vel_imu_ << std::endl;
  return true;
}

bool KittiParser::parseVectorOfDoubles(const std::string& input,
                                       std::vector<double>* output) const {
  output->clear();
  // Parse the line as a stringstream for space-delimeted doubles.
  std::stringstream line_stream(input);
  if (line_stream.eof()) {
    return false;
  }

  while (!line_stream.eof()) {
    std::string element;
    std::getline(line_stream, element, ' ');
    if (element.empty()) {
      continue;
    }
    try {
      output->emplace_back(std::stod(element));
    } catch (const std::exception& exception) {
      std::cout << "Could not parse number in import file.\n";
      return false;
    }
  }
  return true;
}

void KittiParser::loadTimestampMaps() {
  // Load timestamps for poses.
  std::string filename =
      dataset_path_ + "/" + kPoseFolder + "/" + kTimestampFilename;
  loadTimestampsIntoVector(filename, &timestamps_pose_ns_);

  // Velodyne.
  filename = dataset_path_ + "/" + kVelodyneFolder + "/" + kTimestampFilename;
  loadTimestampsIntoVector(filename, &timestamps_vel_ns_);

  // One per camera.
  timestamps_cam_ns_.resize(camera_calibrations_.size());
  for (int i = 0; i < camera_calibrations_.size(); ++i) {
    char buffer[8];
    sprintf(buffer, "%02d", i);

    filename =
        dataset_path_ + "/" + kCameraFolder + buffer + "/" + kTimestampFilename;
    loadTimestampsIntoVector(filename, &timestamps_cam_ns_[i]);
  }
}

bool KittiParser::loadTimestampsIntoVector(
    const std::string& filename, std::vector<uint64_t>* timestamp_vec) const {
  std::ifstream import_file(filename, std::ios::in);
  if (!import_file) {
    return false;
  }

  timestamp_vec->clear();
  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    std::string timestamp_string = line_stream.str();
    std::tm t = {};
    line_stream >> std::get_time(&t, "%Y-%m-%d %H:%M:%S");

    static const uint64_t kSecondsToNanoSeconds = 1e9;
    time_t time_since_epoch = mktime(&t);

    uint64_t timestamp = time_since_epoch * kSecondsToNanoSeconds +
                         std::stoi(timestamp_string.substr(20, 8));
    timestamp_vec->push_back(timestamp);
  }

  std::cout << "Timestamps: " << std::endl
            << timestamp_vec->front() << " " << timestamp_vec->back()
            << std::endl;

  return true;
}

}  // namespace kitty
