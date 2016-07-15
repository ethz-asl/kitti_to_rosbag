#include <iostream>
#include <fstream>

#include "kitti_to_rosbag/kitti_parser.h"

namespace kitti {

const std::string KittiParser::kCamToVelCalibrationFilename =
    "calib_velo_to_cam.txt";
const std::string KittiParser::kCamToCamCalibrationFilename =
    "calib_cam_to_cam.txt";
const std::string KittiParser::kImuToVelCalibrationFilename =
    "calib_imu_to_velo.txt";

KittiParser::KittiParser(const std::string& calibration_path,
                         const std::string& dataset_path, bool rectified)
    : calibration_path_(calibration_path),
      dataset_path_(dataset_path),
      rectified_(rectified) {}

bool KittiParser::loadCalibration() {
  loadCamToVelCalibration();
  return true;
}

bool KittiParser::loadCamToVelCalibration() {
  std::string filename = calibration_path_ + "/" + kCamToVelCalibrationFilename;
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
        T_cam0_vel_.getRotation() = Rotation::fromApproximateRotationMatrix(R);
      }
    } else if (header.compare("T") == 0) {
      // Parse the translation matrix.
      if (parseVectorOfDoubles(data, &parsed_doubles)) {
        Eigen::Vector3d T(parsed_doubles.data());
        T_cam0_vel_.getPosition() = T;
      }
    }
  }
  // How do we return false?
  return true;
}

bool KittiParser::parseVectorOfDoubles(const std::string& input,
                                       std::vector<double>* output) {
  output->clear();
  // Parse the line as a stringstream for space-delimeted doubles.
  std::stringstream line_stream(input);
  if (line_stream.eof()) {
    return false;
  }

  while (!line_stream.eof()) {
    std::string element;
    std::getline(line_stream, element, ' ');
    if (element.size() == 0) {
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

}  // namespace kitty
