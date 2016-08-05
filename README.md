# kitti_to_rosbag
Dataset tools for working with the KITTI dataset raw data ( http://www.cvlibs.net/datasets/kitti/raw_data.php ) and converting it to a ROS bag. Also allows a library for direct access to poses, velodyne scans, and images. 

## Rosbag converter usage example
```
rosrun kitti_to_rosbag kitti_rosbag_converter calibration_path dataset_path output_path
```
(No trailing slashes).

```
rosrun kitti_to_rosbag kitti_rosbag_converter ~/data/kitti/2011_09_26 ~/data/kitti/2011_09_26/2011_09_26_drive_0035_sync ~/data/kitti/2011_09_26/2011_09_26_drive_0035_sync/testbag.bag
```

## Library API Example
```C++
#include <opencv2/highgui/highgui.hpp>

#include "kitti_to_rosbag/kitti_parser.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  
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

  return 0;
}
  
```
