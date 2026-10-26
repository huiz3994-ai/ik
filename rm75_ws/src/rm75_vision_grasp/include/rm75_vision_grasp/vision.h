#pragma once
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
namespace rm75 {
struct Detection {
  std::string name;
  cv::Rect bbox;
  cv::Point2d pixel;
  double score = 0;
};
class ObjectDetector {
public:
  virtual ~ObjectDetector() = default;
  virtual bool detect(const cv::Mat &, Detection &) = 0;
};
class ColorDetector final : public ObjectDetector {
public:
  bool detect(const cv::Mat &, Detection &) override;
  double min_area = 80;
};
// AprilTag 36h11 via OpenCV ArUco. YOLO implementations implement
// ObjectDetector.
class AprilTagDetector final : public ObjectDetector {
public:
  bool detect(const cv::Mat &, Detection &) override;
  int target_id = 0;
};
class DepthToPose {
public:
  static bool project(const cv::Mat &, const sensor_msgs::CameraInfo &,
                      const Detection &, geometry_msgs::PoseStamped &,
                      double min_depth = 0.15, double max_depth = 2.0);
};
} // namespace rm75
