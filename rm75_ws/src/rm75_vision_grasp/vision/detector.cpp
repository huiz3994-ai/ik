#include <opencv2/aruco.hpp>
#include <rm75_vision_grasp/vision.h>
namespace rm75 {
bool ColorDetector::detect(const cv::Mat &bgr, Detection &d) {
  cv::Mat hsv, a, b, mask;
  cv::cvtColor(bgr, hsv, cv::COLOR_BGR2HSV);
  cv::inRange(hsv, cv::Scalar(0, 110, 65), cv::Scalar(10, 255, 255), a);
  cv::inRange(hsv, cv::Scalar(170, 110, 65), cv::Scalar(180, 255, 255), b);
  mask = a | b;
  cv::morphologyEx(mask, mask, cv::MORPH_OPEN,
                   cv::getStructuringElement(cv::MORPH_ELLIPSE, {3, 3}));
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  double area = min_area;
  bool found = false;
  for (const auto &c : contours) {
    double v = cv::contourArea(c);
    if (v <= area)
      continue;
    auto m = cv::moments(c);
    if (m.m00 <= 0)
      continue;
    area = v;
    d.bbox = cv::boundingRect(c);
    d.pixel = {m.m10 / m.m00, m.m01 / m.m00};
    found = true;
  }
  d.name = "red_object";
  d.score = found ? std::min(1.0, area / 1000.0) : 0;
  return found;
}
bool AprilTagDetector::detect(const cv::Mat &bgr, Detection &d) {
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  cv::aruco::detectMarkers(
      bgr, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),
      corners, ids);
  for (size_t i = 0; i < ids.size(); ++i)
    if (ids[i] == target_id) {
      d.name = "tag_" + std::to_string(ids[i]);
      d.bbox = cv::boundingRect(corners[i]);
      d.pixel = {0, 0};
      for (auto p : corners[i]) {
        d.pixel.x += p.x / 4;
        d.pixel.y += p.y / 4;
      }
      d.score = 1;
      return true;
    }
  return false;
}
} // namespace rm75
