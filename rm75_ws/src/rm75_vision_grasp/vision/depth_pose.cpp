#include <algorithm>
#include <cmath>
#include <rm75_vision_grasp/vision.h>
namespace rm75 {
bool DepthToPose::project(const cv::Mat &depth,
                          const sensor_msgs::CameraInfo &k, const Detection &d,
                          geometry_msgs::PoseStamped &pose, double lo,
                          double hi) {
  if (depth.empty() || (depth.type() != CV_16UC1 && depth.type() != CV_32FC1) ||
      !std::isfinite(k.K[0]) || !std::isfinite(k.K[4]) || k.K[0] <= 0 ||
      k.K[4] <= 0 || depth.cols != int(k.width) || depth.rows != int(k.height))
    return false;
  if (!std::isfinite(d.pixel.x) || !std::isfinite(d.pixel.y) ||
      !std::isfinite(k.K[2]) || !std::isfinite(k.K[5]) ||
      d.pixel.x<0 || d.pixel.y<0 || d.pixel.x>=depth.cols || d.pixel.y>=depth.rows ||
      lo<=0 || hi<=lo) return false;
  int u = std::lround(d.pixel.x), v = std::lround(d.pixel.y);
  if (u < 0 || v < 0 || u >= depth.cols || v >= depth.rows)
    return false;
  std::vector<double> zs;
  for (int y = std::max(0, v - 2); y <= std::min(depth.rows - 1, v + 2); ++y)
    for (int x = std::max(0, u - 2); x <= std::min(depth.cols - 1, u + 2);
         ++x) {
      double z = depth.type() == CV_16UC1 ? depth.at<uint16_t>(y, x) * 0.001
                                          : depth.at<float>(y, x);
      if (std::isfinite(z) && z >= lo && z <= hi)
        zs.push_back(z);
    }
  if (zs.size() < 5)
    return false;
  std::sort(zs.begin(), zs.end());
  double z = zs[zs.size() / 2];
  if (zs[zs.size() * 3 / 4] - zs[zs.size() / 4] > 0.04)
    return false;
  pose.header = k.header;
  double x=(d.pixel.x-k.K[2])/k.K[0], y=(d.pixel.y-k.K[5])/k.K[4];
  bool distorted=false;
  for(double coefficient:k.D) {
    if(!std::isfinite(coefficient)) return false;
    distorted=distorted || std::abs(coefficient)>1e-12;
  }
  if(distorted) {
    if(k.distortion_model!="plumb_bob" && k.distortion_model!="rational_polynomial") return false;
    if(k.D.size()!=4 && k.D.size()!=5 && k.D.size()!=8) return false;
    cv::Mat intrinsic=cv::Mat::eye(3,3,CV_64F);
    intrinsic.at<double>(0,0)=k.K[0];intrinsic.at<double>(1,1)=k.K[4];
    intrinsic.at<double>(0,2)=k.K[2];intrinsic.at<double>(1,2)=k.K[5];
    std::vector<cv::Point2d> pixels{d.pixel}, normalized;
    cv::undistortPoints(pixels,normalized,intrinsic,k.D);
    x=normalized[0].x;y=normalized[0].y;
    if(!std::isfinite(x)||!std::isfinite(y)) return false;
  }
  pose.pose.position.x = x*z;
  pose.pose.position.y = y*z;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1;
  return true;
}
} // namespace rm75
