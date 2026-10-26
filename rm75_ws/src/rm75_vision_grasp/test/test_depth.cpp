#include <gtest/gtest.h>
#include <opencv2/aruco.hpp>
#include <rm75_vision_grasp/vision.h>
TEST(Depth, MillimeterMeterEquivalence) {
  sensor_msgs::CameraInfo k;
  k.width = 20;
  k.height = 20;
  k.K[0] = k.K[4] = 100;
  k.K[2] = k.K[5] = 10;
  rm75::Detection d;
  d.pixel = {15, 10};
  geometry_msgs::PoseStamped a, b;
  cv::Mat mm(20, 20, CV_16UC1, cv::Scalar(500)),
      m(20, 20, CV_32FC1, cv::Scalar(.5));
  ASSERT_TRUE(rm75::DepthToPose::project(mm, k, d, a));
  ASSERT_TRUE(rm75::DepthToPose::project(m, k, d, b));
  EXPECT_NEAR(a.pose.position.x, .025, 1e-8);
  EXPECT_NEAR(a.pose.position.z, b.pose.position.z, 1e-8);
}
TEST(Depth, RejectInvalid) {
  sensor_msgs::CameraInfo k;
  k.width = k.height = 20;
  k.K[0] = k.K[4] = 100;
  rm75::Detection d;
  d.pixel = {10, 10};
  geometry_msgs::PoseStamped a;
  cv::Mat m(20, 20, CV_32FC1, cv::Scalar(NAN));
  EXPECT_FALSE(rm75::DepthToPose::project(m, k, d, a));
  m.setTo(0);
  EXPECT_FALSE(rm75::DepthToPose::project(m, k, d, a));
  m.setTo(.5);
  k.K[0] = 0;
  EXPECT_FALSE(rm75::DepthToPose::project(m, k, d, a));
}
TEST(Detector, RedCentroid) {
  cv::Mat image(100, 100, CV_8UC3, cv::Scalar(0, 0, 0));
  cv::rectangle(image, {30, 40, 30, 20}, cv::Scalar(0, 0, 255), -1);
  rm75::ColorDetector detector;
  rm75::Detection d;
  ASSERT_TRUE(detector.detect(image, d));
  EXPECT_NEAR(d.pixel.x, 44.5, 1);
  EXPECT_NEAR(d.pixel.y, 49.5, 1);
}
int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Depth,RejectPixelAndIntrinsicsNaN) {
  sensor_msgs::CameraInfo k; k.width=k.height=20; k.K[0]=k.K[4]=100;
  rm75::Detection d; d.pixel={NAN,10}; geometry_msgs::PoseStamped p;
  cv::Mat depth(20,20,CV_32FC1,cv::Scalar(0.5));
  EXPECT_FALSE(rm75::DepthToPose::project(depth,k,d,p));
  d.pixel={10,10}; k.K[2]=NAN;
  EXPECT_FALSE(rm75::DepthToPose::project(depth,k,d,p));
}

TEST(Detector,AprilTagIdentity) {
  cv::Mat marker, gray(240,240,CV_8UC1,cv::Scalar(255)), bgr;
  cv::aruco::drawMarker(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11),7,160,marker);
  marker.copyTo(gray(cv::Rect(40,40,160,160)));
  cv::cvtColor(gray,bgr,cv::COLOR_GRAY2BGR);
  rm75::AprilTagDetector detector; detector.target_id=7; rm75::Detection d;
  ASSERT_TRUE(detector.detect(bgr,d)); EXPECT_EQ(d.name,"tag_7");
  EXPECT_NEAR(d.pixel.x,119.5,1); detector.target_id=8;
  EXPECT_FALSE(detector.detect(bgr,d));
}

TEST(Depth,CompensateRawColorDistortion) {
  sensor_msgs::CameraInfo k;k.width=k.height=100;k.K[0]=k.K[4]=100;
  k.K[2]=k.K[5]=50;k.distortion_model="plumb_bob";k.D={0.1,0,0,0,0};
  cv::Mat intrinsic=(cv::Mat_<double>(3,3)<<100,0,50,0,100,50,0,0,1);
  std::vector<cv::Point3d> points{{.1,.05,.5}};std::vector<cv::Point2d> pixels;
  cv::projectPoints(points,cv::Vec3d(0,0,0),cv::Vec3d(0,0,0),intrinsic,k.D,pixels);
  rm75::Detection d;d.pixel=pixels[0];geometry_msgs::PoseStamped p;
  cv::Mat depth(100,100,CV_32FC1,cv::Scalar(.5));
  ASSERT_TRUE(rm75::DepthToPose::project(depth,k,d,p));
  EXPECT_NEAR(p.pose.position.x,.1,1e-6);EXPECT_NEAR(p.pose.position.y,.05,1e-6);
}
