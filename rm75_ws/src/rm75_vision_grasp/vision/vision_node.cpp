#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rm75_vision_grasp/vision.h>
#include <std_msgs/String.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
class VisionNode {
  ros::NodeHandle nh, p{"~"};
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener listener{buffer};
  message_filters::Subscriber<sensor_msgs::Image> rgb, depth;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info;
  using Policy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
  std::unique_ptr<message_filters::Synchronizer<Policy>> sync;
  std::unique_ptr<rm75::ObjectDetector> detector;
  ros::Publisher out, camera, detail;
  std::string base, cam, world;
  double max_age;

public:
  VisionNode() {
    std::string type;
    p.param<std::string>("detector", type, "color");
    p.param<std::string>("base_frame", base, "base_link");
    p.param<std::string>("camera_frame", cam, "camera_link");
    p.param<std::string>("world_frame", world, "world");
    p.param("max_age", max_age, 0.5);
    if (type == "color")
      detector.reset(new rm75::ColorDetector);
    else if (type == "apriltag") {
      auto *d = new rm75::AprilTagDetector;
      p.param("tag_id", d->target_id, 0);
      detector.reset(d);
    } else
      throw std::runtime_error("Unsupported detector; YOLO requires an "
                               "ObjectDetector implementation");
    out =
        nh.advertise<geometry_msgs::PoseStamped>("/perception/object_pose", 1);
    camera = nh.advertise<geometry_msgs::PoseStamped>(
        "/perception/object_pose_camera", 1);
    detail = nh.advertise<std_msgs::String>("/perception/detection", 1);
    rgb.subscribe(nh, "/camera/color/image_raw", 3);
    depth.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 3);
    info.subscribe(nh, "/camera/color/camera_info", 3);
    sync.reset(
        new message_filters::Synchronizer<Policy>(Policy(8), rgb, depth, info));
    sync->setMaxIntervalDuration(ros::Duration(0.05));
    sync->registerCallback(boost::bind(&VisionNode::cb, this, _1, _2, _3));
  }
  void cb(const sensor_msgs::ImageConstPtr &a,
          const sensor_msgs::ImageConstPtr &b,
          const sensor_msgs::CameraInfoConstPtr &k) {
    try {
      double age = (ros::Time::now() - a->header.stamp).toSec();
      if (age < 0 || age > max_age)
        return;
      if (a->width != b->width || a->height != b->height ||
          a->header.frame_id != k->header.frame_id) {
        ROS_ERROR_THROTTLE(
            2, "RGB/depth must be aligned and CameraInfo must match RGB");
        return;
      }
      rm75::Detection d;
      if (!detector->detect(cv_bridge::toCvShare(a, "bgr8")->image, d))
        return;
      geometry_msgs::PoseStamped optical, cp, bp;
      auto ci = *k;
      ci.header.stamp = a->header.stamp;
      if (!rm75::DepthToPose::project(cv_bridge::toCvShare(b)->image, ci, d,
                                      optical))
        return;
      buffer.lookupTransform(world, base, a->header.stamp, ros::Duration(0.1));
      cp = buffer.transform(optical, cam, ros::Duration(0.1));
      bp = buffer.transform(cp, base, ros::Duration(0.1));
      camera.publish(cp);
      out.publish(bp);
      std_msgs::String s;
      s.data = d.name + " bbox=" + std::to_string(d.bbox.x) + "," +
               std::to_string(d.bbox.y) + "," + std::to_string(d.bbox.width) +
               "," + std::to_string(d.bbox.height) +
               " pixel=" + std::to_string(d.pixel.x) + "," +
               std::to_string(d.pixel.y);
      detail.publish(s);
    } catch (const tf2::TransformException &e) {
      ROS_ERROR_THROTTLE(2, "TF_FAIL: %s", e.what());
    } catch (const cv_bridge::Exception &e) {
      ROS_ERROR_THROTTLE(2, "VISION_FAIL: %s", e.what());
    } catch (const cv::Exception &e) {
      ROS_ERROR_THROTTLE(2, "VISION_FAIL: %s", e.what());
    }
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "rm75_vision");
  try {
    VisionNode n;
    ros::spin();
  } catch (const std::exception &e) {
    ROS_FATAL("%s", e.what());
    return 1;
  }
  return 0;
}
