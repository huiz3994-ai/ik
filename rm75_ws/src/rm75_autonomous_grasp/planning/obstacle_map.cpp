#include <Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Header.h>
#include <tf2_ros/transform_listener.h>
class ObstacleMap {
  ros::NodeHandle nh, p{"~"};
  tf2_ros::Buffer tf;
  tf2_ros::TransformListener listener{tf};
  ros::Subscriber sub, obj;
  ros::Publisher pub, alive;
  geometry_msgs::PoseStamped target;
  double leaf, mask, age;
  std::string frame;
  bool mask_target;
  std::mutex mutex;

public:
  ObstacleMap() {
    p.param("voxel_size", leaf, 0.015);
    p.param("target_mask_radius", mask, 0.045);
    p.param("max_age", age, 0.5);
    p.param("mask_target", mask_target, false);
    p.param<std::string>("map_frame", frame, "base_link");
    if (leaf <= 0 || mask < 0)
      throw std::runtime_error("Invalid voxel/mask size");
    pub = nh.advertise<sensor_msgs::PointCloud2>("/mapping/cloud", 1);
    alive = nh.advertise<std_msgs::Header>("/mapping/processed", 1);
    obj = nh.subscribe<geometry_msgs::PoseStamped>(
        "/perception/object_pose", 1,
        [this](const geometry_msgs::PoseStampedConstPtr &m) {
          std::lock_guard<std::mutex> l(mutex);
          target = *m;
        });
    sub = nh.subscribe("/camera/depth/color/points", 1, &ObstacleMap::cb, this);
  }
  void cb(const sensor_msgs::PointCloud2ConstPtr &m) {
    double dt = (ros::Time::now() - m->header.stamp).toSec();
    if (dt < 0 || dt > age) {
      ROS_WARN_THROTTLE(2, "Stale obstacle cloud");
      return;
    }
    try {
      auto transform = tf.lookupTransform(frame, m->header.frame_id,
                                          m->header.stamp, ros::Duration(0.1));
      auto &r = transform.transform.rotation;
      auto &t = transform.transform.translation;
      Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
      matrix.block<3, 3>(0, 0) =
          Eigen::Quaternionf(r.w, r.x, r.y, r.z).toRotationMatrix();
      matrix(0, 3) = t.x;
      matrix(1, 3) = t.y;
      matrix(2, 3) = t.z;
      pcl::PointCloud<pcl::PointXYZ>::Ptr input(
          new pcl::PointCloud<pcl::PointXYZ>),
          roi(new pcl::PointCloud<pcl::PointXYZ>);
      pcl::fromROSMsg(*m, *input);
      pcl::transformPointCloud(*input, *input, matrix);
      geometry_msgs::PoseStamped object;
      {
        std::lock_guard<std::mutex> l(mutex);
        object = target;
      }
      bool exclude = mask_target && object.header.frame_id == frame &&
                     (ros::Time::now() - object.header.stamp).toSec() < 1;
      for (const auto &v : *input) {
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
            std::abs(v.x) > 1.3 || std::abs(v.y) > 1.3 || v.z < -.1 ||
            v.z > 1.7)
          continue;
        // Target is represented separately by CollisionObject; this exclusion
        // is bounded.
        if (exclude && std::abs(v.x - object.pose.position.x) < mask &&
            std::abs(v.y - object.pose.position.y) < mask &&
            std::abs(v.z - object.pose.position.z) < mask)
          continue;
        roi->push_back(v);
      }
      if (roi->empty())
        return;
      pcl::VoxelGrid<pcl::PointXYZ> voxel;
      voxel.setInputCloud(roi);
      voxel.setLeafSize(leaf, leaf, leaf);
      pcl::PointCloud<pcl::PointXYZ> filtered;
      voxel.filter(filtered);
      pcl::transformPointCloud(filtered, filtered, matrix.inverse().eval());
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(filtered, output);
      output.header = m->header;
      pub.publish(output);
      alive.publish(output.header);
    } catch (const tf2::TransformException &e) {
      ROS_ERROR_THROTTLE(2, "TF_FAIL mapping: %s", e.what());
    }
  }
};
int main(int argc, char **argv) {
  ros::init(argc, argv, "obstacle_map");
  ObstacleMap n;
  ros::spin();
}
