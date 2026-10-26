#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Joy.h>
#include <signal.h>
#include <atomic>
#include <map>
#include <cmath>
#include <cstdint>
#include <algorithm>
#include <vector>

/*********************************************
 *   ********  *******    ****    ********
 *      **     **        **   *      **
 *      **     *******     ***       **
 *      **     **        *  ***      **
 *      **     *******    ***        **
 * *******************************************/
/*补充说明：
0.本代码基于 test3_real_time_ik_solver.cpp，实现Touch + 视觉伺服共享控制，Touch 主导控制的同时加入视觉伺服辅助控制，追踪 aruco 标记——眼在手上。

1.目前实现：
—————————————————————————————————————————————————————————————————————————————————————————
     控制源     |    控制指令占比(pose.position + pose.orientation) 
---------------|------------------------------------------------------------------------
     启动参数   |  only_touch | aruco_for_pos | aruco_for_ori | 💫realsense_for_ori
---------------|------------------------------------------------------------------------
     Touch     | 100% + 100% |   0%  + 100%  |   100% +  0%   |    100% +  0% 
 aruco_ros(软件)|  0%  +  0%  |  100% +  0%   |    0% + 100%   |    0% + 100%
      合计      |                               100%
—————————————————————————————————————————————————————————————————————————————————————————
数据流：aruco_ros(processed_pose2 --> processed_aruco_marker_pose) 
        >> Touch(processed_pose --> target_pose_msg) 
          >> 机械臂
2.💫已实现realsense深度图提取视野内平面位姿，并驱动机械臂进行姿态对齐（三维视觉伺服）。

【本版新增调试话题】
/debug/ctrl_mode
/debug/ik_solved
/debug/ik_solve_time
/debug/processed_aruco_pose
/debug/processed_realsense_pose
/debug/anchor_pose
*/

//启动参数配置
std::atomic<int> ctrl_mode(1);
#define only_touch          1
#define aruco_for_pos       2
#define aruco_for_ori       3
#define realsense_for_ori   4 
#define mixed_pos           5 

// 全局变量存储目标位姿和接收标志
geometry_msgs::PoseStamped target_pose_msg;
bool new_target_pose_received = false;
bool Anchor_updated = false;
bool back_to_home = false;
volatile sig_atomic_t exit_flag = 0;
double K_touch_pos_FIXED = 0.001;
double K_touch_ori_FIXED = 1;
double K_aruco_pos_FIXED = 1;
double K_aruco_ori_FIXED = 1;
float x_Anchor_arm_received = 0.25;
float y_Anchor_arm_received = 0;
float z_Anchor_arm_received = 0.43;
float delta_x_received = 0;
float delta_y_received = 0;
float delta_z_received = 0;
int last_grey_state = 0;
int last_white_state = 0;
// ==== 共享控制参数 ====
bool ori_init_once = false;
bool aruco_data_active_flag = false;
double K_touch_pos_CTRL = 1;
double K_touch_ori_CTRL = 1;
float K_aruco_pos_CTRL = 1.0f;
double K_aruco_ori_CTRL = 1;
int temp_counter = 0;
// ==== 平滑控制变量 ====
geometry_msgs::PoseStamped current_pose;
const double SMOOTH_FACTOR_POS = 0.02;
const double SMOOTH_FACTOR_ORI = 0.01;
bool first_pose_received = false;
// ==== aruco marker 位姿处理 ====
geometry_msgs::PoseStamped processed_aruco_marker_pose;
geometry_msgs::PoseStamped processed_realsense_marker_pose;

// ========== 新增：用于存储最新 Touch（Geomagic）数据（由回调只做存储） ==========
std::atomic<bool> touch_msg_received(false);
geometry_msgs::PoseStamped latest_touch_msg;
std::atomic<bool> latest_touch_initialized(false);

// 将原来在回调内的 last_orientation/first_run 提升为全局变量以便在 compute 函数中使用
geometry_msgs::Quaternion last_orientation;
std::atomic<bool> last_orientation_inited(false);

// ROS 发布器（主循环中需要）
ros::Publisher joint_pub;
ros::Publisher target_pose_pub;
ros::Publisher k_pub;
ros::Publisher current_pose_pub;

// 新增 debug 发布器
ros::Publisher ctrl_mode_pub;
ros::Publisher ik_solved_pub;
ros::Publisher ik_solve_time_pub;
ros::Publisher processed_aruco_pose_pub;
ros::Publisher processed_realsense_pose_pub;
ros::Publisher anchor_pose_pub;

// 前向声明
void computeTargetPoseFromTouch();

void sigintHandler(int sig) {
    exit_flag = 1;
    ROS_WARN_STREAM("[SHUT DOWN] Captured exit signal, preparing...");
}

// 修改后的 poseCallback：只接收并保存最新 Touch 数据，不进行控制/IK
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    latest_touch_msg = *msg;
    touch_msg_received.store(true);
    latest_touch_initialized.store(true);

    if(!last_orientation_inited.load()){
        last_orientation.w = 1.0;
        last_orientation.x = last_orientation.y = last_orientation.z = 0.0;
        last_orientation_inited.store(true);
    }
}

void poseFROMarucoCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    geometry_msgs::PoseStamped processed_pose2 = *msg;

    /******************
    z1 机械臂    aruco    Touch
        x   ————  z  ————  -z
        y   ———— -x  ————  -x
        z   ———— -y  ————   y
    *******************/
    /******根据机械臂link00绝对空间映射******/
    processed_pose2.pose.position.x = (msg->pose.position.x-0.4)*1;
    processed_pose2.pose.position.y = (msg->pose.position.y)*1;
    processed_pose2.pose.position.z = (msg->pose.position.z-0.055)*1;

    // ========== 姿态映射部分 ==========
    static const tf2::Quaternion q_offset_inv(-0.5, 0.5, 0.5, 0.5);

    tf2::Quaternion q_label(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w);

    tf2::Quaternion q_corrected = q_offset_inv * q_label;
    q_corrected.normalize();

    processed_pose2.pose.orientation.x = q_corrected.z()*1;
    processed_pose2.pose.orientation.y = -q_corrected.x()*1;
    processed_pose2.pose.orientation.z = q_corrected.y()*1;
    processed_pose2.pose.orientation.w = q_corrected.w()*1;

    processed_aruco_marker_pose = processed_pose2;
    processed_aruco_marker_pose.header.stamp = ros::Time::now();
    processed_aruco_marker_pose.header.frame_id = "link00";
    processed_aruco_pose_pub.publish(processed_aruco_marker_pose);

    new_target_pose_received = true;
}

void poseFROMrealsenseCallback(const geometry_msgs::PoseStamped::ConstPtr &msg){
    geometry_msgs::PoseStamped processed_pose3 = *msg;

    /******************
    z1 机械臂    realsense-get    Touch         
        x   ———— -z         ————  -z    
        y   ———— -y         ————  -x      
        z   ———— -x         ————   y        
    *******************/
    /******根据机械臂link00绝对空间映射******/
    processed_pose3.pose.position.x = (msg->pose.position.x-0.4)*1;
    processed_pose3.pose.position.y = (msg->pose.position.y)*1;
    processed_pose3.pose.position.z = (msg->pose.position.z-0.055)*1;

    tf2::Quaternion q_msg;
    tf2::fromMsg(msg->pose.orientation, q_msg);

    tf2::Quaternion qy;
    qy.setRPY(0.0, M_PI_2, 0.0);
    tf2::Quaternion qz;
    qz.setRPY(0.0, 0.0, -M_PI);

    tf2::Quaternion q_total = qz * qy;
    tf2::Quaternion q_new = q_total * q_msg;
    q_new.normalize();
    q_new.setY(-q_new.getY());
    q_new.normalize();

    processed_pose3.pose.orientation = tf2::toMsg(q_new);

    processed_realsense_marker_pose = processed_pose3;
    processed_realsense_marker_pose.header.stamp = ros::Time::now();
    processed_realsense_marker_pose.header.frame_id = "link00";
    processed_realsense_pose_pub.publish(processed_realsense_marker_pose);

    new_target_pose_received = true;
}

void joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
    if(msg->buttons.size() < 2) {
        ROS_WARN_ONCE("Invalid buttons message size!");
        return;
    }

    int current_grey = msg->buttons[0];
    int current_white = msg->buttons[1];

    if(current_grey != last_grey_state) {
        if(current_grey == 1) {
            ROS_INFO_STREAM("Gray button PRESSED");
        } else {
            ROS_INFO_STREAM("Gray button RELEASED");
        }
        last_grey_state = current_grey;
    }

    if(current_white != last_white_state) {
        if(current_white == 1) {
            ROS_INFO_STREAM("White button PRESSED"); 
        } else {
            ROS_INFO_STREAM("White button RELEASED");
        }
        last_white_state = current_white;
    }
}

void AnchorboolCallback(const std_msgs::Bool::ConstPtr& msg) {
    Anchor_updated = msg->data;
}

void arucoflagboolCallback(const std_msgs::Bool::ConstPtr& msg) {
    aruco_data_active_flag = msg->data;
}

void BackToHomeboolCallback(const std_msgs::Bool::ConstPtr& msg) {
    back_to_home = msg->data;
    ROS_INFO_STREAM("[HOME]Received back to home cmd!");
}

void BackToZeroboolCallback(const std_msgs::Bool::ConstPtr& msg) {
    if(msg->data) exit_flag = 1;
    ROS_INFO_STREAM("[SHUT DOWN] Received back to zero cmd!");
}

void poseAnchorCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    if (Anchor_updated)
    {
        x_Anchor_arm_received = msg->pose.position.x;
        y_Anchor_arm_received = msg->pose.position.y;
        z_Anchor_arm_received = msg->pose.position.z;
    }
    Anchor_updated = false;
}

void poseIncrementCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->data.size() >= 3) {
        delta_x_received = msg->data[0];
        delta_y_received = msg->data[1];
        delta_z_received = msg->data[2];
    }
}

/**
 * computeTargetPoseFromTouch()
 * 把原来 poseCallback 中的“处理 Touch 生成 processed_pose -> target_pose_msg”的逻辑移到这里。
 * 该函数使用全局 latest_touch_msg、processed_aruco_marker_pose、processed_realsense_marker_pose 等输入，
 * 计算 processed_pose（与原逻辑完全一致），并把结果写到 target_pose_msg，同时设置 new_target_pose_received = true。
 */
void computeTargetPoseFromTouch() {
    if(!latest_touch_initialized.load()) {
        return;
    }

    geometry_msgs::PoseStamped processed_pose = latest_touch_msg;
    if(!last_orientation_inited.load()){
        last_orientation.w = 1.0;
        last_orientation.x = last_orientation.y = last_orientation.z = 0.0;
        last_orientation_inited.store(true);
    }

    int current_ctrl_mode = ctrl_mode.load();
    switch (current_ctrl_mode)
    {
    case only_touch:
        processed_pose.pose.position.x = x_Anchor_arm_received;
        processed_pose.pose.position.y = y_Anchor_arm_received;
        processed_pose.pose.position.z = z_Anchor_arm_received;
        processed_pose.pose.position.x += K_touch_pos_FIXED * (-delta_z_received);
        processed_pose.pose.position.y += K_touch_pos_FIXED * (-delta_x_received);
        processed_pose.pose.position.z += K_touch_pos_FIXED * (delta_y_received);

        if (last_white_state){
            processed_pose.pose.orientation.x = K_touch_ori_FIXED * (-latest_touch_msg.pose.orientation.z);
            processed_pose.pose.orientation.y = K_touch_ori_FIXED * (-latest_touch_msg.pose.orientation.x);
            processed_pose.pose.orientation.z = K_touch_ori_FIXED * (latest_touch_msg.pose.orientation.y);
            processed_pose.pose.orientation.w = K_touch_ori_FIXED * (latest_touch_msg.pose.orientation.w);
            last_orientation = processed_pose.pose.orientation;
        }
        else{
            processed_pose.pose.orientation = last_orientation;
        }
        break;

    case aruco_for_pos:
        processed_pose.pose.position.x = K_aruco_pos_FIXED * processed_aruco_marker_pose.pose.position.x;
        processed_pose.pose.position.y = K_aruco_pos_FIXED * processed_aruco_marker_pose.pose.position.y;
        processed_pose.pose.position.z = K_aruco_pos_FIXED * processed_aruco_marker_pose.pose.position.z;

        if (last_white_state){
            processed_pose.pose.orientation.x = K_touch_ori_FIXED * (-latest_touch_msg.pose.orientation.z);
            processed_pose.pose.orientation.y = K_touch_ori_FIXED * (-latest_touch_msg.pose.orientation.x);
            processed_pose.pose.orientation.z = K_touch_ori_FIXED * (latest_touch_msg.pose.orientation.y);
            processed_pose.pose.orientation.w = K_touch_ori_FIXED * (latest_touch_msg.pose.orientation.w);
            last_orientation = processed_pose.pose.orientation;
        }
        else{
            processed_pose.pose.orientation = last_orientation;
        }
        break;

    case aruco_for_ori:
        processed_pose.pose.position.x = x_Anchor_arm_received;
        processed_pose.pose.position.y = y_Anchor_arm_received;
        processed_pose.pose.position.z = z_Anchor_arm_received;
        processed_pose.pose.position.x += K_touch_pos_FIXED * (-delta_z_received);
        processed_pose.pose.position.y += K_touch_pos_FIXED * (-delta_x_received);
        processed_pose.pose.position.z += K_touch_pos_FIXED * (delta_y_received);

        if (last_white_state || ori_init_once){
            processed_pose.pose.orientation.x = K_aruco_ori_FIXED * processed_aruco_marker_pose.pose.orientation.x;
            processed_pose.pose.orientation.y = K_aruco_ori_FIXED * processed_aruco_marker_pose.pose.orientation.y;
            processed_pose.pose.orientation.z = K_aruco_ori_FIXED * processed_aruco_marker_pose.pose.orientation.z;
            processed_pose.pose.orientation.w = K_aruco_ori_FIXED * processed_aruco_marker_pose.pose.orientation.w;
            last_orientation = processed_pose.pose.orientation;
        }
        else{
            processed_pose.pose.orientation = last_orientation;
            ori_init_once = true;
        }
        break;

    case realsense_for_ori:
        processed_pose.pose.position.x = x_Anchor_arm_received;
        processed_pose.pose.position.y = y_Anchor_arm_received;
        processed_pose.pose.position.z = z_Anchor_arm_received;
        processed_pose.pose.position.x += K_touch_pos_FIXED * (-delta_z_received);
        processed_pose.pose.position.y += K_touch_pos_FIXED * (-delta_x_received);
        processed_pose.pose.position.z += K_touch_pos_FIXED * (delta_y_received);

        if (last_white_state || ori_init_once){
            processed_pose.pose.orientation.x = K_aruco_ori_FIXED * processed_realsense_marker_pose.pose.orientation.x;
            processed_pose.pose.orientation.y = K_aruco_ori_FIXED * processed_realsense_marker_pose.pose.orientation.y;
            processed_pose.pose.orientation.z = K_aruco_ori_FIXED * processed_realsense_marker_pose.pose.orientation.z;
            processed_pose.pose.orientation.w = K_aruco_ori_FIXED * processed_realsense_marker_pose.pose.orientation.w;
            last_orientation = processed_pose.pose.orientation;
        }
        else{
            processed_pose.pose.orientation = last_orientation;
            ori_init_once = true;
        }
        break;

    case mixed_pos:
    {
        processed_pose.pose.position.x = x_Anchor_arm_received;
        processed_pose.pose.position.y = y_Anchor_arm_received;
        processed_pose.pose.position.z = z_Anchor_arm_received;
        static float K_aruco_pos_CTRL_inner = 1.0f;
        const float target = aruco_data_active_flag ? 0.5f : 1.0f;
        const float smoothing = 0.995f;
        K_aruco_pos_CTRL_inner = smoothing * K_aruco_pos_CTRL_inner + (1 - smoothing) * target;
        K_aruco_pos_CTRL = K_aruco_pos_CTRL_inner;
        ROS_WARN_STREAM("K_aruco_pos_CTRL: " << K_aruco_pos_CTRL_inner);
        processed_pose.pose.position.x += K_aruco_pos_CTRL_inner * K_touch_pos_FIXED * (-delta_z_received);
        processed_pose.pose.position.y += K_aruco_pos_CTRL_inner * K_touch_pos_FIXED * (-delta_x_received);
        processed_pose.pose.position.z += K_aruco_pos_CTRL_inner * K_touch_pos_FIXED * (delta_y_received);

        if (last_white_state)
        {
            processed_pose.pose.orientation.x = K_touch_ori_FIXED * (-latest_touch_msg.pose.orientation.z);
            processed_pose.pose.orientation.y = K_touch_ori_FIXED * (-latest_touch_msg.pose.orientation.x);
            processed_pose.pose.orientation.z = K_touch_ori_FIXED * (latest_touch_msg.pose.orientation.y);
            processed_pose.pose.orientation.w = K_touch_ori_FIXED * (latest_touch_msg.pose.orientation.w);
            last_orientation = processed_pose.pose.orientation;
        }
        else
        {
            processed_pose.pose.orientation = last_orientation;
        }

        std_msgs::Float64 k_msg;
        k_msg.data = K_aruco_pos_CTRL_inner;
        k_pub.publish(k_msg);
        break;
    }

    default:
        ROS_ERROR_STREAM("[ERROR] Invalid control mode!");
        break;
    }

    if(back_to_home){
        processed_pose.pose.position.x = 0.25;
        processed_pose.pose.position.y = 0;
        processed_pose.pose.position.z = 0.43;
        last_orientation.w = 1.0;
        last_orientation.x = last_orientation.y = last_orientation.z = 0.0;
    }

    processed_pose.header.stamp = ros::Time::now();
    if (processed_pose.header.frame_id.empty()) {
        processed_pose.header.frame_id = "link00";
    }

    geometry_msgs::PoseStamped anchor_pose_msg;
    anchor_pose_msg.header.stamp = processed_pose.header.stamp;
    anchor_pose_msg.header.frame_id = "link00";
    anchor_pose_msg.pose.position.x = x_Anchor_arm_received;
    anchor_pose_msg.pose.position.y = y_Anchor_arm_received;
    anchor_pose_msg.pose.position.z = z_Anchor_arm_received;
    anchor_pose_msg.pose.orientation.w = 1.0;
    anchor_pose_pub.publish(anchor_pose_msg);

    target_pose_msg = processed_pose;
    new_target_pose_received = true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "real_time_mixed_ctrl_solver");

  ros::NodeHandle nh("~");
  std::map<std::string, std::int32_t> ctrl_mode_map = {
      {"default"          , only_touch},
      {"aruco_for_pos"    , aruco_for_pos},
      {"aruco_for_ori"    , aruco_for_ori},
      {"realsense_for_ori", realsense_for_ori},
      {"mixed_pos"        , mixed_pos}
  };

  std::string ctrl_mode_str;
  nh.param<std::string>("ctrl_mode", ctrl_mode_str, "default");

  auto it = ctrl_mode_map.find(ctrl_mode_str);
  if (it != ctrl_mode_map.end()) {
      ctrl_mode.store(it->second);
      ROS_INFO_STREAM("Control mode set to: " << ctrl_mode_str << " (" << it->second << ")");
  } else {
      ROS_WARN_STREAM("Invalid control mode parameter: " << ctrl_mode_str << ". Using default.");
      ctrl_mode.store(ctrl_mode_map["default"]);
  }

  signal(SIGINT, sigintHandler);
  signal(SIGTERM, sigintHandler);

  moveit::planning_interface::MoveGroupInterface move_group("manipulator");
  const std::string end_effector_link = move_group.getEndEffectorLink();
  const moveit::core::RobotModelConstPtr &kinematic_model = move_group.getRobotModel();
  const moveit::core::JointModelGroup *joint_model_group = kinematic_model->getJointModelGroup("manipulator");

  const double timeout = 0.1;
  const int attempts = 10;
  double solve_time = 0.0;
  ros::Rate rate(300);

  // ==== 先初始化发布器，再初始化订阅器，避免调试回调发布空句柄 ====
  joint_pub = nh.advertise<std_msgs::Float64MultiArray>("/goal_joint_positions", 10);
  target_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/ttttttttarget_pose", 10);
  k_pub = nh.advertise<std_msgs::Float64>("/debug/k_aruco_pos_ctrl", 10);
  current_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/debug/current_pose", 10);

  ctrl_mode_pub = nh.advertise<std_msgs::Int32>("/debug/ctrl_mode", 10, true);
  ik_solved_pub = nh.advertise<std_msgs::Bool>("/debug/ik_solved", 10);
  ik_solve_time_pub = nh.advertise<std_msgs::Float64>("/debug/ik_solve_time", 10);
  processed_aruco_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/debug/processed_aruco_pose", 10);
  processed_realsense_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/debug/processed_realsense_pose", 10);
  anchor_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/debug/anchor_pose", 10);

  ros::Subscriber pose_sub = nh.subscribe("/Geomagic/pose", 10, poseCallback);
  ros::Subscriber pose_from_aruco_sub = nh.subscribe("/aruco_single/pose", 10, poseFROMarucoCallback);
  ros::Subscriber flag_from_aruco_sub = nh.subscribe("/aruco_single/aruco_detected_flag", 1, arucoflagboolCallback);
  ros::Subscriber pose_from_realsense_sub = nh.subscribe("/detected_plane_pose_link00", 10, poseFROMrealsenseCallback);
  ros::Subscriber sub = nh.subscribe("/Touch_Anchor_updated", 1, AnchorboolCallback);
  ros::Subscriber back_to_home_sub = nh.subscribe("/back_to_home_arm", 1, BackToHomeboolCallback);
  ros::Subscriber back_to_zero_sub = nh.subscribe("/back_to_zero_arm", 1, BackToZeroboolCallback);
  ros::Subscriber pose_Anchor_sub = nh.subscribe("/end_effector_pose", 1, poseAnchorCallback);
  ros::Subscriber increment_sub = nh.subscribe("/Touch_Pose_increment", 10, poseIncrementCallback);
  ros::Subscriber sub_Touch_button = nh.subscribe("/Geomagic/joy", 1, joyCallback);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::Time last_ctrl_mode_pub_time = ros::Time(0);

  while (ros::ok() && !exit_flag)
  {
    if (touch_msg_received.load()) {
        computeTargetPoseFromTouch();
        touch_msg_received.store(false);
    }

    if ((ros::Time::now() - last_ctrl_mode_pub_time).toSec() > 0.1) {
        std_msgs::Int32 mode_msg;
        mode_msg.data = ctrl_mode.load();
        ctrl_mode_pub.publish(mode_msg);
        last_ctrl_mode_pub_time = ros::Time::now();
    }

    if (new_target_pose_received)
    {
      if (!first_pose_received)
      {
        current_pose = target_pose_msg;
        first_pose_received = true;
      }

      target_pose_pub.publish(target_pose_msg);

      current_pose.pose.position.x += SMOOTH_FACTOR_POS *
                                      (target_pose_msg.pose.position.x - current_pose.pose.position.x);
      current_pose.pose.position.y += SMOOTH_FACTOR_POS *
                                      (target_pose_msg.pose.position.y - current_pose.pose.position.y);
      current_pose.pose.position.z += SMOOTH_FACTOR_POS *
                                      (target_pose_msg.pose.position.z - current_pose.pose.position.z);

      tf2::Quaternion q_current, q_target, q_interp;
      tf2::fromMsg(current_pose.pose.orientation, q_current);
      tf2::fromMsg(target_pose_msg.pose.orientation, q_target);
      q_interp = tf2::slerp(q_current, q_target, SMOOTH_FACTOR_ORI);
      q_interp.normalize();
      current_pose.pose.orientation = tf2::toMsg(q_interp);
      current_pose.header = target_pose_msg.header;

      current_pose_pub.publish(current_pose);

      Eigen::Isometry3d target_pose;
      tf2::fromMsg(current_pose.pose, target_pose);

      moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
      moveit::core::RobotState robot_state(*current_state);

      ros::Time start_time = ros::Time::now();

      bool ik_solved = robot_state.setFromIK(
          joint_model_group,
          target_pose,
          end_effector_link,
          attempts,
          timeout);

      ros::Time end_time = ros::Time::now();
      double time_diff = (end_time - start_time).toSec();
      solve_time = time_diff;

      std_msgs::Bool ik_ok_msg;
      ik_ok_msg.data = ik_solved;
      ik_solved_pub.publish(ik_ok_msg);

      std_msgs::Float64 ik_time_msg;
      ik_time_msg.data = solve_time;
      ik_solve_time_pub.publish(ik_time_msg);

      if (ik_solved)
      {
        std::vector<double> joint_values;
        robot_state.copyJointGroupPositions(joint_model_group, joint_values);

        static std::vector<double> last_joint_values(joint_values.size(), 0.0);
        for (size_t i = 0; i < joint_values.size(); ++i)
        {
          joint_values[i] = last_joint_values[i] + 0.5 *
                                                       (joint_values[i] - last_joint_values[i]);
        }
        last_joint_values = joint_values;

        std_msgs::Float64MultiArray joint_positions_msg;
        joint_positions_msg.data = joint_values;
        joint_pub.publish(joint_positions_msg);
      }
      else
      {
        ROS_WARN_STREAM("[WARNING]IK solve failed!");
      }
      new_target_pose_received = false;
    }
    ros::spinOnce();
    rate.sleep();
  }

  if (exit_flag)
  {
    // ROS_WARN_STREAM("[SHUT DOWN] Sending back to zero cmd...");

    // std_msgs::Float64MultiArray joint_positions_msg;
    // joint_positions_msg.data = {0.0, 0.0, -0.005, -0.074, 0.0, 0.0};

    // for (int i = 0; i < 3; ++i)
    // {
    //   joint_pub.publish(joint_positions_msg);
    //   ros::spinOnce();
    //   ros::Duration(0.05).sleep();
    // }

    // ROS_WARN_STREAM("[SHUT DOWN] Cmd sent successfully, shutting down the node...");
    // ros::Duration(0.5).sleep();
  }

  return 0;
}
