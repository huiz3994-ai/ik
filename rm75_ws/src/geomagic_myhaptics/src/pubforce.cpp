/*
新增：
/Touch_Orient_Anchor    ——> 选定的Touch锚定末端姿态四元数[x,y,z,w]
/Touch_Orient_increment ——> Touch实时姿态与锚定姿态的相对旋转（四元数差值）[x,y,z,w]
📅2025.9.4 张耀华
*/

#include "geomagic_myhaptics/pubforce.h"
#include <cmath>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;

// 全局状态
bool flag_whitebutton = false;
bool flag_greybutton = false;
bool prev_greybutton_state = false;
bool prev_whitebutton_state = false;
bool new_anchor_updated = false;
bool new_ori_anchor_updated = false;
bool have_pose = false;
int touch_state = RELEASE;

geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_pose_Anchor;

// 位置锚点
float x_Anchor = 0;
float y_Anchor = 0;
float z_Anchor = 0;
float delta_x = 0;
float delta_y = 0;
float delta_z = 0;

// 姿态锚点
geometry_msgs::Quaternion orient_Anchor = [] {
    geometry_msgs::Quaternion q;
    q.w = 1.0;
    return q;
}();
geometry_msgs::Quaternion delta_orient = [] {
    geometry_msgs::Quaternion q;
    q.w = 1.0;
    return q;
}();

// 优化专用常量
const vector<uint8_t> LOCKED_VEC = {1, 1, 1};
const vector<uint8_t> UNLOCKED_VEC = {0, 0, 0};
ros::Publisher pub_force;
geometry_msgs::Vector3 latest_haptic_command;
ros::WallTime last_haptic_command_time;
bool have_haptic_command = false;
double haptic_command_timeout = 0.1;

// 内部函数声明
namespace {
void handleDualButtonPress();
void handleGreyButtonPress();
void handleWhiteButtonPress();
void applyForceFeedback(geomagic_control::DeviceFeedback& fb);
void hapticCommandCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg);
void publishHapticCommand(bool force_zero = false);

// 工具函数：四元数差值 q_delta = q_anchor⁻¹ * q_current
geometry_msgs::Quaternion quatDiff(
    const geometry_msgs::Quaternion& q_anchor,
    const geometry_msgs::Quaternion& q_current)
{
    tf2::Quaternion qa(q_anchor.x, q_anchor.y, q_anchor.z, q_anchor.w);
    tf2::Quaternion qc(q_current.x, q_current.y, q_current.z, q_current.w);
    qa.normalize();
    qc.normalize();
    tf2::Quaternion qd = qa.inverse() * qc;
    qd.normalize();

    // q and -q describe the same rotation. Keep the published delta on the
    // shortest, continuous hemisphere around the identity quaternion.
    if (qd.w() < 0.0) {
        qd = tf2::Quaternion(-qd.x(), -qd.y(), -qd.z(), -qd.w());
    }

    geometry_msgs::Quaternion q_msg;
    q_msg.x = qd.x();
    q_msg.y = qd.y();
    q_msg.z = qd.z();
    q_msg.w = qd.w();
    return q_msg;
}
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pubforce");
    ros::NodeHandle nh;

    // 订阅话题
    ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(
        "/Geomagic/pose_raw", 3, cb_pose);
    ros::Subscriber sub_button = nh.subscribe<geomagic_control::DeviceButtonEvent>(
        "/Geomagic/button", 3, cb_button);
    pub_force = nh.advertise<geomagic_control::DeviceFeedback>(
        "/Geomagic/force_feedback", 10);
    ros::NodeHandle private_nh("~");
    std::string haptic_command_topic;
    private_nh.param<std::string>("haptic_command_topic", haptic_command_topic,
                                  "/rm75_touch_teleop/haptic_device_command");
    private_nh.param("haptic_command_timeout", haptic_command_timeout, 0.1);
    if (!std::isfinite(haptic_command_timeout) || haptic_command_timeout <= 0.0) {
        ROS_FATAL("~haptic_command_timeout must be positive");
        return 1;
    }
    ros::Subscriber haptic_command_sub = nh.subscribe<geometry_msgs::Vector3Stamped>(
        haptic_command_topic, 1, hapticCommandCallback);

    // 位置锚点与增量
    ros::Publisher pub_Pose_Anchor = nh.advertise<std_msgs::Float64MultiArray>(
        "Touch_Pose_Anchor", 1);
    std_msgs::Float64MultiArray msg_Pose_Anchor;
    std_msgs::MultiArrayDimension dim;
    dim.label = "[x, y, z]";
    dim.size = 3;
    dim.stride = 3;
    msg_Pose_Anchor.layout.dim.push_back(dim);
    msg_Pose_Anchor.data = {0, 0, 0};

    ros::Publisher pub_Pose_increment = nh.advertise<std_msgs::Float64MultiArray>(
        "Touch_Pose_increment", 10);
    std_msgs::Float64MultiArray msg_Pose_increment;
    dim.label = "[delta_x, delta_y, delta_z]";
    msg_Pose_increment.layout.dim.push_back(dim);
    msg_Pose_increment.data = {0, 0, 0};

    // 姿态锚点与增量
    ros::Publisher pub_Orient_Anchor = nh.advertise<std_msgs::Float64MultiArray>(
        "Touch_Orient_Anchor", 1);
    std_msgs::Float64MultiArray msg_Orient_Anchor;
    std_msgs::MultiArrayDimension dim_q;
    dim_q.label = "[x, y, z, w]";
    dim_q.size = 4;
    dim_q.stride = 4;
    msg_Orient_Anchor.layout.dim.push_back(dim_q);
    msg_Orient_Anchor.data = {0, 0, 0, 1};

    ros::Publisher pub_Orient_increment = nh.advertise<std_msgs::Float64MultiArray>(
        "Touch_Orient_increment", 10);
    std_msgs::Float64MultiArray msg_Orient_increment;
    msg_Orient_increment.layout.dim.push_back(dim_q);
    msg_Orient_increment.data = {0, 0, 0, 1};

    // 锚点更新标志
    ros::Publisher pub_anchor_flag = nh.advertise<std_msgs::Bool>(
        "Touch_Anchor_updated", 1);
    std_msgs::Bool anchor_flag_msg;
    ros::Publisher pub_ori_anchor_flag = nh.advertise<std_msgs::Bool>(
        "Touch_Ori_Anchor_updated", 1);
    std_msgs::Bool ori_anchor_flag_msg;

    ros::Rate r(300);
    while(ros::ok())
    {
        geomagic_control::DeviceFeedback fb;

        // 更新并发布位置数据
        msg_Pose_Anchor.data = {x_Anchor, y_Anchor, z_Anchor};
        msg_Pose_increment.data = {delta_x, delta_y, delta_z};
        pub_Pose_Anchor.publish(msg_Pose_Anchor);
        pub_Pose_increment.publish(msg_Pose_increment);

        // 更新并发布姿态数据
        msg_Orient_Anchor.data = {
            orient_Anchor.x, orient_Anchor.y, orient_Anchor.z, orient_Anchor.w
        };
        msg_Orient_increment.data = {
            delta_orient.x, delta_orient.y, delta_orient.z, delta_orient.w
        };
        pub_Orient_Anchor.publish(msg_Orient_Anchor);
        pub_Orient_increment.publish(msg_Orient_increment);

        // 发布锚点更新标志
        if(new_anchor_updated) {
            anchor_flag_msg.data = true;
            pub_anchor_flag.publish(anchor_flag_msg);
            ROS_INFO_STREAM("[STATUS] New anchor set.");
            new_anchor_updated = false;
        }
        if (new_ori_anchor_updated)
        {
            ori_anchor_flag_msg.data = true;
            pub_ori_anchor_flag.publish(ori_anchor_flag_msg);
            ROS_INFO_STREAM("[STATUS] New ori anchor set.");
            new_ori_anchor_updated = false;
        }

        ros::spinOnce();

        // if(flag_whitebutton) {
        //     handleWhiteButtonPress();
        // }

        handleGreyButtonPress();
        handleWhiteButtonPress();
        // This node remains the sole owner of /Geomagic/force_feedback. It
        // publishes nothing until the opt-in teleop haptic path sends its
        // first internal command, preserving legacy non-haptic behaviour.
        if (have_haptic_command) publishHapticCommand();

        // 调试输出（3秒一次）
        ROS_INFO_STREAM_THROTTLE(3.0,"----------------");
        ROS_INFO_STREAM_THROTTLE(3.0,"Current Position: x=" << current_pose.pose.position.x
            << " , y=" << current_pose.pose.position.y
            << " , z=" << current_pose.pose.position.z);
        ROS_INFO_STREAM_THROTTLE(3.0,"Current Orientation: w="<< current_pose.pose.orientation.w
            << " , x=" << current_pose.pose.orientation.x
            << " , y=" << current_pose.pose.orientation.y
            << " , z=" << current_pose.pose.orientation.z);

        r.sleep();
    }
    if (have_haptic_command) {
        for (int i = 0; i < 3; ++i) {
            publishHapticCommand(true);
            ros::WallDuration(0.005).sleep();
        }
    }
    return 0;
}

// 按键回调
void cb_button(const geomagic_control::DeviceButtonEvent::ConstPtr &bt)
{
    flag_whitebutton = (bt->white_button == 1);
    flag_greybutton = (bt->grey_button == 1);
    ROS_INFO_STREAM("[STATION] button changed: grey=" << bt->grey_button
                    << ", white=" << bt->white_button);
}

// 位姿回调
void cb_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{
    current_pose = *msg;
    have_pose = true;
}

// 内部实现
namespace {
void hapticCommandCallback(const geometry_msgs::Vector3Stamped::ConstPtr& msg)
{
    const double values[] = {msg->vector.x, msg->vector.y, msg->vector.z};
    for (double value : values) {
        if (!std::isfinite(value)) {
            latest_haptic_command = geometry_msgs::Vector3();
            have_haptic_command = true;
            last_haptic_command_time = ros::WallTime::now();
            ROS_ERROR_THROTTLE(1.0, "Non-finite haptic command rejected; sending zero");
            return;
        }
    }
    latest_haptic_command = msg->vector;
    have_haptic_command = true;
    last_haptic_command_time = ros::WallTime::now();
}

void publishHapticCommand(bool force_zero)
{
    geomagic_control::DeviceFeedback feedback;
    feedback.lock = UNLOCKED_VEC;
    const bool stale = (ros::WallTime::now() - last_haptic_command_time).toSec() >
                       haptic_command_timeout;
    if (!force_zero && !stale) feedback.force = latest_haptic_command;
    pub_force.publish(feedback);
}

void handleDualButtonPress()
{
    flag_greybutton = flag_whitebutton = false;
}

void handleGreyButtonPress()
{
    bool current_greybutton = flag_greybutton;

    if (!have_pose) {
        // Keep the edge pending: if a button is already held when the first
        // valid pose arrives, that pose must become the anchor.
        prev_greybutton_state = false;
        return;
    }
    
    if (current_greybutton && !prev_greybutton_state) {
        // 设置位置锚点
        current_pose_Anchor = current_pose;
        x_Anchor = current_pose.pose.position.x;
        y_Anchor = current_pose.pose.position.y;
        z_Anchor = current_pose.pose.position.z;

        // // 设置姿态锚点
        // orient_Anchor = current_pose.pose.orientation;

            ROS_INFO_STREAM("[GOAL] New Anchor set: pos=(" 
        << x_Anchor << ", " << y_Anchor << ", " << z_Anchor << ") ");

        // ROS_INFO_STREAM("[GOAL] New Anchor set: pos=(" 
        //     << x_Anchor << ", " << y_Anchor << ", " << z_Anchor << ") "
        //     << "orient=(" << orient_Anchor.x << ", "
        //     << orient_Anchor.y << ", " << orient_Anchor.z << ", "
        //     << orient_Anchor.w << ")");

        new_anchor_updated = true;
    }
    
    if (current_greybutton) {
        // 更新位置差值
        delta_x = current_pose.pose.position.x - x_Anchor;
        delta_y = current_pose.pose.position.y - y_Anchor;
        delta_z = current_pose.pose.position.z - z_Anchor;

        // // 更新姿态差值（四元数差）
        // delta_orient = quatDiff(orient_Anchor, current_pose.pose.orientation);
    }
    
    prev_greybutton_state = current_greybutton;
}

void handleWhiteButtonPress()
{
    bool current_whitebutton = flag_whitebutton;

    if (!have_pose) {
        prev_whitebutton_state = false;
        return;
    }
    
    if (current_whitebutton && !prev_whitebutton_state) {
        // 设置姿态锚点
        orient_Anchor = current_pose.pose.orientation;

        ROS_INFO_STREAM("[GOAL] New Anchor set: " 
            << "orient=(" << orient_Anchor.x << ", "
            << orient_Anchor.y << ", " << orient_Anchor.z << ", "
            << orient_Anchor.w << ")");

        new_ori_anchor_updated = true;
    }
    
    if (current_whitebutton) {
        // 更新姿态差值（四元数差）
        delta_orient = quatDiff(orient_Anchor, current_pose.pose.orientation);
    }
    
    prev_whitebutton_state = current_whitebutton;
}

void applyForceFeedback(geomagic_control::DeviceFeedback& fb)
{
    if(abs(current_pose.pose.position.x) > 100 ||
       abs(current_pose.pose.position.y) > 120 ||
       abs(current_pose.pose.position.z) > 50)
    {
        ROS_INFO_STREAM("超出运动范围，手柄力反馈关闭");
        touch_release();
        return;
    }

    fb.force.x = -((int)current_pose.pose.position.x/100.0)*MAX_FORCE;
    fb.force.y = -((int)current_pose.pose.position.y/100.0)*MAX_FORCE;
    fb.force.z = -((int)current_pose.pose.position.z/50.0)*MAX_FORCE;
    fb.lock = UNLOCKED_VEC;
    pub_force.publish(fb);
}
}

// 工具函数
void touch_release()
{
    geomagic_control::DeviceFeedback fb;
    fb.lock = UNLOCKED_VEC;
    pub_force.publish(fb);
    touch_state = RELEASE;
    ROS_INFO_STREAM("手柄已释放");
}

void touch_lock(geometry_msgs::PoseStamped* pose)
{
    geomagic_control::DeviceFeedback fb;
    fb.position.x=pose->pose.position.x;
    fb.position.y=pose->pose.position.y;
    fb.position.z=pose->pose.position.z;
    fb.lock = LOCKED_VEC;
    pub_force.publish(fb);
    touch_state = LOCK;
    ROS_INFO_STREAM("手柄已锁住");
}
