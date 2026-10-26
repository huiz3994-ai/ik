/*
发送力控指令给Touch手柄

根据手柄按键状态以及位姿进行力反馈操作

初始为自由状态（释放状态）
按下灰色按键进入力反馈状态（需在三轴0点附近按下，勿太靠近底座，避免刚进入即退出）
会根据手柄位置施加相应的力 使其保持在0点附近 超出一定空间范围自动退出力反馈
再次按下灰色按键退出力反馈状态
两个按键同时按下则进入锁死状态  手柄保持在当前位置  不可进行其它操作
再次双按则退出锁死状态  返回自由状态

庐州学子
2025.1.12
*/

#include "geomagic_myhaptics/pubforce.h"

using namespace std;

// 全局状态（与头文件声明匹配）
bool flag_whitebutton = false;
bool flag_greybutton = false;
int touch_state = RELEASE;
geometry_msgs::PoseStamped current_pose;

// 优化专用常量（避免重复内存分配）
const vector<uint8_t> LOCKED_VEC = {1, 1, 1};
const vector<uint8_t> UNLOCKED_VEC = {0, 0, 0};
ros::Publisher pub_force;

// 内部函数声明
namespace {
void handleDualButtonPress();
void handleGreyButtonPress();
void applyForceFeedback(geomagic_control::DeviceFeedback& fb);
}

int main(int argc, char **argv)
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "pubforce");
    ros::NodeHandle nh;

    // 初始化与头文件声明严格一致
    ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>(
        "/Geomagic/pose", 3, cb_pose);
    ros::Subscriber sub_button = nh.subscribe<geomagic_control::DeviceButtonEvent>(
        "/Geomagic/button", 3, cb_button);
    pub_force = nh.advertise<geomagic_control::DeviceFeedback>(
        "/Geomagic/force_feedback", 10);

    ros::Rate r(30);
    while(ros::ok())
    {
        geomagic_control::DeviceFeedback fb;
        ros::spinOnce();

        // 调试输出（3秒一次）
        ROS_INFO_STREAM_THROTTLE(3.0,"----------------");
        ROS_INFO_STREAM_THROTTLE(3.0,"Current Position:     x: " << current_pose.pose.position.x 
            << " , y: " << current_pose.pose.position.y 
            << " , z: " << current_pose.pose.position.z);
        ROS_INFO_STREAM_THROTTLE(3.0,"Current orientation:  w: "<< current_pose.pose.orientation.w 
            << " , x: " << current_pose.pose.orientation.x 
            << " ,y :" << current_pose.pose.orientation.y 
            << " , z: " << current_pose.pose.orientation.z);

        // 状态处理流程
        if(flag_whitebutton) {
            handleDualButtonPress();
            continue;
        }
        
        if(touch_state == LOCK) continue;

        if(flag_greybutton) handleGreyButtonPress();
        
        if(touch_state == FORCE_FEEDBACK) applyForceFeedback(fb);

        r.sleep();
    }
    return 0;
}

// 回调函数严格匹配头文件声明
void cb_button(const geomagic_control::DeviceButtonEvent::ConstPtr &bt)
{
    flag_whitebutton = (bt->white_button == 1);
    flag_greybutton = (bt->grey_button == 1);
    ROS_INFO("按键状态改变：    灰色：%d,白色: %d", bt->grey_button, bt->white_button);
}

void cb_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{
    current_pose = *msg;
}

// 内部实现
namespace {
void handleDualButtonPress()
{
    if(touch_state != LOCK) {
        touch_lock(&current_pose);
    } else {
        touch_release();
    }
    flag_greybutton = flag_whitebutton = false;
}

void handleGreyButtonPress()
{
    if(touch_state != FORCE_FEEDBACK) {
        touch_state = FORCE_FEEDBACK;
        ROS_INFO_STREAM("手柄力反馈开启");
    } else {
        ROS_INFO_STREAM("手柄力反馈关闭");
        touch_release();
    }
    flag_greybutton = false;
}

void applyForceFeedback(geomagic_control::DeviceFeedback& fb)
{   // 运动范围：x、y大概是正负两百，z正负50，具体可见echo /Geomagic/pose的信息
    // 这里取x正负一百，y正负120,z正负50
    // 根据位置 施加反方向的力即可
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

// 头文件声明函数的实现
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