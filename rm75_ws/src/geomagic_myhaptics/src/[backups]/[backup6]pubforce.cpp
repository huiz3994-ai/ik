/*
开机即开始发布：
/Geomagic/... ——————————> Touch末端绝对位姿系列话题
/Touch_Pose_Anchor —————> 用户选定的Touch锚定末端坐标[x,y,z]
/Touch_Pose_increment ——> Touch实时位置坐标与锚定坐标的相对位移[delta_x,delta_y,delta_z]

根据手柄按键状态以及位姿进行力反馈操作
灰色按下：
*/

#include "geomagic_myhaptics/pubforce.h"

using namespace std;

// 全局状态（与头文件声明匹配）
bool flag_whitebutton = false;//两个按键，一灰一白，数值0 表示未按下，1 表示按下。在按键状态改变时会向 /Geomagic/button 话题发送信息。
bool flag_greybutton = false;
bool prev_greybutton_state = false;// 在全局变量区增加按钮状态跟踪变量
int touch_state = RELEASE;
geometry_msgs::PoseStamped current_pose;
geometry_msgs::PoseStamped current_pose_Anchor;
float x_Anchor = 0;
float y_Anchor = 0;
float z_Anchor = 0;
float delta_x = 0;
float delta_y = 0;
float delta_z = 0;

// 优化专用常量（避免重复内存分配）
const vector<uint8_t> LOCKED_VEC = {1, 1, 1};
const vector<uint8_t> UNLOCKED_VEC = {0, 0, 0};
ros::Publisher pub_force;

// 内部函数声明
namespace {
void handleDualButtonPress();
void handleGreyButtonPress();
void handleWhiteButtonPress();
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

    ros::Publisher pub_Pose_Anchor = nh.advertise<std_msgs::Float64MultiArray>(
        "Touch_Pose_Anchor", 1);
    std_msgs::Float64MultiArray msg_Pose_Anchor;
    std_msgs::MultiArrayDimension dim;// 设置维度信息
    dim.label = "[x, y, z]";
    dim.size = 3;               // 数据长度（3个元素）
    dim.stride = 3;             // 步长：从当前维度的某个元素移动到下一个元素时，需要跳过的数据元素数量​​（一维数组连续存储时stride=size）
    msg_Pose_Anchor.layout.dim.push_back(dim); // 添加到维度数组
    msg_Pose_Anchor.data = DEFALT_POSE_FORWARD; // 锚定点数组初始化

    ros::Publisher pub_Pose_increment = nh.advertise<std_msgs::Float64MultiArray>(
        "Touch_Pose_increment", 10);
    std_msgs::Float64MultiArray msg_Pose_increment;
    dim.label = "[delta_x, delta_y, delta_z]";
    msg_Pose_increment.layout.dim.push_back(dim); // 添加到维度数组
    msg_Pose_increment.data = {0, 0, 0}; // 数组初始化

    ros::Rate r(30); //默认30hz
    while(ros::ok())
    {
        geomagic_control::DeviceFeedback fb;
        
        msg_Pose_Anchor.data = {x_Anchor, y_Anchor, z_Anchor};  // 更新锚点数据
        msg_Pose_increment.data = {delta_x, delta_y, delta_z};  // 更新相对位移数据         
        pub_Pose_Anchor.publish(msg_Pose_Anchor);               // 发布锚点
        pub_Pose_increment.publish(msg_Pose_increment);         // 发布相对位移 

        ros::spinOnce();

        // 状态处理流程
        if(flag_whitebutton) {
            handleWhiteButtonPress();
            // continue;    //continue会导致handleGreyButtonPress()的长按逻辑出bug
        }

        // if(touch_state == LOCK) continue;    //continue会导致handleGreyButtonPress()的长按逻辑出bug

        handleGreyButtonPress();  // 此函数现在包含完整的按钮状态处理

        if(flag_greybutton && flag_whitebutton) handleDualButtonPress();

        // if(touch_state == FORCE_FEEDBACK) applyForceFeedback(fb);

        // 调试输出（3秒一次）
        ROS_INFO_STREAM_THROTTLE(3.0,"----------------");
        ROS_INFO_STREAM_THROTTLE(3.0,"Current Position:     x: " << current_pose.pose.position.x
            << " , y: " << current_pose.pose.position.y
            << " , z: " << current_pose.pose.position.z);
        ROS_INFO_STREAM_THROTTLE(3.0,"Current orientation:  w: "<< current_pose.pose.orientation.w
            << " , x: " << current_pose.pose.orientation.x
            << " , y: " << current_pose.pose.orientation.y
            << " , z: " << current_pose.pose.orientation.z);

        r.sleep();
    }
    return 0;
}

// 回调函数严格匹配头文件声明
void cb_button(const geomagic_control::DeviceButtonEvent::ConstPtr &bt)
{
    flag_whitebutton = (bt->white_button == 1);
    flag_greybutton = (bt->grey_button == 1);
    // ROS_INFO("按键状态改变：    灰色：%d,白色: %d", bt->grey_button, bt->white_button);
    ROS_INFO_STREAM("[STATION]button changed: grey button: " << bt->grey_button << ", white button: " << bt->white_button);
}

void cb_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{
    current_pose = *msg;
}

// 内部实现
namespace {
void handleDualButtonPress()//原先锁定功能已注释，现在什么也不做
{
    // if(touch_state != LOCK) {
    //     touch_lock(&current_pose);
    // } else {
    //     touch_release();
    // }

    flag_greybutton = flag_whitebutton = false; //重置所有状态位
}


/***handleGreyButtonPress()灰色按钮按键处理***
每次完整按压周期（按下→保持→松开）只设置一次锚点;
按住期间持续更新位移差值;
支持无限次重复操作;
*******************************************/
void handleGreyButtonPress()
{
    bool current_greybutton = flag_greybutton;
    
    // 上升沿检测：按钮刚被按下
    if (current_greybutton && !prev_greybutton_state) {
        current_pose_Anchor = current_pose;
        x_Anchor = current_pose_Anchor.pose.position.x;
        y_Anchor = current_pose_Anchor.pose.position.y;
        z_Anchor = current_pose_Anchor.pose.position.z;
        ROS_INFO_STREAM("[GOAL]New Anchor setted: x: " << x_Anchor 
            << ", y: " << y_Anchor 
            << ", z: " << z_Anchor);
    }
    
    // 持续计算差值（只要按钮保持按下）
    if (current_greybutton) {
        delta_x = current_pose.pose.position.x - x_Anchor;
        delta_y = current_pose.pose.position.y - y_Anchor;
        delta_z = current_pose.pose.position.z - z_Anchor;
    }
    
    // 更新状态记录
    prev_greybutton_state = current_greybutton;
}


void handleWhiteButtonPress()//锁定手柄位置
{
    if(touch_state != LOCK) {
        touch_lock(&current_pose);
    } else {
        touch_release();
    }
    flag_whitebutton = false;
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