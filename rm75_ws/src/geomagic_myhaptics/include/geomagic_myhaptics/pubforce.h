#ifndef PUBFORCE_H
#define PUBFORCE_H

#include "ros/ros.h"
#include "geomagic_control/DeviceButtonEvent.h"
#include "geometry_msgs/PoseStamped.h"
#include "geomagic_control/DeviceFeedback.h"
#include  <vector>
#include <std_msgs/Float64MultiArray.h> //新增 用于相对位置发布
#include <std_msgs/Bool.h>

// 单轴输出的最大力 官网显示最大3.3N 测试可用1.5N
#define MAX_FORCE   1.5
#define RELEASE 0
#define FORCE_FEEDBACK  1
#define LOCK    2
//笛卡尔空间下的初始化锚定点
#define DEFALT_POSE_ZERO    {0, 0, 0}
#define DEFALT_POSE_FORWARD {0.25, 0, 0.43}

//位姿与按键订阅者的回调函数
void cb_pose(const geometry_msgs::PoseStampedConstPtr& msg);
void cb_button(const geomagic_control::DeviceButtonEvent::ConstPtr &bt);
//释放手柄
void touch_release(void);
//在指定位置锁住touch手柄
void touch_lock(geometry_msgs::PoseStamped* pose);

#endif // !PUBFORCE_H
