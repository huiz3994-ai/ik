#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

/*补充说明：
1.机器人运动学求解器配置在z1_moveit_config/config/kinematics.yaml
2.与 z1_examples/examples/test2_get_end_effector_pose.cpp 不同的是，
    本代码打印末端位姿数据到控制台,而是发布到 geometry_msgs::PoseStamped 话题 /end_effector_pose
*/

/*头文件解释：
move_group_interface.h：提供了与MoveIt!中的“MoveGroup”接口进行交互的功能，用于实现路径规划和执行。
planning_scene_interface.h：用于操作和管理规划场景。
moveit_msgs/DisplayRobotState.h、moveit_msgs/DisplayTrajectory.h等：用于显示机器人状态和轨迹的消息类型。
moveit_visual_tools.h：提供了可视化工具接口，帮助在Rviz中展示机器人操作。
*/

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "get_end_effector_pose");
    ros::AsyncSpinner spinner(1);
    spinner.start();
    
    // 初始化MoveGroupInterface对象，指定规划组名称（如"manipulator"）
    moveit::planning_interface::MoveGroupInterface move_group("manipulator");
    
    // 设置末端执行器Link名称（根据URDF定义，如"end_effector_link"）
    std::string end_effector_link = move_group.getEndEffectorLink();
    
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("end_effector_pose", 10);

    ros::Rate rate(300); // 设置读取频率
    
    while (ros::ok()) {
        // 获取当前末端位姿（类型为geometry_msgs::PoseStamped）并发布出去
        geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose(end_effector_link);
        pose_pub.publish(current_pose);

        // 提取位置信息
        double x = current_pose.pose.position.x;
        double y = current_pose.pose.position.y;
        double z = current_pose.pose.position.z;
        
        // 提取姿态四元数
        double ox = current_pose.pose.orientation.x;
        double oy = current_pose.pose.orientation.y;
        double oz = current_pose.pose.orientation.z;
        double ow = current_pose.pose.orientation.w;
        
        // 打印结果
        // ROS_INFO_STREAM("--------------------------------------------------------");
        // ROS_INFO_STREAM("End Effector Position: ["
        //     << x << ", " << y << ", " << z << "]");
        // ROS_INFO_STREAM("End Effector Orientation (Quaternion): ["
        //     << ox << ", " << oy << ", " << oz << ", " << ow << "]");
        rate.sleep();
    }
    
    return 0;
}