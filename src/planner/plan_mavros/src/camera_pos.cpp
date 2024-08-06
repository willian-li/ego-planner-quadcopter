//ROS 头文件
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Eigen>
#include <tf/transform_datatypes.h>//转换函数头文件
#include <tf/transform_listener.h>

ros::Timer estimation_timer; 

using namespace std;
using namespace Eigen;

Matrix4d cam2world;
Matrix4d cam02body;
Eigen::Quaterniond cam2world_quat;

ros::Publisher pos_pub;

geometry_msgs::PoseStamped _pos;



void cmdCallbck(const geometry_msgs::PoseStamped& msg) 
{
    _pos = msg;
    Matrix4d Pose_receive = Matrix4d::Identity();
    Eigen::Vector3d request_position;
    Eigen::Quaterniond request_pose;
    request_position.x() = msg.pose.position.x;
    request_position.y() = msg.pose.position.y;
    request_position.z() = msg.pose.position.z;
    request_pose.x() = msg.pose.orientation.x;
    request_pose.y() = msg.pose.orientation.y;
    request_pose.z() = msg.pose.orientation.z;
    request_pose.w() = msg.pose.orientation.w;
    Pose_receive.block<3,3>(0,0) = request_pose.toRotationMatrix();
    Pose_receive(0,3) = request_position(0);
    Pose_receive(1,3) = request_position(1);
    Pose_receive(2,3) = request_position(2);

    Matrix4d body_pose = Pose_receive;
    //convert to cam pose
    cam2world = body_pose * cam02body;
    cam2world_quat = cam2world.block<3,3>(0,0);

}

void pubCameraPose(const ros::TimerEvent & event)
{
    geometry_msgs::PoseStamped target_pos;
    target_pos.header = _pos.header;
    target_pos.header.frame_id = "/map";
    target_pos.pose.position.x = cam2world(0,3);
    target_pos.pose.position.y = cam2world(1,3);
    target_pos.pose.position.z = cam2world(2,3);
    target_pos.pose.orientation.w = cam2world_quat.w();
    target_pos.pose.orientation.x = cam2world_quat.x();
    target_pos.pose.orientation.y = cam2world_quat.y();
    target_pos.pose.orientation.z = cam2world_quat.z();
    pos_pub.publish(target_pos); 
  
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camera_pos_node");
    ros::NodeHandle node;
    ros::NodeHandle nh("~");
    ros::Subscriber pos_sub = node.subscribe("/mavros/local_position/pose", 50, cmdCallbck);
    pos_pub = node.advertise<geometry_msgs::PoseStamped>("/camera_pos", 1000);
    double estimate_duration = 1.0 / 30.0;
    estimation_timer  = nh.createTimer(ros::Duration(estimate_duration), pubCameraPose);
    cam02body << 0.0, 0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            0.0, -1.0,0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;
    ros::spin();   
    return 0;
}
