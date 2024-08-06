//ROS 头文件
#include <ros/ros.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <geometry_msgs/PoseStamped.h>
#include "tf/transform_datatypes.h"//转换函数头文件
//#include <geometry_msgs/Pose.h>
//topic 头文件
#include <iostream>

#include <cmath>

double yaw = 0;
double s_t = 0;
double m_t = 1;
using namespace std;

float desire_h = 1.0;

quadrotor_msgs::PositionCommand position;
geometry_msgs::PoseStamped pub_pos;

std_msgs::Header header;
geometry_msgs::Pose pos;
ros::Publisher pos_pub;

geometry_msgs::Quaternion geo_q;
void cmdCallbck(const quadrotor_msgs::PositionCommand& msg) 
{
    position = msg;
    
}

void timeCallback(const ros::TimerEvent& e)
{
    pos.position.x = position.position.x;
    pos.position.y = position.position.y;
    pos.position.z = desire_h;
    m_t = (position.yaw - yaw)*10000;
    if(m_t == 0)
    {
        m_t = 1;
    }
    s_t += 2;
    if(s_t >= abs(m_t))
    {
        s_t = 0;
    }
    yaw += (position.yaw-yaw) * (s_t/(abs(m_t)));//离散
    cout << "yaw:"<< yaw << endl;
    cout << "positon.yaw:"<< position.yaw << endl;
    cout << "s_t:"<< s_t << endl;
    cout << "m_t:"<< m_t << endl;
    geo_q = tf::createQuaternionMsgFromYaw(yaw);
    pos.orientation = geo_q;

    pub_pos.pose = pos;

    header.frame_id = "base_link";

    pub_pos.header = header;

    pos_pub.publish(pub_pos);

}
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>主 函 数<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_target");
    ros::NodeHandle node;
    ros::NodeHandle nh("~");
    ros::Subscriber pos_cmd_sub = node.subscribe("/planning/pos_cmd", 10, cmdCallbck);
    pos_pub = node.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 50);

    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), timeCallback);
//>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>Main Loop<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
    ros::spin();   
    return 0;

}


