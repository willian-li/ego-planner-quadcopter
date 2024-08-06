/*发布目标点  
话题/move_base_simple/goal 
消息类型：geometry_msgs/PoseStamped 
在发布目标点前确认当前飞机和目标点的方向 
缓慢转动机头使其对准目标点，
最后发布目标点给fast_planner
封装为一个对象，外部只需调用函数pub_goal(x,y)
*/
#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
using namespace std;

ros::Publisher goal_pub; //发布目标点
ros::Publisher yaw_pub; //发布角度转变
geometry_msgs::PoseStamped _cmd;
void cmdCallbck(const geometry_msgs::PoseStamped& msg) 
{
    _cmd = msg;
}

double get_yaw(float a, float b)
{

}

void pub_goal(float a, float b) //发布函数
{
    //获取期望姿态
    double target_yaw = 0;
    target_yaw = get_yaw(a,b);
    //

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "send_goal");
    ros::NodeHandle nh("~");

    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);//发布期望坐标
    yaw_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 50);//发布期望角

    ros::Subscriber sub1 = nh.subscribe("/mavros/local_position/pose", 10, cmdCallbck); //获取飞控当前姿态

    ros::spin();   
    return 0;

}