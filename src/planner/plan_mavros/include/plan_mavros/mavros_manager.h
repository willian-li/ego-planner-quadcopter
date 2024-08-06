/*
把重复的代码写成单独的函数，如果有许多重复顺序的函数调用，就再组织成一个函数。如果这些函数有共同的数据，可组织成一个类
功能：
2.发布目标点给fast_planner，再将fast_planner发过来的路径发送给mavros
3.接受目标识别的的位置发送给mavros
4.自动起降功能
5.目标搜寻功能
主循环：起飞模式->搜寻模式->精准定位模式->货物投放模式->(判断三个货物是否投完）？是->前往终点模式->降落模式
                   <-------------------------------------------------否*/
#ifndef _Mavros_Manager_
#define _Mavros_Manager_

#include <iostream>
#include <ros/ros.h>
#include "quadrotor_msgs/PositionCommand.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "tf/transform_datatypes.h"//转换函数头文件
#include <Eigen/Eigen>
#include <mavros_msgs/State.h>
#include <mavros_msgs/SetMode.h>
#include <math.h>
#include <serial/serial.h>
#include <string>
#include <std_msgs/String.h>
#include <vector>
using namespace std;
using namespace Eigen;

class Mavros_Manager
{
private:
    enum FlyState {WAITING,MISSION,TRACKING,DROPING,LANDING};
    //enum CmdState { TAKEOFF, ROLLING ,FASTPLANNER,YOLOTRC, HOVERING};

    
    //CmdState cmdstate = TAKEOFF;

    /* ***************************************参数 ***************************************/
    FlyState flystate = WAITING;

    /*相机坐标转换相关*/
    Matrix4d cam2world;
    Matrix4d cam02body;
    Eigen::Quaterniond cam2world_quat;

    //容器，装目标投放点名称
    vector<string>complete_name[3];
    vector<string>my_name[3];
    string now_name;

    
    //launch动态参数
    string arduino_port;                                    //arduino串口名称
    double desire_h;                                        //执行任务高度
    double waypoints_[50][3];                               //任务点
    int waypoint_num_;                                      //待完成任务点数目
    int complete_waynum = 0;                                //已完成任务点数目
    bool is_2D;
    
    //程序运行控制动态参数
    int time_dex;                                           //离散变量
    int t_delay;                                            //延时变量
    //转头参数
    double start_yaw,end_yaw;
    //追踪参数
    Eigen::Vector3d start_tracpos,end_tracpos;
    //double desire_yaw,temp_yaw,temp_target_yaw,m_t,s_t;
    bool rolling_complete = false;                          //用于将转头和避障先后运行
    bool rolling_init = false;
    bool tracking_init = false;
    bool is_tracking = false;                               //判断是否在途中遇到图片
    int globle_dex=0;
    int local_dex = 0;
    int t1=0;
    Eigen::Vector3d goal_for_test;    
    // double m_x,m_y,s_xy;
    double drop_pos[3];
    //bool roll_init = false; //转头初始化
    
    //发给飞控的变量！！！！！！！！！！！！！！！！！！！！！！！！！
    Eigen::Vector3d _target_pos;    
    double _target_yaw;


    //回调函数中的变量
    Eigen::Vector3d local_fcu_pt_;                          //飞机当前位置
    Eigen::Vector3d yolo_distance_;                         //当前距离图片位置



    quadrotor_msgs::PositionCommand fater_cmd_msg;          //避障路径
    geometry_msgs::PoseStamped local_pos_msg;               //fcu位姿
    mavros_msgs::State current_state;                       //当前状态 offboard
    mavros_msgs::SetMode mode_cmd;                          //降落命令
    geometry_msgs::PoseStamped avoidance_goal_msg;          //给避障算法发送目标点
    geometry_msgs::Quaternion geo_q;                        //四元数
    serial::Serial ser;                                     //串口初始化
    std_msgs::String sub_data;

    /*话题*/
    ros::Subscriber cmd_sub,local_pos_sub,state_sub,yolo_sub,name_sub;
    ros::Publisher target_pos_pub,camera_pub,goal_pub;
    ros::Timer main_timer,cmd_timer;
    ros::ServiceClient set_mode_client;

    void cmdCallbck(const quadrotor_msgs::PositionCommand& msg);
    void local_pos_Callbck(const geometry_msgs::PoseStamped& msg);
    void yolo_cb(const geometry_msgs::Pose& msg);
    void name_cb(const std_msgs::String& msg);
    void state_cb(const mavros_msgs::State& msg);
    void mainloop(const ros::TimerEvent& e);
    void cmdloop(const ros::TimerEvent& e);
    float getyaw(const float x,const float y);
    //bool planNextWaypoint();
    void changeState(FlyState newstate);                    //切换状态的函数
    //bool rollInit();
    //void changecmd(CmdState newcmd);
    void delayPrint(int delay_dex,string mytips);
    bool isRightData();
    double dispersed(double start_value,double end_value,double active_value,double dis_size);
public:
    Mavros_Manager(ros::NodeHandle& nh);
    ~Mavros_Manager();
};


#endif
