/*// 把重复的代码写成单独的函数，如果有许多重复顺序的函数调用，就再组织成一个函数。如果这些函数有共同的数据，可组织成一个类
// 功能：
// 2.发布目标点给fast_planner，再将fast_planner发过来的路径发送给mavros
// 3.接受目标识别的的位置发送给mavros
// 4.自动起降功能
// 5.目标搜寻功能
// 主循环：起飞模式->搜寻模式->精准定位模式->货物投放模式->(判断三个货物是否投完或者是否超时）？是->前往终点模式->降落模式
//                    <——————————————————————————————————————————————————否*/
#include <ros/ros.h>
#include <iostream>
#include "mavros_manager.h"
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "drop_mainner");
    ros::NodeHandle nh("~");
    Mavros_Manager mavros_manager(nh);
    ros::spin();   
    return 0;

}