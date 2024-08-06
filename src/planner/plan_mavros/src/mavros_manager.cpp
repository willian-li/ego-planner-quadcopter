#include "mavros_manager.h"

/********************************************初始化函数**************************************************/
Mavros_Manager::Mavros_Manager(ros::NodeHandle& nh)
{
    /*  fsm param  */
    nh.param("desire_h", desire_h, 1.0);
    nh.param("drop_pos_high", drop_pos[2], 1.0);
    nh.param("waypoint_num", waypoint_num_, -1);
    nh.param("is_2D", is_2D, true);
    nh.param<string>("arduino_port", arduino_port,"/dev/ttyACM0");
    //初始化变量
    time_dex = 0;
    my_name->push_back("BRIDGE");
    my_name->push_back("BL");
    my_name->push_back("ZP");

    _target_pos(0) = 0;
    _target_pos(1) = 0;
    _target_pos(2) = desire_h;

    for (int i = 0; i < waypoint_num_; i++) 
    {
        nh.param("waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
        nh.param("waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
        nh.param("waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    //fater_cmd.position.z = desire_h;//初始fast没发默认是0,给他初始为desire_h
    cam02body << 0.0, 0.0, 1.0, 0.0,
            -1.0, 0.0, 0.0, 0.0,
            0.0, -1.0,0.0, 0.0,
            0.0, 0.0, 0.0, 1.0;

    /*回调函数*/
    cmd_sub = nh.subscribe("/planning/pos_cmd", 50, &Mavros_Manager::cmdCallbck, this);
    local_pos_sub = nh.subscribe("/mavros/local_position/pose", 50, &Mavros_Manager::local_pos_Callbck,this);
    state_sub = nh.subscribe("/mavros/state", 10, &Mavros_Manager::state_cb,this);
    yolo_sub = nh.subscribe("/yolo", 10, &Mavros_Manager::yolo_cb,this);
    name_sub = nh.subscribe("/detect", 10,&Mavros_Manager::name_cb,this);
    /**/
    target_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    camera_pub = nh.advertise<geometry_msgs::PoseStamped>("/camera_pos", 1000);
    goal_pub = nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);//发布期望坐标
    /**/
    main_timer = nh.createTimer(ros::Duration(0.1), &Mavros_Manager::mainloop,this);
    cmd_timer = nh.createTimer(ros::Duration(0.01), &Mavros_Manager::cmdloop,this);
    /**/
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    
    setlocale(LC_ALL, ""); //中文支持
	try 
    { 
        ser.setPort(arduino_port);  //设置端口号
        ser.setBaudrate(9600);  //设置波特率
        serial::Timeout tout = serial::Timeout::simpleTimeout(1000);//设置延时等待 ms
        ser.setTimeout(tout); 
        ser.open(); //打开串口
    } 
    catch (serial::IOException& e) 
    { 
        ROS_ERROR_STREAM("无法打开串口 "); 
    } 
    
    if(ser.isOpen())  //成功打开串口
    { 
        ROS_INFO_STREAM("串口初始化成功"); 
    } 
    else 
    { 
        ROS_ERROR_STREAM("初始化不成功"); 
    }

    cout << "代码版本：V 1.0 " << endl;
}

/*****************************************主逻辑循环***********************************************/


void Mavros_Manager::mainloop(const ros::TimerEvent& e)
{
    switch (flystate)
    {
    case WAITING:{
        if(current_state.mode != "OFFBOARD")
        {
            delayPrint(20,"wait for offboard ");
            time_dex = 0;
            _target_pos(0) = 0;
            _target_pos(1) = 0;
            _target_pos(2) = desire_h;

        }
        else
        {
            if(local_fcu_pt_(2) < desire_h - 0.05) 
            {
                
            }
            else
            {
                cout << "当前高度：" << local_fcu_pt_(2) << endl;
                changeState(MISSION);
                time_dex = 0;
                //cout <<"time_dex已置0" << endl;
            }
    }
        break;
    }
    
    case MISSION:{
        //转角：
        if(rolling_complete == false)
        {
            //转角初始化
            if(rolling_init == false)
            {
                rolling_init = true;
                time_dex = 0;
                if(complete_waynum < waypoint_num_) 
                {
                    //获取目标角度
                    end_yaw = getyaw(waypoints_[complete_waynum][0],waypoints_[complete_waynum][1]);
                    //cout <<end_yaw <<endl;
                    cout << "得到目标：x:"<< waypoints_[complete_waynum][0]<< "    y:"<<waypoints_[complete_waynum][1]<<endl;
                    avoidance_goal_msg.pose.position.x = waypoints_[complete_waynum][0];
                    avoidance_goal_msg.pose.position.y = waypoints_[complete_waynum][1];
                    avoidance_goal_msg.pose.position.z = waypoints_[complete_waynum][2];
                    goal_for_test(0) = waypoints_[complete_waynum][0];
                    goal_for_test(1) = waypoints_[complete_waynum][1];
                    goal_for_test(2) = waypoints_[complete_waynum][2];

                    tf::Quaternion quat;
                    tf::quaternionMsgToTF(local_pos_msg.pose.orientation, quat);
                    double roll, pitch, yaw;
                    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
                    //获取当前角度
                    start_yaw = yaw;
                    
                    complete_waynum += 1;
                }
                else
                {
                    changeState(LANDING);
                }
            }
            //等待转角完成
            else
            {
                _target_yaw = dispersed(start_yaw,end_yaw,time_dex,50);
                time_dex +=1;
                if(time_dex >= abs((end_yaw - start_yaw)*50))
                {
                    rolling_complete = true;
                    rolling_init = false;
                    //发布目标点
                    goal_pub.publish(avoidance_goal_msg);
                    time_dex = 0;
                }
                
            }
        }
        //等待到达目标点
        else
        {
            if((local_fcu_pt_ - goal_for_test).norm() < 0.40)
            {
                rolling_complete = false;
                cout << "已完成第 "<< complete_waynum << "个点" << endl;
                //在目标点有图片
                if (is_tracking == true)
                {
                    bool sucess = isRightData();
                    if(sucess)
                    {
                        changeState(TRACKING);
                    }
                }
            }
            
        }

        //检测是否收到图片
        t1 += 1;
        if(t1 >= 3)
        {
            t1 = 0;
            local_dex = globle_dex;
        }//三轮循环记录一次globle
        if(globle_dex != local_dex) //如果当前不等于三轮前记录就说明在更新
        {
            is_tracking = true;
        }
        else
        {
            is_tracking = false;
        }

        break;
    }
    case TRACKING:{
        //初始化
        if (tracking_init == false)
        {
            tracking_init = true;
            start_tracpos(0) = local_fcu_pt_(0);
            start_tracpos(1) = local_fcu_pt_(1);
            end_tracpos(0) = local_fcu_pt_(0)+ yolo_distance_(0);
            end_tracpos(1) = local_fcu_pt_(1)+ yolo_distance_(1);
            time_dex = 0;
        }
        //等待追踪完成
        else
        {
            time_dex += 1;
            //start_value + (end_value - start_value) * (active_value/(abs(end_value - start_value)*dis_size));(max(abs(end_tracpos(0) - start_tracpos(0),
            _target_pos(0) = start_tracpos(0) + (end_tracpos(0) - start_tracpos(0)) * (time_dex/(max(abs(end_tracpos(0) - start_tracpos(0)),abs(end_tracpos(1)-start_tracpos(1)))*50));
            _target_pos(1) = start_tracpos(1) + (end_tracpos(1) - start_tracpos(1)) * (time_dex/(max(abs(end_tracpos(0) - start_tracpos(0)),abs(end_tracpos(1)-start_tracpos(1)))*50));
            _target_pos(2) = desire_h;
            if (time_dex >= 10)
            {
                time_dex = 0;
                tracking_init = false;
            }
            Eigen::Vector3d h(0,0,yolo_distance_(2)); //消除高度
            if ((yolo_distance_ - h).norm() < 0.40)
            {
                //changeState(DROPING);
            }
        }
        
        break;
    }

    case DROPING:{
        time_dex++;
        if(time_dex > 10&& time_dex <12)
        {
            if(ser.isOpen())  //成功打开串口
            {
                if (now_name == my_name->begin()[0])
                {
                    std::string msg = "1"; //发送数据
                    ser.write(msg); //像单片机发送数据
                }
                else if (now_name == my_name->begin()[1])
                {
                    std::string msg = "2"; //发送数据
                    ser.write(msg); //像单片机发送数据
                }
                else if (now_name == my_name->begin()[2])
                {
                    std::string msg = "3 "; //发送数据
                    ser.write(msg); //像单片机发送数据
                }
                cout << "投放"<< endl;
            }
            else{
                cout << "串口打开失败"<< endl;
            }
        }
        //投放完毕
        if(time_dex > 50)
        {
            changeState(MISSION);//发送一次就切换状态了
            time_dex = 0;
        }

    break;
    }
    case LANDING:{
        /* code */
        mode_cmd.request.custom_mode = "AUTO.LAND";
        set_mode_client.call(mode_cmd);
        break;
    }

    default:
        break;
    }
}


void Mavros_Manager::cmdloop(const ros::TimerEvent& e)
{
    geometry_msgs::PoseStamped local_target_pos;
    local_target_pos.header = local_pos_msg.header;
    local_target_pos.pose.position.x = _target_pos(0);
    local_target_pos.pose.position.y = _target_pos(1);
    local_target_pos.pose.position.z = _target_pos(2);
    geo_q = tf::createQuaternionMsgFromYaw(_target_yaw);
    local_target_pos.pose.orientation = geo_q;
    target_pos_pub.publish(local_target_pos);

/*发布相机坐标*/
    geometry_msgs::PoseStamped camera_pos;
    camera_pos.header = local_pos_msg.header;
    camera_pos.header.frame_id = "/map";
    camera_pos.pose.position.x = cam2world(0,3);
    camera_pos.pose.position.y = cam2world(1,3);
    camera_pos.pose.position.z = cam2world(2,3);
    camera_pos.pose.orientation.w = cam2world_quat.w();
    camera_pos.pose.orientation.x = cam2world_quat.x();
    camera_pos.pose.orientation.y = cam2world_quat.y();
    camera_pos.pose.orientation.z = cam2world_quat.z();
    camera_pub.publish(camera_pos);

}
/*********************************功能函数****************************************/
double Mavros_Manager::dispersed(double start_value,double end_value,double active_value,double dis_size)
{
    double current;
    if (active_value >= abs((end_value - start_value)*dis_size))
    {
        active_value = ((end_value - start_value)*dis_size);
    }
    current = start_value + (end_value - start_value) * (active_value/(abs(end_value - start_value)*dis_size));
    
    return current;
}

void Mavros_Manager::delayPrint(int delay_dex,string mytips)
{
    t_delay += 1;
    if (t_delay >= delay_dex)
    {
        t_delay = 0;
        cout << mytips << endl;
    }
}

float Mavros_Manager::getyaw(const float x,const float y)
{
    float yaw;
    yaw = atan2((y - local_pos_msg.pose.position.y),(x -local_pos_msg.pose.position.x));
    return yaw;
}

void Mavros_Manager::changeState(FlyState newstate)
{
    static string state_str[5] = {"WAITING","MISSION", "TRACKING", "DROPING", "LANDING"};
    int pre_s = int(flystate);
    flystate = newstate;
    cout << "[任务状态]："<<state_str[pre_s] + " ——> " + state_str[int(newstate)] << endl;
}

bool Mavros_Manager::isRightData()
{
    //判断是否已经完成
    bool is_same = false;
    for (int i = 0;i < complete_name->size();i++)
    {
        if(sub_data.data == complete_name->begin()[i])   
        {
            is_same = true;
            return false;
        }
    }
    //判断是否是目标图片
    if (is_same == false)
    {
        for(int i = 0; i < my_name->size();i++)
        {
            if(sub_data.data == my_name->begin()[i]) 
            {
                cout << "填充："<<sub_data.data<<endl;
                now_name = sub_data.data;
                complete_name->push_back(sub_data.data);//加入已经识别过的图片
                return true;
            }
        }
    }
}

/*********************************************回调函数***************************************************/

void Mavros_Manager::cmdCallbck(const quadrotor_msgs::PositionCommand& msg)
{
    fater_cmd_msg = msg;
    if(rolling_complete)
    {
        _target_pos(0) = msg.position.x;
        _target_pos(1) = msg.position.y;
    }
}
void Mavros_Manager::name_cb(const std_msgs::String& msg)
{
    //cout <<"收到消息："<< msg.data << endl;
    //判断是否收到过此消息
    globle_dex +=1;
    if (globle_dex >= 10000)
    {
        globle_dex = 0;
    }
    sub_data = msg;
    

}


void Mavros_Manager::yolo_cb(const geometry_msgs::Pose& msg)
{
    yolo_distance_(0) = msg.position.x;
    yolo_distance_(1) = msg.position.y;
    yolo_distance_(2) = msg.position.z;
}

void Mavros_Manager::local_pos_Callbck(const geometry_msgs::PoseStamped& msg)
{
    local_pos_msg = msg;

    local_fcu_pt_(0) = msg.pose.position.x;
    local_fcu_pt_(1) = msg.pose.position.y;
    local_fcu_pt_(2) = msg.pose.position.z;

    /*坐标转换相关*/
    Matrix4d Pose_receive = Matrix4d::Identity();
    Eigen::Vector3d request_position;
    Eigen::Quaterniond request_pose;
    request_position.x() = msg.pose.position.x+0.12;
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

void Mavros_Manager::state_cb(const mavros_msgs::State& msg)
{
    current_state = msg;
}


Mavros_Manager::~Mavros_Manager()
{
}
