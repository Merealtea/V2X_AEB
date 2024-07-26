#include "stanley_controller.h"

// USB Key，只在生成发布版的时候才解注释
// #include "../../../common/Pwd_8/SoftkeyPWD.h"

StanleyController::StanleyController()
{
    //------成员变量初始化-------------
    path_flag = false;
    temp_point = 0;
    former_steer_cmd = 0;
    target_path.poses.clear();

    inter_steer_error = 0;
    for (int i = 0; i < STEER_ERROR_QUEUE_SIZE; i++)
        steer_error_queue.push(0);

    //------配置动态更改参数服务-------
    cb = boost::bind(&StanleyController::configCallback, this, _1, _2);
    dr_srv.setCallback(cb);

    //------参数初始化-----------------
    ros::NodeHandle pnh("~"); //定义私有节点句柄，用于传递参数;
    pnh.param("reference_distance_", ref_distance, 2.5);
    pnh.param("k_angle_", k_angle, 1.0);
    pnh.param("k_error_", k_error, 3.0);
    pnh.param("Kp_error_", Kp_error, 0.2);
    pnh.param("Ki_error_", Ki_error, 0.2);
    pnh.param("filter_param_", filter_param, 0.6);
    pnh.param("Kp_wheel_", Kp_wheel, 1500.0);

    //------配置订阅和发布的话题-----------

    stanley_pub_steer_cmd = nh.advertise<std_msgs::Float64>("/steer_cmd_stanley", 10);
    stanley_pub_reference_pose = nh.advertise<geometry_msgs::PoseStamped>("reference_pose", 10);
    stanley_pub_target_pose = nh.advertise<geometry_msgs::PoseStamped>("target_pose", 10);
    stanley_pub_target_path = nh.advertise<geometry_msgs::PoseArray>("target_path", 10);
    sub_local_trajectory = nh.subscribe("/control/local_trajectory", 5, &StanleyController::localTrajectoryCallback, this);
    sub_pose = nh.subscribe("/localization/estimation", 1, &StanleyController::poseCallback, this);

    printf("%sNode stanley_controller init done!\n", "\x1B[37m");
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}

StanleyController::~StanleyController()
{
    printf("\n%sNode stanley_controller shut down!\n", "\x1B[31m");
    printf("%s", "\x1B[0m");
}

void StanleyController::localTrajectoryCallback(const cyber_msgs::LocalTrajList::ConstPtr &path_in)
{
    if (path_in->points.size() == 0)
        return;
    //target_path.header = path_in->header;
    target_path.header.stamp = ros::Time::now();
    target_path.header.frame_id = "world";
    target_path.poses.clear();
    target_trajectory.clear();

    geometry_msgs::Pose target_point_;

    for (auto iter = path_in->points.begin(); iter != path_in->points.end(); iter++)
    {
        target_point_.position = iter->position;
        target_point_.orientation = iter->orientation;
        TrajPoint temp(target_point_, iter->mode);
        target_path.poses.push_back(target_point_);
        target_trajectory.push_back(temp);
    }

    temp_point = 0;
    path_flag = true;

    stanley_pub_target_path.publish(target_path);

    //std::cout<<"tmp_local.current point:"<<tmp_localTrajectory.tmp_current_point<<" global current point:"<< global_param.tmp_current_point<<std::endl;
}

void StanleyController::poseCallback(const cyber_msgs::LocalizationEstimate::ConstPtr &pose_in)
{

    if (!path_flag)
        return;

    current_pose.pose = pose_in->pose;
    current_yaw = tf::getYaw(pose_in->pose.orientation);
    current_vel = pose_in->velocity.linear.x;
    current_ang = pose_in->velocity.angular.z;

    //求得上个循环转角指令与实际转角的误差
    double R_cur;
    if (current_vel < 0.5) //防止低速时转弯半径的计算误差过大
    {
        R_cur = 0;
        current_steer_angle = 0;
        temp_steer_error = 0;
    }
    else if (current_ang == 0)
    {
        R_cur = 0;
        current_steer_angle = 0;
        temp_steer_error = former_steer_cmd - current_steer_angle;
    }
    else
    {
        R_cur = current_vel / current_ang;             //向左为正，通过线速度与角速度求得转弯半径
        current_steer_angle = atan(WHEELBASE / R_cur); //向左为正，WHEELBASE为前后轴距
        temp_steer_error = former_steer_cmd - current_steer_angle;
    }
    inter_steer_error = inter_steer_error + temp_steer_error - steer_error_queue.front(); //计算一段时间内前轮转角误差的积分值
    steer_error_queue.push(temp_steer_error);
    steer_error_queue.pop();

    //    ofstream fout;
    //    fout.open("/home/davy/relationship.txt",ios::app);
    //    if (current_steer_angle != 0)
    //        fout << current_steer_angle << "\t" << wheel_output.data <<"\n";
    //    fout.close();

    //计算控制参考点
    reference_pose = current_pose;
    reference_pose.pose.position.x += ref_distance * cos(current_yaw);
    reference_pose.pose.position.y += ref_distance * sin(current_yaw);
    reference_pose.pose.orientation = tf::createQuaternionMsgFromYaw(current_yaw + current_steer_angle);
    stanley_pub_reference_pose.publish(reference_pose);
    //计算控制目标点
    target_pose.header = current_pose.header;
    double temp_dis = sqrt(pow(reference_pose.pose.position.y - target_trajectory[temp_point].point.position.y, 2) + pow(reference_pose.pose.position.x - target_trajectory[temp_point].point.position.x, 2));
    for (int i = temp_point + 1; i < target_trajectory.size(); i++)
    {
        double next_dis = sqrt(pow(reference_pose.pose.position.y - target_trajectory[i].point.position.y, 2) + pow(reference_pose.pose.position.x - target_trajectory[i].point.position.x, 2));
        if (next_dis <= temp_dis)
        {
            temp_point = i;
            temp_dis = next_dis;
        }
        else
            break;
    }

    int further_temp_point = temp_point; //防止由于道路拼接导致的目标点搜索出现卡住（由于折返）
    for (int i = temp_point + 1; i <= temp_point + 20; i++)
    {
        if (i >= target_trajectory.size())
            break;
        double next_dis = sqrt(pow(reference_pose.pose.position.y - target_trajectory[i].point.position.y, 2) + pow(reference_pose.pose.position.x - target_trajectory[i].point.position.x, 2));
        if (next_dis < temp_dis) //使用小于可以防止通过同一个点
        {
            further_temp_point = i;
            temp_dis = next_dis;
        }
    }
    if (further_temp_point != temp_point)
    {
        temp_point = further_temp_point;
        ROS_INFO("Use Further Temp Point!");
    }

    target_pose.pose = target_trajectory[temp_point].point;
    stanley_pub_target_pose.publish(target_pose);

    //计算横向距离偏差与角度偏差
    double target_yaw;
    if (temp_point >= target_trajectory.size() - 1)
        target_yaw = atan2(target_trajectory[temp_point].point.position.y - target_trajectory[temp_point - 1].point.position.y,
                           target_trajectory[temp_point].point.position.x - target_trajectory[temp_point - 1].point.position.x);
    else
        target_yaw = atan2(target_trajectory[temp_point + 1].point.position.y - target_trajectory[temp_point].point.position.y,
                           target_trajectory[temp_point + 1].point.position.x - target_trajectory[temp_point].point.position.x);
    double delta_x = reference_pose.pose.position.x - target_pose.pose.position.x;
    double delta_y = reference_pose.pose.position.y - target_pose.pose.position.y;
    double dis_error = -(delta_x * sin(-target_yaw) + delta_y * cos(-target_yaw)); //向左为正
    double angle_error = target_yaw - current_yaw;                                 //向左为正
    if (angle_error < -M_PI)
        angle_error += 2 * M_PI;
    if (angle_error > M_PI)
        angle_error -= 2 * M_PI;

    //利用Stanley算法计算前轮转角
    double front_vel = current_vel / cos(current_steer_angle); //可以尝试使用current_steer_angle
    if (temp_point == target_path.poses.size() - 1)            //路径结束保持直行
        temp_steer_cmd = 0;
    else if (abs(front_vel) < 0.5)
        temp_steer_cmd = k_angle * angle_error + atan(k_error * dis_error / 0.5);
    else
        temp_steer_cmd = k_angle * angle_error + atan(k_error * dis_error / front_vel); //向左为正

    //前轮转角滤波与方向盘转角计算
    //final_steer_cmd = temp_steer_cmd;// + Kp_error * temp_steer_error + Ki_error * inter_steer_error / STEER_ERROR_QUEUE_SIZE;      //将Stanley算法的计算值叠加上转向误差PI控制
    final_steer_cmd = temp_steer_cmd + Kp_error * temp_steer_error + Ki_error * inter_steer_error / STEER_ERROR_QUEUE_SIZE; //将Stanley算法的计算值叠加上转向误差PI控制
    final_steer_cmd = filter_param * final_steer_cmd + (1 - filter_param) * former_steer_cmd;                               //对所求得的转向控制角进行低通滤波
    if (final_steer_cmd > WHEEL_MAX / Kp_wheel)
        final_steer_cmd = WHEEL_MAX / Kp_wheel;
    if (final_steer_cmd < -WHEEL_MAX / Kp_wheel)
        final_steer_cmd = -WHEEL_MAX / Kp_wheel; //限幅
    wheel_output.data = final_steer_cmd;         //Kp_wheel * final_steer_cmd + WHEEL_ZERO;
    former_steer_cmd = final_steer_cmd;
    ROS_INFO("Stanley Controller:\ndis_error: %f\t\tangle_error: %f\nfront_vel: %f\t\ttemp_steer_cmd: %f\ntemp_steer_error: %f\tinter_steer_error: %f\nfinal_steer_cmd: %f\twheel_output: %f",
             dis_error, angle_error, front_vel, temp_steer_cmd, temp_steer_error, inter_steer_error, final_steer_cmd, wheel_output.data);
    //stanley_pub_steer_cmd.publish(wheel_output);
    stanley_pub_steer_cmd.publish(wheel_output);
}

void StanleyController::configCallback(steer_controller::stanley_paramConfig &config, uint32_t level)
{
    if (config.update_param_)
    {
        ref_distance = config.ref_distance_;
        k_angle = config.k_angle_;
        k_error = config.k_error_;
        Kp_error = config.Kp_error_;
        Ki_error = config.Ki_error_;
        filter_param = config.filter_param_;
        Kp_wheel = config.Kp_wheel_;

        config.update_param_ = false;
    }
}

int main(int argc, char **argv)
{
    // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
    // if(!checkUSBKey()) return 0;

    ros::init(argc, argv, "stanley_controller");

    StanleyController StanleyController_obj;

    return 0;
}
