#include "rear_feedback.h"

// USB Key，只在生成发布版的时候才解注释
// #include "../../../common/Pwd_8/SoftkeyPWD.h"

RearFeedback::RearFeedback()
{
    //------成员变量初始化-------------
    path_flag = false;
    temp_point = 0;
    former_steer_cmd = 0;
    target_path.poses.clear();

    inter_steer_error = 0;
    for (int i = 0; i < STEER_ERROR_QUEUE_SIZE; i++)
        steer_error_queue.push(0);

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
    sub_local_trajectory = nh.subscribe("/control/local_trajectory", 5, &RearFeedback::localTrajectoryCallback, this);
    sub_pose = nh.subscribe("/localization/estimation", 1, &RearFeedback::poseCallback, this);

    printf("%sNode stanley_controller init done!\n", "\x1B[37m");
    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();
}

RearFeedback::~RearFeedback()
{
    printf("\n%sNode stanley_controller shut down!\n", "\x1B[31m");
    printf("%s", "\x1B[0m");
}

void RearFeedback::localTrajectoryCallback(const cyber_msgs::LocalTrajList::ConstPtr &path_in)
{
    if (path_in->points.size() == 0)
        return;
    //target_path.header = path_in->header;
    target_path.header.stamp = ros::Time::now();
    target_path.header.frame_id = "world";
    target_path.poses.clear();
    target_trajectory.points.clear();

    geometry_msgs::Pose target_point_;

    for (auto iter = path_in->points.begin(); iter != path_in->points.end(); iter++)
    {
        target_point_.position = iter->position;
        target_point_.orientation = iter->orientation;
        target_path.poses.push_back(target_point_);
    }

    target_trajectory = *path_in;

    temp_point = 0;
    path_flag = true;

    stanley_pub_target_path.publish(target_path);

    //std::cout<<"tmp_local.current point:"<<tmp_localTrajectory.tmp_current_point<<" global current point:"<< global_param.tmp_current_point<<std::endl;
}

void RearFeedback::poseCallback(const cyber_msgs::LocalizationEstimate::ConstPtr &pose_in)
{

    if (!path_flag)
        return;

    current_pose.header.frame_id = pose_in->header.frame_id;
    current_pose.header.stamp = ros::Time::now();
    current_pose.pose = pose_in->pose;
    stanley_pub_reference_pose.publish(current_pose);
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

    //计算控制目标点
    target_pose.header = current_pose.header;
    double temp_dis = sqrt(pow(current_pose.pose.position.y - target_trajectory.points[temp_point].position.y, 2) + pow(current_pose.pose.position.x - target_trajectory.points[temp_point].position.x, 2));
    for (int i = temp_point + 1; i < target_trajectory.points.size(); i++)
    {
        double next_dis = sqrt(pow(current_pose.pose.position.y - target_trajectory.points[i].position.y, 2) + pow(current_pose.pose.position.x - target_trajectory.points[i].position.x, 2));
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
        if (i >= target_trajectory.points.size())
            break;
        double next_dis = sqrt(pow(current_pose.pose.position.y - target_trajectory.points[i].position.y, 2) + pow(current_pose.pose.position.x - target_trajectory.points[i].position.x, 2));
        if (next_dis < temp_dis)
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

    target_pose.pose.position = target_trajectory.points[temp_point].position;
    target_pose.pose.orientation = target_trajectory.points[temp_point].orientation;
    stanley_pub_target_pose.publish(target_pose);

    //计算横向距离偏差与角度偏差
    double ks = target_trajectory.points[temp_point].kappa;
    double target_yaw = target_trajectory.points[temp_point].theta;
    double delta_x = target_pose.pose.position.x - current_pose.pose.position.x;
    double delta_y = target_pose.pose.position.y - current_pose.pose.position.y;
    double dis_error = delta_x * sin(target_yaw) - delta_y * cos(target_yaw); //向左为正
    double angle_error = current_yaw - target_yaw;                            //向左为正
    if (angle_error < -M_PI)
        angle_error += 2 * M_PI;
    if (angle_error > M_PI)
        angle_error -= 2 * M_PI;

    double ek;
    if (fabs(1 - ks * dis_error) < 0.5)
        ek = 0;
    else
        ek = ks * cos(angle_error) / 1 - ks * dis_error;
    if (fabs(angle_error) < LITTLE)
        angle_error = LITTLE;

    //利用Stanley算法计算前轮转角
    double front_vel;
    if (temp_point == target_path.poses.size() - 1) //路径结束保持直行
        temp_steer_cmd = 0;
    else
        temp_steer_cmd = atan(WHEELBASE * (ek - k_angle * angle_error - k_error * dis_error * sin(angle_error) / angle_error));

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
    ROS_INFO("Rear_feedback Controller:\ndis_error: %f\t\tangle_error: %f\nfront_vel: %f\t\ttemp_steer_cmd: %f\ntemp_steer_error: %f\tinter_steer_error: %f\nfinal_steer_cmd: %f\twheel_output: %f",
             dis_error, angle_error, front_vel, temp_steer_cmd, temp_steer_error, inter_steer_error, final_steer_cmd, wheel_output.data);
    //stanley_pub_steer_cmd.publish(wheel_output);
    stanley_pub_steer_cmd.publish(wheel_output);
}

int main(int argc, char **argv)
{
    // 只在生成发布版的时候才解注释，否则在自己电脑上无法调试
    // if(!checkUSBKey()) return 0;

    ros::init(argc, argv, "parkingcontrol");

    RearFeedback rearfeedback;

    return 0;
}
