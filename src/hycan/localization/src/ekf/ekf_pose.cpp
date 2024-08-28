/**
  *ekf_pose.cpp
  *brief:ekf fusion of gps and slam and imu
  *author:Chen Xiaofeng
  *date:20191028
  **/

#include "ekf_pose.h"

void All_EKF_Pose::gpsStateUpdate(Eigen::Vector3d &Z_gps, const Eigen::Matrix3d &R_gps, double time)
{
    if (!gps_init_flag || !vel_init_flag)
    {
        X.head(3) = Z_gps;
        P.topLeftCorner(3,3) = R_gps;
        time_now = time;
        gps_init_flag = true;
        return;
    }

    if(Z_gps(2) > X(2) + M_PI) Z_gps(2) -= 2 * M_PI;
    else if (Z_gps(2) < X(2) - M_PI) Z_gps(2) += 2 * M_PI;

    //prediction
    double dt = time - time_now;
    time_now = time;
    statePrediction(dt);

    //update
    Eigen::MatrixXd H(3,5);
    Eigen::MatrixXd I(5,5);
    Eigen::MatrixXd S(3,3);
    Eigen::MatrixXd K(5,3);
    Eigen::Vector3d Y;

    H << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0;
    I = Eigen::MatrixXd::Identity(5,5);

    Y = Z_gps - H * X;
    S = H * P * H.transpose() + R_gps;
    K = P * H.transpose() * S.inverse();
    X = X + K * Y;
    P = (I - K * H) * P;

    constrainRadian(X(2));
}

void All_EKF_Pose::slamStateUpdate(Eigen::Vector3d &Z_slam, const Eigen::Matrix3d &R_slam, const double time)
{
    if (!gps_init_flag || !vel_init_flag)
    {
        X.head(3) = Z_slam;
        P.topLeftCorner(3,3) = R_slam;
        time_now = time;
        gps_init_flag = true;
        return;
    }

    if(Z_slam(2) > X(2) + M_PI) Z_slam(2) -= 2 * M_PI;
    else if (Z_slam(2) < X(2) - M_PI) Z_slam(2) += 2 * M_PI;

    if(data_record){
        data_file << std::endl << std::endl;
        data_file << std::fixed << std::setprecision(4)<<"slamState_indata: "<<Z_slam(0)<<'\t'<<Z_slam(1)<<'\t'<<Z_slam(2)/M_PI*180<<std::endl;
        data_file << std::fixed << std::setprecision(4)<<"slamStateUpdate: "<<X(0)<<'\t'<<X(1)<<'\t'<<X(2)/M_PI*180<<'\t'<<X(3)<<'\t'<<X(4)<<std::endl;
    }

    //prediction
    double dt = time - time_now;
    time_now = time;
    statePrediction(dt);
    if(data_record)
        data_file << std::fixed << std::setprecision(4)<<"slamState_aftpre:"<<X(0)<<'\t'<<X(1)<<'\t'<<X(2)/M_PI*180<<'\t'<<X(3)<<'\t'<<X(4)<<std::endl;

    if (Z_slam(2) > X(2) + M_PI)
        Z_slam(2) -= 2 * M_PI;
    else if (Z_slam(2) < X(2) - M_PI)
        Z_slam(2) += 2 * M_PI;

    //update
    Eigen::MatrixXd H(3,5);
    Eigen::MatrixXd I(5,5);
    Eigen::MatrixXd S(3,3);
    Eigen::MatrixXd K(5,3);
    Eigen::Vector3d Y;

    H << 1, 0, 0, 0, 0,
            0, 1, 0, 0, 0,
            0, 0, 1, 0, 0;
    I = Eigen::MatrixXd::Identity(5,5);

    Y = Z_slam - H * X;
    S = H * P * H.transpose() + R_slam;
    K = P * H.transpose() * S.inverse();
    X = X + K * Y;
    P = (I - K * H) * P;

    constrainRadian(X(2));
    if(data_record){
        data_file << std::fixed << std::setprecision(8)<<"Y="<<Y<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"S="<<S<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"S.inverse()="<<S.inverse()<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"K="<<K<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"X="<<X<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"P="<<P<<std::endl;
        data_file << std::fixed << std::setprecision(4)<<"slamState_final:"<<X(0)<<'\t'<<X(1)<<'\t'<<X(2)/M_PI*180<<'\t'<<X(3)<<'\t'<<X(4)<<std::endl;
    }
}

void All_EKF_Pose::velStateUpdate(const Eigen::Vector2d &Z_vel, const Eigen::Matrix2d &R_vel, double time, const bool ang_only)
{
    if (!vel_init_flag)
    {
        if(!ang_only){
            X.tail<2>() = Z_vel;
            P.bottomRightCorner(2,2) = R_vel;
            time_now = time;
            vel_init_flag = true;
        }
        return;
    }
    if(data_record){
        data_file << std::endl << std::endl;
        data_file << std::fixed << std::setprecision(4)<<"velStateIn:"<<'\t'<<Z_vel(0)<<'\t'<<Z_vel(1)<<std::endl;
        data_file << std::fixed << std::setprecision(4)<<"velStateUpdate:"<<'\t'<<X(0)<<'\t'<<X(1)<<'\t'<<X(2)/M_PI*180<<'\t'<<X(3)<<'\t'<<X(4)<<std::endl;
    }

    //prediction
    double dt = time - time_now;
    time_now = time;
    statePrediction(dt);
    if(data_record){
        data_file << std::fixed << std::setprecision(4)<<"velState_aftpre:"<<X(0)<<'\t'<<X(1)<<'\t'<<X(2)/M_PI*180<<'\t'<<X(3)<<'\t'<<X(4)<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"P_aftpre="<<P<<std::endl;
    }

    char z_num = ang_only ? 1 : 2;

    //update
    Eigen::MatrixXd H(z_num,5);
    Eigen::MatrixXd I(5,5);
    Eigen::MatrixXd S(z_num,z_num);
    Eigen::MatrixXd K(5,z_num);
    Eigen::VectorXd Y(z_num);
    Eigen::VectorXd Z(z_num);
    Eigen::MatrixXd R(z_num,z_num);

    if(!ang_only){
        H << 0, 0, 0, 1, 0,
             0, 0, 0, 0, 1;
    }
    else{
        H << 0, 0, 0, 0, 1;
    }

    Z = Z_vel.tail(z_num);
    R = R_vel.bottomRightCorner(z_num,z_num);

    I = Eigen::MatrixXd::Identity(5,5);

    Y = Z - H * X;
    S = H * P * H.transpose() + R;
    K = P * H.transpose() * S.inverse();
    X = X + K * Y;

    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    // P = (I - K * H) * P;

    constrainRadian(X(2));
    if(data_record){
        data_file << std::fixed << std::setprecision(8)<<"Y="<<Y<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"S="<<S<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"S.inverse()="<<S.inverse()<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"K="<<K<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"X="<<X<<std::endl;
        data_file << std::fixed << std::setprecision(8)<<"P="<<P<<std::endl;
        data_file << std::fixed << std::setprecision(4)<<"velState_final:"<<X(0)<<'\t'<<X(1)<<'\t'<<X(2)/M_PI*180<<'\t'<<X(3)<<'\t'<<X(4)<<std::endl;
    }
}

Eigen::VectorXd All_EKF_Pose::readX(double time,bool prediction)
{
    //prediction
    double dt = time - time_now;
    time_now = time;
    if(prediction){
        if(data_record)
            data_file << std::endl;
        statePrediction(dt);
    }

    return X;
}

Eigen::MatrixXd All_EKF_Pose::readP(double time,bool prediction)
{
    //prediction
    double dt = time - time_now;
    time_now = time;
    if(prediction) statePrediction(dt);

    return P;
}


void All_EKF_Pose::timeUpdate(double time)
{
    X(3) = 0.0001;
    X(4) = 0.0001;
    time_now = time;
}

void All_EKF_Pose::statePrediction(double dt)
{
    double x = X(0);
    double y = X(1);
    double yaw = X(2);
    double v = X(3);
    double w = X(4);

    Eigen::MatrixXd F(5,5);
    Eigen::MatrixXd Q(5,5);
    Eigen::MatrixXd G(5,2);
    Eigen::MatrixXd E(2,2);

    X(0) = x + v/w*(sin(yaw+w*dt)-sin(yaw));
    X(1) = y + v/w*(cos(yaw)-cos(yaw+w*dt));
    X(2) = yaw + w*dt;
    X(3) = v;
    X(4) = w;
    if(data_record)
        data_file << std::fixed << std::setprecision(4)<<"statePrediction:"<<'\t'<<X(0)<<'\t'<<X(1)<<'\t'<<X(2)/M_PI*180<<'\t'<<X(3)<<'\t'<<X(4)<<std::endl;

    F << 1, 0, (v*(cos(yaw + dt*w) - cos(yaw)))/w,  (sin(yaw + dt*w) - sin(yaw))/w, (dt*v*cos(yaw + dt*w))/w - (v*(sin(yaw + dt*w) - sin(yaw)))/pow(w,2),
            0, 1, (v*(sin(yaw + dt*w) - sin(yaw)))/w, -(cos(yaw + dt*w) - cos(yaw))/w, (v*(cos(yaw + dt*w) - cos(yaw)))/pow(w,2) + (dt*v*sin(yaw + dt*w))/w,
            0, 0,                                  1,                               0,                                                                   dt,
            0, 0,                                  0,                               1,                                                                    0,
            0, 0,                                  0,                               0,                                                                    1;
    G << cos(yaw)*pow(dt,2)/2, 0,
            sin(yaw)*pow(dt,2)/2, 0,
            0, pow(dt,2)/2,
            dt, 0,
            0, dt;
    E << pow(5,2), 0,
            0, pow(0.1,2);      //5 0.1
    Q = G * E * G.transpose();
    P = F * P * F.transpose() + Q;

    constrainRadian(X(2));
}

void All_EKF_Pose::constrainRadian(double &x)
{
    if (x >= M_PI)
        x -= 2*M_PI;
    if (x < -M_PI)
        x += 2*M_PI;
}

void All_EKF_Pose::resetState()
{
    X.setZero();
    P.setZero();
    gps_init_flag=false;
    vel_init_flag=false;
    time_now=0;
}
