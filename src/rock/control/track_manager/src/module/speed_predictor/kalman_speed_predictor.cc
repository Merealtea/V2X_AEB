/**
 * @file speed_predictor.cc
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief
 * @version 0.1
 * @date 2023-11-21
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include "kalman_speed_predictor.h"
#include <vector>
#include "Eigen/src/Core/Matrix.h"
#include "track_manager_type.h"

namespace cyberc3 {
namespace planning {

void KFBase::predict(const double& stamp) {
  setCurrentTimeStamp(stamp);
  if (this->dt_ < 1e-3) return;
  updatePrediction();
  this->P_ = this->F_ * this->P_ * this->F_.transpose() + this->Q_;
}

void KFBase::update(const Eigen::VectorXd& z) {
  //   updateMeasurement();
  Eigen::VectorXd cur_z;
  Eigen::MatrixXd cur_H;
  Eigen::MatrixXd cur_R;
  if (z.size() == 1) {
    cur_z = this->x().head(1);
    cur_H.resize(1, 3);
    cur_H << 1, 0, 0;
    cur_R = this->R_.block(0, 0, 1, 1);
  } else if (z.size() == 2) {
    cur_z = this->x().tail(2);
    cur_H.resize(2, 3);
    cur_H << 0, 1, 0,
             0, 0, 1;
    cur_R = this->R_.block(1, 1, 2, 2);
  } else if (z.size() == 3) {
    cur_z = this->x();
    cur_H.resize(3, 3);
    cur_H = Eigen::MatrixXd::Identity(3, 3);
    cur_R = this->R_;
  } else {
    return;
  }
  Eigen::MatrixXd S = cur_H * this->P_ * cur_H.transpose() + cur_R;
  Eigen::VectorXd v = z - cur_z;
  for (size_t i = 0; i < angle_mask_.size(); i++) {
    if (angle_mask_[i] == true)
      v(i) = normalizeAngle(v(i));  // 使角度在-pi~+pi之间
  }
  double det = S.determinant();
  this->S_ = S;
  S = S.inverse();
  this->likelihood_ =
      1.0 / sqrt(2 * M_PI * fabs(det)) *
      exp(-0.5 * v.transpose() * S * v);  // 假定模型残差服从高斯分布,计算似然值
  std::cout << "likelihood: " << this->likelihood_ << std::endl;

  S = 0.5 * (S + S.transpose());
  Eigen::MatrixXd K = this->P_ * (cur_H.transpose() * S);

  this->x_ = this->x_ + K * v;
  Eigen::MatrixXd I;
  I.setIdentity(3, 3);
  Eigen::MatrixXd C = (I - K * cur_H);
  this->P_ = C * this->P_ * C.transpose() + K * cur_R * K.transpose();
  this->P_ = this->P_ + 0.0001 * I;
}

void KFBase::updateOnce(const double& stamp, const Eigen::VectorXd* z) {
  if (z == nullptr) {
    predict(stamp);
  } else {
    predict(stamp);
    update(*z);
  }
}

bool KFBase::getStatePredictions(double stamp, double dt, size_t horizon,
                                 std::vector<Eigen::VectorXd>* predicted_x) {
  if (current_time_stamp_ < 1) return false;
  predict(stamp);
  updateMatrixF(dt);
  auto x = this->x();
  predicted_x->clear();
  for (int i = 0; i < horizon; i++) {
    predicted_x->push_back(x);
    x = this->F_ * x;
    // speed need to be positive
    x(1) = std::max(x(1), 0.0);
  }
  return true;
}

void CV::init(const double& stamp, const Eigen::VectorXd& x) {
  if (x.size() != 3) {
    std::cerr << "[error] Dismatch between State and CV model." << std::endl;
    return;
  }
  this->current_time_stamp_ = stamp;
  this->P_.setIdentity(3, 3);
  this->R_.resize(3, 3);
  this->R_ << 0.25, 0,  0,
              0, 0.25,  0,
              0,    0,  1;
  this->x_ = x;
  this->F_.resize(3, 3);
  this->H_.resize(3, 3);
  this->H_ << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  this->angle_mask_ = {false, false, false, false};
  this->z_.resize(3);
}

void CV::updateMatrixF(double dt) {
  this->F_ << 1, dt, 0,
              0,  1, 0,
              0,  0, 0;
}

void CV::updatePrediction() {
  updateMatrixF(dt_);
  this->x_ = this->F_ * this->x_;
  /*--------------------------------------------------------------------*\
  ** CALC Process Noice Q Matrix
  \*--------------------------------------------------------------------*/
  {
    double delta_1 = dt_;
    double delta_2 = dt_ * dt_;
    Eigen::Matrix<double, 3, 1> G;
    G << 1 / 2.0 * delta_2, dt_, 0;
    double E = 400;
    this->Q_ = G * E * G.transpose();
  }
}

void CV::updateMeasurement() {
    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = x_(2);
}


void CA::init(const double &stamp, const Eigen::VectorXd &x) {
    if (x.size() != 3) {
    std::cerr << "[error] Dismatch between State and CA model." << std::endl;
    return;
    }
    this->current_time_stamp_ = stamp;
    this->P_.setIdentity(3, 3);
    this->R_.resize(3, 3);
    this->R_ << 0.25, 0,  0,
                0, 0.25,  0,
                0,    0,  5;
    this->x_ = x;  // s,v,a
    this->F_.resize(3, 3);
    this->H_.resize(3, 3);
    this->H_ << 1, 0,  0,
                0, 1,  0,
                0, 0,  1;
    this->z_.resize(3);
}

void CA::updateMatrixF(double dt) {
    this->F_ <<     1, dt,  1/ 2.0 * dt * dt,
                    0,  1,  dt,
                    0,  0,  1;
}

void CA::updatePrediction() {
    updateMatrixF(dt_);
    this->x_ = this->F_ * this->x_;
    /*--------------------------------------------------------------------*\
    ** CALC Process Noice Q Matrix
    \*--------------------------------------------------------------------*/
    {
        double delta_1 = dt_;
        double delta_2 = dt_ * dt_;
        double delta_3 = dt_ * dt_ * dt_;
        Eigen::Matrix<double, 3, 1> G;
        G <<
                    1 / 6.0 * delta_3,
                    1 / 2.0 * delta_2,
                                  dt_;
        double E = 400;
        this->Q_ = G * E * G.transpose();
    }

}

void CA::updateMeasurement() {
    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = x_(2);
}

void Singer::init(const double& stamp, const Eigen::VectorXd& x) {
    if (x.size() != 3) {
    std::cerr << "[error] Dismatch between State and CA model." << std::endl;
    return;
    }
    this->current_time_stamp_ = stamp;
    this->P_.setIdentity(3, 3);
    this->R_.resize(3, 3);
    this->R_ << 0.25, 0,  0,
                0,  0.2,  0,
                0,    0,  5;
    this->x_ = x;  // s,v,a
    this->F_.resize(3, 3);
    this->H_.resize(3, 3);
    this->H_ << 1, 0,  0,
                0, 1,  0,
                0, 0,  1;
    this->z_.resize(3);
}

void Singer::updateMatrixF(double dt) {
    const double exp = std::exp(-alpha_ * dt);
    this->F_ <<     1, dt,  (alpha_ * dt - 1+exp) / (alpha_*alpha_),
                    0,   1,  (1-exp)/alpha_,
                    0,   0,     exp;
}

void Singer::updatePrediction() {
    updateMatrixF(dt_);
    this->x_ = this->F_ * this->x_;
    /*--------------------------------------------------------------------*\
    ** CALC Process Noice Q Matrix
    \*--------------------------------------------------------------------*/
    {
        double delta_1 = dt_;
        double delta_2 = dt_ * dt_;
        double delta_3 = dt_ * dt_ * dt_;

        double alpha_1 = alpha_;
        double alpha_2 = alpha_*alpha_;
        double alpha_3 = alpha_*alpha_*alpha_;

        double exp_1 = std::exp(-alpha_ * dt_);
        double exp_2 = std::exp(-2 * alpha_ * dt_);

        double q11 = (2 * alpha_3 * delta_3 - 6 * alpha_2 * delta_2 +
                      6 * alpha_1 * delta_1 + 3 -
                      12 * alpha_1 * delta_1 * exp_1 - 3 * exp_2) /
                     (6 * alpha_3 * alpha_2);

        double q12 = (alpha_2 * delta_2 - 2 * alpha_1 * delta_1 + 1 -
                      2 * (1 - alpha_1 * delta_1) * exp_1 + exp_2) /
                     (2 * alpha_2 * alpha_2);

        double q22 =
            (2 * alpha_1 * delta_1 - 3 + 4 * exp_1 - exp_2) / (2 * alpha_3);

        double q13 =
            (1 - 2 * alpha_1 * delta_1 * exp_1 - exp_2) / (2 * alpha_3);

        double q23 = (1 - 2 * exp_1 + exp_2) / (2 * alpha_2);

        double q33 = (1 - exp_2) / (2 * alpha_1);

       Eigen::Matrix<double, 3, 3> Q;
       Q << q11, q12, q13,
            q12, q22, q23,
            q13, q23, q33;

       this->Q_ = 2 * alpha_ * sigma_ * sigma_ * Q;
    }

}

void Singer::updateMeasurement() {
    this->z_(0) = x_(0);
    this->z_(1) = x_(1);
    this->z_(2) = x_(2);
}


void IMM::addModel(const std::shared_ptr<KFBase>& model) {
    this->models_.push_back(model);
    this->model_num_++;
}

void IMM::init (const Eigen::MatrixXd& transfer_prob,
                const Eigen::VectorXd& model_prob,
                const Eigen::VectorXd& x,
                const double& dt) {
    if (this->model_num_ == 0) {
        std::cerr << "[error] No valid model." << std::endl;
        exit(1);
    }

    if (transfer_prob.cols() != this->model_num_ || transfer_prob.rows() != this->model_num_)
    {
        std::cerr << "[error] Dimension of transfer probability matrix is not equal to number of models." << std::endl;
         exit(1);
    }
    if (model_prob.size() != this->model_num_)
    {
        std::cerr << "[error] Dimension of model probability vector is not equal to number of models." << std::endl;
         exit(1);
    }
    this->state_num_ = x.size();
    this->X_.resize(this->state_num_, this->model_num_);


    for (size_t i = 0; i < model_num_; i++) {
         this->models_[i]->init(dt, x);
         this->X_.col(i) = this->models_[i]->x();
    }

    this->transfer_prob_ = transfer_prob;
    this->model_prob_ = model_prob;
    this->x_ = x;
    this->c_.resize(this->model_num_);
}

void IMM::stateInteraction() {
    this->c_ = Eigen::VectorXd::Zero(this->model_num_);
    // c(j) = sum_i(transfer_prob(i,j)*model_prob(i))
    for (size_t j = 0; j < this->model_num_; j++) {
        for (size_t i = 0; i < this->model_num_; i++) {
            this->c_(j) += this->transfer_prob_(i, j) * this->model_prob_(i);
        }
    }
    // u(i,j) = 1 / c(j) * transfer_prob(i,j) * model_prob(i)
    // X(j) = sum_i(X(i) * U(i, j))

    Eigen::MatrixXd U = Eigen::MatrixXd::Zero(this->model_num_, this->model_num_);
    for (size_t i = 0; i < model_num_; i++) {
        this->X_.col(i) = this->models_[i]->x();
    }

    Eigen::MatrixXd X = this->X_;
    this->X_.fill(0);
    for (size_t j = 0; j < this->model_num_; j++) {
        for (size_t i = 0; i < this->model_num_; i++) {
            U(i, j) = 1.0 / this->c_(j) * this->transfer_prob_(i, j) * this->model_prob_(i);
            this->X_.col(j) += X.col(i) * U(i, j);
        }
    }

    for (size_t i = 0; i < this->model_num_; i++) {
        Eigen::MatrixXd P = Eigen::MatrixXd::Zero(this->state_num_, this->state_num_);
        for (size_t j = 0; j < this->model_num_; j++) {
            Eigen::VectorXd s = X.col(i) - this->X_.col(j);
            P += U(i,j) * (this->models_[i]->P() + s * s.transpose());
        }
        this->models_[i]->setStateCoveriance(P);
        this->models_[i]->setState(this->X_.col(i));
    }
}

void IMM::updateState(const double& stamp, const Eigen::VectorXd* z) {
    current_time_stamp_ = stamp;
    for (size_t i = 0; i < this->model_num_; i++) {
        this->models_[i]->predict(stamp);
        if (nullptr != z) {
            this->models_[i]->update(*z);
        }
    }
}

void IMM::updateModelProb() {
    double c_sum = 0;
    for (size_t i = 0; i < this->model_num_; i++) {
        c_sum += this->models_[i]->likelihood() * this->c_(i);
    }

    std::cout << "model_prob: " << std::endl;
    for (size_t i = 0; i < this->model_num_; i++) {
        this->model_prob_(i) = 1 / c_sum * this->models_[i]->likelihood() * this->c_(i);
        std::cout << this->model_prob_(i) << "\t";
    }
    std::cout << std::endl;
}

void IMM::estimateFusion() {
    this->x_ = this->X_ * this->model_prob_;

    for (size_t i = 0; i < this->model_num_; i++) {
        Eigen::MatrixXd v = this->X_.col(i) - this->x_;
        this->P_ = this->models_[i]->P() + v * v.transpose() * this->model_prob_[i];
    }
}

void IMM::updateOnce(const double& stamp, const Eigen::VectorXd* z) {
    if (z == nullptr) {
        stateInteraction();
        updateState(stamp);
        estimateFusion();
    } else {
        stateInteraction();
        updateState(stamp, z);
        updateModelProb();
        estimateFusion();
    }
}

KalmanSpeedPredictor::KalmanSpeedPredictor(){
    kf_ptr_ = std::make_unique<Singer>();
    // kf_ptr_ = std::make_unique<IMM>();
    // kf_ptr_->addModel(std::make_shared<CV>());
    // kf_ptr_->addModel(std::make_shared<CA>());
    // kf_ptr_->addModel(std::make_shared<Singer>());
}

void KalmanSpeedPredictor::FeedTargetPerception(
    const VehicleState& target_state) {
    std::lock_guard<std::mutex> lock(mutex_);
    target_state_ = target_state;
    if (!is_init_) {
        Eigen::VectorXd init_states(3);
        init_states << target_state_.s, target_state_.speed, 0;
        // target_state_.acc_x;
        // kf_ptr_->init(target_state_.timestamp_perception, init_states);
        // Eigen::MatrixXd transfer_prob = Eigen::MatrixXd::Zero(3,3);
        // transfer_prob <<0.8,  0.1, 0.1,
        //                 0.1,  0.8, 0.1,
        //                 0.1,  0.1, 0.8;
        // Eigen::VectorXd model_prob = Eigen::VectorXd::Zero(3);
        // model_prob << 0.5, 0.25, 0.25;
        // kf_ptr_->init(transfer_prob, model_prob, init_states,
        //               target_state_.timestamp_perception);
        kf_ptr_->init(target_state_.timestamp_perception, init_states);
        is_init_ = true;
    } else {
        Eigen::VectorXd measures(1);
        // measures << target_state_.s, target_state_.speed,
        // target_state_.acc_x; Eigen::VectorXd measures(2); measures <<
        // target_state_.s, target_state_.speed;
        measures << target_state_.s;
        kf_ptr_->updateOnce(target_state.timestamp_perception, &measures);
    }
}

void KalmanSpeedPredictor::FeedTargetV2V(const V2VPacket &v2v_msg){
    std::lock_guard<std::mutex> lock(mutex_);
    if(!is_init_) return;
    Eigen::VectorXd measures(2);
    measures << v2v_msg.speed, v2v_msg.acc_x;
    kf_ptr_->updateOnce(v2v_msg.timestamp, &measures);
}

bool KalmanSpeedPredictor::GetPredictedSpeedProfiles(
    double timestamp, std::array<double, N_HORIZON>* predicted_s,
    std::array<double, N_HORIZON>* predicted_speeds) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (predicted_s == nullptr || predicted_speeds == nullptr) {
        AERROR << "input pointer is nullptr!";
        return false;
    }
    if (!CheckTargetStateValidty(timestamp)) {
        return false;
    }
    std::vector<Eigen::VectorXd > predicted_states;
    predicted_states.reserve(N_HORIZON);
    kf_ptr_->getStatePredictions(timestamp, T_step, N_HORIZON,
                                 &predicted_states);
    for (int i = 0; i < N_HORIZON; ++i) {
        predicted_s->at(i) = predicted_states[i](0);
        predicted_speeds->at(i) = predicted_states[i](1);
    }
    return true;
}

bool KalmanSpeedPredictor::GetTargetStates(double timestamp, double* target_s,
                                           double* target_speed,
                                           double* target_acc) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_init_) return false;
    // kf_ptr_->predict(timestamp);
    kf_ptr_->updateOnce(timestamp);
    auto states = kf_ptr_->x();
    *target_s = states(0);
    *target_speed = states(1);
    *target_acc = states(2);
    return true;
}

std::vector<double> KalmanSpeedPredictor::GetIMMProbs() {
    std::lock_guard<std::mutex> lock(mutex_);
    // auto probs = kf_ptr_->getModelProb();
    // std::vector<double> ret;
    // for (int i = 0; i < probs.size(); i++) {
    //     ret.push_back(probs(i));
    // }
    // return ret;
    return {};
}

std::vector<double> KalmanSpeedPredictor::GetStatePredictDebugs(
    double timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Eigen::VectorXd> predicted_states;
    double predict_delta_time = 1.0;
    kf_ptr_->getStatePredictions(timestamp, predict_delta_time, 4,
                                 &predicted_states);
    std::vector<double> debug_speeds;
    // return predicted target speed at 1,2,3 s for debug
    debug_speeds.reserve(3);
    for (int i = 0; i < 3; i++) {
        debug_speeds.push_back(predicted_states[i + 1](1));
    }
    return debug_speeds;
}

}  // namespace planning
}  // namespace cyberc3
