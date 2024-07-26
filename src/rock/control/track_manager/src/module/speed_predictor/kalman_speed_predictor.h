/**
 * @file kalman_speed_predictor.h
 * @author Chen Hongxin (angelochen@live.cn)
 * @brief
 * @version 0.1
 * @date 2023-11-23
 *
 * @copyright Copyright (c) 2023
 *
 */

#pragma once

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include "speed_predictor.h"

namespace cyberc3 {
namespace planning {

class KFBase {
 private:
  /* data */
 public:
  KFBase() = default;
  virtual ~KFBase() = default;
  void predict(const double& stamp);
  void update(const Eigen::VectorXd& z);
  void updateOnce(const double& stamp, const Eigen::VectorXd* z = nullptr);

  virtual void init(const double& stamp, const Eigen::VectorXd& x) {}

  Eigen::VectorXd x() const { return this->x_; }
  Eigen::MatrixXd P() const { return this->P_; }
  Eigen::MatrixXd S() const { return this->S_; }
  void setStateCoveriance(const Eigen::MatrixXd& P) { this->P_ = P; }

  void setState(const Eigen::VectorXd& x) { this->x_ = x; }

  void setCurrentTimeStamp(const double& stamp) {
    this->dt_ = stamp - this->current_time_stamp_;
    // std::cout << std::fixed << "stamp: " << stamp << std::endl;
    // std::cout << std::fixed << "dt: " << dt_ << std::endl;
    // std::cout << std::fixed << "S det:" << this->S().determinant() << std::endl;
    if (this->dt_ < 0)
      this->dt_ = 1e-4;
    else {
      this->current_time_stamp_ = stamp;
    }
  }
  double stamp() const { return this->current_time_stamp_; }
  double likelihood() const { return this->likelihood_; }
  virtual KFBase* clone() = 0;
  // virtual void init(const Eigen::MatrixXd &P, const Eigen::MatrixXd &R, const
  // double &dt) = 0;
  bool getStatePredictions(double stamp, double dt, size_t horizon,
                           std::vector<Eigen::VectorXd>* predicted_x);

 protected:
  Eigen::MatrixXd F_;  // 状态转移矩阵
  Eigen::MatrixXd H_;  // 测量矩阵
  Eigen::MatrixXd Q_;  // 系统协方差矩阵
  Eigen::MatrixXd R_;  // 测量协方差矩阵
  Eigen::MatrixXd J_;  // 雅可比阵
  Eigen::MatrixXd P_;  // 过程协方差矩阵
  Eigen::MatrixXd S_;
  Eigen::VectorXd x_;  // 状态向量
  Eigen::VectorXd z_;  // 测量向量

  std::vector<bool> angle_mask_;
  double likelihood_ = 0.0;
  double dt_ = 0.0;
  double current_time_stamp_ = -1;
  virtual void updateMatrixF(double dt) = 0;
  virtual void updatePrediction() = 0;
  virtual void updateMeasurement() = 0;

  static double normalizeAngle(const double raw_angle)  // 象限问题
  {
    int n = 0;
    double angle = 0;
    n = raw_angle / (M_PI * 2);
    angle = raw_angle - (M_PI * 2) * n;
    if (angle > M_PI) {
      angle = angle - (M_PI * 2);
    } else if (angle <= -M_PI) {
      angle = angle + (M_PI * 2);
    }

    return angle;
  }
};

class CV : public KFBase {
 private:
  void updateMatrixF(double dt) override;
  void updatePrediction() override;
  void updateMeasurement() override;

 public:
  void init(const double& stamp, const Eigen::VectorXd& x) override;
  CV* clone() override { return new CV(*this); }
};

class CA : public KFBase {
 private:
  void updateMatrixF(double dt) override;
  void updatePrediction() override;
  void updateMeasurement() override;

 public:
  void init(const double& stamp, const Eigen::VectorXd& x) override;
  CA* clone() override { return new CA(*this); }
};

class Singer : public KFBase {
 private:
  void updateMatrixF(double dt) override;
  void updatePrediction() override;
  void updateMeasurement() override;
  double alpha_ = 0.5;
  double sigma_ = 2.0;

 public:
  void init(const double& stamp, const Eigen::VectorXd& x) override;
  Singer* clone() override { return new Singer(*this); }
};

class IMM
{
private:
    std::vector<std::shared_ptr<KFBase>> models_;
    Eigen::MatrixXd transfer_prob_;
    Eigen::MatrixXd P_;
    Eigen::MatrixXd X_;
    Eigen::VectorXd c_;
    Eigen::VectorXd model_prob_;
    Eigen::VectorXd x_;
    size_t model_num_;
    size_t state_num_;
    double current_time_stamp_;
public:
    void addModel(const std::shared_ptr<KFBase>& model);
    void init(
        const Eigen::MatrixXd& transfer_prob,
        const Eigen::VectorXd& model_prob,
        const Eigen::VectorXd& x,
        const double& dt);
    void stateInteraction();
    void updateState(const double& stamp, const Eigen::VectorXd* z = nullptr);
    void updateModelProb();
    void estimateFusion();
    void updateOnce(const double& stamp, const Eigen::VectorXd* z = nullptr);
    Eigen::VectorXd x() const {return x_;}
    IMM(const IMM& imm) {
        transfer_prob_ = imm.transfer_prob_;
        P_ = imm.P_;
        X_ = imm.X_;
        c_ = imm.c_;
        model_prob_ = imm.model_prob_;
        x_ = imm.x_;
        model_num_ = imm.model_num_;
        state_num_ = imm.state_num_;
        for (size_t i = 0; i < imm.models_.size(); i++) {
            std::shared_ptr<KFBase> m = std::shared_ptr<KFBase>(imm.models_[i]->clone());
            this->models_.push_back(m);
        }
    }
    double stamp() const {return current_time_stamp_;}
    IMM* clone() {return new IMM(*this);}
    Eigen::VectorXd getModelProb() { return model_prob_; }

    IMM() : model_num_(0), current_time_stamp_(-1){};
    ~IMM()=default;
    };

class KalmanSpeedPredictor : public BaseSpeedPredictor {
 public:
  KalmanSpeedPredictor();
  ~KalmanSpeedPredictor() override = default;
  void FeedTargetV2V(const V2VPacket& v2v_msg) override;
  void FeedTargetPerception(const VehicleState& target_state) override;
  bool GetPredictedSpeedProfiles(
      double timestamp, std::array<double, N_HORIZON>* predicted_s,
      std::array<double, N_HORIZON>* predicted_speeds) override;
  bool GetTargetStates(double timestamp, double* target_s, double* target_speed,
                       double* target_acc) override;  // only for debug

  std::vector<double> GetIMMProbs() override;
  std::vector<double> GetStatePredictDebugs(double timestamp) override;

 private:
  std::unique_ptr<KFBase> kf_ptr_;
  // std::unique_ptr<IMM> kf_ptr_;
  bool is_init_ = false;
};
}  // namespace planning
}  // namespace cyberc3
