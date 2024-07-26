/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 **/

#pragma once

#include <vector>

#include "gtest/gtest_prod.h"

#include "osqp/osqp.h"
#include "spline_2d.h"
#include "spline_2d_solver.h"

namespace cyberc3 {
namespace planning {
class OsqpSpline2dSolver final : public Spline2dSolver {
 public:
  OsqpSpline2dSolver(const std::vector<double>& t_knots, const uint32_t order);

  void Reset(const std::vector<double>& t_knots, const uint32_t order) override;

  // customize setup
  Spline2dConstraint* mutable_constraint() override;
  Spline2dKernel* mutable_kernel() override;
  Spline2d* mutable_spline() override;

  // solve
  bool Solve() override;

  // extract
  const Spline2d& spline() const override;
  bool IsLastProblemSolved() const override { return last_problem_success_; };
  std::vector<double> GetLastSolution() const override { return last_x_; };

 private:
  FRIEND_TEST(OSQPSolverTest, basic_test);

 private:
  OSQPSettings* osqp_settings_ = nullptr;
  OSQPWorkspace* work_ = nullptr;  // Workspace
  OSQPData* data_ = nullptr;       // OSQPData

  int last_num_constraint_ = 0;
  int last_num_param_ = 0;
  bool last_problem_success_ = false;
  std::vector<double> last_x_;
  std::vector<double> last_y_;
};

template <typename T, int M, int N, typename D>
void DenseToCSCMatrix(const Eigen::Matrix<T, M, N>& dense_matrix,
                      std::vector<T>* data, std::vector<D>* indices,
                      std::vector<D>* indptr) {
  static constexpr double epsilon = 1e-9;
  int data_count = 0;
  for (int c = 0; c < dense_matrix.cols(); ++c) {
    indptr->emplace_back(data_count);
    for (int r = 0; r < dense_matrix.rows(); ++r) {
      if (std::fabs(dense_matrix(r, c)) < epsilon) {
        continue;
      }
      data->emplace_back(dense_matrix(r, c));
      ++data_count;
      indices->emplace_back(r);
    }
  }
  indptr->emplace_back(data_count);
}

}  // namespace planning
}  // namespace cyberc3
