/**
 * Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 * @file linear_quadratic_regulator.h
 * @author Chun Lin (sjtulc@sjtu.edu.cn)
 * @brief Solver for discrete-time linear quadratic problem.
 * @version IVFC20
 * @date 2020-09-23
 *
 * @maintainer
 */

#pragma once

#include "Eigen/Core"

/**
 * @namespace cyberc3::common::math
 * @brief cyberc3::common::math
 */
namespace cyberc3 {
namespace common {
namespace math {

/**
 * @brief Solver for discrete-time linear quadratic problem.
 * @param A The system dynamic matrix
 * @param B The control matrix
 * @param Q The cost matrix for system state
 * @param R The cost matrix for control output
 * @param M is the cross term between x and u, i.e. x'Qx + u'Ru + 2x'Mu
 * @param tolerance The numerical tolerance for solving Discrete
 *        Algebraic Riccati equation (DARE)
 * @param max_num_iteration The maximum iterations for solving ARE
 * @param ptr_K The feedback control matrix (pointer)
 */
void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const Eigen::MatrixXd &M, const double tolerance,
                     const uint max_num_iteration, Eigen::MatrixXd *ptr_K);

/**
 * @brief Solver for discrete-time linear quadratic problem.
 * @param A The system dynamic matrix
 * @param B The control matrix
 * @param Q The cost matrix for system state
 * @param R The cost matrix for control output
 * @param tolerance The numerical tolerance for solving Discrete
 *        Algebraic Riccati equation (DARE)
 * @param max_num_iteration The maximum iterations for solving ARE
 * @param ptr_K The feedback control matrix (pointer)
 */
void SolveLQRProblem(const Eigen::MatrixXd &A, const Eigen::MatrixXd &B,
                     const Eigen::MatrixXd &Q, const Eigen::MatrixXd &R,
                     const double tolerance, const uint max_num_iteration,
                     Eigen::MatrixXd *ptr_K);

}  // namespace math
}  // namespace common
}  // namespace cyberc3
