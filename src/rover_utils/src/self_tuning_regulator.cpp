#include "rover_utils/self_tuning_regulator.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Core/util/Constants.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

namespace rover_utils
{

using Eigen::VectorXd;
using Eigen::MatrixXd;

void SelfTuningRegulator::init(int& state_dim, int& input_dim, int& state_history, int& input_history, double forgettingfactor)
{
    n_ = state_dim;
    m_ = input_dim;
    p_ = state_history;
    r_ = input_history;
    s_ = n_ * p_ + m_ * r_;
    lambda_ = forgettingfactor;
    Theta_ = MatrixXd::Random(s_, n_);
    phi_ = VectorXd::Zero(s_);

    // p_states_ = VectorXd::Zero(n_*p_);
    // p_inputs_ = VectorXd::Zero(m_*r_);

    K_ = VectorXd::Zero(s_);
    Cov_ = MatrixXd::Identity(s_, s_) * 1e6;

    A_ = MatrixXd::Identity(n_, n_*p_);
    B_ = MatrixXd::Identity(n_, m_*r_);
    B_current = MatrixXd::Identity(n_, m_);
    B_old = MatrixXd::Identity(n_, m_*(r_ -1));
}

void SelfTuningRegulator::reset()
{
    Theta_ = MatrixXd::Ones(s_, n_);
    phi_ = VectorXd::Zero(s_);

    p_states_ = VectorXd::Zero(n_*p_);
    p_inputs_ = VectorXd::Zero(m_*r_);

    K_ = VectorXd::Random(s_);
    Cov_ = MatrixXd::Identity(s_, s_) * 1e6;

    A_ = MatrixXd::Identity(n_, n_*p_);
    B_ = MatrixXd::Identity(n_, m_*r_);
}

void SelfTuningRegulator::set_bounds(VectorXd& control_bound)
{
  input_bounds = control_bound;
}

void limit(VectorXd& input, VectorXd& bound)
{
  if (input.size() != bound.size()){
    std::cerr << "Input and Bound size's don't match" << std::endl;
    return;
  }
  for (uint i=0; i < input.size(); ++i)
  {
    input(i) = std::clamp(input(i), -bound(i), bound(i));
  }
  return;
}

void SelfTuningRegulator::set_covariance(double& initial_covariance)
{
    Cov_ = MatrixXd::Identity(s_, s_) * initial_covariance;
}

void SelfTuningRegulator::update_forgetting_factor(double& forgettingfactor){
    lambda_ = forgettingfactor;
}

VectorXd SelfTuningRegulator::compute_input(VectorXd& desired, VectorXd& current, VectorXd& outputs, VectorXd& inputs)
{
    phi_ << -outputs, inputs;
    parameter_estimation(current);
    A_ = Theta_.transpose().block(0, 0, n_, n_*p_);
    std::cout << "A matrix: \n" << A_ << std::endl;
    B_ = Theta_.transpose().block(0, n_*p_, n_, m_*r_);
    auto B_past = B_.block(0, m_, n_, m_*(r_-1));
    std::cout << "B past matrix: \n" << B_past << std::endl;
    VectorXd xn(n_*p_);
    xn << current, outputs.segment(0, n_*(p_-1));
    VectorXd gamma = desired + A_ * xn - B_past * inputs.segment(0, m_*(r_-1));
    VectorXd input = step_ahead_control(gamma);
    return input;
}

double SelfTuningRegulator::str(double& desired, double& current, VectorXd& outputs, VectorXd& inputs)
{
    phi_ << -outputs, inputs;
    VectorXd current_state = VectorXd::Ones(n_) * current;
    rls(current_state);
    A_ << Theta_(0), Theta_(1);
    B_old << Theta_(3);
    B_current << Theta_(2);
    VectorXd xn(n_*p_);
    xn << current, outputs(0);
    auto yn = VectorXd::Ones(n_) * desired;
    VectorXd error = yn + A_ * xn - B_old * inputs(0);
    double control = error(0) / B_current(0);
    control = std::clamp(control, -input_bounds(0), input_bounds(0));
    return control;
}

void SelfTuningRegulator::parameter_estimation(VectorXd& current){

    auto prediction_error = current - Theta_.transpose() * phi_;
    // RLS Update:
    auto phiPphi = phi_.transpose() * ( Cov_ * phi_);
    double denominator = lambda_ + phiPphi;
    if (std::abs(denominator) < 1e-6) {
        denominator = std::copysign(1e-6, denominator);
    }
    K_ = (Cov_ * phi_) / denominator;
    Theta_ = Theta_ + K_ * prediction_error.transpose();
    covariance_update();
}

void SelfTuningRegulator::rls(VectorXd& current){

    auto prediction_error = current - Theta_.transpose() * phi_;
    // RLS Update:
    auto phiPphi = phi_.transpose() * Cov_ * phi_;
    double denominator = lambda_ + phiPphi;
    K_ = (Cov_ * phi_) / denominator;

    Theta_ = Theta_ + K_ * prediction_error;
    Theta_ = Theta_.cwiseMin(10.0).cwiseMax(-10.0);
    auto temp = MatrixXd::Identity(s_, s_) - K_ * phi_.transpose();
    Cov_ = temp * Cov_ / lambda_;
}

void SelfTuningRegulator::covariance_update(){

    MatrixXd IKPhi = MatrixXd::Identity(s_, s_) - K_ * phi_.transpose();
    Cov_ = (IKPhi * Cov_ * IKPhi.transpose()) / lambda_;
    Cov_ = (Cov_ + Cov_.transpose()) / 2.0;

    Eigen::SelfAdjointEigenSolver<MatrixXd> eigendecomp(Cov_);
    auto eigen_values = eigendecomp.eigenvalues();
    eigen_values = eigen_values.cwiseMax(1e-6);
    Cov_ = eigendecomp.eigenvectors()*eigen_values.asDiagonal()*eigendecomp.eigenvectors().inverse();
}

VectorXd SelfTuningRegulator::step_ahead_control(VectorXd& error){
    auto B_current = B_.block(0,0,n_,m_);
    std::cout << "B current matrix: \n" << B_current << std::endl;
    Eigen::JacobiSVD<MatrixXd> B_svd(B_current, Eigen::ComputeThinU | Eigen::ComputeThinV);

    double cond = B_svd.singularValues()(0) / B_svd.singularValues()(B_svd.singularValues().size()-1);
    if (cond > 1e6) {
        std::cerr << "WARNING: B matrix is poorly conditioned! condition number = " << cond << std::endl;
    }
    VectorXd input = B_svd.solve(error);
    limit(input, input_bounds);
    return input;
}

std::tuple<VectorXd,VectorXd,VectorXd> SelfTuningRegulator::get_error(const VectorXd& desired, const VectorXd& current){

    // Three Types of Errors we can visualize
    // 1. State Error : desired - current
    // 2. Estimation Error : current - Theta_.T * phi_
    // 3. Control Error : desired - Theta_.T * phi_
    auto prediction = Theta_.transpose() * phi_;
    auto state_error = desired - current;
    auto estimation_error = current - prediction;
    auto control_error = desired - prediction;
    std::tuple<VectorXd,VectorXd,VectorXd> errors = {state_error, estimation_error, control_error};
    return errors;
}

void SelfTuningRegulator::set_pid(std::tuple<double,double,double>gains){
    std::tie(kp_, ki_, kd_) = gains;
}

VectorXd SelfTuningRegulator::pid_controller(VectorXd& desired, VectorXd& current, std::chrono::duration<double> dt){

    auto error = desired - current;
    std::cout << "Computed Error: \n" << error << std::endl;
    integral += error(0) * dt.count();
    double derivative = (error(0) - p_error) / dt.count();
    double control = kp_ * error(0) + ki_ * integral + kd_ * derivative;
    p_error = error(0);
    VectorXd input = VectorXd::Ones(m_) * control;
    limit(input, input_bounds);
    return input;
}

} // namespace rover_utils
