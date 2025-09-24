#include "rover_control/self_tuning_regulator.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include <algorithm>

using Eigen::VectorXd;
using Eigen::MatrixXd;

void printM(MatrixXd M){
  std::cout << M << std::endl << std::endl;
}

void SelfTuningRegulator::init(int& state_dim, int& input_dim, int& state_history, int& input_history, double forgettingfactor)
{
    step_ = 0;
    n_ = state_dim;
    m_ = input_dim;
    p_ = state_history;
    r_ = input_history;
    s_ = n_ * p_ + m_ * r_;

    lambda_ = forgettingfactor;

    Theta_ = MatrixXd::Random(s_, n_);
    phi_ = VectorXd::Zero(s_);

    p_states_ = VectorXd::Zero(n_*p_);
    p_inputs_ = VectorXd::Zero(m_*r_);

    K_ = VectorXd::Zero(s_);
    Cov_ = MatrixXd::Identity(s_, s_) * 1e6;

    A_ = MatrixXd::Identity(n_, n_*p_);
    B_ = MatrixXd::Identity(n_, m_*r_);
}

void SelfTuningRegulator::reset()
{
    step_ = 0;
    Theta_ = MatrixXd::Ones(s_, n_);
    phi_ = VectorXd::Zero(s_);

    p_states_ = VectorXd::Zero(n_*p_);
    p_inputs_ = VectorXd::Zero(m_*r_);

    K_ = VectorXd::Random(s_);
    Cov_ = MatrixXd::Identity(s_, s_) * 1e6;

    A_ = MatrixXd::Identity(n_, n_*p_);
    B_ = MatrixXd::Identity(n_, m_*r_);
}

void SelfTuningRegulator::set_frequency(int& freq)
{
    update_freq_ = freq;
}

void SelfTuningRegulator::set_bounds(double& param_bound, double& control_bound)
{
    theta_bound_ = param_bound;
    u_bound_ = control_bound;
}

void SelfTuningRegulator::set_covariance(double& initial_covariance)
{
    Cov_ = MatrixXd::Identity(s_, s_) * initial_covariance;
}

VectorXd SelfTuningRegulator::compute_input(VectorXd& desired, VectorXd& current, VectorXd& prev_input)
{
    if (step_ < std::max(p_,r_)) {
      phi_update(current, prev_input);
      std::cout << "Not enough history to solve parameter_estimation, skipping" << std::endl;
      step_++;
      auto error = desired - current;
      return VectorXd::Zero(m_);
    }

    phi_ << p_states_, p_inputs_;
    if(step_ % update_freq_ == 0){
        parameter_estimation(current);

        // Extract A and B matrices from current parameter estimate
        A_ = Theta_.transpose().block(0, 0, n_, n_*p_);
        B_ = Theta_.transpose().block(0, n_*p_, n_, m_*r_);
    }

    phi_update(current, prev_input);

    MatrixXd B_old;
    if (r_ > 1){
      B_old = B_.block(0, m_, n_, m_*(r_-1));

    } else {
      B_old = MatrixXd::Zero(n_, m_*(r_-1));
    }
    // error = desired - predicted
    VectorXd gamma = desired - A_ * p_states_ - B_old * p_inputs_.segment(0, m_*(r_-1));

    // VectorXd noise = 0.0001 * VectorXd::Random(m_/* ) */;
    VectorXd input = step_ahead_control(gamma); // + noise;

    step_++;
    return input;
}

void SelfTuningRegulator::phi_update(VectorXd& state, VectorXd& input){

    VectorXd old_states = p_states_.segment(0, n_*(p_-1));
    VectorXd old_inputs = p_inputs_.segment(0, m_*(r_-1));
    p_states_ << state, old_states;
    p_inputs_ << input, old_inputs;
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
    Theta_ = Theta_.cwiseMin(theta_bound_).cwiseMax(-theta_bound_);

    covariance_update();
    // std::cout << "Covariance: \n" << Cov_ << std::endl << std::endl;
}

void SelfTuningRegulator::covariance_update(){

    // Cov_ = MatrixXd::Identity(s_, s_) * 1e6;
    MatrixXd IKPhi = MatrixXd::Identity(s_, s_) - K_ * phi_.transpose();
    Cov_ = (IKPhi * Cov_ * IKPhi.transpose()) / lambda_;
    Cov_ = (Cov_ + Cov_.transpose()) / 2.0;

    Eigen::SelfAdjointEigenSolver<MatrixXd> eigendecomp(Cov_);
    auto eigen_values = eigendecomp.eigenvalues();
    eigen_values = eigen_values.cwiseMax(1e-6);

    Cov_ = eigendecomp.eigenvectors()*eigen_values.asDiagonal()*eigendecomp.eigenvectors().inverse();
}

VectorXd SelfTuningRegulator::step_ahead_control(VectorXd& error){

    MatrixXd B_current = B_.block(0, 0, n_, m_);
    // std::cout << "B current:\n " << B_current << std::endl;
    VectorXd input(m_);

    Eigen::JacobiSVD<MatrixXd> B_svd(B_current);
    if(B_svd.rank() < std::min(B_current.cols(), B_current.rows())){
      input = B_svd.solve(error);
    } else{
      input = B_current.inverse() * error;
    }

    input = input.cwiseMin(u_bound_).cwiseMax(-u_bound_);
    // std::cout << "Computed input: \n" << input << std::endl;
    return input;
}

double SelfTuningRegulator::get_error(const VectorXd& desired){
    auto error = desired - Theta_.transpose()*phi_;
    return std::abs(error(0));
}
