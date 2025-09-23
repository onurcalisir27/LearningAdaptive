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

void SelfTuningRegulator::set_frequency(int& param_freq, int& system_freq)
{
    parameter_update_freq_ = param_freq;
    system_update_freq_ = system_freq;
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
    if (step_ < p_ + r_) {
      phi_update(current, prev_input);
      std::cout << "Not enough history to solve parameter_estimation, skipping..." << std::endl;
      step_++;
      return VectorXd::Zero(m_);
    }

    if(step_ > 10000 && current != desired){
        reset();
        step_  = 0;
        return VectorXd::Zero(m_);
    }

    // Construct phi from previous states and previous inputs (at time k-1)
    phi_ << -p_states_, p_inputs_;
    std::cout << "Phi: \n" << phi_ << std::endl;

    // Update parameter estimate
    if (step_ % parameter_update_freq_ == 0){
        parameter_estimation(current);
    }

    if(step_ % system_update_freq_ == 0){

        // Extract A and B matrices from current parameter estimate
        A_ = Theta_.transpose().block(0, 0, n_, n_*p_);
        B_ = Theta_.transpose().block(0, n_*p_, n_, m_*r_);
    }

    // Append the current state to previous states
    VectorXd old_states = p_states_.segment(0, n_*(p_-1));
    VectorXd old_inputs = p_inputs_.segment(0, m_*(r_-1));
    p_states_ << current, old_states;

    // One-step-ahead control law
    // Model: y(k+1) = A * Y(k) + B * U(k)
    // where Y(k) = [y(k), y(k-1), ..., y(k-p+1)]^T
    // and U(k) = [u(k), u(k-1), ..., u(k-r+1)]^T

    // For one-step ahead: y_ref(k+1) = A * Y(k) + B * U(k)
    // We need to solve for u(k) (the first element of U(k))
    MatrixXd B_old;
    if (r_ > 1){
      // Means we have other input history in our B matrix, we need to
      // dissect B into 2 parts
      B_old = B_.block(0, m_, n_, m_*(r_-1));

    } else {
      B_old = MatrixXd::Zero(n_, m_*(r_-1));
    }

    // error = desired - predicted
    VectorXd gamma = desired - A_ * p_states_ - B_old * old_inputs;
    // std::cout << "System error: \n" << gamma << std::endl;

    VectorXd noise = 0.01 * VectorXd::Random(m_);
    VectorXd input = step_ahead_control(gamma) + noise;

    p_inputs_ << input, old_inputs;
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

    // Compute the current prediction error: y(k) - theta(k-1)^T * phi(k-1)
    auto prediction_error = current - Theta_.transpose() * phi_;
    // std::cout << "Prediction error: \n" << prediction_error << std::endl;

    // RLS Update:
    auto phiPphi = phi_.transpose() * ( Cov_ * phi_);
    double denominator = lambda_ + phiPphi;
    if (std::abs(denominator) < 1e-6) {
        denominator = std::copysign(1e-6, denominator);
        std::cout << "Warning: Near-singular denominator in Kalman gain!" << std::endl;
    }

    K_ = (Cov_ * phi_) / denominator;
    // std::cout << "Gain: \n" << K_ << std::endl;

    Theta_ = Theta_ + K_ * prediction_error.transpose();
    Theta_ = Theta_.cwiseMin(theta_bound_).cwiseMax(-theta_bound_);
    // std::cout << "Parameters: \n" << Theta_ << std::endl;

    covariance_update();
    std::cout << "Covariance: \n" << Cov_ << std::endl << std::endl;
}

void SelfTuningRegulator::covariance_update(){

    Cov_ = Cov_ - K_ * phi_.transpose() * Cov_;
    Cov_ = Cov_ / lambda_;
    Cov_ = (Cov_ + Cov_.transpose()) / 2.0;

    Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(Cov_);
    auto eigenvalues = eigensolver.eigenvalues();
    std::cout << "Eigenvalues of Covariance Matrix: \n" << eigenvalues << std::endl;
}

VectorXd SelfTuningRegulator::step_ahead_control(VectorXd& error){

    // Solve for the new input u(k)
    MatrixXd B_current = B_.block(0, 0, n_, m_);
    VectorXd input(m_);

    // Check if B_current is rank defficient
    Eigen::JacobiSVD<MatrixXd> B_svd(B_current);
    if(B_svd.rank() < B_current.cols()){
      input = B_svd.solve(error);
    } else{
      input = B_current.inverse() * error;
    }
    // Apply control bounds
    input = input.cwiseMin(u_bound_).cwiseMax(-u_bound_);
    std::cout << "Computed input: \n" << input << std::endl;

    return input;
}
