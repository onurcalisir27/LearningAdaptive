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

    // p_states_ = VectorXd::Zero(n_*p_);
    // p_inputs_ = VectorXd::Zero(m_*r_);

    K_ = VectorXd::Zero(s_);
    Cov_ = MatrixXd::Identity(s_, s_) * 1e6;

    A_ = MatrixXd::Identity(n_, n_*p_);
    B_ = MatrixXd::Identity(n_, m_*r_);

    update_freq_ = 1;
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

void SelfTuningRegulator::update_forgetting_factor(double& forgettingfactor){
    lambda_ = forgettingfactor;
}

VectorXd SelfTuningRegulator::compute_input(VectorXd& desired, VectorXd& current, VectorXd& prev_input, std::chrono::duration<double> dt)
{
    if (step_ < std::max(p_,r_)) {
      phi_update(current, prev_input);
      std::cout << "Not enough history to solve parameter_estimation, skipping" << std::endl;
      step_++;
      return pid_controller(desired, current, dt);
    }

    phi_ << -p_states_, p_inputs_;
    if(step_ % update_freq_ == 0){
        parameter_estimation(current);
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

    VectorXd gamma = desired + A_ * p_states_ - B_old * p_inputs_.segment(0, m_*(r_-1));

    VectorXd input = step_ahead_control(gamma);
    step_++;

    return input;
}

double SelfTuningRegulator::str(double& desired, double& current, VectorXd& outputs, VectorXd& inputs)
{
    phi_ << -outputs, inputs;

    VectorXd current_state = VectorXd::Ones(n_) * current;

    rls(current_state);
    A_ << Theta_(0), Theta_(1);

    MatrixXd B_old(1,1);
    B_old << Theta_(3);

    MatrixXd B_current(1,1);
    B_old << Theta_(2);

    VectorXd xn(n_*p_);
    xn << current, outputs(0);

    auto yn = VectorXd::Ones(n_) * desired;
    VectorXd error = yn + A_ * xn - B_old * inputs(0);

    VectorXd control = B_old.inverse() * error;

    return control(0);
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
    if (std::abs(denominator) < 1e-4) {
        denominator = std::copysign(1e-4, denominator);
    }

    K_ = (Cov_ * phi_) / denominator;

    Theta_ = Theta_ + K_ * prediction_error.transpose();
    // Theta_ = Theta_.cwiseMin(theta_bound_).cwiseMax(-theta_bound_);

    covariance_update();
    // std::cout << "Covariance: \n" << Cov_ << std::endl << std::endl;
}

void SelfTuningRegulator::rls(VectorXd& current){

    auto prediction_error = current - Theta_.transpose() * phi_;
    // RLS Update:
    auto phiPphi = phi_.transpose() * Cov_ * phi_;
    double denominator = lambda_ + phiPphi;
    K_ = (Cov_ * phi_) / denominator;

    Theta_ = Theta_ + K_ * prediction_error;

    Cov_ = Cov_ - K_ * phi_.transpose() * Cov_ / lambda_;
}


void SelfTuningRegulator::covariance_update(){

    // Cov_ = MatrixXd::Identity(s_, s_) * 1e6;
    MatrixXd IKPhi = MatrixXd::Identity(s_, s_) - K_ * phi_.transpose();
    Cov_ = (IKPhi * Cov_ * IKPhi.transpose()) / lambda_;
    Cov_ = (Cov_ + Cov_.transpose()) / 2.0;

    // Eigen::SelfAdjointEigenSolver<MatrixXd> eigendecomp(Cov_);
    // auto eigen_values = eigendecomp.eigenvalues();
    // eigen_values = eigen_values.cwiseMax(1e-6);
    //
    // Cov_ = eigendecomp.eigenvectors()*eigen_values.asDiagonal()*eigendecomp.eigenvectors().inverse();
}

VectorXd SelfTuningRegulator::step_ahead_control(VectorXd& error){

    // MatrixXd B_current = B_.block(0, 0, n_, m_);
    MatrixXd B_current(1,1);
    B_current << Theta_(2);
    // std::cout << "B current:\n " << B_current << std::endl;
    VectorXd input(m_);

    Eigen::JacobiSVD<MatrixXd> B_svd(B_current);
    if(B_svd.rank() < std::min(B_current.cols(), B_current.rows())){
      input = B_svd.solve(error);
    } else{
      input = B_current.inverse() * error;
    }

    // input = input.cwiseMin(u_bound_).cwiseMax(-u_bound_);
    // std::cout << "Computed input: \n" << input << std::endl;
    return input;
}

VectorXd SelfTuningRegulator::get_error(const double& desired, const double& current){

    // Three Types of Errors we can visualize
    // 1. State Error : desired - current
    // 2. Estimation Error : current - Theta_.T * phi_
    // 3. Control Error : desired - Theta_.T * phi_
    auto prediction = Theta_.transpose() * phi_;
    auto state_error = desired - current;
    auto estimation_error = current - prediction(0);
    auto control_error = desired - prediction(0);

    VectorXd error_vector(3);
    error_vector << std::abs(state_error), std::abs(estimation_error), std::abs(control_error);

    return error_vector;
}

void SelfTuningRegulator::set_pid(std::tuple<double,double,double>gains){
    std::tie(kp_, ki_, kd_) = gains;
}

VectorXd SelfTuningRegulator::pid_controller(VectorXd& desired, VectorXd& current, std::chrono::duration<double> dt){

    auto error = desired - current;
    std::cout << "Compute Error: \n" << error << std::endl;
    integral += error(0) * dt.count();
    double derivative = (error(0) - p_error) / dt.count();
    double control = kp_ * error(0) + ki_ * integral + kd_ * derivative;
    p_error = error(0);
    VectorXd input = VectorXd::Ones(m_) * control;
    input = input.cwiseMin(u_bound_).cwiseMax(-u_bound_);
    if(kp_ == kd_ == ki_ == 0.0){
        return VectorXd::Zero(m_);
    }
    return input;
}


