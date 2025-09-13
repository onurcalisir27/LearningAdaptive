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

    Theta_ = MatrixXd::Ones(s_, n_);
    phi_ = VectorXd::Zero(s_);

    p_states_ = VectorXd::Zero(n_*p_);
    p_inputs_ = VectorXd::Zero(m_*r_);

    K_ = VectorXd::Random(s_);
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
    // Skip parameter estimation for first few steps until we have enough data
    if (step_ > 0) {
        // Construct phi from previous states and previous inputs (at time k-1)
        phi_ << p_states_, p_inputs_;
        std::cout << "Phi: \n" << phi_ << std::endl;

        // Compute the current prediction error: y(k) - phi^T(k-1) * theta(k-1)
        VectorXd prediction_error = current - Theta_.transpose() * phi_;
        std::cout << "Prediction error: \n" << prediction_error << std::endl;

        // RLS Update: Compute Kalman Gain with numerical stability
        VectorXd P_phi = Cov_ * phi_;
        double denominator = lambda_ + phi_.transpose() * P_phi;

        // Add numerical stability check
        if (std::abs(denominator) < 1e-12) {
            denominator = std::copysign(1e-12, denominator);
            std::cout << "Warning: Near-singular denominator in Kalman gain!" << std::endl;
        }

        K_ = P_phi / denominator;
        std::cout << "Gain: \n" << K_ << std::endl;

        // Update Parameter estimation: theta(k) = theta(k-1) + K * e(k)
        for (int i = 0; i < n_; i++) {
            Theta_.col(i) += K_ * prediction_error(i);
        }

        // Apply parameter bounds
        Theta_ = Theta_.cwiseMin(theta_bound_).cwiseMax(-theta_bound_);
        std::cout << "Parameters: \n" << Theta_ << std::endl;

        // Update Covariance Matrix: P(k) = (P(k-1) - K * phi^T * P(k-1)) / lambda
        Cov_ = (Cov_ - K_ * phi_.transpose() * Cov_) / lambda_;

        // Ensure covariance remains positive definite
        Eigen::SelfAdjointEigenSolver<MatrixXd> eigensolver(Cov_);
        if (eigensolver.eigenvalues().minCoeff() < 1e-12) {
            // Regularize the covariance matrix
            Cov_ += MatrixXd::Identity(s_, s_) * 1e-6;
            std::cout << "Warning: Covariance regularized!" << std::endl;
        }

        std::cout << "Covariance: \n" << Cov_ << std::endl << std::endl;
    }

    // Extract A and B matrices from current parameter estimate
    A_ = Theta_.transpose().block(0, 0, n_, n_*p_);
    B_ = Theta_.transpose().block(0, n_*p_, n_, m_*r_);

    std::cout << "A Matrix: \n";
    printM(A_);
    std::cout << "B Matrix: \n";
    printM(B_);

    // One-step-ahead control law
    // Model: y(k+1) = A * Y(k) + B * U(k)
    // where Y(k) = [y(k), y(k-1), ..., y(k-p+1)]^T
    // and U(k) = [u(k), u(k-1), ..., u(k-r+1)]^T

    // For one-step ahead: y_ref(k+1) = A * Y(k) + B * U(k)
    // We need to solve for u(k) (the first element of U(k))

    VectorXd Y_current = p_states_; // Current state history
    VectorXd U_prev = p_inputs_;    // Previous input history

    // Predicted output without new control: y_pred = A * Y(k)
    VectorXd y_pred = A_ * Y_current;

    // Required correction: error = desired - predicted
    VectorXd error = desired - y_pred;
    std::cout << "Control error: \n" << error << std::endl;

    // Solve B * U(k) = error for the control input
    // Since we're updating only u(k), we need to account for previous inputs
    VectorXd U_new = VectorXd::Zero(m_*r_);

    // Copy previous inputs (shifted)
    if (r_ > 1) {
        U_new.segment(m_, m_*(r_-1)) = U_prev.segment(0, m_*(r_-1));
    }

    // Solve for the new input u(k) using pseudoinverse with better conditioning
    MatrixXd B_current = B_.block(0, 0, n_, m_); // Only the current input part

    Eigen::JacobiSVD<MatrixXd> svd(B_current, Eigen::ComputeFullU | Eigen::ComputeFullV);
    double tolerance = 1e-6 * std::max(B_current.rows(), B_current.cols()) * svd.singularValues().maxCoeff();

    VectorXd input;
    if (svd.singularValues().minCoeff() > tolerance) {
        // B matrix is well-conditioned
        VectorXd B_prev_contribution = VectorXd::Zero(n_);
        if (r_ > 1) {
            B_prev_contribution = B_.block(0, m_, n_, m_*(r_-1)) * U_prev.segment(0, m_*(r_-1));
        }
        input = svd.solve(error - B_prev_contribution);
    } else {
        // Fallback: use damped least squares
        MatrixXd BTB = B_current.transpose() * B_current;
        BTB.diagonal().array() += 1e-6; // Damping
        input = BTB.ldlt().solve(B_current.transpose() * error);
        std::cout << "Warning: Using damped least squares for control!" << std::endl;
    }

    // Apply control bounds
    input = input.cwiseMin(u_bound_).cwiseMax(-u_bound_);
    std::cout << "Computed input: \n" << input << std::endl;

    // Update state and input histories
    // Shift states: [current, previous_states]
    VectorXd new_states = VectorXd::Zero(n_*p_);
    new_states.segment(0, n_) = current;
    if (p_ > 1) {
        new_states.segment(n_, n_*(p_-1)) = p_states_.segment(0, n_*(p_-1));
    }
    p_states_ = new_states;

    // Shift inputs: [new_input, previous_inputs]
    U_new.segment(0, m_) = input;
    p_inputs_ = U_new;

    step_++;
    return input;
}
