#include "rover_control/self_tuning_regulator.hpp"
#include <iostream>
#include <cstdint>
#include <Eigen/Dense>

using Eigen::VectorXd;
using Eigen::MatrixXd;

void SelfTuningRegulator::init(int n, int m, int input_dim, int output_dim, double lambda, double init_cov){

    n_ = n;
    m_ = m;
    system_dim_ = output_dim * (n * output_dim + m * input_dim);
    phi_dim_ = n * output_dim + m * input_dim;
    output_dim_ = output_dim;
    input_dim_ = input_dim;
    lambda_ = lambda;
    initialized_ = false;
    step_count_ = 0;

    // alpha_ = VectorXd(output_dim_);    
    Theta_ = VectorXd::Random(system_dim_);
    L_ = MatrixXd::Zero(system_dim_, output_dim_);
    P_ = MatrixXd::Identity(system_dim_, system_dim_) * init_cov;

    phi_ = VectorXd::Zero(phi_dim_);
    Phi_ =  MatrixXd::Zero(output_dim_, system_dim_);

    previous_outputs_.resize(n_, VectorXd::Zero(output_dim_)); 
    previous_inputs_.resize(m_, VectorXd::Zero(input_dim_));
}

void SelfTuningRegulator::shutdown(){

    initialized_ = false;
}

void SelfTuningRegulator::reset(){

}

void SelfTuningRegulator::start(){
    initialized_ = true;
}   

void SelfTuningRegulator::construct_phi(VectorXd output, VectorXd input) {

    previous_outputs_.push_front(output);
    previous_inputs_.push_front(input);

    if (previous_outputs_.size() > n_) {previous_outputs_.pop_back();}
    if (previous_inputs_.size() > m_) {previous_inputs_.pop_back();}

    for (int i=0; i < previous_outputs_.size(); i++){
    phi_.segment(i*output_dim_, output_dim_) = previous_outputs_[i];
    }
    for (int j=0; j < previous_inputs_.size(); j++){
    phi_.segment(n_*output_dim_ + j*input_dim_, input_dim_) = previous_inputs_[j];
    }

    for (int k=0; k < output_dim_; k++){
        Phi_.block(k, k*phi_dim_, 1, phi_dim_) = phi_.transpose();
    }

}

/*
alpha(k) = Phi(k) x Theta(k) + v(k) 

Update equations -
Theta(k) = Theta(k-1) + L(k-1) x [alpha(k) - Phi(k) x Theta(k)]

L(k-1) = (P(k-1) x Phi^T(k)) / (lambdaI + Phi(k) x P(k-1) * phi(k) )

P(k) = [ I - L(k-1) * phi'(k) ] * P(k-1) / lambda

*/
void SelfTuningRegulator::update(VectorXd output, VectorXd input){

    if (output.size() != output_dim_ || input.size() != input_dim_){
        std::cerr << "Dimension mismatch! \n";
        exit(1);
    }

    MatrixXd L_den = MatrixXd::Identity(output_dim_,output_dim_) * lambda_ + Phi_ * P_ * Phi_.transpose();
    L_ = (P_ * Phi_.transpose() ) * L_den.inverse();

    // L_ = (P_ * Phi_.transpose()) * L_den.ldlt().solve(MatrixXd::Identity(output_dim_, output_dim_));

    Theta_ = Theta_ + L_ * (output - Phi_ * Theta_);

    P_ = (MatrixXd::Identity(system_dim_, system_dim_) - (L_ * Phi_)) * P_ / lambda_;

    construct_phi(output, input); // for next update

    step_count_ ++;
}

void SelfTuningRegulator::estimate(){



}