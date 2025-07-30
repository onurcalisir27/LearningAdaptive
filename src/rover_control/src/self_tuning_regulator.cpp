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

    Theta_ = VectorXd::Random(system_dim_);
    L_ = MatrixXd::Zero(system_dim_, output_dim_);
    P_ = MatrixXd::Identity(system_dim_, system_dim_) * init_cov;

    phi_ = VectorXd::Zero(phi_dim_);
    Phi_ =  MatrixXd::Zero(output_dim_, system_dim_);

    previous_outputs_.resize(n_, VectorXd::Zero(output_dim_)); 
    previous_inputs_.resize(m_, VectorXd::Zero(input_dim_));

    A_ = MatrixXd::Zero(output_dim_, output_dim_*n_);
    Bc_ = MatrixXd::Zero(output_dim_, input_dim_);
    Bp_ = MatrixXd::Zero(output_dim_, input_dim_*m_);

    x_ = VectorXd(output_dim_ * n_);
    up_ = VectorXd(input_dim_ * m_);
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

    std::cout << "phi_ size: " << phi_.size() << std::endl;
    std::cout << "Phi_ size: " << Phi_.rows() << "x" << Phi_.cols() << std::endl;
    std::cout << "phi_dim_: " << phi_dim_ << ", output_dim_: " << output_dim_ << std::endl;
    
    for (int i=0; i < previous_outputs_.size(); i++){
    phi_.segment(i*output_dim_, output_dim_) = previous_outputs_[i];
    }
    for (int j=0; j < previous_inputs_.size(); j++){
    phi_.segment(n_*output_dim_ + j*input_dim_, input_dim_) = previous_inputs_[j];
    }

    for (int k=0; k < output_dim_; k++){
        // Phi_.block(k, k*phi_dim_, 1, phi_dim_) = phi_.transpose();
        Phi_.row(k).segment(k*phi_dim_, phi_dim_) = phi_.transpose();
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

    std::cout << "Updating L, for the :" << step_count_ << "'s time step"<< std::endl;
    MatrixXd L_den = MatrixXd::Identity(output_dim_,output_dim_) * lambda_ + Phi_ * P_ * Phi_.transpose();
    L_ = (P_ * Phi_.transpose() ) * L_den.inverse();
    // L_ = (P_ * Phi_.transpose()) * L_den.ldlt().solve(MatrixXd::Identity(output_dim_, output_dim_));
    
    std::cout << "Updating Theta, for the :" << step_count_ << "'s time step"<< std::endl;
    Theta_ = Theta_ + L_ * (output - Phi_ * Theta_);

    std::cout << "Updating P, for the :" << step_count_ << "'s time step"<< std::endl;
    P_ = (MatrixXd::Identity(system_dim_, system_dim_) - (L_ * Phi_)) * P_ / lambda_;

    construct_phi(output, input); // for next update
}

// void SelfTuningRegulator::estimate(){
//     // y_des = [Theta1; Theta2; Theta3][]
//     //[3 x 1]            [3 x 3n]
//     int idx(0);

//     std::cout << "Constructing x, for the :" << step_count_ << "'s time step"<< std::endl;
//     for (VectorXd i : previous_outputs_){
//         x_.segment(idx*output_dim_, output_dim_) = i; 
//         idx++;
//     }
//     idx = 0;
//     std::cout << "Constructing up, for the :" << step_count_ << "'s time step"<< std::endl;
//     for(VectorXd j : previous_inputs_){
//         up_.segment(idx*input_dim_, input_dim_) = j;
//     }
    
//     std::cout << "Constructing A, Bc, Bp, for the :" << step_count_ << "'s time step"<< std::endl;
//     for(int i = 0; i < output_dim_; i++) {
//         A_.row(i) = Theta_.segment(i*(output_dim_ * n_ + input_dim_ * m_), output_dim_ * n_).transpose();
//         Bc_.row(i) = Theta_.segment(i*(output_dim_ * n_ + input_dim_ * m_) + output_dim_ * n_, input_dim_).transpose();
//         Bp_.row(i) = Theta_.segment(i*(output_dim_ * n_ + input_dim_ * m_) + output_dim_* n_ + input_dim_, (m_ - 1) * input_dim_).transpose();
//     }

// }

void SelfTuningRegulator::estimate(){
    int idx(0);
    std::cout << "Constructing x, for the :" << step_count_ << "'s time step"<< std::endl;
    for (VectorXd i : previous_outputs_){
        x_.segment(idx*output_dim_, output_dim_) = i;
        idx++;
    }
    idx = 0;
    std::cout << "Constructing up, for the :" << step_count_ << "'s time step"<< std::endl;
    for(VectorXd j : previous_inputs_){
        up_.segment(idx*input_dim_, input_dim_) = j;
        idx++;
    }
    
    std::cout << "Constructing A, Bc, Bp, for the :" << step_count_ << "'s time step"<< std::endl;
    std::cout << "Theta_ size: " << Theta_.size() << std::endl;
    std::cout << "A_ size: " << A_.rows() << "x" << A_.cols() << std::endl;
    std::cout << "output_dim_: " << output_dim_ << ", n_: " << n_ << ", m_: " << m_ << ", input_dim_: " << input_dim_ << std::endl;
    
    for(int i = 0; i < output_dim_; i++) {
        int start_A = i*(output_dim_ * n_ + input_dim_ * m_);
        int start_Bc = i*(output_dim_ * n_ + input_dim_ * m_) + output_dim_ * n_;
        int start_Bp = i*(output_dim_ * n_ + input_dim_ * m_) + output_dim_ * n_ + input_dim_;
        
        std::cout << "Row " << i << ": A segment[" << start_A << ":" << start_A + output_dim_ * n_ - 1 << "]" << std::endl;
        std::cout << "Row " << i << ": Bc segment[" << start_Bc << ":" << start_Bc + input_dim_ - 1 << "]" << std::endl;
        std::cout << "Row " << i << ": Bp segment[" << start_Bp << ":" << start_Bp + (m_-1)*input_dim_ - 1 << "]" << std::endl;
        
        A_.row(i) = Theta_.segment(start_A, output_dim_ * n_).transpose();
        Bc_.row(i) = Theta_.segment(start_Bc, input_dim_).transpose();
        Bp_.row(i) = Theta_.segment(start_Bp, (m_ - 1) * input_dim_).transpose();
    }
}

VectorXd SelfTuningRegulator::computeControl(VectorXd& desired, VectorXd& current, VectorXd& prev_input){

    if(desired.size() != output_dim_ || current.size() != output_dim_ || prev_input.size() != input_dim_){
        std::cerr << "Dimension mismatch, check your inputs to the function \n";
        exit(1);
    }

    VectorXd u_c = VectorXd::Random(input_dim_) * 0.01;

    previous_outputs_.push_front(current);
    previous_inputs_.push_front(prev_input);

    if (previous_outputs_.size() > n_) {previous_outputs_.pop_back();}
    if (previous_inputs_.size() > m_) {previous_inputs_.pop_back();}

    if (step_count_ < n_ + m_){
        std::cout << "Not enough information, use different controller \n";

    } else{
        std::cout << "Using estimated parameters to calculate control effort! \n";
        estimate();
        std::cout << "Parameters estimated, for the :" << step_count_ << "'s time step" << std::endl;
        std::cout << "If it fails now, we got an inverse error" << std::endl;
        u_c = u_c + Bc_.inverse() * (desired - A_ * x_ - Bp_ * up_);
    }

    std::cout << "Running update now, for the :" << step_count_ << "'s time step" << std::endl;
    update(current, prev_input);
    step_count_++;

    return u_c;
}