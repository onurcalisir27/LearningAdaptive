#include "rover_control/self_tuning_regulator.hpp"
#include <iostream>
// #include <cstdint>
#include <Eigen/Dense>
#include <algorithm>

using Eigen::VectorXd;
using Eigen::MatrixXd;

#define CLAMP_COVARIANCE

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
    initial_covariance_ = init_cov;

    Theta_ = VectorXd::Random(system_dim_);
    L_ = MatrixXd::Zero(system_dim_, output_dim_);
    P_ = MatrixXd::Identity(system_dim_, system_dim_) * initial_covariance_;

    phi_ = VectorXd::Zero(phi_dim_);
    Phi_ =  MatrixXd::Zero(output_dim_, system_dim_);

    previous_outputs_.resize(n_, VectorXd::Zero(output_dim_)); 
    previous_inputs_.resize(m_, VectorXd::Zero(input_dim_));

    A_ = MatrixXd::Zero(output_dim_, output_dim_*n_);
    x_ = VectorXd(output_dim_ * n_);

    Bc_ = MatrixXd::Zero(output_dim_, input_dim_);

    if (m_ < 2){
        no_input_history = true;
        std::cout << "No input history, Bp_ doesnt exist" << std::endl;
    } else {
        no_input_history = false;
        Bp_ = MatrixXd::Zero(output_dim_, input_dim_ * (m_-1));
        up_ = VectorXd(input_dim_ * (m_-1));
    }
}

void SelfTuningRegulator::shutdown(){

    initialized_ = false;
}

void SelfTuningRegulator::reset(){

    Theta_ = VectorXd::Random(system_dim_);
    L_ = MatrixXd::Zero(system_dim_, output_dim_);
    P_ = MatrixXd::Identity(system_dim_, system_dim_) * initial_covariance_;

    phi_ = VectorXd::Zero(phi_dim_);
    Phi_ =  MatrixXd::Zero(output_dim_, system_dim_);

    previous_outputs_.resize(n_, VectorXd::Zero(output_dim_)); 
    previous_inputs_.resize(m_, VectorXd::Zero(input_dim_));

    A_ = MatrixXd::Zero(output_dim_, output_dim_*n_);
    x_ = VectorXd(output_dim_ * n_);

    Bc_ = MatrixXd::Zero(output_dim_, input_dim_);

    if(!no_input_history){
        Bp_ = MatrixXd::Zero(output_dim_, input_dim_ * (m_-1));
        up_ = VectorXd(input_dim_ * (m_-1));
    }

}

void SelfTuningRegulator::start(){
    initialized_ = true;
}   

void SelfTuningRegulator::construct_phi() {

    for (uint i=0; i < previous_outputs_.size(); i++){
    phi_.segment(i*output_dim_, output_dim_) = previous_outputs_[i];
    }
    for (uint j=0; j < previous_inputs_.size(); j++){
    phi_.segment(n_*output_dim_ + j*input_dim_, input_dim_) = previous_inputs_[j];
    }
    for (uint k=0; k < output_dim_; k++){
        Phi_.row(k).segment(k*phi_dim_, phi_dim_) = phi_.transpose();
    }
    std::cout << "Phi Matrix:" << Phi_ << "\n \n";
}

void SelfTuningRegulator::update(VectorXd desired)
{
    MatrixXd L_den = MatrixXd::Identity(output_dim_,output_dim_) * lambda_ + Phi_ * P_ * Phi_.transpose();
    L_ = (P_ * Phi_.transpose() ) * L_den.inverse();
    // L_ = (P_ * Phi_.transpose()) * L_den.completeOrthogonalDecomposition().pseudoInverse();
    // L_ = (P_ * Phi_.transpose()) * L_den.ldlt().solve(MatrixXd::Identity(output_dim_, output_dim_));
    
    Theta_ = Theta_ + L_ * (desired - Phi_ * Theta_);
    
    double dLimit=0.95;
    for (int iLoop= output_dim_*n_; iLoop < phi_dim_; iLoop++)
    {
        Theta_(iLoop,0) = std::clamp(Theta_(iLoop,0), -dLimit, dLimit);
    }

    std::cout << "Theta Vector:" << Theta_ << "\n";

    P_ = (MatrixXd::Identity(system_dim_, system_dim_) - (L_ * Phi_)) * P_ / lambda_;

	for (int iLoop1=0; iLoop1<system_dim_; iLoop1++)
	{
      double dLimitDiagonal=10000.0;
      double dLimitOffDiagonal=100.0;
#ifdef CLAMP_COVARIANCE     
      P_(iLoop1,iLoop1)=std::clamp(P_(iLoop1,iLoop1), 0.0, dLimitDiagonal);
#endif // CLAMP_COVARIANCE     
      for (int iLoop2=0; iLoop2<system_dim_; iLoop2++)
      {
	if (iLoop1 == iLoop2)
        {
          fprintf(stdout, "%.2lf ", P_(iLoop1,iLoop2)), fflush(stdout);
	  continue;
	}else
    {
#ifdef CLAMP_COVARIANCE     
          P_(iLoop1,iLoop2)=std::clamp(P_(iLoop1,iLoop2), 0.0, dLimitOffDiagonal);
#endif // CLAMP_COVARIANCE     
          fprintf(stdout, "%.2lf ", P_(iLoop1,iLoop2)), fflush(stdout);
	}
      }
      fprintf(stdout, "\n"), fflush(stdout);
    }

    construct_phi(); // for next update
}

void SelfTuningRegulator::estimate(){

    int idx(0);
    for (VectorXd i : previous_outputs_){
        x_.segment(idx*output_dim_, output_dim_) = i; 
        idx++;
    }

    idx = 0;
    if (!no_input_history){
        for(VectorXd j : previous_inputs_){
            if (j == previous_inputs_.back()) {continue;}
            up_.segment(idx*input_dim_, input_dim_) = j;
            idx++;
        }
    }
    
    for(int i = 0; i < output_dim_; i++) {
        A_.row(i) = Theta_.segment((i * phi_dim_), output_dim_ * n_).transpose();

        Bc_.row(i) = Theta_.segment((i * phi_dim_ + (output_dim_ * n_)), input_dim_).transpose();

        if (!no_input_history){
            std::cout << "Building Bp now! \n";
            Bp_.row(i) = Theta_.segment((i * phi_dim_ + (output_dim_* n_ + input_dim_)), (m_ - 1) * input_dim_).transpose();
        } 

    }
}

VectorXd SelfTuningRegulator::computeControl(VectorXd& desired, VectorXd& current, VectorXd& prev_input){

    if(desired.size() != output_dim_ || current.size() != output_dim_ || prev_input.size() != input_dim_){
        std::cerr << "Dimension mismatch, check your inputs to the function \n";
        exit(1);
    }

    VectorXd u_c = VectorXd::Random(input_dim_) * 0.05;
    // VectorXd u_c = VectorXd::Zero(input_dim_);

    previous_outputs_.push_front(current);
    previous_inputs_.push_front(prev_input);

    if (previous_outputs_.size() > n_) {previous_outputs_.pop_back();}
    if (previous_inputs_.size() > m_) {previous_inputs_.pop_back();}

    if (step_count_ < n_ + m_){
        std::cout << "Not enough information, use different controller \n";

    } else{
        std::cout << "Using estimated parameters to calculate control effort! \n";
        estimate();
        
        std::cout << "If it fails now, we got an inverse error" << std::endl;
        if (no_input_history)
        {
            double Bc_temp = std::max(Bc_(0,0),0.001);
            double Bc_inv = 1.0 / Bc_temp;
            u_c = u_c + Bc_inv * (desired - A_ * x_);
        } else{
            MatrixXd Bc_inverse =  Bc_.inverse();
            // MatrixXd Bc_inverse =  Bc_.completeOrthogonalDecomposition().pseudoInverse();
            u_c = u_c + Bc_inverse * (desired - A_ * x_ - Bp_ * up_);
        }
        std::cout << "We computed u_c = " << u_c << " moving on..." << std::endl;
    }

    update(desired);
    step_count_++;

    return u_c;
}
