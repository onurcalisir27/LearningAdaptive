#ifndef SELF_TUNING_REGULATOR_HPP
#define SELF_TUNING_REGULATOR_HPP

#include <memory>
#include <string>
#include <cmath>

#include <deque>
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SelfTuningRegulator{

    public:

        SelfTuningRegulator() = default;
        ~SelfTuningRegulator() = default;

        void init(int n, int m, int input_dim, int output_dim, double lambda, double init_cov);

        void shutdown();

        void reset();

        void start();

        void construct_phi();

        void update(VectorXd desired);
        
        VectorXd get_theta_parameters() {return Theta_;}

        MatrixXd get_covariance_matrix() {return P_;}

        void estimate();

        VectorXd computeControl(VectorXd& desired, VectorXd& current, VectorXd& input);

    private:
    // input dim: p, output dim: k, system_dim: k x (nk + mp)
        uint n_, m_;
        uint system_dim_, phi_dim_, output_dim_, input_dim_;
        double lambda_, initial_covariance_;
        bool initialized_;
        int step_count_;
        bool no_input_history;

        // System ID params
        VectorXd Theta_;    // [system_dim]
        MatrixXd L_;        // [system_dim x output_dim_ ]
        MatrixXd P_;        // [system_dim x system_dim]

        VectorXd phi_;      // [system_dim/output_dim]
        MatrixXd Phi_;      // [output_dim x system_dim]

        std::deque<VectorXd> previous_outputs_; 
        std::deque<VectorXd> previous_inputs_;

        // Controller Params
        MatrixXd A_;
        VectorXd x_;  // past outputs
        MatrixXd Bc_; // B current
        MatrixXd Bp_; // B past
        VectorXd up_; // past inputs
};

#endif // SELF_TUNING_REGULATOR_HPP