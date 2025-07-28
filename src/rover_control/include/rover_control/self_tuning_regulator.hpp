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

        void construct_phi(VectorXd output, VectorXd input);

        void update(VectorXd output, VectorXd input);

        void estimate();

    private:
    // input dim: p, output dim: k, system_dim: k x (nk + mp)
        int n_, m_;
        int system_dim_, phi_dim_, output_dim_, input_dim_;
        double lambda_;
        bool initialized_;
        int step_count_;

        // System ID params
        // VectorXd alpha_;    // [output_dim]
        VectorXd Theta_;    // [system_dim]
        MatrixXd L_;        // [system_dim x output_dim_ ]
        MatrixXd P_;        // [system_dim x system_dim]

        VectorXd phi_;      // [system_dim/output_dim]
        MatrixXd Phi_;      // [output_dim x system_dim]

        std::deque<VectorXd> previous_outputs_; 
        std::deque<VectorXd> previous_inputs_;

        // Controller Params

};

#endif // SELF_TUNING_REGULATOR_HPP