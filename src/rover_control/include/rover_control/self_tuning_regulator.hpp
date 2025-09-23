#ifndef SELF_TUNING_REGULATOR_HPP
#define SELF_TUNING_REGULATOR_HPP
#include <Eigen/Dense>

using Eigen::MatrixXd;
using Eigen::VectorXd;

class SelfTuningRegulator{

    public:

        SelfTuningRegulator() = default;
        ~SelfTuningRegulator() = default;

        void init(int& state_dim, int& input_dim, int& state_history, int& input_history, double forgettingfactor);

        void reset();

        void set_frequency(int& param_freq, int& system_freq);

        void set_bounds(double& param_bound, double& control_bound);

        void set_covariance(double& initial_covariance);

        MatrixXd get_theta() {return Theta_;}

        MatrixXd get_covariance() {return Cov_;}

        VectorXd get_phi() {return phi_;}

        VectorXd compute_input(VectorXd& desired, VectorXd& current, VectorXd& prev_input);

    private:

        //Helper Functions
        void phi_update(VectorXd& state, VectorXd& input);

        void parameter_estimation(VectorXd& current);

        void system_update();

        void covariance_update();

        VectorXd step_ahead_control(VectorXd& error);

        // System Dimensions
        int n_, m_, s_;
        int p_, r_;

        // Update frequencies
        int step_;
        int parameter_update_freq_;
        int system_update_freq_;

        // Forgetting Factor
        double lambda_;

        // Parameter Array
        MatrixXd Theta_;

        // Data Vector
        VectorXd phi_;
        VectorXd p_states_, p_inputs_;

        // Kalman Gain
        VectorXd K_;

        // Covariance Matrix
        MatrixXd Cov_;

        // State-Input Matrix estimate
        MatrixXd A_;
        MatrixXd B_;

        // Bound parameters and input values to realistic values
        double theta_bound_, u_bound_;
};

#endif // SELF_TUNING_REGULATOR_HPP
