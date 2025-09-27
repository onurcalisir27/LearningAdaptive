#ifndef SELF_TUNING_REGULATOR_HPP
#define SELF_TUNING_REGULATOR_HPP
#include <Eigen/Dense>
#include <tuple>
#include <chrono>
using Eigen::MatrixXd;
using Eigen::VectorXd;

class SelfTuningRegulator{

    public:

        SelfTuningRegulator() = default;
        ~SelfTuningRegulator() = default;

        void init(int& state_dim, int& input_dim, int& state_history, int& input_history, double forgettingfactor);

        void reset();

        void set_frequency(int& freq);

        void set_bounds(double& param_bound, double& control_bound);

        void set_covariance(double& initial_covariance);

        void set_theta(MatrixXd& Theta_desired){Theta_ = Theta_desired;}

        void set_gain(VectorXd& K_desired){K_ = K_desired;}

        void update_forgetting_factor(double& forgettingfactor);

        MatrixXd const get_theta() {return Theta_;}

        MatrixXd const get_covariance() {return Cov_;}

        VectorXd const get_phi() {return phi_;}

        VectorXd get_error(const double& desired, const double& current);

        //**
        // @brief:
        // @params:
        //
        VectorXd compute_input(VectorXd& desired, VectorXd& current, VectorXd& prev_input, std::chrono::duration<double> dt);
        void set_pid(std::tuple<double,double,double>gains);
        VectorXd pid_controller(VectorXd& desired, VectorXd& current, std::chrono::duration<double> dt);

        double str(double& desired, double& current, VectorXd& outputs, VectorXd& inputs);
    private:

        //Helper Functions
        void phi_update(VectorXd& state, VectorXd& input);

        void parameter_estimation(VectorXd& current);

        void system_update();

        void rls(VectorXd& current);

        void covariance_update();

        VectorXd step_ahead_control(VectorXd& error);

        // System Dimensions
        int n_, m_, s_;
        int p_, r_;

        // Update frequencies
        int step_;
        int update_freq_;

        // Forgetting Factor
        double lambda_;

        // Parameter Array
        MatrixXd Theta_;

        // Data Vector
        VectorXd phi_;
        VectorXd p_states_, p_inputs_;

        // Gain
        VectorXd K_;

        // Covariance Matrix
        MatrixXd Cov_;

        // State-Input Matrix estimate
        MatrixXd A_;
        MatrixXd B_;

        // Bound parameters and input values to realistic values
        double theta_bound_, u_bound_;

        // PID params
        double kp_, kd_, ki_;
        double p_error;
        double integral;
};

#endif // SELF_TUNING_REGULATOR_HPP
