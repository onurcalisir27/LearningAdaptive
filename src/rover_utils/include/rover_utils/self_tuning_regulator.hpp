#ifndef SELF_TUNING_REGULATOR_HPP
#define SELF_TUNING_REGULATOR_HPP
#include <Eigen/Dense>
#include <tuple>
#include <chrono>
namespace rover_utils
{
using Eigen::MatrixXd;
using Eigen::VectorXd;
class SelfTuningRegulator{
public:

  SelfTuningRegulator() = default;
  ~SelfTuningRegulator() = default;

  /**
   * @brief: Initialize the Self Tuning Regulator by passing on the system dimensions and forgetting factor
   * @params: state_dim: size of the control state, input_dim: size of the control effort, state/input history
   * are the system orders (i.e. how many prior information to take), forgettingfactor for the RLS algorithm
   * @output: -
   */
  void init(int& state_dim, int& input_dim, int& state_history, int& input_history, double forgettingfactor);

  /**
   * @brief: Reset all state matrices and RLS estimation to zero, in between runs if needed
   * @params: -
   * @output: -
   */
  void reset();

  /**
   * @brief: Pass in a Vector of limits you want to enforce on your control effort. The size of control_bound should
   * match the size of input_dim. Each element in input will be bounded by the corresponding control_bound element
   * @params: Vector control_bound, an array of bounds for control
   * @output: -
   */
  void set_bounds(VectorXd& control_bound);

  /**
   * @brief: Covariance matrix can be initialized  or reset to an identity matrix with initial_covariance in the diagonal
   * @params: double initial_covariance: the value that will be on the identity diagonal
   * @output: -
   */
  void set_covariance(double& initial_covariance);

  /**
   * @brief: If there is a good idea of stable Theta parameters, this function can be used to pass values directly at initialization
   * @params: Matrix Theta_desired, matrix of size [sxn]
   * @output: -
   */
  void set_theta(MatrixXd& Theta_desired){Theta_ = Theta_desired;}

  /**
   * @brief: Can change the forgettingfactor after initialization, or online
   * @params: - double forgettingfactor value
   * @output:
   */
  void update_forgetting_factor(double& forgettingfactor);

    /**
   * @brief: Access current Parameter estimate,
   * @params: -
   * @output: Matrix Parameters
   */
  MatrixXd const get_parameters() {return Theta_;}

  /**
   * @brief: Access the current covariance matrix
   * @params: -
   * @output: Matrix Covariance
   */
  MatrixXd const get_covariance() {return Cov_;}

  /**
   * @brief: Access the History Matrix the estimator internally has seen
   * @params: -
   * @output: Vector passed history data
   */
  VectorXd const get_phi() {return phi_;}

  /**
   * @brief: Access the current error of the system. The error can be interpreted in 3 different ways:
   * 1. State Error: error between the desired_state - current_state,
   * 2. Estimate Error: error between current_state - estimated_state
   * 3. Control Error: error between the desired_state - estimated_state
   * All errors measure a separate metric of the controller, so this function returns all the errors for the user
   * to parse through and decide on which to use.
   * @params: Vector Desired State equal to the size of n, Vector Current State equal to the size of n
   * @output: Return type is a tuple of Eigen::VectorXd, use std::tie() or .get<> to access errors desired.
   * The returned errors are in the same order as specified above in the brief,
   */
  std::tuple<VectorXd,VectorXd,VectorXd> get_error(const VectorXd& desired, const VectorXd& current);

  /**
   * @brief: The main entrypoint of the controller, will update the parameters through Recursive Least squares, and compute an
   * error minimizing control input using an One Step Ahead control law.
   * @params: Vector desired state, Vector current state, Vector previous outputs, Vector previous inputs,
   * Desired and Current should be of size n, while outputs is required to be of size n * p, and inputs of size
   * m * r
   * @output: Vector inputs equal to size m, of bounded control input minimizing the control error (3)
   */
  VectorXd compute_input(VectorXd& desired, VectorXd& current, VectorXd& outputs, VectorXd& inputs);

  /**
   * @brief: Allows for online tuning of a pid controller as gains can be passed in a tuple
   * @params: tuple<double,double,double> gains = [kp, ki, kd]
   * @output: -
   */
  void set_pid(std::tuple<double,double,double>gains);

  /**
   * @brief: Using the defined gains, this uses a PID control law to minimize the error between desired and current,
   * this function does not use parameter estimation, or adaptive control, and its sole use could be for stabilizing control
   * @params: Vector desired, current, and chrono::duration dt the sampling rate
   * @output: Error minimizing Vector output
   */
  VectorXd pid_controller(VectorXd& desired, VectorXd& current, std::chrono::duration<double> dt);

  /**
   * @brief: Simplified version of compute_input for 1 dimensional state. Handles 1 dimensional edge cases by hardcoding and
   * assumpsions. Should only be used for initial testing, as comput_input is also programmed to handle 1D cases
   * @params: double desired and current state, 1 dimensional states. Vector outputs and inputs of previous state/input pairs
   * @output: 1 dimensional control input, bounded by control_bound
   */
  double str(double& desired, double& current, VectorXd& outputs, VectorXd& inputs);

private:
   /**
   * @brief: This function utilizes the Recursive Least Squares algorithm to update the Parameter
   * estimates based on the current state and how close the internal estimate of the state is. The function allows the
   * SelfTuningRegulator to learn the system parameters online with updates.
   * @params: Vector current state
   * @output: -
   */
  void parameter_estimation(VectorXd& current);

   /**
   * @brief: Simplified version of parameter_estimation, specialized for 1 dimensional state
   * @params: VectorXd, current state
   * @output: -
   */
  void rls(VectorXd& current);

  /**
   * @brief: Covariance Matrix Update ensuring PSD properties
   * @params: -
   * @output: -
   */
  void covariance_update();

  /**
   * @brief: Calculates the input based on the one step ahead control law.
   * @params: Vector error calculated with estimated system parameters
   * @output: Vector input, the error minimizing bounded control effort
   */
  VectorXd step_ahead_control(VectorXd& error);

  // Recursive Least Squares Arrays
  double lambda_;
  MatrixXd Theta_;
  VectorXd phi_;
  VectorXd p_states_, p_inputs_;
  VectorXd K_;
  MatrixXd Cov_;

  // Nonlinear System Matrices
  int n_, m_, s_, p_, r_;
  MatrixXd A_;
  MatrixXd B_;
  MatrixXd B_current;
  MatrixXd B_old;
  VectorXd input_bounds;

  // PID control params
  double kp_, kd_, ki_;
  double p_error;
  double integral;

}; // class SelfTuningRegulator

} // namespace rover_utils

#endif // SELF_TUNING_REGULATOR_HPP
