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
   * @brief Initialize the Self Tuning Regulator Controller for the system being used through dimension definitions
   * @param state_dim Dimension of the controlled state
   * @param input_dim Dimension of the control effort
   * @param state_history State order how many steps back in state data should the system know
   * @param input_history Input order, how many steps back in input data should the system know
   * @param forgettingfactor Forgetting Factor of the recursive least squares algorithm
  */
  void init(int& state_dim, int& input_dim, int& state_history, int& input_history, double forgettingfactor);

  /**
   * @brief Reset all state matrices and RLS estimation to zero, in between runs if needed
   */
  void reset();

  /**
   * @brief  Pass in a Vector of limits you want to enforce on your control effort. The size of control_bound should
   * match the size of input_dim. Each element in input will be bounded by the corresponding control_bound element
   * @param control_bound Vector containing element wise bounds for control effort limitting
   */
  void set_bounds(VectorXd& control_bound);

  /**
   * @brief Covariance matrix can be initialized  or reset to an identity matrix with initial_covariance in the diagonal
   * @param initial_covariance Value that will be on the identity diagonal
   */
  void set_covariance(double& initial_covariance);


  /**
   * @brief If there is a good idea of stable Theta parameters, this function can be used to pass values directly at initialization
   * @param Theta_desired Matrix of containing initial guess for system parameters
   */
  void set_theta(MatrixXd& Theta_desired){Theta_ = Theta_desired;}

  /**
   * @brief Can change the forgettingfactor after initialization, or online
   * @param forgettingfactor Desired value for the forgettingfactor in double
   */
  void update_forgetting_factor(double& forgettingfactor);

  /**
   * @brief Access current Parameter estimate
   * @return The parameters computed so far in the estimator
   */
  MatrixXd const get_parameters() {return Theta_;}

  /**
   * @brief Access the current covariance matrix
   * @return Current Covariance Matrix where each row corresponds to parameters
   */
  MatrixXd const get_covariance() {return Cov_;}

  /**
   * @brief Access the current error of the system. The error can be interpreted in 3 different ways:
   * 1. State Error: error between the desired_state - current_state,
   * 2. Estimate Error: error between current_state - estimated_state
   * 3. Control Error: error between the desired_state - estimated_state
   * All errors measure a separate metric of the controller, so this function returns all the errors for the user
   * to parse through and decide on which to use.
   * @param desired Desired State Vector
   * @param current Current State Vector
   * @return Return type is a tuple of Eigen::VectorXd, use std::tie() or .get<> to access errors desired.
   * The returned errors are in the same order as specified above in the brief,
   */
  std::tuple<VectorXd,VectorXd,VectorXd> get_error(const VectorXd& desired, const VectorXd& current);

  /**
   * @brief The main entrypoint of the controller, will update the parameters through Recursive Least squares, and compute an
   * error minimizing control input using an One Step Ahead control law.
   * @param desired Vector desired state, [n x 1]
   * @param current Vector current state, [n x 1]
   * @param outputs Vector containing previous states in the order [n*p x 1]
   * @param inputs Vector containing previous inputs in the order [m*r x 1]
   * @return Vector inputs equal to size m, of bounded control input minimizing the control error (3)
   */
  VectorXd compute_input(VectorXd& desired, VectorXd& current, VectorXd& outputs, VectorXd& inputs);

  /**
   * @brief Allows for online tuning of a pid controller as gains can be passed in a tuple
   * @param gains A tuple of <double,double,double> representing [kp, ki, kd]
   */
  void set_pid(std::tuple<double,double,double>gains);

  /**
   * @brief  Using the defined gains, this uses a PID control law to minimize the error between desired and current,
   * this function does not use parameter estimation, or adaptive control, and its sole use could be for stabilizing control
   * @param desired Vector desired state of the system
   * @param current Vector current state of the system
   * @param dt chrono::duration sampling rate
   * @return Error minimizing Input Vector
   */
  VectorXd pid_controller(VectorXd& desired, VectorXd& current, std::chrono::duration<double> dt);

  /**
   * @brief Simplified version of compute_input for 1 dimensional state. Handles 1 dimensional edge cases by hardcoding and
   * assumpsions. Should only be used for initial testing, as comput_input is also programmed to handle 1D cases
   * @param desired double desired state, [1 x 1]
   * @param current double current state, [1 x 1]
   * @param outputs Vector containing previous states in the order [p x 1]
   * @param inputs Vector containing previous inputs in the order [r x 1]
   * @return 1 dimensional control input, bounded by control_bound
   */
  double str(double& desired, double& current, VectorXd& outputs, VectorXd& inputs);

private:
   /**
   * @brief Helper function implementing the Recursive Least Squares algorithm to update the Parameter
   * estimates based on the current state and how close the internal estimate of the state is. The function allows the
   * SelfTuningRegulator to learn the system parameters online with updates.
   * @param current Vector current state of the system
   */
  void parameter_estimation(VectorXd& current);

  /**
   * @brief Simplified version of parameter_estimation, specialized for 1 dimensional state
   * @param current Vector Current stete of the system
   * @return
   */
  void rls(VectorXd& current);

  /**
   * @brief Covariance Matrix Update ensuring PSD properties
   */
  void covariance_update();

  /**
   * @brief Helper Function which calculates the control input based on the one step ahead control law.
   * @param error Vector calculated with estimated system parameters
   * @return Vector input, the error minimizing bounded control effort
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
