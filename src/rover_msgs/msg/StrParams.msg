# Self Tuning Regulator Parameters Message

# The current parameter estimates from the least squares estimator
float64[] estimate

# The current covariance matrix flattened, of the estimated parameters
float64[] covariance

# Self Tuning Regulator has 3 different types of errors,
# It could be useful to access all kinds of errors in the system
float64[] state_error
float64[] estimate_error
float64[] control_error
