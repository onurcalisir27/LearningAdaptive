## New Adaptive and Learning-Adaptive Control Techniques based on an Extension of the Generalized Secant Method
[[New Adaptive and Learning-Adaptive Control ]]
### Introduction
Using an extension of the [[generalized secant method]] [1] of solving linear equations. Two proposed new controllers:
- First controller is [[Adaptive Controller]] and operates in the time domain
- Second is [[Learning-Adaptive Controller]] and it is formulated to deal with repetitive systems with repetitive disturbances, operates in the repetition domain
An analog network increases the speed of the controllers which is useful for the learning-adaptive controller since this network parallelizes most of the computation necessary for establishing the control, in hardware form.
---
*Most dynamic systems are too complex to be modeled properly. An ideal control system would be a system which gives the best performance for the least user-provided amount of information about the system being controlled.* --> Adaptive Control Systems 

Two well known adaptive controllers are the Model Reference and Self Tuning Regulator [2].  
First proposal, a solution to the adaptive control problem considering a general discrete time linear, time invariant model given by the following equation:
$$y(t+1) = \underbrace{CA}_{\hat{A}}x(t) + \underbrace{CB}_{\hat{B}}u(t) + w(t)$$
$$ \hat{A} = CA,\space\hat{B} = CB $$
State x is n-dimensional, control u is m-dimensional, output y is q-dimensional, and t is the time step. Also $\hat{A}, \hat{B}$ and w(t) are assumed to be unknown - otherwise one could determine in advance what control to use to minimize the tracking error. 

For simplicity the order of the system is assumed to be known but generalization to just knowing an upper bound on the order is easily considered.

When control systems are given a task to perform repeatedly, they will usually repeat the same errors in executing the command, expect for some random noise effects. Learning controllers on the other hand can improve their performance at a given task with each repetition of the task. [5][6]

Presenting a new [[Learning-Adaptive Controller]] which improves the performance of a dynamic system as the desired task is performed by the system over and over again. In this approach, a time-variant, discrete-time model of the system is devised in the form of a system of linear algebraic equations ==relating the change in the state of the system to the change in the control action from one repetition of the task to the next==. 

**This set of linear equations gives the transition between any two repetitions.** This system is solved using a generalized secant method for the appropriate control action and parameter estimates that will ==minimize the tracking error of the controlled dynamic system, only requiring the availability of the order of the system, without any prior knowledge of the system parameters.== 

### Practical Parallel Implementation
##### Degree of Linear Independence
As a practical precaution for stability and quick convergence after evaluating the vector $v_{\tau}$ using [Eq.7] in the adaptive case (or [Eq.41] in the learning-adaptive case), a linear independence test was made by evaluating:
[Eq.55]
$$\omega_{\tau} = \frac{|z_{\tau}^{T}v_{\tau}|}{||z_{\tau}||_{\epsilon}||v_{\tau}||_{\epsilon}}$$
**Here, $\tau$ should be replaced with $t$ for the adaptive controller and with k for the learning-adaptive controller.** 
[Eq. 56]
If $\omega_{\tau} < \rho$  and   $0 < \rho < 1$                   

In the simulations of this paper, $\rho =0.0001$, then the control for the step is changed such as to make $\omega_{\tau}> \rho$ . This is done using the information provided by the orthogonalization method.[7] 

##### Parallel Implementation
The most computation intensive part of the learning-adaptive algorithm is the evaluation of the pseudo-inverse in [Eq.41] Although, the nature of this computation is such that it could be done between two repetitions, it would be nice to be able to solve [Eq.41] in a more effective and quicker fashion. By nature, pseudo-inversion is a minimization problem such that the Frobenius norm of the error $\|P^{k}v^{k}+e^{k}\|_{\mathcal{F}}$ is minimized. This error, E, could also be formulated in terms of a minimization problem in $v^k$ ,
[Eq. 57]
$$\frac{dv^{k}}{d\xi}=-M(\xi)\nabla E(v^{k})$$ [Eq.58]
$$\nabla E(v^{k}(\xi)) = {P^{k}}^{T}(P^{k}v^{k}(\xi) + e^{k})$$
[Eq.59]
$$v^{k}(0) = v^{k}_{0}$$
This is a possible minimization formulation of the pseudo-inverse. If the weighting factor $M(\xi)$ is set to the identity matrix, the minimization problem reduces to the steepest descent problem.

For convergence of the minimization problem, M should be chosen to be positive definite. Reference [10] shows a network which recursively goes through the calculations of [Eq.57-59]. 

Figure 1. shows a parallel network which basically solves this minimization problem. However, since this is a parallel analog network, the calculations are done much more quickly.

A proof of convergence of [Eq.41 and 42] is given above. This proof of convergence is independent of the solution for the change in the control vector. The change in the control input vector converges if M is picked to be positive definite.

Therefore, the two problems are almost decoupled and if both converge, the system will converge to the optimal solution.

### Conclusions
##### [[Adaptive Controller]]
The secant adaptive-controller converges much more quickly than the self tuning regulator and has a consistently lower overall error in the sum of squares sense. It is very robust to have been applied to such non-linear systems at highly non-linear trajectories displaying a very good performance and does not require any more computation than the self tunning regulator. With the practical introduction of the restrictions on $\omega_{t}$ of [Eq.55], the secant controller takes a few steps to learn the parameters of the system before it greatly affects the dynamics of it. Once it realizes the system parameters, it quickly reduces the error.

##### [[Learning-Adaptive Controller]]
Theoretical evidence shows that the Generalized Secant method requires the least number of repetitions for convergence in this framework. The method presented here will theoretically converge in at most $(mp)$ repetitions in the absence of non-repetitive noise.

**Results show that without rejection of non-informative control steps, the system may become unstable and that the use of a $p$ which is too small in [Eq. 56] could result in a big reduction in the rate of convergence.** Therefore the trade-off in the magnitude of $p$ needs to be considered.  Also in practice, the magnitude of steps which replace rejected control steps should be made small enough to be considered a perturbation which does not cause instabilities in the system. 

In other words, these perturbations should be small enough that the standard stabilizing controller of the system would be able to handle. In general, the Generalized Secant Learning-Adaptive Controller has shown to be very stable and robust in a practical sense when applied to the nonlinear systems of this paper.

The [[generalized secant method]] learning-adaptive controller was also combined with a parallel scheme for evaluating the pseudo-inversion required in that learning control algorithm. This makes the waiting period between two consequent repetitions shorter. The convergence of the two methods is shown to be almost independent. The results of simulations support the fact that using this parallel network for obtaining the pseudo-inverse is very practical.

However, these is one problem of practicality which is still outstanding and that is the fact that for each time step there should be a connection built in the network. However if networks are developed for this type of controller, a maximum size network can work for any number of time steps less that or equal to its maximum capacity.

### References
- 
---

## Nonlinear Piezo-Actuator Control by Learning Self-Tuning Regulator
[[Self Tuning Regulator]]
### Abstract
A learning self-tuning regulator (LSTR) which improves the tracking performance of itself while performing repetitive tasks. The controller is a self-tuning regulator based on ==learning parameter estimation.== 
Nomenclature:

|  Symbol   | Nomenclature                                                         |
| :-------: | -------------------------------------------------------------------- |
| $\alpha$  | output of the plant                                                  |
| $\gamma$  | forgetting factor in the repetition domain                           |
| $\lambda$ | forgetting factor in the time domain                                 |
|  $\phi$   | data vector                                                          |
| $\theta$  | parameter vector                                                     |
|    $A$    | polynomial of the system poles                                       |
|    $B$    | polynomial of the system zeroes                                      |
|    $C$    | vector of generalized forces due to centrifugal and Coriolis forces? |
|    $c$    | damping constant                                                     |
|    $F$    | vector of generalized viscous friction forces                        |
|    $G$    | vector of generalized gravitational forces                           |
|    $I$    | identity matrix                                                      |
|    $k$    | discrete time step                                                   |
|    $P$    | covariance matrix of errors in parameter estimates                   |
|    $p$    | no. of sampling intervals for the trajectory                         |
| $q^{-d}$  | d step delay operator                                                |
|    $r$    | repetition number                                                    |
|    $t$    | time step                                                            |
|    $u$    | control actions (plant input)                                        |
|    $v$    | equation error                                                       |

### Introduction
A STR consists of a parameter estimator and a controller. The parameter estimator estimates the parameters of an approximated model of the controlled system by utilizing a recursive estimation scheme. Based on the approximate model and the estimated parameters, the controller adjusts its actions to maintain its performance. Therefore, the performance of a STR depends greatly on the **parameter estimator used**

Recursive Least Squares(RLS) algorithms do not utilize their past experience in estimating repetitive parameters, which means they will keep making the same errors at corresponding times in the duration of each repetition. This paper presents a learning adaptive control scheme based on a learning estimator which ==utilizes the information from past performances of a repetitive task to refine the estimate of the plant parameters. Consequently, the learning adaptive controller improves its performance throughout the repetitions.==

Since the adaptation of parameters at all sampling instants is made over the repetitions of the task, the estimator is free to follow the changes of parameters over time.

###  The Self Tuning Regulator
A STR models a plant with a difference equation such as follows and uses a recursive estimator to estimate the parameter vector $\theta(t)$ at each sampling instant and adjusts its control action accordingly. Therefore the overall performance of the control system is dependent on how close these estimates are to the real system parameters.
[Eq.1]$$\alpha(k) = \phi'(k) \theta(k) + v(k)$$
where:
$$\begin{align}
\theta'(k) &= [\theta_{1}(k), \theta_{2}(k), \theta_{3}(k), ..., \theta_{N+M}(k)] \\
\phi'(k) &= [-\alpha(k-1), -\alpha(k-2), ...,-\alpha(k-N), u(k-1), u(k-2), ..., u(k-M)]
\end{align}$$
To find a $\theta$ vector which would minimize
[Eq.2]$$v_{T}(\theta)=\sum_{k=0}^T\limits{\lambda^{T-k}[\alpha(k) - \theta'(k) \phi(k)]^2}$$
the following algorithm can be used 
[Eq.3a-c]
$$\begin{align}
\theta(k) &= \theta(k-1) + L(k-1)[\alpha(k) - \theta '(k) \phi(k)] \\[8pt]

L(k-1) &= \frac{P(k-1) \phi(k)}{\lambda + \phi'(k) P(k-1) \phi(k)} \\[8pt]

P(k) &= \frac{[I - L(k-1) \phi'(k)] P(k-1)}{\lambda} 
\end{align}$$
Typically an initial estimate of the parameters has to be given to the estimator to start the recursion, which is different from the real parameters. A tracking error will be produced until the parameter estimator catches up which may never be achieved if the parameters are varying from one sampling instant to the next. 

Consequently, a persistent tracking error resulted from the lagging of the estimate of the parameters at all times is inevitable. If the same task is executed repeatedly, the controller repeats it without looking back at how it had performed previously. Therefore, the controller will make the same errors at corresponding times of repetitions.

The learning estimator provides a solution which will allow the use of the information acquired from previous repetitions to improve the estimate of the parameters over repetitions of a task. With better estimates of parameters the self-tuning controller will be able to reduce the tracking errors over the repetitions of a task. 

### Learning Recursive LSE and Control Law
 ![[Pasted image 20250430140727.png|300x200]]
Figure 1 illustrates the complete closed-loop system which consists of the learning parameter estimator and the One-Step-Ahead controller which was selected for its simplicity[9].

Let us look at time step k of repetition r, and assume that there is a linear difference equation which adequately describes the dynamics of the plant.
[Eq.4]$$\alpha^{r}(k) = \underline{\phi}^{r'}\space(k)\space \underline{\theta}^{r}(k) + v^{r}(k)$$
where,$$\begin{align}
\underline{\theta}^{r'}(k) &= [\theta^{r}_{1}(k), \theta^{r}_{2}(k), \theta^{r}_{3}(k), ..., \theta^{r}_{N+M}(k)] \\
\underline{\phi}^{r'}(k) &= [-\alpha^{r}(k-1), -\alpha^{r}(k-2), ...,-\alpha^{r}(k-N), u^{r}(k-1), u^{r}(k-2), ..., u^{r}(k-M)]
\end{align}$$
For $r = 0, 1, 2,...,r$ [Eq.4] gives a set of liner algebraic equations which may be written in the matrix form as
$$\underline{\alpha}^{r}(k) = \Phi^{r}(k)\space\theta^{r}(k) + v^{r}(k)$$
where $\alpha^{r}$ and $v^{r}$ are repetition trajectory arrays, and [Eq.5]$$\Phi^{r'}(k) = [\underline{\phi^{0}}\quad \underline{\phi^{1}}\quad ...\quad \underline{\phi^{r}}]$$
It can easily be shown that the least square estimate of $\theta^{r}(k)$ minimizing the sum of the squares of the equation errors, $v^{r}(k)$s can be calculated recursively as follows:
[Eq.7-9]$$\begin{align}
\underline{\hat{\theta}}^{r+1}(k) &= \underline{\hat{\theta}}^{r}(k) + \underline{L}^{r}(k)[\alpha^{r+1}(k) - \underline{\hat{\theta}}^{r'}(k)\space \underline{\phi}^{r+1}(k)] \\[8pt]

\underline{L}^{r}(k) &= \frac{P^{r}(k) \underline{\phi}^{r+1}(k)}{\gamma + \underline{\phi}^{r+1'}(k)\space P^{r}(k) \space\underline{\phi}^{r+1}(k)} \\[8pt]

P^{r+1}(k) &= \frac{[I - \underline{L}^{r}(k)\space\underline{\phi}^{r+1'}(k)]\space P^{r} (k)}{\gamma} 
\end{align}$$
For a repetitive task which takes $p$ sampling interval ($p+1$ sampling instants) to complete, one would need ($p+1$) linear difference equations to approximate the real equation of motions. The
($p+1$) set of parameters of these difference equations can be updated independently by [Eq.7] from one repetition to the next. Since this updating does not have to be done on-line as long as it is done before the next repetition is started, the demand on the on-line computing power is lower.

### Conclusions and Future Dev
Results show that the LSTR is a quick learning controller which reduces tracking errors of the nonlinear system over repetitions. Also, the fact that the parameter updating part could be done off-line between repetitions, makes the control algorithm very feasible(Obsolete now, can make the model larger).

**The experimental results have shown that the proposed LSTR can be applied to the control of a nonlinear system. Also, the notion of a learning parameter estimator could be applied to other parameter estimators than the RLS estimator.**


---
[[Implementing in the Robot]]  ||  [[Autonomous Robot Project]]

