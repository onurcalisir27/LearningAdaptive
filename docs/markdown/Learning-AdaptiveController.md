### Problem Formulation
Consider a general discrete-time linear time-variant or time-invariant system,
[Eq.23]$$ x^k(t+1) = A(t)x^k(t) + B(t)u^k(t) + w^k(t) $$
[Eq.24] $$ y^k(t+1) = C^k(t+1)x^kk(t+1) \quad t= 0, 1,2,..., p-1 \quad k=0,1,2,...$$
where the state x is n-dimensional, the control u is m-dimensional, the output y is q-dimensional $(q \geq m)$, t is the time step in a p-step repetitive operation, and k is the repetition number. 

==In the learning control problem (as contrasted with the repetitive control problem in [5]) the system is assumed to always start from the same initial state in each repetition of the task.== Matrix A includes any states or output feedback control present in the system and the symbol $u^k$ is reserved for the signal added to the control for learning purposes. 

A time-variant model is considered because many applications such as in robotics and machining involve nonlinear dynamic systems which when linearized, produce linear models with coefficients that vary with the time step and vary in the same manner each repetition.

In such repetitive operations it is often the case that there will be disturbances $w^k(t)$ that repeat with each repetition of the task and the learning can be made to also correct for this source of errors in a natural way. ==One of the purposes of a feedback control is to handle any non-repetitive disturbances, and these will be ignored for purposes of designing the learning controller==. The solution to [Eq.23] can be written as:
[Eq.25]
$$
x^k(t+1)=\left(\prod_{j=0}^t A(j)\right) x^k(0)+\sum_{j=0}^t\left(\left(\prod_{r=j+1}^t A(r)\right)\left(B(j) u^k(j)+w^k(j)\right)\right)
$$
where the product symbol is taken to give the identity matrix if the lower limit is larger than the upper limit. Let $p$ be the total number of time steps in one repetition of a given task.

Writing [Eq.25] for successive repetitions $k$ and $k+1$ and subtracting the equation for repetition $k$ form that of repetition $k+1$. Assuming that disturbances $w_k$ are repetitive with a period of $p$ or a period of any integral divisor of $p$, and that the initial conditions are also repetitive from repetition k to k+1, we may write the following relation for the output of two consecutive repetitions,
[Eq.26-29]
$$
\begin{aligned}
&\begin{aligned}
& \mathbf{y}^{k+1}-\mathbf{y}^k=P \underbrace{\left(\mathbf{u}^{k+1}-\mathbf{u}^k\right)}_{\mathbf{v}^k} \\
& P=\left[\begin{array}{cccc}
C(1) B(0) & 0 & \cdots & 0 \\
C(2) A(1) B(0) & C(2) B(1) & \cdots & 0 \\
\vdots & \vdots & \ddots & \vdots \\
C(p) A(p-1) \cdots A(1) B(0) & \cdots & \cdots & C(p) B(p-1)
\end{array}\right]
\end{aligned}\\
&\text { The system transition matrix } P \text { is given by [Eq.27] } \text {, and, }\\
&\begin{aligned}
& \mathbf{y}^k=\left[\begin{array}{llll}
y^{k^T}(1) & y^{k^T}(2) & \cdots & y^{k^T}(p)
\end{array}\right]^T \\
& \mathbf{u}^k=\left[\begin{array}{llll}
u^{k^T}(0) & u^{k^T}(1) & \cdots & u^{k^T}(p-1)
\end{array}\right]^T
\end{aligned}
\end{aligned}
$$
Defining the error $e^k \equiv y^k - y_d$  where $y_d$ is the discrete desired trajectory, equation 26 can be written as,
[Eq.30] $$ Pv^k = e^{k+1} - e^k $$ At any repetition $k$, suppose there exists some change in our input vector, $\tilde{v}^k$, which would lead to a zero tracking error at the next repetition, $k+1$, namely, $e_{k+1}=0$ . Then [Eq.30] for the given change of input would be as follows,
[Eq.31]$$ P\tilde{v}^{k}= 0 - e^{k}$$
If $P^k$ is an approximation to P at the $k^{th}$ repetition, then one can write.
[Eq.32] $$ P^{k} v^{k} = -e^k $$
where $v^k$ is a change in the control vector at repetition $k$ that would use the information in the approximation to $P$ $(P_k)$ and it would result in an $e^{k+1}$ which is zero (or is minimum F norm as will discussed later.)

Let us assume that we are provided with an initial guess for the P matrix at the zeroth repetition,
[Eq.33]$$P^{0}v^{0}=-e^{0}$$ then, $v^{0}$ may be solved for, by using the error vector $e^{0}$ provided by the real system using control vector $u^{0}$. Generally, an exact $v^{0}$ may not exist, but a minimum error solution may be obtained for $v^{0}$ through using the Moore-Penrose pseudo-inverse in the sense that $v^{0}$ would minimize,

[Eq.34]$$
\left\|P^0 \mathbf{v}^0+\mathbf{e}^0\right\|_{\mathcal{F}}
$$

namely,
[Eq.35]
$$
\mathbf{v}^0=-P^{0^{\dagger}} \mathbf{e}^0
$$


If we keep 32 satisfied for all $k$, then we may write,
[Eq.36]
$$
\mathbf{v}^k=-P^{k+} \mathbf{e}^k \quad \text { for all } k
$$

minimizing $\left\|P^k \mathbf{v}^k+\mathbf{e}^k\right\|_{\mathcal{F}}$.
At any repetition $k$, the actual system parameters, $P$, could be written as,
[Eq.37]
$$
P=P^k+D^k
$$
where $D^{k}$ is a matrix of corrections for $P^{k}$ at each repetition k. Substituting for $P$ in [Eq.30] from [Eq.37]
[Eq.38]
$$(P^{k}+ D^{k} ) v^{k} = e^{k+1} - e^{k}$$
Solve for $D^{k}v^{k}$ from [Eq.38]
[Eq.39]$$D^{k} v^{k} = e^{k+1} - e^{k} - P^{k} v^{k}$$ Since $e^{k+1}$ is the error through the introduction of $u^{k+1} =u^{k}+ v^{k}$, then the only unknown in [Eq.39] is the correction matrix $D^{k}$, one solution to [Eq.39] would be
[Eq.40]$$ D^{k} = \frac {(e^{k+1}-e^{k}-P^{k}v^{k}) {z^{k}}^{T}}{{z^{k}}^Tv^{k}}$$
where $z^{k}$ are secant projection vectors, chosen in the following way [1]:
- If $k \geq mp-1$ then $z^{k}$ is chosen orthogonal to the previous $mp-1$ control steps, $v^{k-(mp-1)}, ..., v^{k-1}$
-  If $k < mp-1$ then $z^{k}$ is chosen orthogonal to the available t vectors, $v^0, ..., v^{k-1}$ 

One possibility is to pick $z^{k}$ as a linear combination of $v^{0}, ..., v^{k}$ which would be orthogonal to all  $v^{0}, ..., v^{k-1}$. Similar to the case of the adaptive control discussed earlier, any orthogonalization technique may be used for the actual evaluation of the $z$ vectors.

With the above formulation in mind, a recursive generalized secant learning adaptive control scheme is given by the following two recursive equations and the use of an orthogonalization method such as Gram-Schmidt:
[Eq.41]$$v^{k}= -P^{k+}e^{k}$$
[Eq.42]$$P^{k+1}=P^{k} + \frac {(e^{k+1}-e^{k}-P^{k}v^{k}) {z^{k}}^{T}}{{z^{k}}^Tv^{k}}$$
