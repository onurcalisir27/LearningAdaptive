**Link** back to Control's Paper:   [[Adaptive Control Research]]
### Problem Formulation
[Eq.1]
$$y(t+1) = \hat{A} x(t) + \hat{B}u(t) + w(t)$$
Let us assume that the elements in u(t) could be arranged in such a way that all the elements which are dependent on time t are placed at the bottom portion of the vector u(t) and all the elements which signify the effects of the previous $0 \leq \tau < t$ are placed on the top portion of the u(t) vector, namely:
[Eq.2]$$u(t) = [u_1^T(t-1)  |   u_2^T(t)]^T $$ [Eq.3]$$y(t+1)=\hat{A} x(t)+\underbrace{\left[\hat{B}^1 \mid \hat{B}^2\right]}_B\left[\begin{array}{c}u^1(t-1) \\ u^2(t)\end{array}\right]+w(t)$$[Eq.4]$$y(t+1)=\underbrace{\left[\begin{array}{cc}\hat{A} & \hat{B}\end{array}\right]}_S \underbrace{\left[\begin{array}{l}x(t) \\ u(t)\end{array}\right]}_{v(t)}+w(t)$$
 Let us assume for now that there is some control actions $u^2(t)$ such that when applied to the real system, S, it would generate an output $y_{t+1}$ equal to the desired value $y_{t+1}^d$ and let us call the $v$ vector containing that control, $\tilde{v_t}$. Therefore,
  [Eq. 5] $$y_{t+1}^d = S \tilde{v_t}$$
 Now let us assume that there exists some combination of the system matrix S and the control input  $u^2(t)$ such that when placed in the v vector, it provides the same result as in equation above, and call this pair $S_t$ and $v_t$.
 [Eq. 6] $$y_{t+1}^d = S_t {v_t}$$
  There might not exists such a pair of S and v that would get the output at time t+1 to be equal to the desired value. However given an $S_t$, one could always find some $u^2(t)$ such that when placed in v, the Frobenius norm of the difference between the actual output and the desired output at time t+1 is minimized. This $u^2(t)$ is given by the following equation:
   [Eq. 7] $$u^2(t)=\hat{B}_t^{2^+}\left(y^d(t+1)-\left[\hat{A}_t \mid \hat{B}_t^1\right]\left[\begin{array}{c}x(t) \\ u^1(t-1)\end{array}\right]\right)$$
   **$u^2(t)$ given by [Eq.7] minimizes the Frobenius norm of the error**, $||y^d(t+1) - y(t+1)||_{\mathcal{F}}$ .

Through the application of the control given by [Eq.7], to the actual system, at time t,  some output $y_{t+1}$ will be generated. 

Let us assume that at time t, the best estimate of the system matrix available to us is $S_t$. Therefore, the difference between the true system matrix S and $S_t$ could be denoted by $\Delta S_t$ . 

Now let us impose the condition that $S_{t+1}$ would generate the same output $y_{t+1}$ as would the real system, S, with the given $v_t$. Therefore: 
[Eq. 8] $y_{t+1} = S_{t+1} v_t$

Rewriting equation 8 in terms of $S_t$ and the difference between $S_t$ and the difference between $S_t$ and $S_{t+1}$ such that equation 8 holds,
[Eq. 9] 
$$y_{t+1} = (S_t + \Delta S_t) v_t$$

One way to solve equation above is to update the system matrix in one direction each time using the generalized secant update [1] in the following fashion:
[Eq. 10] $$ S_{t+1} = S_t + \Delta S_t$$ 
where: [Eq. 11] $$ \Delta S_t = \frac{(y(t+1) - S_t v_t) z_t^T}{z_t^T v_t} $$

$z_t$'s are secant projection vectors, chosen in the following way:
- If $t \geq n+m-1$ then $z_t$ is chosen orthogonal to the previous n+m-1 vectors, $v_{t-(n+m-1)}, ..., v_{t-1}$
-  If $t < n+m-1$ then $z_t$ is chosen orthogonal to the available t vectors, $v_0, ..., v_{t-1}$ 

One possibility is to pick $z_t$ as a linear combination of $v_0, ..., v_{t}$ which would be orthogonal to all $v_0, ..., v_{t-1}$ . A few orthogonalization methods are available in the literature which may be used for the actual evaluation  of the z vectors. 

These methods include the well-known Gram-Schmidt orthogonalization process [7] and a more advanced technique due to Fletcher [8] in which the number of vectors in the set are likely to be increased or decreased. 

Therefore using an orthogonalization method and [Eq.7&10] in a recursive fashion; a control could be evaluated using [Eq. 7]  and after being applied to the real system, based on the response of the system, a better estimate of the system matrices may be obtained using [Eq. 10] .