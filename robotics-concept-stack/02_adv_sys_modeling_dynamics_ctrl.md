# **Advanced System Modeling, Dynamics and Control**

### **What Actually Matters When You're Building Something**

**Focus:** practical depth: what you use in robotics, autonomy, and real control work.

---

# Why this subject exists in your stack

Every robot you build has the same problem: you have a physical system that evolves over time, you can only measure some of it, and you need to make it do what you want reliably.

That problem has three parts:

1. **Model it** - write down the equations that describe how the system evolves.
2. **Analyze it** - understand whether it's stable, what you can control, what you can observe.
3. **Control it** - design an input signal that drives it to the desired behavior.

This subject gives you the mathematical language and tools to do all three rigorously.

In industry; robotics, aerospace, automotive, energy systems; this is the toolkit that underlies almost every controller that runs in production. Model predictive control, LQG, Kalman filters, trajectory optimization: all of it sits on this foundation.

---

# How to use this document

## Mode 1: Quick refresh before an interview or design review

Read the Core Formula Sheet and each section's "What this means in practice" block.

## Mode 2: Design session

Read the relevant section fully - modeling, controllability check, observer design, LQR tuning.

## Mode 3: Full rebuild

Read everything linearly. Every section builds on the previous one.

# Core Formula Sheet

## The Model

$$
\dot{x} = Ax + Bu, \qquad y = Cx + Du
$$

$$
x \in \mathbb{R}^n \text{ (state)}, \quad u \in \mathbb{R}^m \text{ (input)}, \quad y \in \mathbb{R}^p \text{ (output)}
$$

## System Response

$$
x(t) = e^{At}x_0 + \int_0^t e^{A(t-\tau)}Bu(\tau)\,d\tau
$$

## Transfer Function

$$
H(s) = C(sI - A)^{-1}B + D
$$

## Controllability

$$
P = \begin{bmatrix} B & AB & A^2B & \cdots & A^{n-1}B \end{bmatrix}, \qquad \text{controllable} \Leftrightarrow \text{rank}(P) = n
$$

## Observability

$$
Q = \begin{bmatrix} C \\ CA \\ CA^2 \\ \vdots \\ CA^{n-1} \end{bmatrix}, \qquad \text{observable} \Leftrightarrow \text{rank}(Q) = n
$$

## Stability

$$
\text{Asymptotically stable} \Leftrightarrow \text{Re}(\lambda_i(A)) < 0 \quad \forall\, i
$$

## State Feedback + Observer

$$
u = -K\hat{x} + Gr, \qquad \dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x})
$$

$$
\text{eig}(A-BK) = \text{controller poles}, \qquad \text{eig}(A-LC) = \text{observer poles}
$$

## LQR

$$
J = \int_0^\infty \left[x^T Q x + u^T R u\right] dt \quad \Rightarrow \quad u^* = -K^* x, \quad K^* = R^{-1}B^T P^*
$$

where $P^*$ solves $A^T P + PA - PBR^{-1}B^T P + Q = 0$.

# Section 1: Modeling - Building the State Space

## What it is

You take a physical system - a robot joint, a drone, a motor - and write it as:

$$
\dot{x} = Ax + Bu, \qquad y = Cx + Du
$$

The state $x$ is everything the system needs to know about itself to predict its future. For a mechanical system with position and velocity, that's $x = [q, \dot{q}]^T$. For a thermal system, it might be temperatures at different nodes.

## How to build it from a mechanical system

**Three steps, always the same:**

1. Choose state variables - typically positions and velocities.
2. Draw a free body diagram, write Newton's law.
3. Rearrange into first-order form $\dot{x} = Ax + Bu$.

**Standard spring-mass-damper** ($m\ddot{y} + c\dot{y} + ky = F$):

Let $x_1 = y$, $x_2 = \dot{y}$:

$$
\dot{x} = \begin{bmatrix} 0 & 1 \\ -k/m & -c/m \end{bmatrix} x + \begin{bmatrix} 0 \\ 1/m \end{bmatrix} u, \qquad y = \begin{bmatrix} 1 & 0 \end{bmatrix} x
$$

This is the template for any single DOF mechanical system. Robot joints, actuators, suspension systems - all start here.

## From transfer function to state space

If you have $H(s) = \frac{b_{n-1}s^{n-1} + \cdots + b_0}{s^n + a_{n-1}s^{n-1} + \cdots + a_0}$, the **Controller Canonical Form** gives you a state space immediately:

$$
A_{CCF} = \begin{bmatrix}
0 & 1 & 0 & \cdots & 0 \\
0 & 0 & 1 & \cdots & 0 \\
\vdots & & & \ddots & \vdots \\
0 & 0 & 0 & \cdots & 1 \\
-a_0 & -a_1 & -a_2 & \cdots & -a_{n-1}
\end{bmatrix}, \quad
B_{CCF} = \begin{bmatrix} 0 \\ 0 \\ \vdots \\ 0 \\ 1 \end{bmatrix}
$$

$$
C_{CCF} = \begin{bmatrix} b_0 & b_1 & \cdots & b_{n-1} \end{bmatrix}, \quad D_{CCF} = 0
$$

The bottom row of $A_{CCF}$ is the negative of the denominator coefficients. The $C$ row is the numerator coefficients. Read them off directly.

## What this means in practice

In robotics you almost never write a transfer function. You write $\dot{x} = f(x, u)$ directly from the dynamics, then linearize. The CCF form matters when you're working with a system identified from frequency response data - which does happen in motor control and structural dynamics.

The key habit: **always define your state vector explicitly before writing any equations.** Sloppy state definitions cause bugs that are almost impossible to trace.

## Discrete-time version

For digital controllers running at sample rate $T_s$:

$$
x(k+1) = A_d x(k) + B_d u(k), \qquad y(k) = Cx(k) + Du(k)
$$

Convert from continuous:

$$
A_d = e^{AT_s}, \qquad B_d = \int_0^{T_s} e^{A\tau} d\tau \cdot B
$$

MATLAB: `[Ad, Bd] = c2d(A, B, Ts)`.

Rule of thumb: sample rate should be at least 10–20× the closed-loop bandwidth. Faster is safer.

# Section 2: Linearization - Making Nonlinear Systems Tractable

## What it is

Almost every real system is nonlinear: pendulums, robot arms, quadrotors, cars. Linear control theory can't be applied directly. Linearization is the bridge.

You expand the nonlinear dynamics around an operating point using a first-order Taylor expansion. The result is a local linear approximation that is valid in a neighborhood of that point.

## The procedure

Start with:

$$
\dot{x} = f(x, u), \qquad y = h(x, u)
$$

Pick an **equilibrium** (operating point): find $x_{eq}$, $u_{eq}$ such that $f(x_{eq}, u_{eq}) = 0$.

Define deviations: $x_\delta = x - x_{eq}$, $u_\delta = u - u_{eq}$.

The linearized system:

$$
\dot{x}_\delta = A x_\delta + B u_\delta, \qquad y_\delta = C x_\delta + D u_\delta
$$

where the Jacobians are evaluated at the equilibrium:

$$
A = \frac{\partial f}{\partial x}\bigg|_{x_{eq},\, u_{eq}}, \quad
B = \frac{\partial f}{\partial u}\bigg|_{x_{eq},\, u_{eq}}, \quad
C = \frac{\partial h}{\partial x}\bigg|_{x_{eq},\, u_{eq}}, \quad
D = \frac{\partial h}{\partial u}\bigg|_{x_{eq},\, u_{eq}}
$$

## Worked example - nonlinear spring-mass-damper

System: $m\ddot{y} = F - ky^3 - c\dot{y}$, $m = c = k = 1$.

States: $x_1 = y$, $x_2 = \dot{y}$.

$$
f(x, u) = \begin{bmatrix} x_2 \\ u - x_1^3 - 3x_2 \end{bmatrix}
$$

Equilibrium at $\tilde{x} = [1,\, 0]^T$, $\tilde{u} = 1$ (since $1 - 1^3 - 0 = 0$):

$$
A = \begin{bmatrix} 0 & 1 \\ -3x_1^2 & -3 \end{bmatrix}\bigg|_{x_1=1} = \begin{bmatrix} 0 & 1 \\ -3 & -3 \end{bmatrix}, \qquad B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}
$$

Now you have an LTI system you can design a controller for.

## Pendulum - two equilibria

$\dot{x} = \begin{bmatrix} x_2 \\ -(g/l)\sin(x_1) + u/(ml^2) \end{bmatrix}$

Equilibria at $u = 0$: $x_1 = 0$ (pendulum down) and $x_1 = \pi$ (pendulum up).

**At $x_{eq} = [0, 0]^T$ (pendulum down):**

$$
A = \begin{bmatrix} 0 & 1 \\ -g/l & 0 \end{bmatrix} \quad \Rightarrow \text{ eigenvalues } \pm j\sqrt{g/l}, \text{ marginally stable (undamped)}
$$

**At $x_{eq} = [\pi, 0]^T$ (pendulum up):**

$$
A = \begin{bmatrix} 0 & 1 \\ +g/l & 0 \end{bmatrix} \quad \Rightarrow \text{ eigenvalues } \pm\sqrt{g/l}, \text{ unstable}
$$

## What this means in practice

Every nonlinear controller you write, model predictive control, feedback linearization, LQR on a drone, uses this step. You identify the operating regime, compute the Jacobian, and get an LTI model you can work with.

In robotics: the manipulator Jacobian $J(q)$ is exactly this $A$ matrix for the kinematics. When you compute $J(q)$ at the current configuration and use it for velocity-level control, you are doing real-time linearization at every timestep.

**Lyapunov's indirect method:** If the Jacobian $A$ at an equilibrium is Hurwitz (all eigenvalues have negative real part), the nonlinear system is locally asymptotically stable at that equilibrium. If any eigenvalue has positive real part, the equilibrium is unstable. This is how you determine stability of the upright pendulum, the drone hover, or the robot standing configuration, analytically, before you ever run a simulation.

# Section 3: System Response: What the System Actually Does

## Matrix exponential

The free response of a linear system is:

$$
x(t) = e^{At} x_0
$$

The matrix exponential is computed as:

$$
e^{At} = \mathcal{L}^{-1}\left[(sI - A)^{-1}\right]
$$

For a diagonal $A = \text{diag}(\lambda_1, \ldots, \lambda_n)$:

$$
e^{At} = \text{diag}(e^{\lambda_1 t}, \ldots, e^{\lambda_n t})
$$

In practice you never compute this by hand for large systems, MATLAB's `expm(A*t)` handles it. But you need to understand what it means: each mode of the system decays (or grows) at the rate of its eigenvalue.

## Full response

$$
x(t) = \underbrace{e^{At}x_0}_{\text{zero-input}} + \underbrace{\int_0^t e^{A(t-\tau)}Bu(\tau)\,d\tau}_{\text{zero-state}}
$$

The zero-input part is the natural decay from initial conditions. The zero-state part is the response to the input. They add linearly, this is superposition, and it only holds because the system is linear.

In frequency domain:

$$
Y(s) = \underbrace{C(sI-A)^{-1}x_0}_{\text{zero-input}} + \underbrace{\left[C(sI-A)^{-1}B + D\right] U(s)}_{\text{zero-state} = H(s)\cdot U(s)}
$$

## What this means in practice

When you simulate a robot after a perturbation (initial condition $x_0 \neq 0$) with no input, you're computing $e^{At}x_0$. When you apply a command, you're computing the convolution integral. Your simulation engine (MATLAB's `lsim`, ROS with `ode45`) is numerically computing exactly this.

The modes of $e^{At}$ tell you the natural frequencies and damping of your system before any control is applied. If those are badly placed, too slow, too oscillatory, or unstable, that's your plant dynamics problem, separate from your controller design problem.

# Section 4: Stability: Will It Blow Up?

## The only criterion you need for LTI systems

A linear system $\dot{x} = Ax$ is **asymptotically stable** if and only if all eigenvalues of $A$ have strictly negative real parts. $A$ is then called **Hurwitz**.

$$
\text{Re}(\lambda_i(A)) < 0 \quad \forall\, i \quad \Leftrightarrow \quad \lim_{t \to \infty} x(t) = 0 \text{ for any } x_0
$$

That's it. Check the eigenvalues. MATLAB: `eig(A)`.

## Marginal stability: the dangerous middle ground

If any eigenvalue sits exactly on the imaginary axis ($\text{Re}(\lambda) = 0$), the system is marginally stable, it neither grows nor decays. In practice this is fragile. Any small perturbation, noise, or model error can push it unstable.

Marginally stable systems driven at their natural frequency produce **resonance**, unbounded output from bounded input. The Tacoma Narrows Bridge collapse in 1940 was resonance. Any undamped oscillator hit at $\omega_n$ is the same phenomenon. Never design a controller that leaves poles on the imaginary axis.

## Lyapunov stability check (for when you need a certificate)

For $\dot{x} = Ax$, pick any positive definite $Q$ (usually $Q = I$) and solve:

$$
A^T P + PA = -Q
$$

for $P$. If $P$ is positive definite, the system is asymptotically stable.

This is the **Lyapunov matrix equation**. In MATLAB: `P = lyap(A', Q)`.

Why bother when you can just check eigenvalues? Two reasons:

1. For nonlinear systems you can't check eigenvalues globally, a Lyapunov function gives you a stability certificate with a region of attraction.
2. The Lyapunov equation is the backbone of LQR (Section 7), you'll solve a version of it for every optimal controller you design.

## Stability of the closed-loop system

Once you close the loop with $u = -Kx$:

$$
\dot{x} = (A - BK)x
$$

Stability is now determined by the eigenvalues of $(A - BK)$, not $A$. Your job as a control designer is to choose $K$ so those eigenvalues are where you want them, this is pole placement, covered in Section 6.

# Section 5: Controllability and Observability: What You Can and Can't Do

## Controllability

**Question:** Can you drive the system from any initial state to any target state in finite time?

**Answer:** Yes, if and only if the controllability matrix has full rank.

$$
P = \begin{bmatrix} B & AB & A^2B & \cdots & A^{n-1}B \end{bmatrix}
$$

$$
\text{rank}(P) = n \quad \Leftrightarrow \quad \text{system is controllable}
$$

MATLAB: `rank(ctrb(A, B))`.

If rank$(P) < n$, some states are unreachable no matter what input you apply. Pole placement and LQR both require full controllability.

**Physical intuition:** Controllability fails when some part of the system is dynamically disconnected from the input. A motor that can only spin one axis can't control the other two. A thermal actuator on one side of a symmetric system can't independently control both halves if the structure only passes the average temperature.

**Checking in practice:** Don't just compute `rank(ctrb(A,B))`, check the condition number of $P$ too. A nearly rank-deficient controllability matrix means some modes are weakly controllable, which translates to very large required inputs to move those modes. In hardware that means actuator saturation.

## Observability

**Question:** Can you reconstruct the full state from output measurements alone?

**Answer:** Yes, if and only if the observability matrix has full rank.

$$
Q = \begin{bmatrix} C \\ CA \\ CA^2 \\ \vdots \\ CA^{n-1} \end{bmatrix}
$$

$$
\text{rank}(Q) = n \quad \Leftrightarrow \quad \text{system is observable}
$$

MATLAB: `rank(obsv(A, C))`.

If rank$(Q) < n$, some states are invisible to your sensors. No observer or filter, not even a Kalman filter, can recover them.

**Physical example:** A 2-DOF robot arm where you only have an encoder on joint 1. Is joint 2's velocity observable? Depends on the coupling, if joint 2 dynamics are reflected through joint 1 (via inertial coupling), you might be able to infer it. If they're fully decoupled, you can't. The observability matrix tells you the answer analytically before you build anything.

## Duality: the key relationship

Controllability of $(A, B)$ is mathematically identical to observability of $(A^T, B^T)$.

This means every algorithm for controller design has a direct dual for observer design. Design $K$ for control → the same procedure gives you $L$ for the observer, just transposed. You'll use this constantly.

## What this means in practice

Before you start any control design:

1. Build your model.
2. Check `rank(ctrb(A,B)) == n`. If not, figure out what's uncontrollable and why.
3. Check `rank(obsv(A,C)) == n`. If not, figure out what's unobservable and why, or add a sensor.

Skipping this check and going straight to pole placement is a common mistake. MATLAB's `place()` will fail silently or give garbage if the system isn't fully controllable.

# Section 6: Pole Placement: Designing the Closed-Loop Response

## What you're doing

With state feedback $u = -Kx$, the closed-loop system is $\dot{x} = (A - BK)x$. You choose $K$ to put the eigenvalues of $(A - BK)$ exactly where you want them.

**This requires the system to be controllable.** If it's not, you can't place all poles freely.

## Choosing where to put the poles

Poles in the left half of the complex plane make the system stable. The further left, the faster the decay. Complex poles add oscillation.

For a **second-order system** the poles are $s_{1,2} = -\zeta\omega_n \pm j\omega_n\sqrt{1-\zeta^2}$ where:

- $\omega_n$ - natural frequency: controls how fast the response is.
- $\zeta$ - damping ratio: controls how much it oscillates.

**The design formulas you use repeatedly:**

From specs → pole locations:

$$
\zeta = \frac{-\ln(\%OS/100)}{\sqrt{\pi^2 + \ln^2(\%OS/100)}}
$$

$$
\omega_n = \frac{4}{\zeta \cdot t_s} \quad \text{(2\% settling time criterion)}
$$

$$
\omega_d = \omega_n\sqrt{1-\zeta^2} \quad \text{(damped natural frequency)}
$$

$$
s_{1,2} = -\zeta\omega_n \pm j\omega_d
$$

Once you have the desired poles, compute $K$:

MATLAB: `K = place(A, B, desired_poles)`.

For repeated poles, use `K = acker(A, B, desired_poles)` (Ackermann's formula).

## Higher-order systems: dominant pole approximation

For systems higher than 2nd order, place two dominant poles where the 2nd-order specs require, and push all remaining poles at least 5–10× further left so they die out fast and don't affect the response.

Rule: if $|\text{Re}(s_{\text{fast}})| > 5 \times |\text{Re}(s_{\text{dominant}})|$, the fast poles are negligible in the transient.

**Example:** 3rd-order system, specs give dominant poles at $s_{1,2} = -2 \pm 2j$. Place third pole at $s_3 = -20$. The $e^{-20t}$ component dies in $< 0.2s$, the dominant poles give the $e^{-2t}$ envelope.

## Steady-state tracking: the feedforward gain $G$

Pole placement alone doesn't guarantee the output reaches the reference. You need to set the DC gain correctly.

With $u = -Kx + Gr$, the steady-state output for a unit step reference is $H_{cl}(0)$. To make this equal to 1:

$$
G = \frac{-1}{C(A-BK)^{-1}B}
$$

MATLAB: `G = -inv(C * inv(A - B*K) * B)`.

Always set this. Otherwise your output will settle at the wrong value.

## Bass-Gura formula (when you need to do it by hand)

For SISO systems: let the open-loop characteristic polynomial be $s^n + a_{n-1}s^{n-1} + \cdots + a_0$ and the desired polynomial be $s^n + \hat{a}_{n-1}s^{n-1} + \cdots + \hat{a}_0$.

In Controller Canonical Form coordinates:

$$
K_{CCF} = \begin{bmatrix} \hat{a}_0 - a_0 & \hat{a}_1 - a_1 & \cdots & \hat{a}_{n-1} - a_{n-1} \end{bmatrix}
$$

Convert back: $K = K_{CCF} \cdot T_{CCF}^{-1}$.

In practice you use MATLAB's `place()`. But you need to understand that pole placement is just a polynomial matching problem, you're choosing $K$ so that $\det(sI - (A-BK))$ equals your desired polynomial.

# Section 7: LQR: The Industrial Standard for State Feedback

## Why LQR instead of pole placement

Pole placement works for low-order SISO systems. For MIMO systems or systems with more than 3–4 states, manually choosing pole locations becomes guesswork. LQR gives you a systematic way to trade off performance against effort, with optimality guarantees.

In industry: model predictive control (MPC), LQG, and most modern control architectures are built on LQR principles. If you understand LQR, you understand the foundation of all of them.

## The problem

Minimize the infinite-horizon quadratic cost:

$$
J = \int_0^\infty \left[x^T Q x + u^T R u\right] dt
$$

subject to $\dot{x} = Ax + Bu$.

- $Q \succeq 0$ - penalizes state deviation. How much do you care about each state being away from zero?
- $R \succ 0$ - penalizes control effort. How expensive is actuation?

## The solution

The optimal control law is linear state feedback:

$$
u^* = -K^* x, \qquad K^* = R^{-1} B^T P^*
$$

where $P^*$ is the unique positive definite solution to the **Algebraic Riccati Equation (ARE)**:

$$
A^T P + PA - P B R^{-1} B^T P + Q = 0
$$

MATLAB: `[K, P, e] = lqr(A, B, Q, R)`.

The closed-loop eigenvalues are returned in `e`, check them.

## How to tune $Q$ and $R$

This is the engineering judgment that makes LQR practical.

**Start here:** $Q = C^T C$, $R = \rho I$ where $\rho$ is a scalar. This penalizes output error directly. Increase $\rho$ to use less control effort (slower, smoother). Decrease $\rho$ to use more effort (faster, more aggressive).

**State weighting:** $Q = \text{diag}(q_1, q_2, \ldots, q_n)$. Set $q_i$ large for states you care most about tracking. If position matters 100× more than velocity, $Q = \text{diag}(100, 1)$.

**Bryson's rule (standard starting point):**

$$
Q_{ii} = \frac{1}{(\text{max acceptable deviation of } x_i)^2}, \qquad R_{jj} = \frac{1}{(\text{max acceptable value of } u_j)^2}
$$

Normalize each state and input by its acceptable range. Start there, then tune.

**Cheap vs expensive control:**

| $Q$ large, $R$ small | $Q$ small, $R$ large |
|---|---|
| "Cheap control" | "Expensive control" |
| Aggressive, fast, large $u$ | Conservative, slow, small $u$ |
| Good for fast disturbance rejection | Good for actuator-limited systems |

## LQR robustness guarantees

This is why engineers prefer LQR over pole placement: it comes with guaranteed robustness margins.

For SISO systems, a properly designed LQR controller guarantees:
- Gain margin: $\geq 6$ dB (can double or halve the loop gain before instability)
- Phase margin: $\geq 60°$

Pole placement gives you none of these guarantees by default.

## Practical LQR workflow

1. Build and validate your model $(A, B, C)$.
2. Check controllability.
3. Set initial $Q$ and $R$ using Bryson's rule.
4. Run `lqr(A, B, Q, R)`, check eigenvalues of $(A - BK)$.
5. Simulate step response, disturbance rejection, noise sensitivity.
6. Adjust $Q/R$ ratio until specs are met.
7. Check actuator saturation in simulation. If $u$ saturates, increase $R$ or reduce $Q$.

# Section 8: Observer Design: Estimating What You Can't Measure

## The problem

In every real system, you don't have access to the full state. Encoders give position, not velocity (usually). IMUs give acceleration and angular rate, not absolute pose. Pressure sensors give one scalar, not the full fluid state.

You need to reconstruct the state from the available outputs, that's the observer.

## The Luenberger Observer

Build a virtual copy of the system, but inject the output error to correct it:

$$
\dot{\hat{x}} = A\hat{x} + Bu + L(y - \hat{y}), \qquad \hat{y} = C\hat{x}
$$

Define the estimation error $e = x - \hat{x}$:

$$
\dot{e} = (A - LC)e
$$

The error dynamics are governed by $(A - LC)$. By choosing $L$, you place the eigenvalues of $(A - LC)$, exactly like pole placement, but for the observer.

**This requires the system to be observable.** If it's not, there exist initial states that produce zero output, you can never recover them.

## Designing $L$

By duality, observer design is pole placement on the dual system. The same Bass-Gura formula applies:

MATLAB: `L = place(A', C', desired_observer_poles)'`.

Note the transpose: you're placing poles for $(A - LC)^T = A^T - C^T L^T$, then transposing back.

## Where to place observer poles

Observer poles should be **faster** than controller poles, typically 3–5× further left in the complex plane.

Why? The controller needs the state estimate to be accurate before it acts. If the observer is too slow, the controller is acting on stale, inaccurate estimates, which degrades performance or causes instability.

Why not infinitely fast? Faster observer poles mean higher observer gain $L$, which amplifies sensor noise. There's a direct trade-off: fast convergence vs noise sensitivity. This trade-off is exactly what the Kalman filter optimizes.

**Rule of thumb:**
- If controller poles are at $\text{Re} = -\sigma$, place observer poles at $\text{Re} = -(3\text{ to }5)\sigma$.
- For noisy sensors, prefer the Kalman filter over manual observer pole placement.

## Luenberger vs Kalman filter

| | Luenberger | Kalman Filter |
|---|---|---|
| Design method | Pole placement for $(A-LC)$ | Minimize estimation error variance |
| Handles noise? | No (deterministic) | Yes (optimal for Gaussian noise) |
| MATLAB | `place(A', C', poles)'` | `kalman()` or `lqe()` |
| Use when | Noise is not the main concern | Sensor noise / process noise present |

The Kalman filter is a Luenberger observer where $L$ is computed optimally given noise covariances. Same structure, optimal gain.

# Section 9: Separation Principl: Putting It All Together

## The architecture

The full observer-based controller:

$$
u = -K\hat{x} + Gr \qquad \text{(controller uses estimated state)}
$$

$$
\dot{\hat{x}} = A\hat{x} + Bu + L(y - C\hat{x}) \qquad \text{(observer updates estimate)}
$$

## Why it works: separation principle

Write the augmented system in terms of $x$ (true state) and $e = x - \hat{x}$ (error):

$$
\begin{bmatrix} \dot{x} \\ \dot{e} \end{bmatrix} =
\begin{bmatrix} A-BK & BK \\ 0 & A-LC \end{bmatrix}
\begin{bmatrix} x \\ e \end{bmatrix} +
\begin{bmatrix} BG \\ 0 \end{bmatrix} r
$$

The system matrix is **block upper triangular**. Its eigenvalues are:

$$
\text{eig}(A-BK) \cup \text{eig}(A-LC)
$$

**The controller poles and observer poles are completely independent.** Design them separately, combine them freely. This is the separation principle.

## What this means in practice

This is the architecture of almost every deployed controller in robotics and aerospace:

- Design your state feedback gain $K$ (via LQR or pole placement) as if you had perfect state access.
- Design your observer gain $L$ (Luenberger or Kalman) as if you had no controller.
- Combine. The combined system is stable if and only if both $(A-BK)$ and $(A-LC)$ are stable.

This architecture is called **LQG (Linear Quadratic Gaussian)** when you use LQR for $K$ and Kalman filter for $L$. It is the most widely deployed optimal controller in engineering.

**In ROS/real hardware:** your state estimator node (running the observer/Kalman filter) is separate from your controller node (running the feedback law). The separation principle is the theoretical justification for that modular architecture.

# Section 10: Second-Order System Specs - The Language of Control Engineers

## Why second order matters

Most real systems, when designed well, behave approximately like second-order systems near their operating point. The resonance of a joint, the attitude dynamics of a drone, the pitch of a car suspension: all second-order-dominated in the frequency range that matters.

Even for higher-order systems, you use dominant pole approximation to reduce to second-order and then apply these specs. So these formulas come up constantly.

## Standard form

$$
H(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}
$$

For the spring-mass-damper: $\omega_n = \sqrt{k/m}$, $\zeta = c/(2\sqrt{km})$.

## Response specs for underdamped systems ($0 < \zeta < 1$)

| Spec | Formula | What it means |
|---|---|---|
| %OS (percent overshoot) | $e^{-\pi\zeta/\sqrt{1-\zeta^2}} \times 100$ | How much output exceeds final value |
| Settling time $t_s$ | $4/(\zeta\omega_n)$ | Time to stay within 2% of final value |
| Peak time $t_p$ | $\pi/\omega_d$ | Time to reach first peak |
| Damped frequency | $\omega_d = \omega_n\sqrt{1-\zeta^2}$ | Frequency of oscillation |

**Inverse: from specs to pole locations:**

$$
\zeta = \frac{-\ln(\%OS/100)}{\sqrt{\pi^2 + \ln^2(\%OS/100)}}, \qquad \omega_n = \frac{4}{\zeta \cdot t_s}, \qquad s_{1,2} = -\zeta\omega_n \pm j\omega_d
$$

## Damping cases: physical meaning

| $\zeta$ | Behavior | Real-world analog |
|---|---|---|
| $\zeta > 1$ | Overdamped - slow, no overshoot | Heavy fluid suspension |
| $\zeta = 1$ | Critically damped - fastest without overshoot | Optimal door hinge |
| $0 < \zeta < 1$ | Underdamped - fast but oscillates | Spring with light damping |
| $\zeta = 0$ | Undamped - oscillates forever | Ideal LC circuit |
| $\zeta < 0$ | Unstable - diverges | Inverted pendulum without control |

## Typical design targets

In robotics and motion control:

- **$\zeta = 0.7$** is the sweet spot: about 4.3% overshoot, near-optimal settling. Most industrial servo controllers start here.
- **$\zeta = 0.5$** gives 16% overshoot, too much for precise positioning, fine for force control.
- **$\zeta = 1.0$** is ideal when you cannot tolerate any overshoot, wafer steppers, surgical robots.

High $\omega_n$ = fast system = requires more powerful actuators. Always check whether your hardware can support the bandwidth you're designing for.

# Section 11: Transfer Functions and BIBO - What You Need to Know

## Poles, zeros, and stability

The transfer function:

$$
H(s) = \frac{b_m s^m + \cdots + b_0}{s^n + a_{n-1}s^{n-1} + \cdots + a_0} = C(sI-A)^{-1}B + D
$$

- **Poles:** roots of the denominator. Determine stability and transient response.
- **Zeros:** roots of the numerator. Affect how quickly the output responds and whether it has undershoot (right-half-plane zeros).

**All poles of $H(s)$ are eigenvalues of $A$. Not all eigenvalues are poles**, pole-zero cancellations can hide modes.

## BIBO stability

A system is BIBO (Bounded-Input Bounded-Output) stable if bounded inputs always produce bounded outputs.

For LTI systems: BIBO stable $\Leftrightarrow$ all poles of $H(s)$ have negative real parts.

**Key practical warning:** BIBO stability is not the same as internal stability.

If your state space model has a pole-zero cancellation, an unstable eigenvalue that cancels out of the transfer function, the system looks BIBO stable but has an internally unstable mode. That mode will grow, it just won't appear in the output until it saturates or breaks something.

**This happens in practice** when you have an unobservable or uncontrollable mode. A non-minimal realization can be BIBO stable and internally unstable simultaneously.

**Rule:** Always verify stability at the state space level (eigenvalues of $A$), not just at the transfer function level (poles of $H(s)$). Use minimal realizations.

# Section 12: Workflow Summary: How to Actually Design a Controller

This is the sequence you follow in practice. Every section above is one step here.

## Step 1: Model

- Write the equations of motion or dynamics.
- Define state variables explicitly.
- Build $(A, B, C, D)$.
- If nonlinear: linearize around the operating point. Get Jacobians.
- For digital implementation: discretize with `c2d(A, B, Ts)`.

## Step 2: Analyze

- Check eigenvalues of $A$: is the open-loop plant stable? (`eig(A)`)
- Check controllability: `rank(ctrb(A,B))` must equal $n$.
- Check observability: `rank(obsv(A,C))` must equal $n$.
- If either fails, fix the system architecture, add actuators/sensors or redesign the model.

## Step 3: Design the controller

**Option A: Pole placement (small systems, explicit specs):**
- Convert %OS and $t_s$ to $\zeta$ and $\omega_n$.
- Compute desired poles.
- `K = place(A, B, desired_poles)`.
- Set `G = -inv(C*inv(A-B*K)*B)` for unit DC gain.

**Option B: LQR (larger systems, systematic tuning):**
- Set $Q$ and $R$ using Bryson's rule.
- `[K, P, e] = lqr(A, B, Q, R)`.
- Check `e` (closed-loop poles).
- Simulate and adjust $Q/R$ ratio.

## Step 4: Design the observer

- Decide: Luenberger (low noise) or Kalman filter (significant noise).
- For Luenberger: place observer poles 3–5× faster than controller poles.
  `L = place(A', C', observer_poles)'`
- For Kalman: specify process noise $Q_w$ and measurement noise $R_n$.
  `[L, P, e] = lqe(A, eye(n), C, Qw, Rn)` or `kalman()`.

## Step 5: Combine and verify

- Build the augmented observer-based controller.
- Verify closed-loop eigenvalues = eig$(A-BK)$ ∪ eig$(A-LC)$ (separation principle check).
- Simulate:
  - Step response: check %OS, $t_s$, steady-state value.
  - Disturbance rejection: inject a step disturbance, check recovery.
  - Noise sensitivity: inject measurement noise, check output quality.
  - Initial condition response: start from $\hat{x}(0) \neq x(0)$, check observer convergence.
- Check actuator saturation: if $u$ saturates in simulation, your gains are too aggressive.

## Step 6: Robustness check

- For LQR: gain and phase margins are guaranteed ($\geq$ 6dB, $\geq$ 60°).
- For pole placement: compute margins explicitly with `margin(H_ol)`.
- Test with $\pm 20\%$ model parameter variation, does the closed-loop remain stable?

# Section 13: Robotics and Industry Connections

## Robot arm joint control

Each joint is a second-order system: motor inertia $J$, damping $b$, driven by torque $\tau$:

$$
J\ddot{\theta} + b\dot{\theta} = \tau
$$

State: $x = [\theta, \dot{\theta}]^T$. Encoder reads $\theta$ only. Velocity $\dot{\theta}$ is observed.

Design: LQR or pole placement for the joint controller. Luenberger or Kalman for velocity estimation. The separation principle lets you do these independently.

In industrial manipulators: each joint runs an independent observer-based PID/state feedback loop. The coupling between joints is treated as a disturbance, handled by the disturbance rejection properties of the controller.

## Quadrotor attitude control

Attitude dynamics linearized around hover give three decoupled second-order systems (roll, pitch, yaw). Each is:

$$
\ddot{\phi} = \frac{1}{I_{xx}}\tau_\phi
$$

Three separate LQR controllers, one per axis. The full 6-DOF quadrotor is the nonlinear system; each flight controller is the linearized version around hover, updated fast enough that the linearization stays valid.

## Kalman filter in state estimation

The Luenberger observer is the deterministic version of the Kalman filter. The Kalman filter is an observer where $L$ is computed to minimize the estimation error variance given process noise covariance $Q_w$ and measurement noise covariance $R_n$:

The Kalman gain satisfies the same Riccati equation structure as LQR, by duality. LQR and Kalman filter are literally the same mathematical problem, just one is for control and one is for estimation. LQG combines both.

This connection matters: if you understand LQR tuning (how $Q$ and $R$ affect the trade-off), you understand Kalman filter tuning.

## Model Predictive Control (MPC): where this leads

MPC is LQR with a finite horizon and constraints:

$$
\min_{u_0, \ldots, u_{N-1}} \sum_{k=0}^{N-1} \left[x_k^T Q x_k + u_k^T R u_k\right] + x_N^T P_f x_N
$$

subject to $x_{k+1} = A_d x_k + B_d u_k$, actuator limits, state limits.

MPC is what runs in autonomous vehicles, industrial process control, and many legged robots. It requires exactly the discretized state space model from Section 1 and the LQR cost structure from Section 7. You now have everything you need to understand MPC at a conceptual level.

---

# Quick Reference

## Modeling

| Task | Formula / Command |
|---|---|
| Build state space from ODE | Choose $x$, write $\dot{x} = f(x,u)$, derive $A$, $B$, $C$, $D$ |
| Linearize at equilibrium | $A = \partial f/\partial x$, $B = \partial f/\partial u$ at $(x_{eq}, u_{eq})$ |
| Discretize | `[Ad, Bd] = c2d(A, B, Ts)` |
| Transfer function | $H(s) = C(sI-A)^{-1}B + D$ |

## Analysis

| Task | Formula / Command |
|---|---|
| Stability | `eig(A)` - all Re $< 0$? |
| Controllability | `rank(ctrb(A,B)) == n` |
| Observability | `rank(obsv(A,C)) == n` |
| Lyapunov check | `P = lyap(A', eye(n))`, is $P \succ 0$? |

## Controller Design

| Task | Formula / Command |
|---|---|
| %OS → $\zeta$ | $\zeta = -\ln(\%OS/100)/\sqrt{\pi^2 + \ln^2(\%OS/100)}$ |
| $t_s$ → $\omega_n$ | $\omega_n = 4/(\zeta t_s)$ |
| Desired poles | $s_{1,2} = -\zeta\omega_n \pm j\omega_n\sqrt{1-\zeta^2}$ |
| Pole placement | `K = place(A, B, poles)` |
| DC gain correction | `G = -inv(C * inv(A-B*K) * B)` |
| LQR | `[K, P, e] = lqr(A, B, Q, R)` |
| Bryson's rule | $Q_{ii} = 1/x_{i,max}^2$, $R_{jj} = 1/u_{j,max}^2$ |

## Observer Design

| Task | Formula / Command |
|---|---|
| Observer pole rule | Place 3–5× further left than controller poles |
| Luenberger | `L = place(A', C', obs_poles)'` |
| Kalman filter | `[L, P, e] = lqe(A, I, C, Qw, Rn)` |
| Closed-loop verification | `eig(A-L*C)`, all Re $< 0$? |

## Combined System Check

| Task | Check |
|---|---|
| Separation principle | eig(augmented) = eig$(A-BK)$ ∪ eig$(A-LC)$ |
| Steady-state | $y_{final} = H_{cl}(0) \cdot r$, set $G$ to make this 1 |
| Actuator limits | Simulate: is $\|u(t)\|$ within hardware limits? |

---