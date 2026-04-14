# **Advanced System Modeling, Dynamics and Control**

### **From First Principles to Expert Intuition**

**Focus:** durable understanding, not temporary exam memory.

---

# Why this course matters in robotics

If linear algebra is the operating system of robotics, then systems and control theory is its nervous system.

Every robot must answer three questions at every moment in time:

1. **What is my state right now?** (estimation, observability, observers)
2. **Where do I want to go?** (reference, trajectory)
3. **What do I do to get there?** (control law, stability, design)

This course builds the machinery to answer all three with precision.

You see it everywhere:

- **State-space models** are how you represent a robot arm, drone, or autonomous car as a set of equations.
- **Linearization** is how you turn a nonlinear pendulum or quadrotor into something you can analyze.
- **Controllability** tells you whether you can actually move the robot to any target state.
- **Observability** tells you whether you can infer what you cannot directly measure (like velocity when you only have position).
- **Pole placement and LQR** are how you make the closed-loop system fast, smooth, and stable.
- **Observer/Luenberger design** is the precursor to the Kalman filter — the backbone of robot state estimation.
- **Stability analysis** tells you whether your system will converge or explode.

This is not abstract theory. It is direct engineering infrastructure.

---

# How to use this document

## Mode 1: 5-minute refresh

Read:
- Big Picture Map
- Core Formula Sheet
- Robotics Connections

## Mode 2: 1-hour revision

Read module summaries in order:

1. System classification and modeling
2. Laplace transform and transfer functions
3. State space representation and solution
4. Coordinate transformation and canonical forms
5. Controllability and observability
6. Stability (internal and BIBO)
7. Control and observer design

## Mode 3: Deep rebuild

Read everything, including proofs, examples, and mental checks.

---

# Big Picture Map

This entire course answers five giant questions:

## 1. How do we describe a dynamical system mathematically?

> That is the world of **state space**, ODEs, Laplace transforms, and transfer functions.

## 2. Can the system be steered anywhere we want?

> That is the world of **controllability** and the controllability matrix $P$.

## 3. Can we infer the system's internal state from outputs alone?

> That is the world of **observability** and the observability matrix $Q$.

## 4. Will the system stay near an equilibrium or blow up?

> That is the world of **Lyapunov stability**, eigenvalue location, and Hurwitz conditions.

## 5. How do we design a controller that achieves the response we want?

> That is the world of **pole placement**, **LQR**, and **observer-based compensators**.

---

# Core Formula Sheet

## State Space

$$\dot{x} = Ax + Bu, \quad x \in \mathbb{R}^n, \; u \in \mathbb{R}^m$$
$$y = Cx + Du, \quad y \in \mathbb{R}^p$$

## State Equation Solution

$$x(t) = \underbrace{e^{At}x_0}_{\text{zero-input}} + \underbrace{\int_0^t e^{A(t-\tau)} Bu(\tau)\,d\tau}_{\text{zero-state}}$$

## Transfer Function

$$H(s) = C(sI - A)^{-1}B + D$$

## Controllability Matrix

$$P = \begin{bmatrix} B & AB & A^2B & \cdots & A^{n-1}B \end{bmatrix} \in \mathbb{R}^{n \times nm}$$

System controllable $\Leftrightarrow$ $\text{rank}(P) = n$

## Observability Matrix

$$
Q = \begin{bmatrix} C \\
 CA \\
 CA^2 \\
 \vdots \\
 CA^{n-1} \end{bmatrix} \in \mathbb{R}^{np \times n}
$$

System observable $\Leftrightarrow$ $\text{rank}(Q) = n$

## Asymptotic Stability

$$x_{\text{eq}} = 0 \text{ is asymptotically stable} \Leftrightarrow A \text{ is Hurwitz} \Leftrightarrow \text{Re}(\lambda_i) < 0 \; \forall i$$

## Closed-Loop System (State Feedback)

$$u = -Kx + Gr \implies \dot{x} = (A - BK)x + BGr$$

## Closed-Loop Poles

$$\text{eig}(A - BK) \quad \text{(designed via pole placement or LQR)}$$

## Luenberger Observer

$$\dot{\hat{x}} = A\hat{x} + Bu + L(y - \hat{y}), \quad \hat{y} = C\hat{x}$$

Error dynamics: $\dot{e} = (A - LC)e$

## LQR Cost

$$J = \int_0^\infty \left[ x^T Q x + u^T R u \right] dt, \quad Q \succeq 0, \; R \succ 0$$

Optimal gain: $u^* = -K^*x$, solved via algebraic Riccati equation.

---

# Module 1: System Classification and Modeling

## 1.1 What is a Dynamical System?

A **dynamical system** is a system in which a function describes the time dependence of a point in a geometrical space.

The state of the system encodes everything needed to predict future behavior.

Classic example — Lorenz system:

$$\dot{x}_1 = \sigma(x_2 - x_1), \quad \dot{x}_2 = x_1(\rho - x_3) - x_2, \quad \dot{x}_3 = x_1 x_2 - \beta x_3$$

An **input-output system** has quantifiable signals as inputs and outputs, with possible disturbances:

$$u(t) \longrightarrow \boxed{\text{system}} \longrightarrow y(t)$$

## 1.2 Open-Loop vs. Closed-Loop Control

**Open-loop:** the input does not depend on the output.

$$r \longrightarrow \boxed{\text{controller}} \longrightarrow u \longrightarrow \boxed{\text{plant}} \longrightarrow y$$

Limitations of open-loop: disturbances, model uncertainty, potential instability, slow response.

**Closed-loop (feedback) control:** the input is computed from the error between reference and measured output.

$$r \longrightarrow \oplus \longrightarrow \boxed{\text{controller}} \longrightarrow u \longrightarrow \boxed{\text{plant}} \longrightarrow y \longrightarrow \text{(sensor)} \longrightarrow$$

The error $e(t) = r(t) - y(t)$ drives the controller.

Closed-loop is everywhere: thermostats, automotive cruise control, human motor control, medical dosing systems.

## 1.3 System Classifications

This course focuses exclusively on **LTI, lumped parameter, deterministic, causal systems** — both continuous- and discrete-time.

| Classification | Two Options |
|---|---|
| Parameter type | Lumped (ODEs) vs. Distributed (PDEs) |
| Uncertainty | Deterministic vs. Stochastic |
| Time domain | Continuous-time vs. Discrete-time |
| Linearity | Linear vs. Nonlinear |
| Time dependence | Time-invariant vs. Time-variant |
| Forcing | Homogeneous (no input) vs. Non-homogeneous |
| Causality | Causal vs. Non-causal |

### Linear Systems — Superposition Principle

A system is linear if and only if it satisfies superposition:

$$u_1(t) \to y_1(t), \quad u_2(t) \to y_2(t)$$
$$\implies \alpha u_1(t) + \beta u_2(t) \to \alpha y_1(t) + \beta y_2(t)$$

### Time-Invariant Systems

If $u(t) \to y(t)$, then $u(t - T) \to y(t - T)$ for any delay $T > 0$.

The system parameters do not change with time.

### Causal Systems

Current output depends only on past and current inputs:

$$y(k+1) = u(k) + u(k-1) \quad \checkmark \text{ causal}$$
$$y(k+1) = u(k) + u(k+2) \quad \times \text{ non-causal (depends on future)}$$

## 1.4 Modeling Mechanical Systems

### Common Elements

**Spring (potential energy storage):**

Translational: $F_s = ky$ (Hooke's Law), $k > 0$ is spring constant.

Rotational (torsion spring): $T_s = k\theta$.

Note: the positive direction of the force must be defined consistently. If you flip the positive direction, you flip the sign of the formula. The physical direction does not change.

**Damper (energy dissipator):**

Translational: $F_d = c\dot{y}$, $c \geq 0$ is damping coefficient.

Rotational: $T_D = D\dot{\theta}$.

**Mass/Inertia (kinetic energy storage):**

Translational: $F = m\ddot{y}$ (Newton's 2nd law).

Rotational: $T = J\ddot{\theta}$ (rotational Newton's 2nd law).

### Three-Step Derivation

1. **Assign variables** — define displacement, angle, and their positive directions.
2. **Draw free body diagram** — identify all forces acting on each mass.
3. **Apply Newton's law** — sum of forces = mass times acceleration.

### Standard Spring-Mass-Damper Example

$$m\ddot{y} + c\dot{y} + ky = F$$

With input $u = F$ and output $y$, this is a linear time-invariant system.

Setting $x_1 = y$, $x_2 = \dot{y}$:

$$
\dot{x} = \begin{bmatrix} 0 & 1 \\
 -k/m & -c/m \end{bmatrix} x + \begin{bmatrix} 0 \\
 1/m \end{bmatrix} u, \qquad y = \begin{bmatrix} 1 & 0 \end{bmatrix} x
$$

### Multi-body Mechanical System

For a two-mass system with spring $k_1$, $k_2$ and damper $c$:

$$m_1 \ddot{y}_1 = f + k_2(y_2 - y_1) - k_1 y_1$$
$$m_2 \ddot{y}_2 = -c\dot{y}_2 - k_2(y_2 - y_1)$$

Choose state $x = [y_1, y_2, \dot{y}_1, \dot{y}_2]^T$ and write in state space form.

The key insight: **each second-order equation needs two state variables** (position and velocity).

---

# Module 2: Laplace Transform and Transfer Functions

## 2.1 Laplace Transform

The Laplace transform converts a time-domain signal into the frequency domain:

$$\mathcal{L}[y(t)] = \int_0^\infty y(t)\, e^{-st}\, dt = Y(s), \qquad s = \sigma + j\omega$$

The inverse:

$$y(t) = \mathcal{L}^{-1}[Y(s)]$$

The transform exists only when $s$ is in the **region of convergence (ROC)** — values of $s$ for which the integral converges.

### Unit Step Function

$$
u_s(t) = \begin{cases} 1 & t \geq 0 \\
 0 & t < 0 \end{cases}
$$

$$\mathcal{L}[u_s(t)] = \frac{1}{s}, \qquad \text{Re}(s) > 0$$

### Key Laplace Transform Pairs

| $y(t)$ | $Y(s)$ |
|---|---|
| $\delta(t)$ (impulse) | $1$ |
| $u_s(t)$ (unit step) | $1/s$ |
| $e^{at}$ | $1/(s-a)$ |
| $t \cdot e^{at}$ | $1/(s-a)^2$ |
| $\sin(\omega t)$ | $\omega/(s^2 + \omega^2)$ |
| $\cos(\omega t)$ | $s/(s^2 + \omega^2)$ |
| $e^{at}\sin(\omega t)$ | $\omega/((s-a)^2 + \omega^2)$ |
| $e^{at}\cos(\omega t)$ | $(s-a)/((s-a)^2 + \omega^2)$ |

### Key Properties

**Time derivative:**

$$\mathcal{L}[\dot{y}(t)] = sY(s) - y(0)$$
$$\mathcal{L}[\ddot{y}(t)] = s^2 Y(s) - sy(0) - \dot{y}(0)$$

**Time shift:** $\mathcal{L}[y(t - T)] = e^{-sT} Y(s)$

**Convolution:** $\mathcal{L}[f(t) * g(t)] = F(s) \cdot G(s)$

**Final value theorem:** If $\lim_{t \to \infty} y(t)$ exists,

$$\lim_{t \to \infty} y(t) = \lim_{s \to 0} s Y(s)$$

**Initial value theorem:**

$$y(0^+) = \lim_{s \to \infty} s Y(s)$$

### Using Laplace to Solve ODEs

Example: $\ddot{y} + \dot{y} + y = u_s(t)$, with $y(0) = \dot{y}(0) = 0$

$$s^2 Y(s) + s Y(s) + Y(s) = \frac{1}{s}$$

$$Y(s) = \frac{1}{s(s^2 + s + 1)}$$

Then use **partial fraction expansion** to invert:

$$\frac{A}{s} + \frac{Bs + C}{s^2 + s + 1}$$

And look up each term in the Laplace table.

## 2.2 Transfer Functions

For a linear time-invariant SISO system with zero initial conditions:

$$a_n y^{(n)} + \cdots + a_1 \dot{y} + a_0 y = b_m u^{(m)} + \cdots + b_1 \dot{u} + b_0 u$$

Taking the Laplace transform with zero initial conditions:

$$H(s) = \frac{Y(s)}{U(s)} = \frac{b_m s^m + \cdots + b_1 s + b_0}{a_n s^n + \cdots + a_1 s + a_0}$$

**Proper TF:** $n \geq m$ (causal).

**Strictly proper TF:** $n > m$.

**Poles:** roots of the denominator.

**Zeros:** roots of the numerator.

### From State Space to Transfer Function

$$H(s) = C(sI - A)^{-1}B + D$$

This is the exact formula. For a $2 \times 2$ system:

$$(sI - A)^{-1} = \frac{1}{\det(sI - A)} \text{adj}(sI - A)$$

The denominator $\det(sI - A)$ is the **characteristic polynomial**, and its roots are the eigenvalues of $A$.

> **Key fact:** All poles of $H(s)$ are eigenvalues of $A$, but not all eigenvalues of $A$ are necessarily poles of $H(s)$. Pole-zero cancellation can occur.

### Partial Fraction Expansion

Used to invert $Y(s)$ back to $y(t)$:

$$\frac{s+3}{(s+1)(s+2)} = \frac{a}{s+1} + \frac{b}{s+2}$$

Solve: $a(s+2) + b(s+1) = s+3$, giving $a = -1$, $b = 2$.

$$y(t) = \mathcal{L}^{-1}[Y(s)] = -e^{-t} + 2e^{-2t}$$

---

# Module 3: State Space Representation

## 3.1 State Variables

The **state** is a minimal set of variables that, together with the future input, completely determines the future behavior of the system.

For an $n$th-order system: $n$ state variables are needed.

**State equation (continuous-time):**

$$\dot{x} = Ax + Bu$$

**Output equation:**

$$y = Cx + Du$$

Where:
- $x \in \mathbb{R}^n$ — state vector
- $u \in \mathbb{R}^m$ — input vector
- $y \in \mathbb{R}^p$ — output vector
- $A \in \mathbb{R}^{n \times n}$ — system matrix
- $B \in \mathbb{R}^{n \times m}$ — input matrix
- $C \in \mathbb{R}^{p \times n}$ — output matrix
- $D \in \mathbb{R}^{p \times m}$ — feedforward matrix

**Discrete-time version:**

$$x(k+1) = Ax(k) + Bu(k), \qquad y(k) = Cx(k) + Du(k)$$

## 3.2 Converting ODE to State Space

### Method 1: Direct (when only $y$ appears on the right-hand side)

Given $\dddot{y} + a_2 \ddot{y} + a_1 \dot{y} + a_0 y = b_0 u$:

Choose $x = [y, \dot{y}, \ddot{y}]^T$:

$$
\dot{x} = \begin{bmatrix} 0 & 1 & 0 \\
 0 & 0 & 1 \\
 -a_0 & -a_1 & -a_2 \end{bmatrix} x + \begin{bmatrix} 0 \\
 0 \\
 b_0 \end{bmatrix} u
$$

### Method 2: Via transfer function (when derivatives of $u$ appear)

Given $\dddot{y} + a_2 \ddot{y} + a_1 \dot{y} + a_0 y = b_2 \ddot{u} + b_1 \dot{u} + b_0 u$:

1. Compute $H(s) = \frac{b_2 s^2 + b_1 s + b_0}{s^3 + a_2 s^2 + a_1 s + a_0}$

2. Introduce intermediate variable $W(s)$: $H(s) = H_1(s) \cdot H_2(s)$, where $H_1(s) = \frac{1}{s^3 + a_2 s^2 + a_1 s + a_0}$ and $H_2(s) = b_2 s^2 + b_1 s + b_0$.

3. $H_1(s)$ gives the state vector $x = [w, \dot{w}, \ddot{w}]^T$ from $\dddot{w} + a_2 \ddot{w} + a_1 \dot{w} + a_0 w = u$.

4. $H_2(s)$ gives the output equation $y = b_2 \ddot{w} + b_1 \dot{w} + b_0 w = b_2 x_3 + b_1 x_2 + b_0 x_1$.

$$
\dot{x} = \begin{bmatrix} 0 & 1 & 0 \\
 0 & 0 & 1 \\
 -a_0 & -a_1 & -a_2 \end{bmatrix} x + \begin{bmatrix} 0 \\
 0 \\
 1 \end{bmatrix} u, \qquad y = \begin{bmatrix} b_0 & b_1 & b_2 \end{bmatrix} x
$$

This is exactly the **Controller Canonical Form** (covered in Module 4).

## 3.3 State Equation Solution

### Homogeneous Case (no input, $u = 0$)

Scalar analogy: $\dot{x} = ax \implies x(t) = e^{at} x_0$.

Matrix case: $\dot{x} = Ax \implies x(t) = e^{At} x_0$.

### Matrix Exponential

$$e^{At} = I + At + \frac{(At)^2}{2!} + \frac{(At)^3}{3!} + \cdots$$

**Key properties:**

| Property | Formula |
|---|---|
| At $t=0$ | $e^{A \cdot 0} = I$ |
| Composition | $e^{A(t_1 + t_2)} = e^{At_1} e^{At_2}$ |
| Inverse | $(e^{At})^{-1} = e^{-At}$ |
| Derivative | $\frac{d}{dt} e^{At} = A e^{At}$ |
| Commuting | $e^{(A+B)t} = e^{At} e^{Bt}$ **only if** $AB = BA$ |
| Eigenvalue relation | $A e^{At} = e^{At} A$ |

**Computing $e^{At}$:**

$$e^{At} = \mathcal{L}^{-1}\left[ (sI - A)^{-1} \right]$$

For a diagonal matrix $A = \text{diag}(\lambda_1, \ldots, \lambda_n)$:

$$e^{At} = \text{diag}(e^{\lambda_1 t}, \ldots, e^{\lambda_n t})$$

### Full Solution (Non-Homogeneous)

$$x(t) = \underbrace{e^{At} x_0}_{\text{zero-input (ZI)}} + \underbrace{\int_0^t e^{A(t-\tau)} Bu(\tau)\, d\tau}_{\text{zero-state (ZS)}}$$

$$y(t) = \underbrace{Ce^{At} x_0}_{y_{zi}(t)} + \underbrace{\int_0^t C e^{A(t-\tau)} Bu(\tau)\,d\tau + Du(t)}_{y_{zs}(t)}$$

### Laplace Domain Solution

$$X(s) = (sI - A)^{-1} x_0 + (sI - A)^{-1} B U(s)$$
$$Y(s) = C(sI - A)^{-1} x_0 + \left[C(sI - A)^{-1}B + D\right] U(s)$$

The second bracket is exactly $H(s)$.

### Example

$$
\dot{x} = \begin{bmatrix} 0 & 1 \\
 -2 & -3 \end{bmatrix} x + \begin{bmatrix} 0 \\
 1 \end{bmatrix} u, \quad x(0) = \begin{bmatrix} -1 \\
 1 \end{bmatrix}, \quad y = \begin{bmatrix} 1 & 1 \end{bmatrix} x
$$

With unit step input $U(s) = 1/s$:

$$Y_{zi}(s) = C(sI - A)^{-1} x_0 = 0 \text{ (for this initial condition)}$$

$$Y_{zs}(s) = H(s) \cdot \frac{1}{s} = \frac{s+1}{s(s^2+3s+2)} = \frac{1}{s(s+2)} = \frac{0.5}{s} - \frac{0.5}{s+2}$$

$$y(t) = 0.5 - 0.5 e^{-2t}$$

---

# Module 4: Linearization of Nonlinear Systems

## 4.1 Nonlinear State Space

A general nonlinear, time-invariant system:

$$\dot{x}(t) = f(x(t), u(t)), \qquad y(t) = h(x(t), u(t))$$

where $f$ and $h$ are vector-valued nonlinear functions.

### Pendulum Example

Let $x = [\theta, \dot{\theta}]^T$, input $u = T$ (external torque):

$$
\dot{x} = \begin{bmatrix} x_2 \\
 -\frac{g}{l}\sin(x_1) + \frac{1}{ml^2} u \end{bmatrix} = f(x, u)
$$

$$y = x_1 = \theta$$

## 4.2 Linearization around a Nominal Trajectory

**Nominal trajectory:** $\tilde{x}(t)$, $\tilde{u}(t)$, $\tilde{y}(t)$ — any reference path.

**Deviation variables:**

$$x_\delta(t) = x(t) - \tilde{x}(t), \quad u_\delta(t) = u(t) - \tilde{u}(t), \quad y_\delta(t) = y(t) - \tilde{y}(t)$$

**Taylor expansion** of $f$ around $\tilde{x}(t)$, $\tilde{u}(t)$:

$$\dot{x}_\delta = A(t) x_\delta + B(t) u_\delta$$
$$y_\delta = C(t) x_\delta + D(t) u_\delta$$

where the **Jacobian matrices** are:

$$A(t) = \frac{\partial f}{\partial x}\bigg|_{\tilde{x}(t),\tilde{u}(t)}, \quad B(t) = \frac{\partial f}{\partial u}\bigg|_{\tilde{x}(t),\tilde{u}(t)}$$

$$C(t) = \frac{\partial h}{\partial x}\bigg|_{\tilde{x}(t),\tilde{u}(t)}, \quad D(t) = \frac{\partial h}{\partial u}\bigg|_{\tilde{x}(t),\tilde{u}(t)}$$

This is a **linear time-variant (LTV)** system.

## 4.3 Linearization around an Equilibrium (LTI Special Case)

If the nominal trajectory is constant — an **equilibrium state** $x_{eq}$ where $f(x_{eq}, u_{eq}) = 0$ — then $A(t)$, $B(t)$, $C(t)$, $D(t)$ are all constant matrices.

The result is a **linear time-invariant (LTI)** system:

$$\dot{x}_\delta = A x_\delta + B u_\delta, \qquad y_\delta = C x_\delta + D u_\delta$$

### Finding Equilibrium States

Set $\dot{x} = f(x_{eq}, u_{eq}) = 0$ and solve for $x_{eq}$.

**Pendulum example** with $u_{eq} = 0$:

$$
\begin{bmatrix} x_{2,eq} \\
 -\frac{g}{l}\sin(x_{1,eq}) \end{bmatrix} = \begin{bmatrix} 0 \\
 0 \end{bmatrix}
$$

$$\implies x_{2,eq} = 0, \quad \sin(x_{1,eq}) = 0 \implies x_{1,eq} = n\pi$$

Two equilibria: $x_{eq} = [0, 0]^T$ (pendulum down, stable) and $x_{eq} = [\pi, 0]^T$ (pendulum up, unstable).

### Pendulum Linearized at $x_{eq} = [0, 0]^T$

$$
A = \begin{bmatrix} 0 & 1 \\
 -g/l & 0 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\
 1/(ml^2) \end{bmatrix}
$$

### Nonlinear Spring-Mass-Damper Example

System: $m\ddot{y} = F - ky^3 - c\dot{y}$, with $m = c = k = 1$.

States: $x_1 = y$, $x_2 = \dot{y}$. Equilibrium at $\tilde{x} = [1, 0]^T$, $\tilde{u} = 1$:

$$
A = \begin{bmatrix} 0 & 1 \\
 -3x_1^2 & -c/m \end{bmatrix}\bigg|_{x_1=1, x_2=0} = \begin{bmatrix} 0 & 1 \\
 -3 & -3 \end{bmatrix}
$$

$$
B = \begin{bmatrix} 0 \\
 1/m \end{bmatrix}\bigg|_{\tilde{x}, \tilde{u}} = \begin{bmatrix} 0 \\
 1 \end{bmatrix}
$$

> **Robotics connection:** Linearization around an operating point is how you apply linear control theory (pole placement, LQR) to fundamentally nonlinear robotic systems. The Jacobian $A$ is literally the Jacobian matrix from linear algebra — the same one that appears in robot Jacobians and numerical methods.

---

# Module 5: Coordinate Transformation and Canonical Forms

## 5.1 Coordinate Transformation

Given a state space representation with state $x$, apply the invertible (non-singular) transformation $x = Tz$:

$$\dot{z} = \hat{A}z + \hat{B}u, \qquad y = \hat{C}z + \hat{D}u$$

where:

$$\hat{A} = T^{-1}AT, \quad \hat{B} = T^{-1}B, \quad \hat{C} = CT, \quad \hat{D} = D$$

This is a **similarity transformation**.

### What is preserved under coordinate transformation?

| Property | Preserved? |
|---|---|
| Eigenvalues of $A$ | ✓ Yes |
| Transfer function $H(s)$ | ✓ Yes |
| Controllability (rank of $P$) | ✓ Yes |
| Observability (rank of $Q$) | ✓ Yes |
| Poles | ✓ Yes |

**Proof (eigenvalues):** $|\lambda I - \hat{A}| = |\lambda I - T^{-1}AT| = |T^{-1}(\lambda I - A)T| = |T^{-1}||\lambda I - A||T| = |\lambda I - A|$

So the characteristic polynomial is unchanged.

**Why transform at all?** Because some canonical forms are easier to analyze, decompose, or use for design. You choose the form that makes your task simplest.

## 5.2 Diagonal Canonical Form (DCF)

If $A$ is **diagonalizable**, it can be transformed into:

$$\hat{A}_{DCF} = \text{diag}(\lambda_1, \lambda_2, \ldots, \lambda_n)$$

**When is $A$ diagonalizable?**

$A$ is diagonalizable if and only if **algebraic multiplicity (A.M.) = geometric multiplicity (G.M.)** for every eigenvalue.

- **A.M.** of $\lambda$ = number of times $\lambda$ appears as a root of the characteristic polynomial.
- **G.M.** of $\lambda$ = dimension of the null space of $(\lambda I - A)$ = number of linearly independent eigenvectors.

If all $n$ eigenvalues are **distinct**, then $A$ is always diagonalizable (each eigenvalue has A.M. = G.M. = 1).

**Jordan form** (non-diagonalizable case): When A.M. > G.M. for some eigenvalue, the best we can do is the Jordan normal form — block-diagonal with Jordan blocks along the diagonal.

### DCF Transformation Steps

1. Compute eigenvalues $\lambda_1, \ldots, \lambda_n$ of $A$.
2. Compute eigenvectors $v_1, \ldots, v_n$ of $A$.
3. Construct $T_{DCF} = [v_1 \; v_2 \; \cdots \; v_n]$.
4. Then:

$$
\hat{A}_{DCF} = T_{DCF}^{-1} A T_{DCF} = \begin{bmatrix} \lambda_1 & & \\
 & \ddots & \\
 & & \lambda_n \end{bmatrix}
$$

$$\hat{B}_{DCF} = T_{DCF}^{-1} B, \quad \hat{C}_{DCF} = C T_{DCF}, \quad \hat{D}_{DCF} = D$$

### Why DCF is useful

In DCF, the state equation decouples into $n$ independent scalar equations:

$$\dot{z}_i = \lambda_i z_i + \hat{b}_i u \quad \text{for each } i$$

Each is solvable independently. This makes computing $e^{\hat{A}t}$ trivial:

$$e^{\hat{A}_{DCF} t} = \text{diag}(e^{\lambda_1 t}, \ldots, e^{\lambda_n t})$$

### Example: DCF

$$
A = \begin{bmatrix} 0 & 1 \\
 -2 & -3 \end{bmatrix}, \quad B = \begin{bmatrix} 0 \\
 1 \end{bmatrix}, \quad C = \begin{bmatrix} 1 & 1 \end{bmatrix}
$$

Eigenvalues: $\lambda_1 = -1$, $\lambda_2 = -2$.

Eigenvectors: $v_1 = [1, -1]^T$, $v_2 = [-1, 2]^T$.

$$
T_{DCF} = \begin{bmatrix} 1 & -1 \\
 -1 & 2 \end{bmatrix}, \quad T_{DCF}^{-1} = \begin{bmatrix} 2 & 1 \\
 1 & 1 \end{bmatrix}
$$

$$
\hat{A}_{DCF} = \begin{bmatrix} -1 & 0 \\
 0 & -2 \end{bmatrix}, \quad \hat{B}_{DCF} = \begin{bmatrix} 1 \\
 1 \end{bmatrix}, \quad \hat{C}_{DCF} = \begin{bmatrix} 0 & 1 \end{bmatrix}
$$

## 5.3 Controller Canonical Form (CCF)

For a SISO system with transfer function:

$$H(s) = \frac{b_{n-1}s^{n-1} + \cdots + b_1 s + b_0}{s^n + a_{n-1}s^{n-1} + \cdots + a_1 s + a_0}$$

The CCF state matrices are:

$$
A_{CCF} = \begin{bmatrix} 0 & 1 & 0 & \cdots & 0 \\
 0 & 0 & 1 & \cdots & 0 \\
 \vdots & & & \ddots & \vdots \\
 0 & 0 & 0 & \cdots & 1 \\
 -a_0 & -a_1 & -a_2 & \cdots & -a_{n-1} \end{bmatrix}, \quad B_{CCF} = \begin{bmatrix} 0 \\
 0 \\
 \vdots \\
 0 \\
 1 \end{bmatrix}
$$

$$C_{CCF} = \begin{bmatrix} b_0 & b_1 & \cdots & b_{n-1} \end{bmatrix}, \quad D_{CCF} = 0$$

**A system in CCF is always controllable** (by construction — the controllability matrix $P_{CCF}$ has determinant $\pm 1$).

### Converting to CCF (4 Steps)

1. Compute $P = [B \; AB \; \cdots \; A^{n-1}B]$ and verify controllability ($\text{rank}(P) = n$).
2. Compute characteristic polynomial: $\det(sI - A) = s^n + a_{n-1}s^{n-1} + \cdots + a_0$.  
   Compute $P_{CCF}^{-1}$ (see formula below).
3. Compute transformation: $T_{CCF} = P \cdot P_{CCF}^{-1}$.
4. Write CCF directly from the $a_i$ and $b_i$ coefficients.

The inverse of the CCF controllability matrix is:

$$
P_{CCF}^{-1} = \begin{bmatrix} a_1 & a_2 & \cdots & a_{n-1} & 1 \\
 a_2 & a_3 & \cdots & 1 & 0 \\
 \vdots & & \ddots & & \vdots \\
 a_{n-1} & 1 & \cdots & 0 & 0 \\
 1 & 0 & \cdots & 0 & 0 \end{bmatrix}
$$

### CCF Example

$$
\dot{x} = \begin{bmatrix} -1 & -2 \\
 0 & -1 \end{bmatrix} x + \begin{bmatrix} 1 \\
 1 \end{bmatrix} u, \quad y = \begin{bmatrix} 1 & 0 \end{bmatrix} x
$$

1. $P = [B \; AB] = \begin{bmatrix} 1 & -3 \\ 1 & -1 \end{bmatrix}$, $|P| = 2 \neq 0$ → controllable.

2. $\det(sI - A) = s^2 + 2s + 1 \implies a_0 = 1$, $a_1 = 2$.  
   $P_{CCF}^{-1} = \begin{bmatrix} 2 & 1 \\ 1 & 0 \end{bmatrix}$

3. $T_{CCF} = P \cdot P_{CCF}^{-1} = \begin{bmatrix} 1 & -3 \\ 1 & -1 \end{bmatrix}\begin{bmatrix} 2 & 1 \\ 1 & 0 \end{bmatrix} = \begin{bmatrix} -1 & 1 \\ 1 & 1 \end{bmatrix}$

4. CCF: $\dot{z} = \begin{bmatrix} 0 & 1 \\ -1 & -2 \end{bmatrix} z + \begin{bmatrix} 0 \\ 1 \end{bmatrix} u$, $\quad y = \begin{bmatrix} 1 & 0 \end{bmatrix} z$

## 5.4 Observer Canonical Form (OCF)

The OCF is the dual of the CCF. For the same $H(s)$:

$$
A_{OCF} = \begin{bmatrix} 0 & 0 & \cdots & 0 & -a_0 \\
 1 & 0 & \cdots & 0 & -a_1 \\
 0 & 1 & \cdots & 0 & -a_2 \\
 \vdots & & \ddots & \vdots & \vdots \\
 0 & 0 & \cdots & 1 & -a_{n-1} \end{bmatrix} = A_{CCF}^T
$$

$$
B_{OCF} = \begin{bmatrix} b_0 \\
 b_1 \\
 \vdots \\
 b_{n-1} \end{bmatrix} = C_{CCF}^T, \quad C_{OCF} = \begin{bmatrix} 0 & 0 & \cdots & 0 & 1 \end{bmatrix} = B_{CCF}^T, \quad D_{OCF} = 0
$$

**A system in OCF is always observable** (by construction).

### Converting to OCF (4 Steps)

1. Compute $Q$ and verify observability ($\text{rank}(Q) = n$).
2. Compute characteristic polynomial and $Q_{OCF}^{-1}$.
3. $T_{OCF} = Q_{OCF}^{-1} \cdot Q$.
4. Write OCF.

Note: $Q_{OCF}^{-1} = (P_{CCF}^{-1})^T$.

## 5.5 Duality

The original system $\{A, B, C, D\}$ has a **dual system** $\{A^T, C^T, B^T, D^T\}$.

Key duality relationships:

| Original system | Dual system |
|---|---|
| Controllability matrix $P = [B, AB, \ldots]$ | Observability matrix $Q^T$ of dual |
| Observability matrix $Q = [C^T, A^T C^T, \ldots]^T$ | Controllability matrix $P^T$ of dual |
| System is controllable | Dual system is observable |
| System is observable | Dual system is controllable |
| Transfer function $H(s)$ | Dual transfer function $H_d(s) = H(s)^T$ |

Duality lets you solve observer design problems using controller design tools and vice versa.

---

# Module 6: Controllability and Observability

## 6.1 Reachability and Controllability

### Definitions

**Reachability:** Can the system be steered from the origin to any state $x_f$ in finite time?

**Controllability:** Can the system be steered from any state $x_0$ to the origin in finite time?

For **continuous-time LTI systems**, reachability $\Leftrightarrow$ controllability (they are equivalent regardless of rank of $A$).

For **discrete-time LTI systems**, reachability $\Rightarrow$ controllability, but controllability $\not\Rightarrow$ reachability in general. If $A$ is full rank, they are equivalent.

### The Controllability Matrix

$$P = \begin{bmatrix} B & AB & A^2B & \cdots & A^{n-1}B \end{bmatrix} \in \mathbb{R}^{n \times nm}$$

$$\text{System is controllable} \Leftrightarrow \text{rank}(P) = n$$

MATLAB: `ctrb(A, B)`.

**Intuition:** The image of $P$ is exactly the reachable space — the set of all states reachable from the origin.

### Controllability Gramian

$$W(t_0, t_f) = \int_{t_0}^{t_f} e^{A(t_0 - \tau)} B B^T e^{A^T(t_0 - \tau)}\, d\tau$$

The system is controllable if and only if $W(t_0, t_f)$ is non-singular for any finite $t_f > t_0$.

### Minimum Energy Steering Input

Given a controllable system, the input that steers from $x(t_0) = x_0$ to $x(t_f) = x_f$ with minimum control energy is:

$$u(t) = B^T e^{A^T(t_0 - t)} W^{-1}(t_0, t_f) \left(e^{A(t_0 - t_f)} x_f - x_0\right)$$

The energy (control effort) is:

$$\int_{t_0}^{t_f} u^T(t) u(t)\, dt$$

## 6.2 Canonical Decomposition for Uncontrollable Systems

If $\text{rank}(P) = q < n$, the system has an uncontrollable part.

**Standard form:**

$$
A = \begin{bmatrix} A_{11} & A_{12} \\
 0 & A_{22} \end{bmatrix}, \quad B = \begin{bmatrix} B_1 \\
 0 \end{bmatrix}
$$

where $(A_{11}, B_1)$ is the **controllable $q$-dimensional subsystem**.

Input $u$ cannot influence $x_2$ (the uncontrollable states) — directly or indirectly.

### Decomposition Procedure

1. Compute $P$, find $q = \text{rank}(P) < n$.
2. Construct $T = [t_1, \ldots, t_q \;|\; t_{q+1}, \ldots, t_n]$ where the first $q$ columns are linearly independent columns of $P$, and the remaining $n-q$ columns are chosen to make $T$ non-singular.
3. Compute $\hat{A} = T^{-1}AT$, $\hat{B} = T^{-1}B$, $\hat{C} = CT$.

## 6.3 Observability

### Definition

**Observability:** Can the initial state $x(t_0)$ be uniquely determined from measurements of $u(t)$ and $y(t)$ over a finite time interval?

If yes, the system is observable.

### The Observability Matrix

$$
Q = \begin{bmatrix} C \\
 CA \\
 CA^2 \\
 \vdots \\
 CA^{n-1} \end{bmatrix} \in \mathbb{R}^{np \times n}
$$

$$\text{System is observable} \Leftrightarrow \text{rank}(Q) = n$$

MATLAB: `obsv(A, C)`.

The **unobservable space** is $\ker(Q)$ — initial states that produce zero output (indistinguishable from origin).

### Observability Gramian

$$M(t_0, t_f) = \int_{t_0}^{t_f} e^{A^T(\tau - t_0)} C^T C\, e^{A(\tau - t_0)}\, d\tau$$

The system is observable if and only if $M(t_0, t_f)$ is non-singular for any finite $t_f > t_0$.

### Recovering Initial State

For an observable system:

$$x_0 = M^{-1}(t_0, t_f) \int_{t_0}^{t_f} e^{A^T(\tau - t_0)} C^T\, y_{zi}(\tau)\, d\tau$$

where $y_{zi}(t) = y(t) - y_{zs}(t)$ is the zero-input output (obtained by subtracting the known forced response).

## 6.4 Canonical Decomposition for Unobservable Systems

If $\text{rank}(Q) = q < n$, the system has an unobservable part.

**Standard form:**

$$
\hat{A} = \begin{bmatrix} \hat{A}_{11} & 0 \\
 \hat{A}_{21} & \hat{A}_{22} \end{bmatrix}, \quad \hat{C} = \begin{bmatrix} \hat{C}_1 & 0 \end{bmatrix}
$$

where $(\hat{A}_{11}, \hat{C}_1)$ is the **observable $q$-dimensional subsystem**.

### Decomposition Procedure (via Duality)

1. Compute $Q$, find $q = \text{rank}(Q) < n$.
2. Construct $T^{-1}$ using the rows of $Q$ (analogous to using columns of $P$ for controllability).
   - First $q$ rows of $T^{-1}$: linearly independent rows of $Q$.
   - Remaining $n - q$ rows: chosen to make $T^{-1}$ non-singular.
3. Alternatively: apply the controllability decomposition procedure to the dual system $(A^T, C^T)$, then transpose back.

## 6.5 Minimality

A **minimal realization** of $H(s)$ is a state space form of the least possible dimension.

For SISO systems:

- $H(s)$ is **irreducible** if it has no pole-zero cancellations (numerator and denominator share no common factors).
- An $n$-dimensional realization is minimal if and only if $H(s)$ is irreducible.
- **A realization is minimal if and only if it is both controllable and observable.**

**Implication:** Pole-zero cancellations hide uncontrollable or unobservable modes. A non-minimal realization may appear stable (BIBO stable) while having internally unstable hidden modes — dangerous in practice.

### Example of Non-minimal Realization

$$H(s) = \frac{s+2}{(s+1)(s+2)} = \frac{1}{s+1}$$

The 2-dimensional CCF realization is unobservable, and the OCF realization is uncontrollable. The minimal realization is 1-dimensional: $\dot{x} = -x + u$, $y = x$.

---

# Module 7: Internal Stability

## 7.1 Definitions (Lyapunov Stability)

For an autonomous system $\dot{x} = f(x)$ with equilibrium at the origin $x_{eq} = 0$:

**Lyapunov Stable:** For any $\varepsilon > 0$, there exists $\delta > 0$ such that $\|x(0)\| < \delta \implies \|x(t)\| < \varepsilon$ for all $t \geq 0$.

States that start "close enough" stay "close enough" forever.

**Asymptotically Stable:** Stable, and there exists $\delta_0 > 0$ such that $\|x(0)\| < \delta_0 \implies \lim_{t \to \infty} x(t) = 0$.

**Globally Asymptotically Stable:** Asymptotically stable for any initial condition (no restriction on $\|x(0)\|$).

**Marginally Stable:** Stable but not asymptotically stable.

**Unstable:** Not stable.

The stability hierarchy: globally asymptotically stable $\subset$ asymptotically stable $\subset$ marginally stable $\subset$ stable $\subset$ unstable.

## 7.2 LTI Stability Conditions

For an LTI system $\dot{x} = Ax$, the solution is $x(t) = e^{At} x(0)$.

The entries of $e^{At}$ contain terms of the form $t^k e^{\lambda t}$ where $\lambda$ are eigenvalues of $A$.

### Asymptotic Stability (Hurwitz Condition)

$$x_{eq} = 0 \text{ is asymptotically stable} \Leftrightarrow A \text{ is Hurwitz} \Leftrightarrow \text{Re}(\lambda_i) < 0 \; \forall i$$

For LTI systems, asymptotic stability = global asymptotic stability.

### Marginal Stability

$x_{eq} = 0$ is stable (marginally) if and only if every eigenvalue $\lambda$ of $A$ satisfies:

- $\text{Re}(\lambda) < 0$, OR
- $\text{Re}(\lambda) = 0$ **and** $\lambda$ has A.M. = G.M.

When $\text{Re}(\lambda) = 0$ but A.M. > G.M. → **unstable** (polynomial growth $t^k e^0 = t^k \to \infty$).

### Summary Table

| Eigenvalue location | A.M. vs G.M. | Stability |
|---|---|---|
| $\text{Re}(\lambda) < 0$ | any | Asymptotically stable |
| $\text{Re}(\lambda) = 0$ | A.M. = G.M. | Marginally stable |
| $\text{Re}(\lambda) = 0$ | A.M. > G.M. | Unstable |
| $\text{Re}(\lambda) > 0$ | any | Unstable |

### Lyapunov's Indirect Method

For a nonlinear system $\dot{x} = f(x)$ linearized to $\dot{x}_\delta = A x_\delta$:

- If $A$ is Hurwitz $\Rightarrow$ nonlinear system is locally asymptotically stable.
- If $A$ has eigenvalues with $\text{Re}(\lambda) > 0$ $\Rightarrow$ nonlinear system is unstable.
- If $\text{Re}(\lambda) = 0$ for some eigenvalues $\Rightarrow$ no conclusion from this method.

## 7.3 Phase Portraits

For 2D systems $\dot{x} = Ax$ with $A$ diagonalizable ($\lambda_1$, $\lambda_2$ eigenvalues):

| Eigenvalue type | Condition | Portrait | Stability |
|---|---|---|---|
| Real, same sign | $\lambda_1, \lambda_2 < 0$, $\lambda_1 \neq \lambda_2$ | **Stable node** (sink) | Asymptotically stable |
| Real, same sign | $\lambda_1, \lambda_2 > 0$, $\lambda_1 \neq \lambda_2$ | **Unstable node** (source) | Unstable |
| Real, opposite sign | $\lambda_1 < 0 < \lambda_2$ | **Saddle** | Unstable |
| Real, one zero | $\lambda_1 = 0$, $\lambda_2 \neq 0$ | Lines/rays | Marginally stable or unstable |
| Complex | $\text{Re}(\lambda) < 0$ | **Stable focus** (spiral in) | Asymptotically stable |
| Complex | $\text{Re}(\lambda) > 0$ | **Unstable focus** (spiral out) | Unstable |
| Pure imaginary | $\text{Re}(\lambda) = 0$ | **Center** | Marginally stable |
| Real, equal | $\lambda_1 = \lambda_2$, A.M. = G.M. | **Stable/unstable star** | Depends on sign |

The direction of rotation (clockwise vs counterclockwise) for complex eigenvalues is determined by the off-diagonal entry $a_{21}$: counterclockwise if $a_{21} > 0$.

## 7.4 Energy-Based Stability — Lyapunov Direct Method

### Motivation

Energy intuition: for a spring-mass-damper, the energy is $E = \frac{1}{2}kx_1^2 + \frac{1}{2}mx_2^2$. If $\dot{E} < 0$, energy decreases — stable.

### Positive/Negative Definite Matrices

For symmetric $P \in \mathbb{R}^{n \times n}$:

- **Positive definite** ($P \succ 0$): $x^T P x > 0$ for all $x \neq 0$. Equivalently: all eigenvalues $> 0$.
- **Positive semi-definite** ($P \succeq 0$): $x^T P x \geq 0$ for all $x$.
- **Negative definite** ($P \prec 0$): $x^T P x < 0$ for all $x \neq 0$.

**Sylvester's criterion:** $P \succ 0$ if and only if all **leading principal minors** are positive:

$$
p_{11} > 0, \quad \begin{vmatrix} p_{11} & p_{12} \\
 p_{21} & p_{22} \end{vmatrix} > 0, \quad \ldots, \quad \det(P) > 0
$$

### Lyapunov's Direct Method

For $\dot{x} = f(x)$ with $x_{eq} = 0$, propose a **Lyapunov function candidate** $V(x)$ in a neighborhood $D$ of 0.

A function $V(x)$ is a Lyapunov function if:
1. $V(0) = 0$
2. $V(x) > 0$ for $x \neq 0$ (positive definite)
3. $\dot{V}(x) = \nabla V \cdot f(x) \leq 0$ (at least negative semi-definite)

**Theorem:**
- If $\dot{V}(x) \leq 0$ in $D$ → **stable**.
- If $\dot{V}(x) < 0$ for $x \neq 0$ in $D$ → **asymptotically stable**.
- If $\dot{V}(x) > 0$ somewhere → $V$ is not a Lyapunov function; try another.

This is a **sufficient condition** — failure to find a Lyapunov function does not prove instability.

### Lyapunov Functions for LTI Systems

For $\dot{x} = Ax$, use the quadratic Lyapunov function candidate $V(x) = x^T P x$ with $P \succ 0$:

$$\dot{V}(x) = \dot{x}^T P x + x^T P \dot{x} = x^T A^T P x + x^T P A x = x^T (A^T P + PA) x$$

Let $Q = -(A^T P + PA)$:

$$\dot{V}(x) = -x^T Q x$$

If $Q \succ 0$, then $\dot{V}(x) < 0$ → asymptotically stable.

This gives the **Lyapunov matrix equation:**

$$A^T P + PA = -Q$$

**Theorem (necessary and sufficient):** For any symmetric positive definite $Q$, the system is asymptotically stable if and only if there exists a unique symmetric positive definite $P$ satisfying $A^T P + PA = -Q$.

### Procedure

1. Choose $Q = I$ (or any $Q \succ 0$).
2. Solve the Lyapunov equation $A^T P + PA = -Q$ for $P$.
3. Check if $P \succ 0$ (Sylvester or eigenvalues).
4. If $P \succ 0$ → asymptotically stable. If not → not asymptotically stable.

MATLAB: `lyap(A', Q)` returns $P$.

---

# Module 8: BIBO Stability

## 8.1 Bounded-Input-Bounded-Output (BIBO) Stability

A signal $f(t)$ is **bounded** if there exists a finite constant $M$ such that $|f(t)| \leq M$ for all $t \geq 0$.

An LTI system with zero initial conditions is **BIBO stable** if every bounded input produces a bounded output (zero-state response).

### Impulse Response

The output of an LTI system for an impulse input $u(t) = \delta(t)$:

$$h(t) = \mathcal{L}^{-1}[H(s)] = Ce^{At}B + D\delta(t)$$

The zero-state response via convolution:

$$y_{zs}(t) = \int_0^\infty h(\tau)\, u(t - \tau)\, d\tau$$

### BIBO Stability Criteria

**Method 1 — Impulse response:** The SISO system is BIBO stable if and only if $h(t)$ is absolutely integrable:

$$\int_0^\infty |h(t)|\, dt < \infty$$

**Method 2 — Transfer function poles:** The SISO system with transfer function $H(s)$ is BIBO stable if and only if all poles of $H(s)$ have negative real parts.

The impulse response contains terms of the form $t^k e^{st}$ for each pole $s$ of $H(s)$. These are absolutely integrable only when $\text{Re}(s) < 0$.

## 8.2 Relationship: BIBO vs. Internal Stability

$$\text{Asymp. stable} \implies \text{BIBO stable}$$
$$\text{BIBO stable} \not\Rightarrow \text{Asymp. stable}$$

**Why?** All poles of $H(s)$ are eigenvalues of $A$, but not all eigenvalues of $A$ are poles. Uncontrollable/unobservable modes may be hidden from the transfer function by pole-zero cancellation.

**Special case:** If the state space realization is **minimal** (both controllable and observable):

$$\text{Asymp. stable} \Leftrightarrow \text{BIBO stable}$$

### Summary

| Realization | Asymp. stable? | BIBO stable? |
|---|---|---|
| Minimal | Yes | Yes |
| Minimal | No | No |
| Non-minimal | No | Possible (if unstable mode is unobservable) |

### Resonance — BIBO Unstable Example

A marginally stable system (poles on imaginary axis) driven by an input at the natural frequency produces unbounded output.

For $m\ddot{y} + ky = F_0 \cos(\omega_n t)$ with $\omega_n = \sqrt{k/m}$:

$$y_p(t) = \frac{F_0}{2m\omega_n} t \sin(\omega_n t) \quad \text{(unbounded)}$$

This is the phenomenon that collapsed the Tacoma Narrows Bridge in 1940.

---

# Module 9: Linear State Feedback Control

## 9.1 Control Architecture

The full state feedback control law:

$$u = -Kx + Gr, \qquad K \in \mathbb{R}^{m \times n}, \; G \in \mathbb{R}^{m \times p}$$

where $r$ is the reference signal, $K$ is the feedback gain matrix, and $G$ is the feedforward (pre-filter) gain.

### Closed-Loop System

Substituting $u = -Kx + Gr$ into $\dot{x} = Ax + Bu$:

$$\dot{x} = (A - BK)x + BGr$$
$$y = (C - DK)x + DGr$$

**Closed-loop transfer function:**

$$H_{cl}(s) = (C - DK)[sI - (A - BK)]^{-1}BG + DG$$

**Closed-loop poles** = eigenvalues of $(A - BK)$.

**Open-loop poles** = eigenvalues of $A$.

By choosing $K$, we can move the closed-loop poles to desired locations — as long as the system is **controllable**.

## 9.2 Pole Placement (Eigenvalue Assignment)

**Theorem:** For a controllable SISO system, the eigenvalues of $(A - BK)$ can be placed at any desired locations by appropriate choice of $K$.

### Bass-Gura Formula (for SISO Systems)

**Step 1:** Transform to CCF (if not already in CCF):

$x = T_{CCF} z$, so the system becomes the CCF form.

**Step 2:** Choose desired eigenvalues $\lambda_1, \ldots, \lambda_n$ and compute the desired characteristic polynomial:

$$\phi(s) = (s - \lambda_1)(s - \lambda_2)\cdots(s - \lambda_n) = s^n + \hat{a}_{n-1}s^{n-1} + \cdots + \hat{a}_0$$

**Step 3:** In CCF coordinates, the feedback gain is:

$$K_{CCF} = [\hat{a}_0 - a_0, \; \hat{a}_1 - a_1, \; \ldots, \; \hat{a}_{n-1} - a_{n-1}]$$

where $a_i$ are the open-loop characteristic polynomial coefficients.

**Step 4:** Convert back to original coordinates:

$$K = K_{CCF} \cdot T_{CCF}^{-1} = K_{CCF} \cdot P_{CCF}^{-1} \cdot P^{-1}$$

This is the **Bass-Gura formula**:

$$K = [\hat{a}_0 - a_0, \; \ldots, \; \hat{a}_{n-1} - a_{n-1}] \cdot P_{CCF}^{-1} \cdot P^{-1}$$

MATLAB: `place(A, B, desired_eigs)`.

## 9.3 Steady-State Tracking (Feedforward Gain $G$)

**Goal:** Set $G$ such that the steady-state output equals the reference: $y_{final} = r$.

Using the **final value theorem** with a unit step reference $R(s) = 1/s$:

$$y_{final} = \lim_{t \to \infty} y(t) = \lim_{s \to 0} s Y(s) = H_{cl}(0)$$

Setting $y_{final} = 1$ (unit reference tracking):

$$H_{cl}(0) = 1 \implies G = -\frac{1}{C(A - BK)^{-1}B} \quad \text{(for } D = 0\text{)}$$

This is the **DC gain of the closed-loop system inverted**.

MATLAB: `G = -1 / (C * inv(A - B*K) * B)`.

---

# Module 10: Second-Order System Dynamics

## 10.1 Standard Second-Order System

The canonical second-order transfer function:

$$H(s) = \frac{\omega_n^2}{s^2 + 2\zeta\omega_n s + \omega_n^2}$$

where:
- $\omega_n > 0$ — **natural frequency** (rad/s)
- $\zeta \geq 0$ — **damping ratio** (dimensionless)

For the spring-mass-damper $m\ddot{y} + c\dot{y} + ky = F$:

$$\omega_n = \sqrt{k/m}, \qquad \zeta = \frac{c}{2\sqrt{km}}$$

### Poles

$$s_{1,2} = -\zeta\omega_n \pm \omega_n\sqrt{\zeta^2 - 1}$$

| Damping case | Condition | Pole type | Stability |
|---|---|---|---|
| Overdamped | $\zeta > 1$ | Two distinct real, negative | Asymp. stable |
| Critically damped | $\zeta = 1$ | Repeated real, negative | Asymp. stable |
| Underdamped | $0 < \zeta < 1$ | Complex conjugate, $\text{Re} < 0$ | Asymp. stable |
| Undamped | $\zeta = 0$ | Pure imaginary | Marginally stable |
| Negative damping | $\zeta < 0$ | $\text{Re} > 0$ | Unstable |

## 10.2 Transient Response Specifications

For an **underdamped** system ($0 < \zeta < 1$) with unit step input:

**Damped natural frequency:**

$$\omega_d = \omega_n \sqrt{1 - \zeta^2}$$

**Decay rate:** $\sigma = \zeta \omega_n$

**Unit step response:**

$$y(t) = 1 - e^{-\zeta\omega_n t}\left(\cos(\omega_d t) + \frac{\zeta}{\sqrt{1-\zeta^2}}\sin(\omega_d t)\right)$$

### Design Specifications

**Peak time $t_p$** — time to reach maximum:

$$t_p = \frac{\pi}{\omega_d} = \frac{\pi}{\omega_n\sqrt{1-\zeta^2}}$$

**Percent overshoot (%OS):**

$$\%OS = \frac{y_{max} - y_{final}}{y_{final}} \times 100 = e^{-\pi\zeta/\sqrt{1-\zeta^2}} \times 100$$

Inverted: given %OS, find $\zeta$:

$$\zeta = \frac{-\ln(\%OS/100)}{\sqrt{\pi^2 + \ln^2(\%OS/100)}}$$

**Settling time $t_s$** (2% criterion):

$$t_s \approx \frac{4}{\zeta\omega_n}$$

**Rise time $t_r$** (10% to 90%):

$$t_r \approx \frac{1 - 0.4167\zeta + 2.917\zeta^2}{\omega_n} \quad \text{(approximation)}$$

### Design Mapping

From %OS and $t_s$ specifications, compute $\zeta$ and $\omega_n$, then place dominant poles at:

$$s_{1,2} = -\zeta\omega_n \pm j\omega_d = -\sigma \pm j\omega_d$$

In the complex plane: the poles lie on a ray at angle $\theta = \arccos(\zeta)$ from the negative real axis.

## 10.3 PD Controller Example

For the spring-mass-damper with $m = c = k = 1$:

Open loop: $H(s) = \frac{1}{s^2 + s + 1}$, $\omega_n = 1$, $\zeta = 0.5$ → %OS = 16.3%, $t_s = 8s$.

PD control law $u = -k_p y - k_d \dot{y} + Gr$, i.e., $u = -Kx + Gr$ with $K = [k_p, k_d]$.

Closed-loop characteristic polynomial:

$$s^2 + (1 + k_d)s + (1 + k_p) = s^2 + 2\zeta_{des}\omega_{n,des} s + \omega_{n,des}^2$$

From desired specs (4% OS, $t_s = 2s$):

$$\zeta_{des} \approx 0.716, \quad \omega_{n,des} = \frac{4}{\zeta_{des} \cdot t_s} \approx 2.79$$

Solve:

$$k_d = 2\zeta_{des}\omega_{n,des} - 1 \approx 2.8, \quad k_p = \omega_{n,des}^2 - 1 \approx 6.8$$

Then set $G = -1/(C(A-BK)^{-1}B)$ for unit tracking.

---

# Module 11: Higher-Order Controller Design

## 11.1 Dominant Pole Approximation

A higher-order system can often be approximated by its dominant poles — the poles closest to the imaginary axis (slowest to decay).

**Rule:** A pole $s_2$ is dominated by $s_1$ if:

$$|\text{Re}(s_2)| > 10 \cdot |\text{Re}(s_1)|$$

i.e., $s_2$ is at least 10 times further to the left in the complex plane.

In that case, the contribution of $s_2$ to the transient dies out 10 times faster and is negligible.

**Design strategy for 3rd-order systems:** Place two dominant poles to satisfy the 2nd-order specs, and place the third pole 10+ times further left.

### Example

Desired 2nd-order specs: 6% OS, $t_s = 3s$.

$$\zeta_{des} = 0.667, \quad \omega_{n,des} = \frac{4}{\zeta_{des} \cdot t_s} = 2$$

Dominant poles: $s_{1,2} = -1.333 \pm j1.49$.

Third pole (dominated): $s_3 = -13.33$ (10× further left).

Desired 3rd-order characteristic polynomial:

$$\phi(s) = (s^2 + 2.667s + 4)(s + 13.33) = s^3 + 16s^2 + 39.55s + 53.32$$

Apply Bass-Gura formula to find $K$.

## 11.2 Pole Placement for Any Controllable System

Even without CCF, the Bass-Gura formula works:

$$K = K_{CCF} \cdot T_{CCF}^{-1} = K_{CCF} \cdot P_{CCF}^{-1} \cdot P^{-1}$$

This is also written as: $K = [\hat{a}_0 - a_0, \ldots, \hat{a}_{n-1} - a_{n-1}] (P_{CCF})^{-1} P^{-1}$.

MATLAB: `K = place(A, B, desired_eigs)`. Use slightly perturbed eigenvalues if any are repeated.

---

# Module 12: Observer Design

## 12.1 State Estimation Problem

In practice, we cannot always measure the full state $x$. We only measure the output $y = Cx + Du$.

**Goal:** Build an **observer** — a dynamical system that takes inputs $u$ and outputs $y$ and produces an estimate $\hat{x}$ that converges to the actual state $x$.

## 12.2 Idea 1: Open-Loop Copy

Simply replicate the system:

$$\dot{\hat{x}} = A\hat{x} + Bu, \quad \hat{x}(0) = 0$$

**Error:** $e = x - \hat{x}$, $\dot{e} = Ae$. Error dynamics: $e(t) = e^{At} e(0)$.

Problem: This works only if $A$ is Hurwitz (asymptotically stable). We cannot control how fast the error converges — and it fails completely if $A$ is unstable.

## 12.3 Idea 2: Luenberger Observer

Inject the output error $y - \hat{y}$ as a correction:

$$\dot{\hat{x}} = A\hat{x} + Bu + L(y - \hat{y}), \qquad \hat{y} = C\hat{x} + Du$$

where $L \in \mathbb{R}^{n \times p}$ is the **observer gain matrix**.

**Error dynamics:**

$$\dot{e} = (A - LC)e$$

The error converges to zero if and only if $(A - LC)$ is Hurwitz.

By choosing $L$, we can place the eigenvalues of $(A - LC)$ — and therefore choose how fast estimation errors decay.

## 12.4 Observer Eigenvalue Placement

By duality with the controller design problem:

$$A - LC \longleftrightarrow A - BK \quad \text{(with } B \leftrightarrow C^T \text{, } K \leftrightarrow L^T\text{)}$$

We can arbitrarily place the eigenvalues of $(A - LC)$ if and only if the system is **observable**.

**Bass-Gura formula for observer gain:**

$$L = \left([\hat{a}_0 - a_0, \ldots, \hat{a}_{n-1} - a_{n-1}] \cdot P_{d,CCF}^{-1} \cdot P_d^{-1}\right)^T$$

where $P_d = Q^T$ is the controllability matrix of the dual system.

MATLAB: `L = place(A', C', desired_eigs)'`.

## 12.5 Typical Design Rule

Observer poles are placed faster than controller poles (typically 3–5× faster in the real part), so the estimation error decays before it significantly affects the control performance.

---

# Module 13: Observer-Based Compensator and Separation Principle

## 13.1 Observer-Based Compensator

Combine controller and observer:

**Controller:** $u = -K\hat{x} + Gr$

**Observer:** $\dot{\hat{x}} = A\hat{x} + Bu + L(y - \hat{y})$

### Augmented System

Write the system in terms of $x$ (true state) and $e = x - \hat{x}$ (estimation error):

$$
\begin{bmatrix} \dot{x} \\
 \dot{e} \end{bmatrix} = \begin{bmatrix} A - BK & BK \\
 0 & A - LC \end{bmatrix} \begin{bmatrix} x \\
 e \end{bmatrix} + \begin{bmatrix} BG \\
 0 \end{bmatrix} r
$$

The augmented system matrix is **block upper triangular**.

## 13.2 Separation Principle

$$
\text{eig}\!\begin{pmatrix} \begin{bmatrix} A - BK & BK \\
 0 & A - LC \end{bmatrix} \end{pmatrix} = \text{eig}(A - BK) \cup \text{eig}(A - LC)
$$

The eigenvalues of the combined system are the union of the controller eigenvalues and the observer eigenvalues.

**Consequence:** Controller design and observer design can be done independently:

- Design $K$ to place eigenvalues of $(A - BK)$ for desired closed-loop response.
- Design $L$ to place eigenvalues of $(A - LC)$ for desired estimation convergence.
- They do not interfere with each other.

This is the **Separation Principle** — the cornerstone of modern control and the predecessor to Kalman filter + LQG control.

---

# Module 14: Linear Quadratic Regulator (LQR)

## 14.1 Problem Formulation

**LQR** optimally trades off control performance and control effort:

$$J = \int_0^\infty \left[ x^T(t) Q\, x(t) + u^T(t) R\, u(t) \right] dt$$

Subject to: $\dot{x} = Ax + Bu$, $x(0) = x_0$.

Where:
- $Q \in \mathbb{R}^{n \times n}$ — **state cost matrix** ($Q \succeq 0$, symmetric)
- $R \in \mathbb{R}^{m \times m}$ — **input cost matrix** ($R \succ 0$, symmetric)

**LQR is:**
- **Linear:** system is linear, control law is linear $u = -K^* x$
- **Quadratic:** cost function is quadratic
- **Regulator:** reference $r = 0$ (drive state to origin)

## 14.2 Optimal Solution

The optimal control law is:

$$u^* = -K^* x, \qquad K^* = R^{-1} B^T P^*$$

where $P^*$ is the unique positive definite solution to the **Algebraic Riccati Equation (ARE)**:

$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$

MATLAB: `[K, P, e] = lqr(A, B, Q, R)`.

## 14.3 Tuning $Q$ and $R$

**State cost $Q$:** encodes how much you penalize deviations in each state variable.

Example: $Q = \text{diag}(100, 1)$ penalizes $x_1$ deviations 100× more than $x_2$.

**Input cost $R$:** encodes how expensive control effort is.

**Trade-off:**
- **Large $Q$, small $R$** ("cheap control"): aggressive controller, fast convergence, large $u$.
- **Small $Q$, large $R$** ("expensive control"): conservative controller, slow convergence, small $u$.

### Example

For spring-mass-damper ($m = k = c = 1$), $A = \begin{bmatrix} 0 & 1 \\ -1 & -1 \end{bmatrix}$, $B = \begin{bmatrix} 0 \\ 1 \end{bmatrix}$:

| $Q$ | $R$ | Strategy | $K^*$ |
|---|---|---|---|
| $I$ | $10$ | Expensive | $[0.0488, \; 0.0945]$ |
| $I$ | $1$ | Balanced | $[0.4142, \; 0.6818]$ |
| $1000I$ | $1$ | Cheap | $[30.64, \; 6.96]$ |

## 14.4 Why LQR over Pole Placement?

| | Pole Placement | LQR |
|---|---|---|
| Design freedom | Full (place anywhere) | Constrained by cost |
| Intuition | Direct eigenvalue spec | Indirect (tune $Q$, $R$) |
| Robustness guarantees | None inherent | Guaranteed margins |
| Scalability | Hard for MIMO | Natural for MIMO |

LQR provides guaranteed gain margin of $\geq 6$ dB and phase margin of $\geq 60°$ under certain conditions — something pole placement does not guarantee.

---

# Module 15: Common Mistakes and Mental Checks

## On State Space

- **Mistake:** Choosing $x = y$ as state for a 2nd-order system. You need both $y$ and $\dot{y}$.
- **Check:** The number of state variables must equal the order of the highest derivative in the ODE.
- **Mistake:** Forgetting $D$ when computing the transfer function. $H(s) = C(sI-A)^{-1}B + D$.
- **Check:** $D = 0$ for strictly proper TFs. $D \neq 0$ adds a direct feedthrough term.

## On Eigenvalues and Stability

- **Mistake:** Assuming all eigenvalues of $A$ are poles of $H(s)$. They aren't — pole-zero cancellations can occur.
- **Check:** Poles of $H(s) \subseteq$ eigenvalues of $A$. Equality only for minimal realizations.
- **Mistake:** Concluding stability from $\text{Re}(\lambda) = 0$ without checking A.M. vs G.M.
- **Check:** $\text{Re}(\lambda) = 0$ + A.M. = G.M. → marginally stable. A.M. > G.M. → unstable.

## On Controllability and Observability

- **Mistake:** Confusing controllability (steering to origin) with reachability (steering from origin). For continuous-time LTI, they are equivalent.
- **Check:** Always use $\text{rank}(P) = n$ for controllability, $\text{rank}(Q) = n$ for observability.
- **Mistake:** Thinking a non-minimal realization that is BIBO stable is also internally stable.
- **Check:** BIBO stable + non-minimal ≠ asymptotically stable. Hidden unstable modes can exist.

## On Controller and Observer Design

- **Mistake:** Designing $K$ for a system that is not controllable. Pole placement requires full controllability.
- **Check:** Verify $\text{rank}(P) = n$ before pole placement.
- **Mistake:** Designing $L$ for a system that is not observable. Observer design requires full observability.
- **Check:** Verify $\text{rank}(Q) = n$ before observer design.
- **Mistake:** Placing observer poles at the same speed as controller poles. The observer should be faster.
- **Rule of thumb:** Observer poles 3–5× further left than controller poles in the complex plane.

## On 2nd-Order Specs

- **Mistake:** Using %OS formula $e^{-\pi\zeta/\sqrt{1-\zeta^2}}$ for overdamped systems ($\zeta \geq 1$). No overshoot exists.
- **Check:** All 2nd-order spec formulas assume $0 < \zeta < 1$ (underdamped).
- **Mistake:** Forgetting to set feedforward gain $G$ — the closed-loop DC gain without $G$ is not 1.
- **Check:** Always compute $G = -1/(C(A-BK)^{-1}B)$ for unit steady-state tracking.

---

# Module 16: Robotics Connections

## State Space → Robot Dynamics

A robot manipulator's dynamics in joint space:

$$M(q)\ddot{q} + C(q, \dot{q})\dot{q} + g(q) = \tau$$

This is nonlinear. But after **linearization** around an operating point, it becomes:

$$\dot{x}_\delta = A x_\delta + B u_\delta$$

The state $x = [q, \dot{q}]^T$ (joint positions and velocities), input $u = \tau$ (joint torques).

## Controllability → Can the Robot Reach Any Configuration?

If $\text{rank}(P) = n$, the linearized robot model is controllable — you can drive the joints anywhere in a neighborhood of the equilibrium.

## Observability → State Estimation from Sensors

In many robots, only positions are measurable (encoders), not velocities. Is the velocity observable from position measurements?

For the simple integrator model $\ddot{q} = \tau/m$ with $C = [1, 0]$ (only position measured):

$$
Q = \begin{bmatrix} C \\
 CA \end{bmatrix} = \begin{bmatrix} 1 & 0 \\
 0 & 1 \end{bmatrix}
$$

$\text{rank}(Q) = 2$ → observable. Velocity can be estimated from position measurements via a Luenberger observer (or Kalman filter).

## Observer Design → Kalman Filter

The Luenberger observer with $L = P^* C^T R_n^{-1}$ (where $P^*$ solves the observer Riccati equation and $R_n$ is the noise covariance) becomes the **Kalman filter** — the optimal state estimator for systems with Gaussian noise.

Every robot running GPS, IMU fusion, or visual odometry is running a Kalman filter under the hood — the Luenberger observer with statistically optimal gain.

## LQR → Optimal Robot Control

LQR provides a principled way to balance state regulation performance against actuator effort:

- $Q$ encodes how much joint tracking error matters.
- $R$ encodes how expensive motor torque is.

LQR is widely used in **quadrotor attitude control**, **legged robot balance control**, and **spacecraft attitude control**.

## Separation Principle → Modern Estimation + Control

The separation principle tells you to design the estimator (Kalman filter) and the controller (LQR) independently, then combine them into the **LQG (Linear Quadratic Gaussian)** controller — the most commonly deployed optimal controller in engineering.

---

# Module 17: Compact Revision Map

## Topic Dependency Chain

```
Mechanical Modeling → ODE → Laplace Transform
                                   ↓
           State Space ←→ Transfer Function
                   ↓
         Matrix Exponential (e^At) → Solution x(t)
                   ↓
         Coordinate Transformation → Canonical Forms
                   ↓           (DCF, CCF, OCF)
     Controllability (P)    Observability (Q)
             ↓                      ↓
   Canonical Decomp           Canonical Decomp
             ↓                      ↓
      Minimality ←─────────────────→
                         ↓
           Internal Stability (Lyapunov, Hurwitz)
                         ↓
              BIBO Stability ↔ Internal Stability
                         ↓
              State Feedback Control (K)
                         ↓
              2nd Order Specs + Dominant Poles
                         ↓
         Observer Design (L) → Separation Principle
                         ↓
                         LQR
```

## One-Line Summaries

| Topic | One-line summary |
|---|---|
| State space | $\dot{x} = Ax + Bu$, $y = Cx + Du$: the universal model. |
| Matrix exponential | $e^{At}$: the free evolution operator, computed via eigendecomposition or Laplace. |
| Linearization | Jacobians around equilibrium: convert nonlinear to LTI. |
| DCF | Diagonalize $A$ using eigenvectors. Decoupled subsystems. |
| CCF | Companion form. Always controllable. Bottom row = $-a_i$. |
| OCF | Dual of CCF. Always observable. Last column = $-a_i$. |
| Controllability | $\text{rank}(P) = n$: can steer to any state. |
| Observability | $\text{rank}(Q) = n$: can infer any initial state. |
| Duality | Controllability of original = observability of dual. |
| Minimality | Controllable + observable = minimal realization. |
| Asymp. stability | All eigenvalues have $\text{Re} < 0$. $A$ is Hurwitz. |
| Marginal stability | Eigenvalues on imaginary axis with A.M. = G.M. |
| Lyapunov | $V(x) > 0$, $\dot{V}(x) \leq 0$ → stable. $\dot{V} < 0$ → asymp. stable. |
| BIBO stability | All poles of $H(s)$ have $\text{Re} < 0$. Asymp. stable ⟹ BIBO stable. |
| Pole placement | Set $K$ so eig($A - BK$) = desired locations. Requires controllability. |
| Steady-state tracking | Set $G$ to invert DC gain of closed-loop. |
| 2nd-order specs | %OS → $\zeta$, $t_s$ → $\omega_n$, then place poles at $-\zeta\omega_n \pm j\omega_d$. |
| Observer | $\dot{\hat{x}} = A\hat{x} + Bu + L(y - \hat{y})$. Error: $(A - LC)e$. Requires observability. |
| Separation principle | eig(augmented) = eig($A - BK$) ∪ eig($A - LC$). Design independently. |
| LQR | Minimize $\int x^T Q x + u^T R u \, dt$. Optimal $K^* = R^{-1}B^T P^*$. |

---

# Quick Reference: Key Formulas

## Controllability

$$P = \begin{bmatrix} B & AB & \cdots & A^{n-1}B \end{bmatrix}, \quad \text{rank}(P) \stackrel{?}{=} n$$

$$W(t_0, t_f) = \int_{t_0}^{t_f} e^{A(t_0-\tau)} BB^T e^{A^T(t_0-\tau)}\, d\tau, \quad W \text{ invertible} \Leftrightarrow \text{controllable}$$

## Observability

$$
Q = \begin{bmatrix} C \\
 CA \\
 \vdots \\
 CA^{n-1} \end{bmatrix}, \quad \text{rank}(Q) \stackrel{?}{=} n
$$

$$M(t_0, t_f) = \int_{t_0}^{t_f} e^{A^T(\tau-t_0)} C^T C\, e^{A(\tau-t_0)}\, d\tau, \quad M \text{ invertible} \Leftrightarrow \text{observable}$$

## Canonical Forms

| Form | $A$ | $B$ | $C$ | Always |
|---|---|---|---|---|
| DCF | $\text{diag}(\lambda_i)$ | $T_{DCF}^{-1}B$ | $CT_{DCF}$ | Diag if diagonalizable |
| CCF | Companion (superdiagonal) | $[0,\ldots,0,1]^T$ | $[b_0,\ldots,b_{n-1}]$ | Controllable |
| OCF | Companion (subdiagonal) | $[b_0,\ldots,b_{n-1}]^T$ | $[0,\ldots,0,1]$ | Observable |

## Stability

$$A \text{ Hurwitz} \Leftrightarrow \text{Re}(\lambda_i(A)) < 0 \; \forall i \Leftrightarrow \exists P \succ 0: A^T P + PA \prec 0$$

## 2nd Order Response

$$\zeta = \frac{-\ln(\%OS/100)}{\sqrt{\pi^2 + \ln^2(\%OS/100)}}, \quad \omega_n = \frac{4}{\zeta t_s}, \quad t_p = \frac{\pi}{\omega_d}, \quad \omega_d = \omega_n\sqrt{1-\zeta^2}$$

## Closed Loop

$$u = -Kx + Gr \implies \dot{x} = (A-BK)x + BGr$$
$$G = \frac{-1}{C(A-BK)^{-1}B} \quad (D = 0, \text{ unit reference})$$

## Observer

$$\dot{\hat{x}} = (A - LC)\hat{x} + Bu + Ly$$

## LQR

$$A^T P + PA - PBR^{-1}B^T P + Q = 0, \quad K^* = R^{-1}B^TP^*$$

---

*Built from MAE 506 lecture notes (Lectures 0–26), textbook: Williams & Lawrence — Linear State-Space Control Systems, and the Robotics Concept Stack.*
