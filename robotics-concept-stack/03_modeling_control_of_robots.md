# **Modeling and Control of Robots**

### **From Robot Geometry to Robot Dynamics to Robot Control**

**Focus:** the manipulator pipeline: how robot arms are represented, how their motion is computed, how forces and torques map through the mechanism, and how joint-level control is built on top of that structure.

---

# Why this subject matters in our stack

This is the course where robotics stops being abstract and becomes mechanical.

It is the bridge between:
- **geometry** - frames, rotations, rigid motion,
- **kinematics** - where the robot is and how it moves,
- **statics and dynamics** - what forces and torques are required,
- **control** - how to actually make the arm do the thing.

If linear algebra gives we the language and system dynamics/control gives we the control toolkit, this course gives we the **robot-specific structure** those tools operate on.

For robot manipulators, almost everything flows through the same chain:

1. **Describe the mechanism** - links, joints, degrees of freedom, configuration.
2. **Map joint variables to pose** - forward kinematics.
3. **Invert the geometry** - inverse kinematics.
4. **Map joint velocities to end-effector twist** - Jacobian and velocity kinematics.
5. **Map end-effector forces to joint torques** - statics.
6. **Write the equations of motion** - dynamics.
7. **Design joint-level or task-level behavior** - robot control.

This is core infrastructure for manipulators, mobile manipulation, humanoids, aerial manipulators, surgical robots, industrial automation, and contact-rich interaction.

# How to use this document

## Mode 1: Quick revision

Read:
- Core Formula Sheet
- the “What this means in practice” blocks
- Quick Reference

## Mode 2: Homework / exam rebuild

Read the sections in order:
- configuration
- rigid-body motion
- forward kinematics
- inverse kinematics
- Jacobian / singularity
- statics
- dynamics
- control

## Mode 3: Robotics design mode

Use this as a manipulator cheat-sheet:
- need pose? → FK
- need joint solution? → IK
- need velocity map? → Jacobian
- need wrench-to-torque map? → statics
- need equations of motion? → dynamics
- need actual behavior? → control

---

# Big Picture Map

This course is really one long chain.

## 1. Configuration
What are the robot’s generalized coordinates? How many independent degrees of freedom does it have?

## 2. Rigid-body kinematics
How do we represent translation, rotation, and pose using matrices and frames?

## 3. Forward kinematics
Given joint variables, where is the end-effector?

## 4. Inverse kinematics
Given desired pose, which joint variables achieve it?

## 5. Velocity kinematics
Given joint rates, what linear/angular velocity does the end-effector have?

## 6. Jacobian and singularity
Which task-space velocities are achievable right now? Which directions are lost at singular configurations?

## 7. Statics
How do endpoint forces map to joint torques?

## 8. Dynamics
How do inertia, Coriolis/centrifugal terms, and gravity determine robot motion?

## 9. Joint control
How do we actually command the arm robustly, usually one joint at a time, in real hardware?

That is the course arc.

# Core Formula Sheet

## Configuration / DOF

For a mechanism with:
- ambient rigid-body motion dimension `m` (`m=3` planar, `m=6` spatial),
- `L` links,
- `J` joints,
- joint freedoms `f_i`,

the degree-of-freedom count is:

$$
\text{dof} = m(L-1-J) + \sum_{i=1}^{J} f_i
$$

## Rotation matrix properties

$$
R^T R = I, \qquad R^{-1} = R^T, \qquad |\det R| = 1
$$

For a proper rotation matrix:

$$
\det R = 1
$$

## Homogeneous transformation

$$
T =
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
$$

with pose composition:

$$
T_0^2 = T_0^1 T_1^2
$$

## Forward kinematics

$$
T_0^e(q) = A_0^1(q_1)A_1^2(q_2)\cdots A_{n-1}^n(q_n)T_n^e
$$

## Velocity kinematics

End-effector twist:

$$
v_e =
\begin{bmatrix}
\dot p_e \\
\omega_e
\end{bmatrix} 
= J(q)\dot q
$$

with:

$$
J(q) =
\begin{bmatrix}
J_P(q) \\
J_O(q)
\end{bmatrix}
$$

For joint `i`:

**Prismatic:**

$$
J_{P,i} = z_{i-1}, \qquad J_{O,i} = 0
$$

**Revolute:**

$$
J_{P,i} = z_{i-1} \times (p_e - p_{i-1}), \qquad J_{O,i} = z_{i-1}
$$

## Statics

Virtual work gives the wrench-to-torque mapping:

$$
\tau = J^T(q)\,\gamma_e
$$

where:

$$
\gamma_e =
\begin{bmatrix}
f_e \\
\mu_e
\end{bmatrix}
$$

## Singularity / redundancy

If:

$$
v_e = J(q)\dot q, \qquad J \in \mathbb{R}^{r \times n}
$$

then:

$$
\dim R(J) = r, \qquad \dim N(J) = n-r
$$

Redundancy means `n > r`. Singularity means rank drops.

## Inverse velocity kinematics

Square nonsingular case:

$$
\dot q = J^{-1} v_e
$$

Redundant / non-square case:

$$
\dot q = J^+ v_e
$$

with secondary objective injection:

$$
\dot q = J^+ v_e + (I - J^+J)z
$$

## Dynamics

Standard manipulator dynamics:

$$
M(q)\ddot q + C(q,\dot q)\dot q + g(q) = \tau
$$

Equivalent compressed form often written as:

$$
B(q)\ddot q + n(q,\dot q) = \xi
$$

with inertia matrix positive definite.

## Joint control idea

At the joint level, the control architecture is usually some form of:

$$
\tau = \tau_{ff} + \tau_{fb}
$$

where feedforward handles known dynamics and feedback handles tracking error and disturbances.

# Section 1: Robot Configuration - What the Coordinates Really Mean

## What it is

A robot’s **configuration** is the minimal information needed to specify its pose completely.

For a planar rigid body, the configuration is typically:

$$
(x, y, \theta)
$$

For a spatial rigid body, it is position plus orientation.

For a robot arm, configuration is usually expressed in **joint coordinates**:

$$
q = [q_1, q_2, \dots, q_n]^T
$$

Those coordinates are the true variables of the mechanism.

## Degrees of freedom

DOF means the number of independent coordinates required to specify the configuration.

This course makes we think about DOF in a precise mechanical way, not as vague “how many ways it can move” intuition.

For mechanisms:

$$
\text{dof} = m(L-1-J) + \sum_{i=1}^{J} f_i
$$

where `m=3` in planar motion and `m=6` in spatial motion.

## Why this matters

Before doing any kinematics or control, we need to know:
- how many independent coordinates the robot really has,
- whether the mechanism is underactuated, fully actuated, or redundant,
- whether our task dimension is smaller than our configuration dimension.

## What this means in practice

This is the first place where robotics becomes structure-aware.

we stop asking “how many joints are there?” and start asking:
- how many **independent generalized coordinates** exist,
- what constraints remove motion,
- what the configuration space actually looks like.

That matters immediately for manipulators, mobile robots, parallel mechanisms, and constrained systems.

# Section 2: Rigid-Body Motion - Translation, Rotation, and Pose

## What it is

A robot arm is a stack of rigid bodies. So before we can model the robot, we need a clean way to represent the motion of a rigid body in 2D and 3D.

The building blocks are:
- vectors,
- rotation matrices,
- coordinate frames,
- homogeneous transformations.

## Rotation matrices

A rotation matrix maps coordinates from one frame to another.

Its core properties are:

$$
R^T R = I, \qquad R^{-1} = R^T, \qquad \det R = 1
$$

This means rotation preserves lengths and angles.

## Elementary rotations

In 3D, we build rotations from elementary axis rotations such as:

$$
R_z(\alpha), \qquad R_y(\beta), \qquad R_x(\gamma)
$$

Then we assemble more complex orientation descriptions:
- Euler angles,
- angle-axis,
- quaternions.

## Homogeneous transformations

To combine rotation and translation in one object:

$$
T =
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
$$

This lets we compose poses by matrix multiplication.

That is the real machinery behind link-to-link transformations.

## What this means in practice

This is the geometry engine of robot arms.

Everything later depends on this:
- FK uses chained transforms,
- Jacobians use frame axes and positions,
- dynamics use COM positions and link orientations,
- control relies on representing desired vs actual pose consistently.

If our frame bookkeeping is wrong, everything downstream gets cursed.

# Section 3: Forward Kinematics - From Joint Space to End-Effector Pose

## What it is

Forward kinematics answers:

> Given joint variables `q`, where is the end-effector?

This is the direct map:

$$
q \mapsto T_0^e(q)
$$

For serial manipulators, we compute it by chaining link transformations.

## Core structure

$$
T_0^e(q) = A_0^1(q_1)A_1^2(q_2)\cdots A_{n-1}^n(q_n)T_n^e
$$

Each matrix contributes one joint/link transformation. Their product gives the final pose.

## Why FK matters

FK is the base map of the manipulator.

Without it, we cannot:
- know where the tool is,
- derive the Jacobian,
- formulate IK,
- compute COM locations for dynamics,
- do trajectory tracking in Cartesian space.

## What this means in practice

FK is the “easy direction” mathematically, but it is still the source of a lot of bugs.

The mature habit is:
- define frames clearly,
- keep transformation order consistent,
- never mix point coordinates from different frames casually,
- verify with simple limiting configurations.

In robotics work, FK is often the first thing we validate in simulation before touching IK or control.

# Section 4: Inverse Kinematics - From Desired Pose to Joint Coordinates

## What it is

Inverse kinematics answers:

> Given a desired end-effector pose, what joint variables achieve it?

This is the reverse map:

$$
T_0^e(q) = T_d
$$

Unlike FK, IK is usually:
- nonlinear,
- nonunique,
- sometimes impossible.

## Analytic IK

For special arm structures, we can derive closed-form solutions.

Typical workflow:
1. separate position and orientation,
2. compute the wrist center,
3. solve geometry for early joints,
4. solve wrist orientation for the remaining joints.

That is the clean classical manipulator IK pipeline.

## Key realities of IK

A desired pose can have:
- one solution,
- many solutions,
- no solution.

Multiple solutions correspond to different robot postures:
- elbow up / elbow down,
- shoulder left / shoulder right,
- wrist flip / non-flip.

## Workspace logic

Before solving IK, always ask:
- is the target inside reachable workspace?
- is it near singularity?
- is the requested orientation compatible with arm geometry?

## What this means in practice

IK is not “invert the matrix and go home.”
It is a geometry problem with branch structure.

This is why real robot software often needs:
- posture selection rules,
- joint limit checks,
- collision-aware filtering,
- numerical fallback when closed form becomes messy.

# Section 5: Velocity Kinematics - How Motion Propagates Through the Chain

## What it is

Pose tells we where the robot is.
Velocity kinematics tells we how fast it is moving.

The course develops this carefully by differentiating rigid-body motion and rotation matrices.

For a time-varying rotation matrix:

$$
R = R(t), \qquad R^T R = I
$$

Differentiation gives the skew-symmetric angular-velocity structure:

$$
\dot R = S(\omega)R
$$

where:

$$
S(\omega) = [\omega \times]
$$

## Why this matters

This is the bridge between geometry and motion.

It lets we derive:
- angular velocity from frame motion,
- linear velocity relationships between links,
- recursive velocity propagation,
- eventually the Jacobian.

## What this means in practice

Velocity kinematics is the part that makes rigid-body motion feel alive instead of static.

Once we understand:
- `\dot R = S(\omega)R`,
- linear velocity recursion,
- angular velocity recursion,

we are ready for Jacobians, statics, and dynamics.

# Section 6: Jacobian - The Most Important Matrix in Manipulator Robotics

## What it is

The manipulator Jacobian maps joint velocity to end-effector twist:

$$
v_e =
\begin{bmatrix}
\dot p_e \\
\omega_e
\end{bmatrix}
= J(q)\dot q
$$

This is the local linearization of manipulator motion in joint space.

## Structure

$$
J(q) =
\begin{bmatrix}
J_P(q) \\
J_O(q)
\end{bmatrix}
$$

For joint `i`:

**Prismatic joint**

$$
J_{P,i} = z_{i-1}, \qquad J_{O,i} = 0
$$

**Revolute joint**

$$
J_{P,i} = z_{i-1} \times (p_e - p_{i-1}), \qquad J_{O,i} = z_{i-1}
$$

This is one of the highest-value formulas in the whole course.

## Why Jacobian matters

The Jacobian answers all of these immediately:
- What end-effector velocity results from a given `\dot q`?
- Which Cartesian directions are achievable right now?
- Where is the manipulator singular?
- How do endpoint forces map to joint torques?
- How do we do velocity IK?

## Analytical Jacobian

If we parameterize orientation with Euler angles or similar variables, we often use an analytical Jacobian:

$$
\dot x_e = J_A(q)\dot q
$$

where the angular velocity block is mapped into orientation-rate coordinates.

## What this means in practice

For robot arms, the Jacobian is basically the gearbox between joint space and task space.

In research and industry it keeps showing up:
- resolved-rate motion control,
- IK solvers,
- singularity detection,
- force control,
- impedance control,
- manipulability analysis,
- redundancy resolution.

If linear algebra gave we respect for rank and nullspace, the Jacobian is where that respect cashes out.

# Section 7: Singularity and Redundancy - When the Robot Loses or Gains Freedom

## Singularity

A singularity is a configuration where the Jacobian loses rank.

That means the manipulator loses one or more instantaneous task-space motion directions.

Even if the arm still physically exists in that posture, the local motion map becomes degenerate.

## Redundancy

If:

$$
J \in \mathbb{R}^{r \times n}, \qquad n > r
$$

then the robot is kinematically redundant for that task.

we have more joint coordinates than required task coordinates.

The key dimension statement is:

$$
\dim R(J) = r, \qquad \dim N(J) = n-r
$$

So redundancy creates a nontrivial nullspace.

## Nullspace meaning

If:

$$
J(q)\dot q = v_e
$$

then all solutions are of the form:

$$
\dot q = \dot q_p + \dot q_n, \qquad \dot q_n \in N(J)
$$

That means the robot can change posture without changing the instantaneous end-effector motion.

## What this means in practice

Singularity is where control effort can explode and velocity IK becomes unstable.

Redundancy is where robotics gets elegant.
It lets we do secondary objectives such as:
- joint limit avoidance,
- obstacle clearance,
- posture shaping,
- manipulability maximization.

This is why nullspace methods matter so much in modern manipulation.

# Section 8: Inverse Velocity Kinematics - Solving Motion Commands in Joint Space

## What it is

Instead of solving pose-level IK, we solve velocity-level IK:

> Given desired end-effector velocity, what joint velocity should I command?

That means solving:

$$
J(q)\dot q = v_e
$$

## Cases

### 1. Square, nonsingular Jacobian

$$
\dot q = J^{-1} v_e
$$

### 2. Overdetermined or inconsistent request
Use least-squares logic.

### 3. Redundant robot
Use pseudoinverse or optimization-based solutions.

$$
\dot q = J^+ v_e
$$

### 4. Weighted or constrained solution
Use weighted pseudoinverse / quadratic optimization.

This is where the robotics-control-linear algebra triangle locks together.

## What this means in practice

Velocity-level control is often easier and more numerically stable than direct pose inversion.

That is why many practical controllers are built as:
- desired Cartesian velocity,
- Jacobian-based joint velocity command,
- low-level joint servo closes the loop.

This is also the entry point to operational-space motion control.

# Section 9: Numerical Inverse Kinematics - Iterative IK When Closed Form Is Ugly

## What it is

When analytic IK is unavailable or inconvenient, we solve IK iteratively.

Start with a guess `q_k`, compute task error, linearize locally using the Jacobian, update, repeat.

## Core idea

At each iteration:

$$
\Delta x \approx J(q_k)\Delta q
$$

so we choose an update law such as:
- Jacobian inverse,
- Jacobian pseudoinverse,
- Jacobian transpose,
- damped / weighted variants.

Typical conceptual form:

$$
q_{k+1} = q_k + \Delta q
$$

## Why numerical IK matters

Because in real robots:
- exact symbolic IK is often too messy,
- constraints and limits matter,
- target changes online,
- iterative solvers are easier to embed in software stacks.

## What this means in practice

Numerical IK is not a fallback for weaklings. It is normal.

It is what we use when:
- the arm is redundant,
- we want secondary objectives,
- we want one solver for many robot structures,
- the target changes online.

The trade-offs are convergence speed, singularity behavior, and sensitivity to initialization.

# Section 10: Statics - How Forces Travel Back Through the Robot

## What it is

Statics asks:

> If the end-effector experiences a force/moment, what joint torques are required to balance it?

This comes from virtual work.

Let endpoint wrench be:

$$
\gamma_e = \begin{bmatrix} f_e \\
\mu_e \end{bmatrix}
$$

Then the joint torque relation is:

$$
\tau = J^T(q)\gamma_e
$$

## Why this is profound

The same Jacobian that maps motion forward also maps forces backward through its transpose.

That is one of the cleanest dualities in robotics.

## What this means in practice

This equation underlies:
- force control,
- contact reasoning,
- grasp wrench mapping,
- impedance / admittance control intuition,
- actuator sizing.

If we know the desired endpoint wrench, `J^T` tells we how much each joint has to suffer for the cause.

# Section 11: Dynamics - The Equations of Motion of a Manipulator

## What it is

Dynamics asks:

> Given torques, how will the robot move?

or the inverse question:

> Given desired motion, what torques are needed?

This course builds robot dynamics through the Lagrangian route using kinetic and potential energy.

## Standard manipulator form

$$
M(q)\ddot q + C(q,\dot q)\dot q + g(q) = \tau
$$

Equivalent notation often compresses terms as:

$$
B(q)\ddot q + n(q,\dot q) = \xi
$$

with the inertia matrix positive definite.

## Physical meaning of the terms

- `M(q)` / `B(q)` - configuration-dependent inertia matrix.
- `C(q,\dot q)\dot q` - Coriolis and centrifugal effects.
- `g(q)` - gravity term.
- `\tau` - actuator torque input.

## Why this matters

Kinematics tells we what geometry permits.
Dynamics tells we what physics permits.

Two poses may be kinematically reachable but dynamically very different in effort, speed, and stability.

## What this means in practice

This is the backbone of:
- inverse dynamics,
- model-based control,
- gravity compensation,
- simulation,
- trajectory tracking,
- optimal control for manipulators.

This is also where robotics stops being just moving triangles in RViz and becomes actual mechanics.

# Section 12: Robot Control - Why Joint-Level Control Usually Wins First

## What it is

Once we have dynamics, we still need a control architecture that works on real hardware.

This course points toward **decentralized joint control** / **independent joint control**:
- treat each joint as a controlled subsystem,
- compensate the most important dynamics,
- use feedback for tracking and disturbance rejection.

## Why this architecture is popular

Because full coupled nonlinear control is elegant but expensive.
Independent joint control is simpler, robust, and good enough for many industrial systems.

The philosophy is:
- model the motor + gear + link behavior reasonably well,
- close a fast servo loop per joint,
- treat inter-joint coupling as disturbance or partially compensate it.

## Common ingredients

Real joint control stacks often include some mix of:
- P / PD / PID structure,
- velocity feedback,
- gravity compensation,
- feedforward terms,
- motor-side dynamics,
- disturbance rejection.

## What this means in practice

This is why industrial manipulators are so effective.
They do not always solve one giant heroic nonlinear control problem every millisecond.

Instead they build strong joint-level loops, then layer task-space planning and motion generation on top.

That is a very grown-up engineering move.

# Section 13: Motion and Trajectory Planning - The Missing Bridge Between Geometry and Control

## What it is

The intro lecture explicitly places **motion and trajectory planning** in the course arc.

That is important, because after we know:
- where the robot is,
- what torques it needs,
- how to control joints,

we still need to decide **how the motion should evolve over time**.

## Conceptual role

Trajectory planning chooses:
- desired path in joint or task space,
- timing law,
- smoothness of velocity and acceleration,
- feasibility with respect to dynamics and actuator limits.

## Why it matters

Without planning, control has nothing meaningful to track.
Without control, planning stays fantasy.

## What this means in practice

The clean stack is:
- plan desired motion,
- use IK / FK / Jacobians as needed,
- compute feasible reference trajectory,
- track it with joint or task-space control.

That full stack is the real manipulator pipeline.

# Section 14: Robotics Connections

## 1. Industrial robot arms

This course is basically the theory operating under industrial manipulators.

- FK tells the controller where the tool is.
- IK maps desired tool poses into joint references.
- Jacobians map velocities and forces.
- Dynamics informs feedforward and compensation.
- Decentralized joint control keeps the machine stable and accurate.

## 2. Mobile manipulation

Once a mobile base carries an arm, all this arm theory still survives. It just gets embedded inside a larger whole-body problem.

## 3. Humanoids and legged robots

Even when the robot is not “just an arm,” the same ingredients appear everywhere:
- kinematic chains,
- Jacobians,
- singularity structure,
- wrench mapping,
- dynamics,
- task-space vs joint-space control.

## 4. Force and contact tasks

Statically,

$$
\tau = J^T\gamma_e
$$

is the entry point to contact reasoning.
That is why Jacobians matter not only for moving the end-effector, but also for interacting with the world.

## 5. Redundancy resolution and optimization

As soon as a robot is redundant, nullspace methods become optimization machinery.

That is where this course touches modern topics like:
- secondary-objective IK,
- constrained optimization,
- whole-body control,
- manipulation under joint limits.

# Quick Reference

## Geometry and pose

| Task | Formula / idea |
|---|---|
| Rotation properties | $R^TR = I$, $R^{-1}=R^T$, $\det R = 1$ |
| Pose representation | $T$ |
| Compose poses | multiply transformations in frame order |

## Configuration and DOF

| Task | Formula / idea |
|---|---|
| Mechanism DOF | $\text{dof} = m(L-1-J)+\sum f_i$ |
| Configuration variables | $q = [q_1,\dots,q_n]^T$ |
| Redundancy test | task dimension smaller than joint dimension |

## Kinematics

| Task | Formula / idea |
|---|---|
| Forward kinematics | $T_0^e(q)=A_0^1A_1^2\cdots A_{n-1}^nT_n^e$ |
| Pose-level IK | solve $T_0^e(q)=T_d$ |
| Velocity kinematics | $v_e = J(q)\dot q$ |
| Revolute Jacobian column | $J_{P,i}=z_{i-1}\times(p_e-p_{i-1}),\; J_{O,i}=z_{i-1}$ |
| Prismatic Jacobian column | $J_{P,i}=z_{i-1},\; J_{O,i}=0$ |

## Singularity / redundancy

| Task | Formula / idea |
|---|---|
| Singularity | rank drop in $J(q)$ |
| Redundancy | $n>r$ for $J\in\mathbb{R}^{r\times n}$ |
| Nullspace dimension | $\dim N(J)=n-r$ |
| Nullspace projector | use $(I-J^+J)$ for secondary motion |

## Inverse velocity kinematics

| Task | Formula / idea |
|---|---|
| Square nonsingular case | $\dot q = J^{-1}v_e$ |
| General least-squares solution | $\dot q = J^+ v_e$ |
| Secondary objective | $\dot q = J^+v_e + (I-J^+J)z$ |
| Numerical IK | iterative Jacobian-based update |

## Statics and dynamics

| Task | Formula / idea |
|---|---|
| Wrench to joint torque | $\tau = J^T\gamma_e$ |
| Manipulator dynamics | $M(q)\ddot q + C(q,\dot q)\dot q + g(q)=\tau$ |
| Lagrangian idea | derive from kinetic and potential energy |
| Inertia matrix property | positive definite |

## Control

| Task | Formula / idea |
|---|---|
| Joint-level control | feedforward + feedback |
| Independent joint control | treat coupling as disturbance or partial compensation |
| Why it works | simple, robust, industrially practical |

# What to Never Forget

## 1. A manipulator is a geometry chain before it is a control problem.
Frames, rotations, and transformations come first.

## 2. FK is the forward map; IK is the inverse geometry problem.
One is direct. The other is nonlinear, multi-solution, and sometimes impossible.

## 3. The Jacobian is the local motion map.

$$
v_e = J(q)\dot q
$$

This one matrix controls velocity, singularity, redundancy, and force mapping.

## 4. Singularities are rank problems.
When rank drops, motion directions disappear and numerical trouble begins.

## 5. Redundancy is nullspace.
That extra freedom is not noise. It is design opportunity.

## 6. Statics is Jacobian transpose.

$$
\tau = J^T\gamma_e
$$

That duality is one of the cleanest facts in robotics.

## 7. Dynamics is where physics enters.

$$
M(q)\ddot q + C(q,\dot q)\dot q + g(q)=\tau
$$

Kinematics tells we what is geometrically possible. Dynamics tells we what is physically demanded.

## 8. Real robot control is layered.
Planning, kinematics, dynamics, and joint servo loops all stack together.

# 30-Years-Later Refresher

If future-me forgets everything, remember this skeleton:

1. Define the robot’s **configuration** and DOF.
2. Represent pose using **frames, rotations, and homogeneous transforms**.
3. Use **forward kinematics** to get end-effector pose from joint coordinates.
4. Use **inverse kinematics** to recover joint coordinates from desired pose.
5. Differentiate the motion to get **velocity kinematics**.
6. Package that local motion map into the **Jacobian**.
7. Use rank and nullspace to reason about **singularity and redundancy**.
8. Use the Jacobian transpose for **statics**.
9. Use energy-based modeling to derive **dynamics**.
10. Use layered servo design for **robot control**.

If that skeleton is alive, the rest can be rebuilt.

# Final Robotics Takeaway

This course teaches one deep lesson:

A robot arm is not “just a bunch of joints.”

It is:
- a constrained geometric chain,
- a differential motion map,
- a force-transmission structure,
- a dynamic mechanical system,
- and a controlled physical machine.

The real jump from beginner to roboticist happens when we stop seeing kinematics, Jacobians, dynamics, and control as separate chapters and start seeing them as one continuous pipeline.

That is what this course builds.

---

# Minimal Core Formula Sheet

$$
\text{dof} = m(L-1-J)+\sum f_i
$$

$$
R^TR = I, \qquad R^{-1}=R^T, \qquad \det R=1
$$

$$
T = \begin{bmatrix}R & p\\
0 & 1\end{bmatrix}
$$

$$
T_0^e(q)=A_0^1(q_1)A_1^2(q_2)\cdots A_{n-1}^n(q_n)T_n^e
$$

$$
v_e = J(q)\dot q
$$

$$
J_{P,i}=z_{i-1},\; J_{O,i}=0 \quad \text{(prismatic)}
$$

$$
J_{P,i}=z_{i-1}\times(p_e-p_{i-1}),\; J_{O,i}=z_{i-1} \quad \text{(revolute)}
$$

$$
\tau = J^T(q)\gamma_e
$$

$$
\dim R(J)=r, \qquad \dim N(J)=n-r
$$

$$
\dot q = J^+ v_e + (I-J^+J)z
$$

$$
M(q)\ddot q + C(q,\dot q)\dot q + g(q)=\tau
$$

---