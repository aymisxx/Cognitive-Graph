# **Linear Algebra in Engineering**

### **From First Principles to Expert Intuition**

**Focus:** durable understanding, not temporary exam memory  .

---

# Why this course matters in robotics

Linear algebra is not just "math machinery." It is the language of robotics.

If robotics is the science of representing, estimating, commanding, and transforming motion, then linear algebra is the operating system underneath it.

You see it everywhere:

- **State vectors** in dynamics and control.
- **Jacobian matrices** in manipulator kinematics.
- **Linearization** around equilibria.
- **Least-squares estimation** in calibration and perception.
- **Covariance matrices** in filtering.
- **Eigenvalues** in stability analysis.
- **SVD** in IK, vision, PCA, compression, denoising, low-rank modeling.
- **Orthogonal matrices** in rotations.
- **Matrix exponentials** in continuous-time system solutions.

So this course is not a side quest. It is core infrastructure.

# How to use this document

This file is meant to work at three speeds.

## Mode 1: 5-minute refresh

Read:
- Big Picture Map.
- Core Formula Sheet.
- Robotics Connections.
- Common Mistakes.

## Mode 2: 1-hour revision

Read module summaries in order:

1. Linear equations.
2. Vector spaces and subspaces.
3. Orthogonality, projections, least squares, QR.
4. Determinants.
5. Eigenvalues and eigenvectors.
6. Singular Value Decomposition.

## Mode 3: Deep rebuild

Read everything and work the mental checks and examples.

# Big Picture Map

At the highest level, this course answers six giant questions:

## 1. When does a linear system have a solution?

> That is the world of **Ax = b**, elimination, pivots, rank, and column space.

## 2. If there are solutions, how many are there?

That is the world of **nullspace**, free variables, and the complete solution:

$$
x = x_p + x_n
$$

where $x_p$ is one particular solution and $x_n \in N(A)$.

## 3. What geometric structure lives inside a matrix?

That is the world of the **four fundamental subspaces**:

- column space.
- nullspace.
- row space.
- left nullspace.

## 4. What if the system is not exactly solvable?

> That is the world of **orthogonality, projections, least squares, and QR**.

## 5. What internal directions does a matrix preserve?

That is the world of **eigenvalues and eigenvectors**.

## 6. How do we understand any matrix, even rectangular ones?

That is the world of **SVD**, the final boss with clean shoes.

# Notation and foundational objects

We work mostly over real numbers.

**Vector** in $\mathbb{R}^n$:

$$
x = \begin{bmatrix}x_1 \\ x_2 \\ \vdots \\ x_n\end{bmatrix}
$$

**Matrix** $A \in \mathbb{R}^{m \times n}$: maps $x \in \mathbb{R}^n$ to $Ax \in \mathbb{R}^m$

**Linear system:**

$$
Ax = b
$$

**Dot product:**

$$
x^T y
$$

**Norm:**

$$
\|x\| = \sqrt{x^T x}
$$

---

# Module 1: Linear Equations

## 1.1 What is a linear system really?

A linear system

$$
Ax = b
$$

is a set of linear constraints on unknowns.

Example:

$$
2x - y = 0,\quad -x + 2y = 3
$$

This can be written as:

```math
\begin{bmatrix}
2 & -1\\
-1 & 2
\end{bmatrix}
\begin{bmatrix}
x\\
y
\end{bmatrix}
=
\begin{bmatrix}
0\\
3
\end{bmatrix}
```

This equation has two interpretations.

### Row picture

Each equation is a geometric object:

- in 2D: a line.
- in 3D: a plane.

The solution is the intersection of those objects.

### Column picture
Write:

$$
x_1 a_1 + x_2 a_2 + \cdots + x_n a_n = b
$$

The question becomes:

**Can the columns of $A$ be combined to produce $b$?**

This column picture becomes much more important in high dimensions.

### Robotics meaning

- Row picture feels like constraints.
- Column picture feels like achievable outputs.

For a robot:

- columns of the Jacobian tell you which instantaneous motion directions are available.
- asking whether a task-space velocity is possible is literally asking whether that desired vector lies in the column space.

That is not poetry. That is the thing.

## 1.2 Linear combinations and independence

A **linear combination** of vectors $a,b$ is:

$$
\alpha a + \beta b
$$

A set of vectors is **linearly independent** if none can be formed from the others.

This matters because:

- independent columns create new directions.
- dependent columns do not add new capability.
- dependence is the root of singularity.

### Geometric intuition
- one independent vector spans a line.
- two independent vectors span a plane.
- three independent vectors in $\mathbb{R}^3$ span the whole 3D space.

### Robotics meaning

If Jacobian columns become dependent, the manipulator loses freedom in certain task directions. That is the kinematic singularity story in embryo.

## 1.3 Systematic solving: elimination

For hand solving and algorithmic solving, we use **Gaussian elimination**.

**Goal**:
transform $A$ into upper triangular form $U$, then solve with back-substitution.

### Why elimination works

We are not changing the solution set. We are only replacing equations by equivalent combinations of equations.

### Pivot

A **pivot** is the leading nonzero entry used to eliminate entries below it.

If a pivot position is zero:

- maybe swap rows.
- if no good row exists, the system may be singular.

### Singular outcomes

During elimination, singularity reveals itself as:

- **0 = nonzero**  
  inconsistent system, no solution.

- **0 = 0** in a redundant row  
  not enough independent equations, infinitely many solutions possible.

### Robotics meaning

This is exactly how numerical solvers figure out whether your linearized problem is healthy, ill-conditioned, or structurally degenerate.

## 1.4 Elimination matrices and LU factorization

Every elimination step can be represented by multiplying by an **elementary matrix**.

After enough elimination:

$$
E_k \cdots E_2 E_1 A = U
$$

Invert the eliminations:

$$
A = LU
$$

where:
- $L$ is lower triangular with ones on the diagonal.
- $U$ is upper triangular.

If row exchanges are needed:

$$
PA = LU
$$

where $P$ is a permutation matrix.

### Why LU matters
Once factored, solving many systems with the same $A$ but different $b$'s becomes cheap:
1. solve $Ly = Pb$.
2. solve $Ux = y$.

### Robotics meaning
This is huge in:
- repeated linear solves in MPC.
- simulation.
- finite element methods.
- optimization loops.
- bundle adjustment style systems.

## 1.5 Partial pivoting

In practice we often do **partial pivoting**:
swap the current row with one below that has the largest absolute pivot candidate.

Why?

- numerical stability.
- avoids dividing by tiny numbers.
- reduces floating-point disaster.

### Engineering truth

Theoretical solvability is one thing. Numerical solvability is another beast with sharper teeth.

## 1.6 Cost of direct solving

Dense Gaussian elimination costs roughly:

$$
\frac{2}{3}n^3
$$

operations for large $n$.

This becomes absurdly expensive for huge systems.

### Engineering implication

For large real robotic, fluid, SLAM, or optimization problems, you almost never want blind dense inversion. You exploit:
- sparsity.
- structure.
- iterative solvers.
- approximate methods.

Never casually invert giant matrices like a chaos goblin.

## 1.7 Matrix inverse

The inverse $A^{-1}$ exists if:

$$
A^{-1}A = AA^{-1} = I
$$

Then:

$$
x = A^{-1}b
$$

But the important mental correction is this:

### Mature viewpoint
The inverse is theoretically elegant, but computationally you almost always solve systems rather than explicitly compute the inverse.

### Invertibility means
- unique solution to $Ax=b$ for every $b$.
- no nonzero vector satisfies $Ax=0$.
- all pivots exist.
- determinant is nonzero.
- full rank for square matrix.

### Robotics meaning
Inverse appears everywhere conceptually:
- inverse kinematics.
- inverse dynamics.
- coordinate transforms.
- covariance propagation.

But numerically, direct inversion is often replaced by better structured solves or pseudoinverses.

## 1.8 Transpose, symmetry, and $LDL^T$

The transpose flips rows and columns:

$$
(A^T)_{ij} = A_{ji}
$$

A matrix is **symmetric** if:

$$
A^T = A
$$

For invertible symmetric matrices, a useful factorization is:

$$
A = LDL^T
$$

This matters because many matrices in engineering are symmetric:
- Hessians.
- covariance matrices.
- inertia-like constructions.
- normal equations $A^T A$.

---

# Module 2: Vector Spaces and Subspaces

---

## 2.1 What is a vector space?

A vector space is a set closed under:
- addition
- scalar multiplication

plus the usual algebraic rules.

Examples:
- $\mathbb{R}^2$, $\mathbb{R}^3$, $\mathbb{R}^n$
- sets of matrices of fixed size
- sets of functions satisfying linear conditions

A **subspace** is a nonempty subset that is itself a vector space.

### Fast test for subspace
A set is a subspace if it contains zero and is closed under:
- addition
- scalar multiplication

### Not every natural-looking set is a subspace
Examples of non-subspaces:
- half-planes not through origin
- invertible matrices
- sets defined by nonlinear constraints

### Robotics meaning
Subspaces are the correct language for:
- feasible motion directions
- constraint manifolds after linearization
- tangent spaces
- null motions
- observable and controllable directions in linear systems

---

## 2.2 The four fundamental subspaces

For $A \in \mathbb{R}^{m \times n}$, the four fundamental subspaces are:

### 1. Column space $C(A)$

All vectors of the form $Ax$.  
A subspace of $\mathbb{R}^m$.

It answers:
**Which right-hand sides $b$ are achievable?**

### 2. Nullspace $N(A)$
All vectors satisfying:

$$
Ax = 0
$$

A subspace of $\mathbb{R}^n$.

It answers:
**Which inputs produce zero output?**

### 3. Row space $C(A^T)$
Span of the rows of $A$.  
A subspace of $\mathbb{R}^n$.

### 4. Left nullspace $N(A^T)$
All vectors $y$ satisfying:

$$
A^T y = 0
$$

A subspace of $\mathbb{R}^m$.

## 2.3 Column space: solvability in one sentence

The most important solvability statement is:

$$
Ax=b \text{ is solvable if and only if } b \in C(A)
$$

That is the whole street law.

### Robotics meaning
If a desired task-space velocity, force, or measurement residual is outside the column space, the exact problem is impossible.

Then you do one of three things:
- relax the target.
- project it.
- solve least squares.

Which leads naturally to module 3.

## 2.4 Nullspace: hidden freedom

The nullspace contains motions invisible to the output map.

If $x_n \in N(A)$ and $x_p$ is a particular solution of $Ax=b$, then:

$$
x = x_p + x_n
$$

is also a solution.

So the complete solution is:

$$
\boxed{x = x_p + N(A)}
$$

### Robotics meaning
This is the heart of **redundancy resolution**.

For manipulator kinematics:

$$
J(q)\dot q = v
$$

If the robot is redundant, then:

$$
\dot q = \dot q_p + \dot q_n,\quad \dot q_n \in N(J)
$$

Interpretation:
- $\dot q_p$ achieves the task.
- $\dot q_n$ changes posture without changing end-effector instantaneous motion.

That is how posture optimization, joint limit avoidance, and secondary objectives sneak in through the back door like elegant criminals.

## 2.5 Reduced row echelon form and free variables

To understand nullspace and solution structure, reduce to **rref**.

Key facts:
- $R = \text{rref}(A)$ is unique.
- $A$ and $R$ have the same nullspace.
- pivot columns correspond to dependent equations on variables.
- non-pivot columns correspond to **free variables**.

### Special solutions
Set one free variable to 1, the others to 0, then solve for pivots.
These produce basis vectors for the nullspace.

### Memory anchor
- pivot variables are determined.
- free variables are chosen.
- nullspace dimension = number of free variables.

## 2.6 Rank

The **rank** $r$ of a matrix is the number of pivots.

Interpretations:
- number of independent columns.
- dimension of the column space.
- dimension of the row space.
- true number of independent constraints/output directions.

### Rank-nullity
For $A \in \mathbb{R}^{m \times n}$:

$$
\dim N(A) = n-r
$$

This is one of the most useful formulas in the whole course.

### Robotics meaning
For a Jacobian $J \in \mathbb{R}^{m \times n}$:

- rank tells you how many independent task-space directions can be generated.
- nullity $= n-r$ tells you how much instantaneous redundancy is left.

If rank drops unexpectedly, welcome to singularity town.

## 2.7 Basis and dimension

A **basis** is a set of vectors that is:
- linearly independent.
- spanning.

The **dimension** is the number of vectors in any basis.

### Mature mental model
Dimension is not "number of coordinates written down."
It is the number of independent directions actually available.

### Example
A matrix may have many columns, but if only 3 are independent, the subspace it spans is still 3-dimensional.

### Robotics meaning
This is the correct way to think about:
- degrees of freedom.
- effective task dimension.
- reduced-order state representations.
- constraint dimension.
- observability rank.

---

# Module 3: Orthogonality, Projections, Least Squares, QR

## 3.1 Inner product and orthogonality

The inner product is:

$$
x^T y = \sum_i x_i y_i
$$

Two vectors are orthogonal if:

$$
x^T y = 0
$$

### Why orthogonality is precious
Orthogonality means clean separation.
No overlap. No interference. No mixed contribution.

That makes geometry, estimation, and decomposition much cleaner.

## 3.2 Orthogonal subspaces

The famous relationships:

$$
C(A^T) \perp N(A)
$$

$$
C(A) \perp N(A^T)
$$

This is not just cute theory. It is a master key.

### Dimensional identities

$$
\dim C(A^T) + \dim N(A) = n
$$

$$
\dim C(A) + \dim N(A^T) = m
$$

### Deep interpretation
Input space splits into:
- useful part seen by $A$.
- invisible part killed by $A$.

Output space splits into:
- achievable outputs.
- impossible directions orthogonal to them.

## 3.3 Orthogonal complement

For a subspace $V$, its orthogonal complement $V^\perp$ is the set of all vectors orthogonal to everything in $V$.

If $V \subset \mathbb{R}^n$ has dimension $d$, then:

$$
\dim(V^\perp) = n-d
$$

Also, every vector $x$ can be decomposed as:

$$
x = v + w,\quad v \in V,\; w \in V^\perp
$$

This is exactly the logic behind projection.

## 3.4 Projection onto a line

If a line is spanned by vector $a$, the projection matrix is:

$$
P = \frac{aa^T}{a^T a}
$$

Projection of $b$ onto that line:

$$
Pb = a \frac{a^T b}{a^T a}
$$

### Key properties
- $P^2 = P$  
  projecting twice does nothing new.
- $I-P$ projects onto the perpendicular complement.

### Robotics meaning
Projection is everywhere:
- constrain motion to allowed directions.
- separate normal and tangential components.
- split force into contact-normal and tangential parts.
- separate task-space motion from nullspace motion.
- remove gravity direction or surface normal effects.

## 3.5 Projection onto a subspace

If columns of $A$ are linearly independent and span a subspace, then projection onto that subspace is:

$$
P = A(A^T A)^{-1}A^T
$$

### What this does
Given $b$, $Pb$ is the closest vector in $C(A)$ to $b$.

Residual:

$$
e = b - Pb
$$

lies in:

$$
N(A^T)
$$

This is the precise geometry behind least squares.

## 3.6 Least squares

When $Ax=b$ is unsolvable because $b \notin C(A)$, we solve instead:

$$
\min_x \|Ax-b\|
$$

This finds the best approximate solution.

### Normal equations
Least-squares solutions satisfy:

$$
A^T A \hat x = A^T b
$$

If columns of $A$ are independent:

$$
\hat x = (A^T A)^{-1}A^T b
$$

### Geometric interpretation
$A\hat x$ is the projection of $b$ onto $C(A)$.

Residual:

$$
e = b - A\hat x
$$

is orthogonal to the column space:

$$
A^T e = 0
$$

### Robotics meaning
Least squares is absolutely everywhere:
- overdetermined calibration.
- sensor fusion.
- parameter estimation.
- line/plane fitting.
- pose estimation.
- bundle adjustment.
- inverse kinematics with impossible exact target.
- SLAM linear subproblems.
- regression and learning.

It is one of the most employed ideas in engineering civilization.

## 3.7 Orthonormal bases and orthogonal matrices

A set of vectors is **orthonormal** if:
- each vector has norm 1.
- different vectors are orthogonal.

If matrix $Q$ has orthonormal columns:

$$
Q^TQ = I
$$

If $Q$ is square:

$$
Q^{-1} = Q^T
$$

### Meaning
Orthogonal matrices preserve:
- lengths.
- angles.

### Robotics meaning
Rotation matrices are orthogonal.  
That is why they preserve rigid-body geometry.

Whenever you see a valid 3D rotation matrix $R$, you expect:

$$
R^T R = I,\quad \det(R)=1
$$

If not, something is cooked.

## 3.8 Gram-Schmidt and QR factorization

Gram-Schmidt converts independent vectors into an orthonormal basis.

Given matrix $A$, this gives:

$$
A = QR
$$

where:
- $Q$: orthonormal columns.
- $R$: upper triangular.

### Why QR matters
It solves least squares more cleanly than normal equations.

Since:

$$
A = QR
$$

then:

$$
\min_x \|Ax-b\| = \min_x \|QRx-b\|
$$

and because $Q$ preserves norms nicely, the problem becomes cleaner.

### Engineering truth
Normal equations square the conditioning and can be numerically nasty. QR is often better.

### Robotics meaning
QR matters in:
- least-squares state estimation.
- numerically stable regression.
- orthonormal basis generation.
- factorization-based solvers.
- online estimation routines.

---

# Module 4: Determinants

## 4.1 What the determinant is

The determinant is a scalar attached to a square matrix.

It encodes:
- invertibility.
- orientation sign.
- volume scaling.

### High-value fact

$$
\det(A)=0 \iff A \text{ is singular}
$$

So determinant zero means:
- no inverse.
- rank deficiency.
- columns are dependent.
- volume collapsed into lower dimension.

## 4.2 Determinant properties worth remembering

Key rules:

- $\det(I)=1$.
- row exchange flips sign.
- triangular matrix determinant = product of diagonal entries.
- $\det(AB)=\det(A)\det(B)$
- $\det(A^T)=\det(A)$
- singular matrix has determinant zero.

### Pivot formula
If $PA=LU$, then determinant is basically signed product of pivots:

$$
\det(A)=\pm \prod_i u_{ii}
$$

The sign comes from the permutation.

## 4.3 Geometric meaning: volume scaling

If columns of $A$ define edges of a box or parallelepiped, then:

$$
|\det(A)|
$$

is the area/volume scaling factor.

### Interpretations
- $|\det(A)| > 1$: expansion.
- $|\det(A)| < 1$: contraction.
- $\det(A)=0$: collapse into lower dimension.
- negative sign: orientation flip.

### Robotics meaning
Determinant shows up in:
- change of variables.
- Jacobian volume scaling.
- manipulability intuition.
- local compression/expansion of maps.
- rigid transform sanity checks.

For rotation matrices:

$$
\det(R)=1
$$

For reflections:

$$
\det(R)=-1
$$

## 4.4 Cofactors and inverse formula

For small matrices:

$$
A^{-1} = \frac{C^T}{\det(A)}
$$

where $C$ is the cofactor matrix.

This is elegant for theory and small examples, but awful for large computation.

### Cramer's rule
For small systems:

$$
x_j = \frac{\det(A_j)}{\det(A)}
$$

where $A_j$ is matrix $A$ with column $j$ replaced by $b$.

Beautiful. Terrible at scale.

## 4.5 Mature viewpoint on determinants

Determinants are conceptually important.
They are not usually the best computational tool for solving large systems.

Use them for:
- theory.
- geometry.
- invertibility tests in small cases.
- proofs.
- symbolic work.

Do not treat them like the universal hammer.

---

# Module 5: Eigenvalues and Eigenvectors

## 5.1 The basic idea

An eigenvector of $A$ is a nonzero vector $x$ such that:

$$
Ax = \lambda x
$$

Interpretation:
$A$ acts on $x$ without changing its direction, only scaling it by $\lambda$.

That special direction is an eigenvector.  
That scaling is the eigenvalue.

### Why this matters
Most transformations mix directions.
Eigenvectors reveal the privileged directions the transformation respects.

## 5.2 Characteristic equation

To find eigenvalues:

$$
(A-\lambda I)x = 0
$$

For nonzero $x$, the matrix must be singular:

$$
\det(A-\lambda I)=0
$$

This is the **characteristic equation**.

Then for each eigenvalue $\lambda_i$, solve:

$$
(A-\lambda_i I)x = 0
$$

to get eigenvectors.

## 5.3 Useful fast facts

For small matrices:

- trace = sum of eigenvalues:

$$
\mathrm{tr}(A)=\sum_i \lambda_i
$$

- determinant = product of eigenvalues:

$$
\det(A)=\prod_i \lambda_i
$$

For triangular matrices, the eigenvalues are the diagonal entries.

That is a ridiculously useful shortcut.

## 5.4 Diagonalization

If a matrix has enough independent eigenvectors, collect them into columns of $S$:

$$
S = [x_1\; x_2\; \cdots \; x_n]
$$

Then:

$$
AS = S\Lambda
$$

and if $S$ is invertible:

$$
A = S\Lambda S^{-1}
$$

This is **diagonalization**.

### Meaning
A complicated transformation becomes:
1. change coordinates into eigenbasis.
2. scale independently by eigenvalues.
3. transform back.

This is gorgeous because powers and exponentials become easy.

## 5.5 When is diagonalization possible?

Not always.

If a matrix lacks enough independent eigenvectors, it is not diagonalizable.

### Distinct eigenvalues guarantee diagonalizability
If an $n \times n$ matrix has $n$ distinct eigenvalues, then it is diagonalizable.

### Defective matrix
A defective matrix has fewer eigenvectors than needed.

This matters because repeated eigenvalues do **not** automatically mean trouble, but they can.

## 5.6 Matrix powers

If:

$$
A = S\Lambda S^{-1}
$$

then:

$$
A^k = S\Lambda^k S^{-1}
$$

That means eigenvalues get powered:

$$
\lambda_i \mapsto \lambda_i^k
$$

### Why this matters
Repeated application of a system is controlled by eigenvalues.

### Robotics and dynamics meaning
For discrete-time systems:

$$
x_{k+1} = Ax_k
$$

long-term behavior depends on magnitudes of eigenvalues:
- $|\lambda|<1$: decays.
- $|\lambda|>1$: grows.
- $|\lambda|=1$: marginal/oscillatory possibilities.

That is control theory DNA.

## 5.7 Matrix exponential

Define:

$$
e^A = \sum_{k=0}^{\infty}\frac{A^k}{k!}
$$

If $x$ is an eigenvector of $A$ with eigenvalue $\lambda$, then it is also an eigenvector of $e^A$ with eigenvalue:

$$
e^\lambda
$$

If $A=S\Lambda S^{-1}$, then:

$$
e^A = S e^\Lambda S^{-1}
$$

### Why this matters
Continuous-time linear systems:

$$
\dot x = Ax
$$

have solution:

$$
x(t)=e^{At}x(0)
$$

### Robotics meaning
This underlies:
- linear system response.
- continuous-time state transition.
- local stability reasoning.
- rigid motion on Lie groups in more advanced settings.
- discretization of continuous models.

## 5.8 Spectral theorem

If $A$ is real and symmetric:

$$
A^T=A
$$

then:
- all eigenvalues are real.
- eigenvectors can be chosen orthonormal.
- $A$ is diagonalizable as:

$$
A = Q\Lambda Q^T
$$

This is massively important.

### Why symmetric matrices are elite
They behave cleanly.
No nonsense.
No complex drama unless you ask for it.

### Positive definite and semidefinite
If symmetric and all eigenvalues are:
- $>0$: positive definite.
- $\ge 0$: positive semidefinite.

### Robotics meaning
Positive definite matrices show up everywhere:
- covariance matrices.
- quadratic costs.
- inertia-like matrices.
- Hessians near minima.
- energy functions.
- $A^TA$ constructions.

If your cost Hessian is positive definite, the problem is locally bowl-shaped instead of cursed.

## 5.9 Stability intuition

For linear ODEs and difference equations, eigenvalues tell the long-term story.

### Continuous time

$$
\dot x = Ax
$$

- negative real parts: decay.
- positive real parts: growth/instability.
- imaginary parts: oscillation.

### Discrete time

$$
x_{k+1}=Ax_k
$$

- eigenvalues inside unit circle: stable decay.
- outside: unstable.
- on unit circle: marginal, structure-dependent.

### Robotics meaning
This is the mathematical backbone of:
- linear control stability.
- observer stability.
- local dynamics near equilibrium.
- vibration modes.
- oscillation analysis.

---

# Module 6: Singular Value Decomposition

## 6.1 Why SVD exists

Eigenvalue decomposition is beautiful, but it needs a square matrix and behaves especially well for symmetric matrices.

Real engineering matrices are often:
- rectangular.
- nonsymmetric.
- rank deficient.
- noisy.

SVD handles all of them.

For any $A \in \mathbb{R}^{m \times n}$:

$$
A = U\Sigma V^T
$$

where:
- $U$: left singular vectors, orthonormal.
- $V$: right singular vectors, orthonormal.
- $\Sigma$: diagonal-like matrix of singular values $\sigma_1 \ge \sigma_2 \ge \cdots \ge 0$.

This factorization is insanely powerful.

## 6.2 What singular values and vectors mean

Right singular vectors $v_i$ are orthonormal eigenvectors of:

$$
A^T A
$$

Left singular vectors $u_i$ are orthonormal eigenvectors of:

$$
AA^T
$$

Singular values satisfy:

$$
A v_i = \sigma_i u_i
$$

### Intuition
SVD says any linear map can be understood as:
1. rotate/reflect in input space via $V^T$.
2. scale along orthogonal axes via $\Sigma$.
3. rotate/reflect in output space via $U$.

That is one of the cleanest structural descriptions in all of applied math.

## 6.3 Rank and SVD

The rank of $A$ equals the number of nonzero singular values.

If rank is $r$, then:

$$
A = \sum_{i=1}^{r}\sigma_i u_i v_i^T
$$

So any rank-$r$ matrix is a sum of $r$ rank-1 outer products.

### This is huge

It means the matrix is built from weighted mode pairs.

## 6.4 Low-rank approximation

If singular values decay quickly, then:

$$
A \approx \sum_{i=1}^{k}\sigma_i u_i v_i^T
\quad\text{for }k \ll r
$$

This gives a compressed approximation.

### Why it works
Later singular values correspond to weaker modes.
Throwing them away often preserves most of the important structure.

### Applications
- image compression.
- denoising.
- latent structure discovery.
- model reduction.
- memory savings.
- fast approximate computation.

### Robotics meaning
Low-rank ideas appear in:
- reduced-order modeling.
- perception embeddings.
- dimensionality reduction.
- compression of maps or features.
- latent dynamics models.
- subspace identification.

## 6.5 Pseudoinverse

Most matrices do not have an inverse.
SVD gives the correct generalization.

If:

$$
A = U\Sigma V^T
$$

then pseudoinverse is:

$$
A^+ = V\Sigma^+ U^T
$$

where $\Sigma^+$ replaces every nonzero $\sigma_i$ by $1/\sigma_i$.

### Why pseudoinverse matters
It gives:
- least-squares solutions.
- minimum norm solutions.
- generalized inversion for rectangular systems.

### Key facts
- if $A$ is invertible square, then $A^+=A^{-1}$.
- $AA^+$ projects onto column space.
- $A^+A$ projects onto row space.

### Robotics meaning
This is one of the most important formulas in robotics.

For inverse kinematics:

$$
\dot q = J^+ v
$$

This gives the least-squares joint velocity producing desired task velocity when exact inversion is not available.

If redundant:
- pseudoinverse gives minimum norm solution
- nullspace term adds secondary behavior:

$$
\dot q = J^+v + (I-J^+J)z
$$

That single formula basically runs half of academic robotics.

## 6.6 Least squares via pseudoinverse

For unsolvable $Ax=b$, a least-squares solution is:

$$
\hat x = A^+ b
$$

This remains valid even when columns are dependent, where normal equations may become singular or awkward.

### Robotics meaning
This is the practical version of:
- IK under redundancy or inconsistency.
- overconstrained calibration.
- sensor alignment.
- data fitting.
- state estimation.

## 6.7 PCA and data interpretation

PCA is built from centered data and SVD.

Goal:
find orthogonal directions of maximum variance.

### Pipeline
1. collect data matrix.
2. center the features.
3. compute SVD.
4. use top singular vectors as principal axes.
5. project data into lower-dimensional coordinates.

### Interpretation
- large singular values = strong variation directions.
- small singular values = weak variation directions.

### Why PCA matters
It finds structure that raw data tables hide.

### Robotics meaning
PCA matters in:
- dimensionality reduction.
- feature compression.
- latent state analysis.
- perception preprocessing.
- anomaly detection.
- map/trajectory embedding.
- exploratory data analysis.

# The Full Concept Chain

This course is easiest to remember as a logical ladder.

## Ladder

1. Start with **Ax = b**.
2. Ask if solution exists -> **column space**.
3. Ask how many solutions -> **nullspace**.
4. Ask structure -> **rank, basis, dimension**.
5. Ask geometry of impossibility -> **orthogonality**.
6. Ask best approximate solution -> **projection and least squares**.
7. Ask numerically stable orthogonal structure -> **QR**.
8. Ask when square map is invertible and volume-preserving/scaling -> **determinant**.
9. Ask intrinsic preserved directions -> **eigenvalues/eigenvectors**.
10. Ask dynamics and repeated application -> **diagonalization and matrix exponential**.
11. Ask for universal factorization for any matrix -> **SVD**.
12. Ask for generalized inverse and modern data tools -> **pseudoinverse and PCA**.

That is the whole course arc.

---

# Robotics Connection Sheet

## A. Manipulator Jacobian

$$
v = J(q)\dot q
$$

- column space of $J$: achievable task velocities.
- nullspace of $J$: self-motions.
- rank of $J$: effective task dimension.
- singularity: rank drop.
- pseudoinverse: least-squares IK.
- SVD of $J$: manipulability directions and weakness modes.

## B. Linear control systems

$$
\dot x = Ax + Bu
$$

- eigenvalues of $A$: local stability and oscillation.
- matrix exponential $e^{At}$: state transition.
- rank ideas: controllability/observability frameworks later on.
- symmetric positive definite matrices: Lyapunov and quadratic cost structures.

## C. Estimation and filtering
- least squares for parameter fitting.
- covariance matrices are symmetric positive semidefinite.
- orthogonality of residuals matters.
- QR and SVD improve numerical stability.
- PCA can reduce feature/state dimension.

## D. Computer vision and perception
- SVD for compression, denoising, low-rank structure.
- least squares for fitting lines, planes, homographies, poses.
- PCA for embeddings and compact representations.
- orthogonal matrices for rotations.
- projection matrices for geometry and subspace reasoning.

## E. Optimization
- Hessian structure.
- positive definiteness.
- conditioning.
- linear solves instead of inverse.
- nullspace methods for constrained optimization.

# What to Never Forget

## 1. Solvability

$$
Ax=b \iff b \in C(A)
$$

## 2. Complete solution

$$
x = x_p + x_n,\quad x_n \in N(A)
$$

## 3. Rank-nullity

$$
\dim N(A)=n-r
$$

## 4. Orthogonality relations

$$
C(A^T)\perp N(A),\quad C(A)\perp N(A^T)
$$

## 5. Projection onto column space

$$
P=A(A^TA)^{-1}A^T
$$

when columns are independent

## 6. Least squares

$$
A^TA\hat x=A^Tb
$$

## 7. Orthogonal matrix

$$
Q^TQ=I,\quad Q^{-1}=Q^T
$$

## 8. Determinant and singularity

$$
\det(A)=0 \iff A \text{ singular}
$$

## 9. Eigenvalue definition

$$
Ax=\lambda x
$$

## 10. Diagonalization

$$
A=S\Lambda S^{-1}
$$

## 11. Continuous-time solution

$$
x(t)=e^{At}x(0)
$$

## 12. SVD

$$
A=U\Sigma V^T
$$

## 13. Pseudoinverse

$$
A^+=V\Sigma^+U^T
$$

## 14. IK-style minimum norm solution

$$
\dot q = J^+ v
$$

---

# Intuition Anchors by Topic

## Linear system
A matrix is a machine.
It takes input directions and manufactures output directions.

## Column space
The machine's menu of possible outputs.

## Nullspace
Inputs the machine swallows without visible output.

## Rank
How many genuinely different output directions the machine can create.

## Orthogonality
Perfect separation of influence.

## Projection
Closest legal version of an illegal request.

## Least squares
When reality says "exactly no," but still offers the best possible compromise.

## Determinant
How much volume the machine inflates, shrinks, flips, or crushes.

## Eigenvector
A lucky direction that survives the machine without changing direction.

## Eigenvalue
How much that lucky direction is stretched.

## Diagonalization
Rewrite the machine in its favorite coordinates.

## SVD
Universal x-ray of a matrix.

## Pseudoinverse
The inverse's smarter cousin who works even when life is messy.

---

# 30-Years-Later Refresher

If future-me forgot almost everything, remember this skeleton:

1. **Ax = b** is the core question.
2. Exact solvability depends on **column space**.
3. Non-uniqueness depends on **nullspace**.
4. Number of independent directions is **rank**.
5. Impossible targets are handled by **projection**.
6. Best approximate solutions come from **least squares**.
7. Clean coordinate systems come from **orthogonality** and **QR**.
8. Invertibility and volume are tied to **determinant**.
9. Preserved directions are **eigenvectors**.
10. Dynamics and repeated action are controlled by **eigenvalues** and **matrix exponential**.
11. The all-purpose decomposition is **SVD**.
12. The practical inverse for robotics is often the **pseudoinverse**.

If this skeleton is alive, the rest can be rebuilt.

---

# Final Robotics Takeaway

This course teaches one deep lesson:

A matrix is not just a table of numbers.

It is:
- a map between spaces,
- a geometry transformer,
- a constraint engine,
- a mode selector,
- a stability storyteller,
- and a numerical personality.

The real jump from beginner to expert is when you stop seeing matrices as arithmetic objects and start seeing them as structured linear operators with geometry, algebra, rank, modes, and computational consequences.

That is when linear algebra stops being a subject and starts becoming instinct.

---

# Minimal Core Formula Sheet

$$
Ax=b
$$

$$
Ax=b \iff b\in C(A)
$$

$$
N(A)=\{x:Ax=0\}
$$

$$
x = x_p + x_n,\quad x_n \in N(A)
$$

$$
\text{rank}(A)=r
$$

$$
\dim N(A)=n-r
$$

$$
C(A^T)\perp N(A),\quad C(A)\perp N(A^T)
$$

$$
P_{\text{line}}=\frac{aa^T}{a^Ta}
$$

$$
P_{\text{subspace}}=A(A^TA)^{-1}A^T
$$

$$
A^TA\hat x=A^Tb
$$

$$
A=QR
$$

$$
\det(A)=0 \iff A \text{ singular}
$$

$$
Ax=\lambda x
$$

$$
\det(A-\lambda I)=0
$$

$$
A=S\Lambda S^{-1}
$$

$$
A^k=S\Lambda^kS^{-1}
$$

$$
e^{At}=Se^{\Lambda t}S^{-1}
$$

$$
A=Q\Lambda Q^T \quad \text{for symmetric } A
$$

$$
A=U\Sigma V^T
$$

$$
A^+=V\Sigma^+U^T
$$

$$
\dot q = J^+ v + (I-J^+J)z
$$

---

# End Note

This is not "just linear algebra."

It's the grammar of robotics and AI.

---