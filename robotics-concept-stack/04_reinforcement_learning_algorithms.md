# **Reinforcement Learning Algorithms**

### **From Bellman Intuition to Modern Continuous Control**

**Focus:** revision that preserves conceptual structure, practical workflow, and robotics relevance.

---

# Why this course matters in robotics

Reinforcement learning is not just “an agent gets rewards.”

It is a control-and-decision framework for situations where:
- the system evolves over time,
- decisions affect future opportunities,
- outcomes are delayed,
- the environment may be uncertain,
- and hand-designed control logic becomes brittle or impossible.

That makes RL deeply relevant to robotics.

You see it in:
- **locomotion** where actions affect long-horizon stability,
- **manipulation** where grasp success depends on sequential choices,
- **autonomous navigation** under uncertainty,
- **drone control** with high-dimensional observation and action spaces,
- **humanoid control** where exact dynamic programming is impossible,
- **offline robotic policy learning** from logged transitions,
- **continuous control** where actor-critic methods dominate.

This course matters because it connects three worlds that are usually taught separately:
- **optimal control / dynamic programming**,
- **machine learning / function approximation**,
- **robot control under uncertainty**.

The real lesson is that RL is not magic. It is structured approximation of optimal sequential decision-making.

# How to use this document

This file is meant to work at three speeds.

## Mode 1: 5-minute refresh

Read:
- Big Picture Map.
- Core Formula Sheet.
- Robotics Connection Sheet.
- What to Never Forget.

## Mode 2: 1-hour revision

Read module summaries in order:

1. policy, value, and MDP intuition.
2. Bellman equation and dynamic programming.
3. tabular Q-learning and SARSA.
4. deep Q-learning and function approximation.
5. failure modes: exploration, overestimation, variance.
6. actor-critic and continuous control.
7. DDPG, TD3, and SAC.
8. evaluation and implementation workflow.

## Mode 3: Deep rebuild

Read everything and mentally connect:
- Bellman recursion,
- bootstrapping,
- approximation,
- exploration,
- critic stability,
- actor learning,
- robotics deployment constraints.

# Big Picture Map

At the highest level, this course answers eight giant questions.

## 1. What is the agent actually trying to learn?

> A **policy** for choosing actions, and some notion of **value** that tells how good states or actions are.

## 2. Why is RL a sequential decision problem instead of a one-step prediction problem?

Because current action changes future state, future reward, and future decisions.

## 3. What is the master equation behind optimal decision-making?

> The **Bellman equation** and the **principle of optimality**.

## 4. Why can’t we just solve everything exactly with dynamic programming?

Because of the **curse of dimensionality**.

## 5. How do we learn values directly from interaction data without a model?

> With **temporal-difference learning**, **Q-learning**, and **SARSA**.

## 6. What happens when the state or action space becomes huge or continuous?

> Replace tables with **function approximation**, usually neural networks.

## 7. Why do deep RL methods become unstable?

Because of some cursed combination of:
- bootstrapping,
- off-policy learning,
- function approximation,
- exploration noise,
- bias and variance.

## 8. How do modern robotics methods handle continuous control?

> With **actor-critic methods**, especially **DDPG**, **TD3**, **SAC**, and related policy-gradient methods.

# Notation and foundational objects

We work in the reinforcement learning setting of sequential interaction.

**State:**

$$
s \in \mathcal{S}
$$

**Action:**

$$
a \in \mathcal{A}
$$

**Reward:**

$$
r_t
$$

**Policy:**

$$
\pi(a \mid s)
$$

**Discount factor:**

$$
\gamma \in [0,1)
$$

**State-value function:**

$$
V^{\pi}(s)
$$

**Action-value function:**

$$
Q^{\pi}(s,a)
$$

**Optimal action-value function:**

$$
Q^*(s,a)
$$

**One-step transition tuple:**

$$
(s_t, a_t, r_{t+1}, s_{t+1})
$$

**Objective:** maximize expected discounted return.

---

# Module 1: Reinforcement Learning Foundations

## 1.1 What reinforcement learning is really about

Reinforcement learning studies how an agent should act through repeated interaction with an environment in order to maximize long-term reward.

That sounds innocent. It is not.

Unlike supervised learning:
- labels are not handed to you,
- data depends on your own actions,
- consequences are delayed,
- and exploration can hurt before it helps.

The class notes frame RL around two basic ingredients:
- **policy generation**: what action should be taken in a situation,
- **value estimation**: how good a state or action is. Those are presented right at the introduction of RL.

## 1.2 Policy: the agent’s strategy

A policy is the rule for choosing actions.

A **stochastic policy** is a conditional distribution over actions given a state:

$$
\pi(a\mid s)=\Pr(A_t=a\mid S_t=s)
$$

A **deterministic policy** picks one action for each state:

$$
\pi(s)=a
$$

That distinction matters later:
- DDPG and TD3 use deterministic actors,
- SAC uses stochastic actors,
- Q-learning usually induces a greedy deterministic target policy,
- exploration often adds stochasticity on top. The notes formalize this policy definition in the early modules.

## 1.3 Value: why “good now” is not enough

RL is not greedy one-step optimization.

A good action is good because of the future it opens.

That is why we use value functions:
- **state value** says how good it is to be in a state,
- **action value** says how good it is to take a specific action in a state.

In practice, the course leans heavily on **Q-functions**, because once we know a good approximation of

$$
Q(s,a),
$$

we can act greedily by choosing the action with highest value.

## 1.4 The motivating examples are not baby examples

The deterministic routing example and the 3-by-4 gridworld are doing a lot of hidden work.

They teach:
- policy vs value,
- deterministic vs stochastic transitions,
- effect of reward design,
- how penalties reshape behavior,
- why uncertainty changes the optimal route.

That gridworld section is a miniature of real robotics:
- walls become constraints,
- traps become unsafe states,
- action slippage becomes stochastic dynamics,
- reward shaping becomes controller preference design. The notes explicitly contrast deterministic and stochastic gridworlds, and show how reward magnitudes change the optimal policy.

## 1.5 Robotics meaning

In robotics, RL is not just “try actions and get points.”

It is often:
- **closed-loop control under uncertain dynamics**,
- **policy learning when exact models are weak or unavailable**,
- **high-dimensional decision-making from sensors**,
- **learning behavior that trades short-term sacrifice for long-term success**.

That is why the course begins with simple examples. They are the clean version of a very messy engineering problem.

# Module 2: The Bellman Equation and Dynamic Programming

## 2.1 Bellman’s real insight

The Bellman equation is the backbone of the course.

Its core principle is the **principle of optimality**:

> if a policy is optimal, then the remaining decisions after the first action must also be optimal for the resulting state.

This is what makes sequential problems recursively solvable. The class notes make this the conceptual center of Module 3.

## 2.2 Why recursion appears naturally

Suppose a state can lead to several next states.

Then the optimal cost-to-go or reward-to-go from the current state is:
- immediate reward/cost of taking an action,
- plus the optimal future value from the next state.

That recursive structure is Bellman.

For cost minimization language, the notes write the routing problem using immediate cost plus optimal remaining cost-to-go, then minimize over feasible next states. 

## 2.3 Dynamic programming

Dynamic programming takes Bellman recursion and turns it into a systematic computational method.

The notes emphasize that DP:
- breaks a multistage problem into single-stage problems,
- works backward from the end,
- produces an optimal closed-loop policy,
- and is fundamentally an **offline** method when the model is known.

## 2.4 Why DP is both beautiful and doomed

DP is beautiful because it gives exact optimal solutions when:
- state space is manageable,
- action space is manageable,
- transitions/rewards are known,
- and discretization is acceptable.

DP is doomed because those conditions collapse fast in realistic robotics.

The notes explicitly quantify the combinatorial explosion: once state dimension and action dimension grow, state-action evaluations explode rapidly. They even point out how absurd this becomes for DMC-style humanoid settings with high-dimensional observations and actions.

## 2.5 Curse of dimensionality

This phrase is not academic decoration.

It means the computational burden scales exponentially enough that exact DP becomes impossible long before the real problem becomes interesting.

This is the bridge from classical optimal control to RL:
- exact Bellman solution is ideal,
- but exact representation is impossible,
- so we learn or approximate instead.

## 2.6 Robotics meaning

DP is still conceptually essential in robotics even when it is computationally dead on arrival.

Why?

Because RL algorithms are basically trying to approximate Bellman-consistent solutions without solving the full exact DP problem.

Bellman is the north star.

Approximation is the survival mechanism.

# Module 3: Tabular Q-Learning

## 3.1 What Q-learning learns

Q-learning learns the optimal action-value function directly: the best expected discounted return obtainable from taking action $a$ in state $s$ and then acting optimally thereafter.

Once that function is known, the policy simply picks the action with the largest value at each state.

In plain form:

`a_star(s) = argmax_a Q(s,a)`

and the policy applies that choice at state $s$.

The class notes define Q-learning as model-free and off-policy, and connect it directly to the Bellman optimality fixed point.

## 3.2 Bellman optimality equation for Q

The target object is the Bellman optimality relation for the action-value function.

In plain form:

`Q_star(s,a) = E[ r + gamma * max over a' of Q_star(s',a') | s,a ]`

This says:
- take action now,
- get immediate reward,
- then assume perfect greedy continuation afterward.

That is why Q-learning is an off-policy control method.

## 3.3 The update rule

Tabular Q-learning updates one entry at a time:

$$
Q_{t+1}(s_t,a_t)
\leftarrow
Q_t(s_t,a_t)+\alpha_t\Big[r_{t+1}+\gamma\max_{a'}Q_t(s_{t+1},a')-Q_t(s_t,a_t)\Big]
$$

This is a bootstrapped update:
- part old estimate,
- part one-step target.

The notes present this exact structure and emphasize its affine-combination intuition.

## 3.4 Why Q-learning is model-free

It does not require:
- transition probabilities,
- explicit reward model,
- a known simulator equation inside the update.

It only needs sampled experience tuples.

That is a massive reason RL became practical for complex environments.

## 3.5 Why Q-learning is off-policy

The target uses:

$$
\max_{a'} Q(s',a')
$$

not the action actually taken by the behavior policy.

So the algorithm can:
- behave with exploration,
- but learn about the greedy target policy.

That matters for:
- experience replay,
- learning from logged data,
- offline RL ideas,
- decoupling exploration from the target objective. The notes explicitly discuss offline learning from transition tuples as a natural extension of off-policy RL.

## 3.6 Convergence in the tabular world

In finite MDPs, tabular Q-learning converges to the optimal Q-function if:
- every state-action pair is visited infinitely often,
- learning rates satisfy Robbins-Monro conditions,
- discount factor is less than 1.

This result is beautiful and important.

It is also fragile outside the tabular setting.

## 3.7 Practical tabular workflow

The notes give the basic online recipe:
1. initialize Q-values,
2. choose an action using exploration such as epsilon-greedy,
3. execute action and observe reward and next state,
4. update Q,
5. decay exploration and maybe learning rate,
6. repeat until values stabilize.

## 3.8 Terminal states matter

For terminal transitions, the bootstrap target collapses to just the reward:

$$
y=r
$$

because there is no future action to evaluate.

This tiny implementation detail is conceptually small and practically huge.

## 3.9 Pitfalls

The notes already warn about the two classic monsters:
- **poor exploration**,
- **Q-value overestimation**.

These are not minor bugs.

They are structural failure modes.

## 3.10 Robotics meaning

Tabular Q-learning itself is usually too small-scale for serious robotics.

But conceptually it teaches almost everything that survives later:
- Bellman bootstrapping,
- greedy improvement,
- off-policy learning,
- exploration scheduling,
- terminal handling,
- value estimation as the basis of control.

# Module 4: SARSA and On-Policy Learning

## 4.1 Why SARSA exists

SARSA updates using the actual next action taken under the current policy:

$$
Q(s_t,a_t)
\leftarrow
Q(s_t,a_t)+\alpha\Big[r_{t+1}+\gamma Q(s_{t+1},a_{t+1})-Q(s_t,a_t)\Big]
$$

The name comes from the update sequence:
- state,
- action,
- reward,
- next state,
- next action. The notes spell this out explicitly.

## 4.2 On-policy vs off-policy

This distinction is one of the most important conceptual splits in RL.

### SARSA
Learns the value of the policy it is currently following.

### Q-learning
Learns the value of a greedy target policy while possibly behaving differently.

So:
- SARSA is **on-policy**,
- Q-learning is **off-policy**.

## 4.3 Why SARSA can be safer

Because SARSA includes exploratory behavior in its own value updates, it can prefer less risky strategies when exploration itself can cause accidents.

The notes explicitly mention the intuition that Q-learning may favor a risky shortest path, while SARSA accounts for the consequences of exploratory slips.

That is not just a classroom curiosity.

That is a robotics safety lesson.

## 4.4 What survives conceptually

This module forces you to understand:
- what policy is being evaluated,
- whether behavior policy equals target policy,
- why exploration changes the learned values,
- and why “optimal in theory” is not always “safe during learning.”

## 4.5 Robotics meaning

In real robotic learning, the on-policy vs off-policy split affects:
- sample reuse,
- logged-data compatibility,
- safety during exploration,
- stability of learning pipelines,
- data efficiency.

It also determines which algorithm families are practical under hardware constraints.

# Module 5: From Tables to Neural Networks

## 5.1 Why tables die

Tabular methods assume we can store explicit values for every state-action pair.

That collapses immediately when:
- states are continuous,
- actions are continuous,
- observations are images,
- robot sensors are high-dimensional,
- or state space is just too large.

The notes make this jump very explicit: robotics problems often involve continuous states and controls, so Q-tables are infeasible.

## 5.2 Function approximation replaces the table

We instead learn:

$$
Q_\theta(s,a) \approx Q^*(s,a)
$$

where the parameters $\theta$ are neural network weights.

Now value learning becomes a supervised-style regression problem built on bootstrapped targets.

## 5.3 Training target and loss

For a transition $(s,a,r,s')$, a standard one-step target is:

$$
y=r+\gamma\max_{a'}Q_{\theta^-}(s',a')
$$

and the loss is usually:

$$
L(\theta)=\big(y-Q_\theta(s,a)\big)^2
$$

The notes repeatedly frame deep Q-learning as learning from regression targets built from Bellman-style bootstrapping.

## 5.4 Replay buffer

Experience replay stores transition tuples and samples them later in random mini-batches.

Why it matters:
- breaks strong temporal correlation,
- improves data reuse,
- stabilizes updates,
- makes off-policy learning practical.

## 5.5 Target network

The target network is a delayed or slowly updated copy of the Q-network.

Why it matters:
- if the target moves too fast, learning becomes self-chasing chaos,
- delayed targets stabilize Bellman regression.

The notes identify replay and target networks as the core DQN stabilizers.

## 5.6 Choice of network

The notes point out the basic architecture logic:
- low-dimensional vector state -> MLP,
- image state -> CNN.

That is the first mature lesson in representation:

RL algorithm alone is not enough.

Input structure matters.

## 5.7 Why this is still off-policy

Even with neural nets, the underlying Q-learning flavor remains off-policy if the targets are greedy and updates learn from replayed tuples.

So deep Q-learning inherits both:
- the strengths of off-policy reuse,
- the instability risks of off-policy bootstrapping with approximation.

## 5.8 Robotics meaning

This transition from table to approximator is the moment RL becomes relevant to actual robots.

Without function approximation:
- no humanoid control,
- no visual navigation,
- no torque policy over continuous sensors,
- no learned manipulation from image streams.

With function approximation:
- scale becomes possible,
- stability becomes the new boss fight.

# Module 6: Deep Q-Networks and Training Mechanics

## 6.1 DQN as the default deep-Q template

DQN is the template where:
- a neural network approximates Q-values,
- replay buffer stores experience,
- target network stabilizes targets,
- training minimizes TD-style regression loss.

The notes treat DQN as the canonical example of Q-function learning with neural nets.

## 6.2 Two output styles for Q networks

For discrete actions, a network can take state as input and output one Q-value per action.

For more general formulations, the network can take the state-action pair as input and output a scalar Q-value.

The notes mention both styles explicitly, using gridworld for the discrete-output version and the state-action scalar version as the more general form.

## 6.3 Exploration is not optional

A good Q-network trained on bad coverage learns confident nonsense.

That is why the notes repeatedly stress:
- epsilon-greedy exploration,
- sufficient state-action coverage,
- dataset quality in offline learning,
- and exploration shutdown during evaluation.

## 6.4 Evaluation is part of the algorithmic discipline

The notes are unusually practical here.

Evaluation should:
- disable exploration,
- freeze network weights,
- use multiple seeds,
- average episodic return over rollouts,
- plot learning curves,
- inspect bias/variance behavior.

That is not paperwork.

That is how you separate “it trained once” from “it actually works.”

## 6.5 Loss functions in the RL context

The course also detours into loss functions for a reason.

### MSE
Good default regression loss, sensitive to large errors.

### MAE
More robust to outliers, but less smooth.

### Huber
Hybrid of MSE and MAE, often better for noisy RL targets.

### Cross-entropy
Important in classification and in policy-distribution learning; also helps build intuition for entropy terms used later in SAC. The notes explicitly connect cross-entropy intuition to entropy regularization in RL.

## 6.6 Robotics meaning

In robotics, training mechanics matter as much as the algorithm name.

A pretty acronym with:
- poor replay quality,
- bad reward scaling,
- no seed averaging,
- and no evaluation discipline

is just expensive self-deception.

# Module 7: Why Deep Q-Learning Breaks

## 7.1 The deadly triad

The notes mention the classic deadly triad:
- **off-policy learning**,
- **bootstrapping**,
- **function approximation**.

Each one is useful.

Together they can become a spectacular instability machine.

## 7.2 Overestimation bias

The notes give a clean explanation.

If multiple actions have noisy estimated values and you take a max over them, the max tends to prefer upward noise.

So even if each estimate is unbiased individually, the max operation creates upward bias in the target.

That becomes inflated Q-values, overconfident policies, and unstable learning.

## 7.3 Variance problem

Even if bias is controlled, high variance in Q estimates is still destructive.

The notes emphasize sources of variance:
- stochastic environment transitions,
- noisy rewards,
- bootstrapped targets,
- exploration randomness,
- mini-batch sampling,
- imperfect neural approximation.

And the key identity is the usual bias-variance decomposition of mean squared error.

## 7.4 Why this matters for actor-critic too

Noisy critics do not stay politely isolated.

If the actor uses critic gradients, critic noise contaminates policy learning.

Then:
- actor oscillates,
- critic oscillates,
- both may diverge together.

That is why critic quality is life-or-death in modern continuous control.

## 7.5 Remedies introduced in the notes

The notes mention several stabilization ideas:
- Double DQN,
- twin critics in TD3,
- averaging / ensembles,
- entropy regularization in SAC,
- Polyak averaging / soft targets,
- n-step methods,
- target policy smoothing.

These are not random bag-of-tricks hacks.

They are targeted interventions against bias, variance, and moving-target instability.

## 7.6 Robotics meaning

In robotics, unstable value learning is not just ugly curves.

It means:
- hardware risk,
- poor sample efficiency,
- fragile deployment,
- reward hacking into unsafe behavior,
- and policies that look smart until the world deviates slightly.

# Module 8: Actor-Critic Reinforcement Learning

## 8.1 Why actor-critic appears

Deep Q-learning struggles badly when actions are continuous, because computing

$$
\max_{a'}Q(s',a')
$$

becomes hard or impossible.

Actor-critic solves this by splitting the job.

### Actor
Represents the policy.

### Critic
Estimates how good actions or states are.

The notes introduce actor-critic as the major bridge from discrete-action RL toward real continuous-control robotics.

## 8.2 What the actor does

The actor maps state to action, either:
- deterministically, or
- stochastically.

It is the control law.

## 8.3 What the critic does

The critic estimates expected return under the current policy.

In practice it is trained by minimizing TD/Bellman-style error using neural regression, just like value learning before.

## 8.4 Policy gradient theorem

The notes give the classical policy gradient expression:

$$
\nabla_\theta J(\theta)=
\mathbb{E}_{s\sim d^{\pi_\theta},a\sim\pi_\theta}
\left[\nabla_\theta \log \pi_\theta(a\mid s) Q^{\pi_\theta}(s,a)\right]
$$

This is one of the defining equations of modern RL.

Interpretation:
- push up probability of actions that the critic says are good,
- push down probability of actions that the critic says are bad.

## 8.5 Why the critic stabilizes policy learning

Pure policy-gradient methods can be high variance and unstable.

The critic gives structured feedback about long-term consequences, making actor updates less blind.

That is why actor-critic became so central in practical RL.

## 8.6 Deterministic vs stochastic actors

The notes explicitly separate the two main forms:
- deterministic policy outputs a specific action,
- stochastic policy outputs a distribution.

This split leads directly to:
- DDPG / TD3 on the deterministic side,
- SAC and PPO-ish logic on the stochastic side.

## 8.7 Robotics meaning

Actor-critic is the architecture that makes continuous control practical.

That is why it appears everywhere in robotics:
- manipulation,
- locomotion,
- aerial control,
- prosthetics and wearable robotics,
- autonomous systems with continuous actuators.

The notes explicitly list these application families in the actor-critic modules.

# Module 9: DDPG

## 9.1 What DDPG is

DDPG is:
- actor-critic,
- model-free,
- off-policy,
- deterministic-policy,
- continuous-control RL.

The course presents it as one of the major continuous-control descendants of deep Q-learning ideas.

## 9.2 Why deterministic policy matters

The actor is:

$$
\mu_\theta(s): s \mapsto a
$$

This avoids the need to search over actions inside a max operator.

Instead, the actor directly outputs the candidate action.

## 9.3 Actor objective

DDPG tries to maximize expected critic value under the actor:

$$
J(\theta)=\mathbb{E}_{s\sim \rho^\mu}[Q^\mu(s,\mu_\theta(s))]
$$

The notes then use chain rule to derive the deterministic policy gradient idea.

## 9.4 Critic objective

The critic learns by Bellman regression with target networks:

$$
y=r+\gamma Q_{\phi^-}(s',\mu_{\theta^-}(s'))
$$

and minimizes squared error between predicted and target Q-values.

## 9.5 Structural importance

DDPG basically fuses:
- actor-critic,
- replay buffer,
- target networks,
- deterministic policy gradients.

So conceptually it is not random new magic.

It is the continuous-control mutation of earlier ideas.

## 9.6 Why DDPG was a big deal

Before methods like DDPG, deep RL was famous mostly for discrete-action problems.

DDPG pushed deep RL into the world of:
- torques,
- steering commands,
- continuous force control,
- robotic actuation. The notes make this historical point explicitly.

## 9.7 Limitation

DDPG works, but critic overestimation and training instability can be nasty.

That is exactly why TD3 and SAC arrive next.

# Module 10: TD3 and SAC

## 10.1 TD3: fixing DDPG’s optimism problem

TD3 adds three famous repairs:
- **twin critics**,
- **target policy smoothing**,
- **delayed policy updates**.

The notes describe TD3 as a direct extension of DDPG that explicitly addresses function approximation error and overestimation.

### Twin critics
Estimate two Q-functions and use the minimum.

This counters optimistic overestimation.

### Target policy smoothing
Perturb target actions with clipped noise.

This discourages the critic from exploiting razor-thin unrealistically sharp peaks.

### Delayed actor update
Update critic more frequently than actor.

This gives the policy a less chaotic value landscape.

## 10.2 SAC: stochastic actor plus entropy regularization

SAC is also off-policy actor-critic, but with a stochastic actor and an entropy term in the objective.

The notes emphasize that SAC and TD3 are siblings in spirit: both try to stabilize continuous-control learning and improve exploration.

## 10.3 SAC objective intuition

Standard RL objective says: maximize return.

SAC says: maximize return **and** maintain action entropy.

So SAC prefers policies that are not prematurely overconfident.

The entropy-regularized objective in the notes adds:

$$
\alpha \mathcal{H}(\pi(\cdot\mid s_t))
$$

to the reward objective.

## 10.4 Why entropy matters

Entropy helps exploration by discouraging the policy from collapsing too early into brittle deterministic choices.

This is not free lunch.

Too much entropy -> chaotic behavior.

Too little entropy -> premature exploitation.

That is why SAC has the temperature parameter \(\alpha\), and the notes mention auto-tuning with target entropy.

## 10.5 TD3 vs SAC mental model

### TD3
- deterministic,
- bias-control heavy,
- strong baseline when deterministic action output is desired.

### SAC
- stochastic,
- exploration-friendly,
- often robust as a default choice for new continuous-control problems.

The notes explicitly say SAC may be a robust starting point for new continuous-control tasks, while TD3 remains appropriate when deterministic controllers are wanted.

## 10.6 Robotics meaning

This module is the real continuous-control heart of the course.

If your robot uses:
- joint torques,
- steering angles,
- force vectors,
- continuous control inputs,

then this is where the course stops being theoretical and starts sounding like real deployment logic.

# Module 11: Practical Workflow and Evaluation

## 11.1 The implementation workflow hidden across the notes

If we compress the course into a practical RL workflow, it becomes:

1. define state, action, and reward,
2. decide whether the problem is discrete or continuous,
3. pick tabular / DQN / actor-critic accordingly,
4. design replay and exploration scheme,
5. stabilize learning with target networks and good losses,
6. train across multiple seeds,
7. evaluate without exploration,
8. compare mean and standard deviation of returns,
9. inspect learning curves and failure cases.

This workflow is spread across the Q-learning, DQN, actor-critic, and assignment slides.

## 11.2 Reward design is part of control design

The early gridworld examples already teach this.

Change the penalty structure and the optimal policy changes.

That means reward design is not decorative bookkeeping.

It is objective design.

In robotics, bad rewards can induce:
- risky shortcuts,
- jerky control,
- reward hacking,
- unstable or unsafe behavior.

## 11.3 Evaluation discipline

The notes strongly emphasize:
- multiple seeds,
- mean ± std learning curves,
- separate training vs evaluation conditions,
- reproducibility and clarity over raw reward bragging rights.

That is very grown-up engineering.

## 11.4 The assignment is a clue to what the course values

The cart-pole assignment asked for an actor-critic continuous-control method, reproducible code, hyperparameters, training with multiple seeds, evaluation with a separate seed, and learning curves with mean ± std.

That tells you the course is not merely about knowing acronyms.

It values:
- correctness,
- reproducibility,
- evaluation hygiene,
- and understanding implementation details.

## 11.5 Robotics meaning

A robotics RL pipeline that ignores evaluation discipline is basically speedrunning self-delusion.

The robot does not care how pretty the average return looked on the one lucky seed.

---

# The Full Concept Chain

This course is easiest to remember as a logical ladder.

## Ladder

1. Start with **agent, state, action, reward**.
2. Define **policy** and **value**.
3. Understand **optimal value through Bellman recursion**.
4. Solve small exact problems with **dynamic programming**.
5. Watch exact methods die from the **curse of dimensionality**.
6. Learn values from interaction using **tabular Q-learning**.
7. Compare **off-policy Q-learning** with **on-policy SARSA**.
8. Replace tables with **function approximation**.
9. Stabilize deep Q-learning with **replay** and **target networks**.
10. Face **bias and variance** in learned Q-values.
11. Move to **actor-critic** for continuous actions.
12. Use **DDPG** for deterministic continuous control.
13. Repair critic optimism with **TD3**.
14. Add entropy-driven exploration with **SAC**.
15. Evaluate with seeds, rollouts, and learning curves like a civilized engineer.

That is the course arc.

# Robotics Connection Sheet

## A. Locomotion and humanoid control

- continuous state and action spaces kill tabular methods.
- actor-critic methods become natural.
- bias/variance in critics directly affect stability of learned gait.
- replay, target updates, and normalization matter a lot.

## B. Manipulation and grasping

- delayed rewards make one-step reactive control insufficient.
- Q-values and policy gradients reason over longer horizons.
- entropy regularization can prevent brittle premature exploitation.
- offline tuples can be valuable when real interaction is expensive.

## C. Autonomous driving and drones

- stochastic transitions resemble slippage / wind / uncertainty.
- reward design shapes risk behavior.
- continuous actions favor TD3 / SAC / DDPG-style methods.
- evaluation under multiple seeds is essential because rare failures matter.

## D. Offline robotic learning

- off-policy methods can learn from logged transitions.
- distribution mismatch becomes a serious issue.
- poor coverage means poor Q estimates.
- conservative critics and better uncertainty handling become important.

## E. Safe robotics

- exploration is not harmless.
- on-policy vs off-policy affects what is learned under exploratory behavior.
- risky shortest-path behavior in gridworld is the toy version of unsafe real-world policy learning.
- reward shaping and admissible policy restrictions matter in safety-critical systems.

## F. Modern control perspective

- RL can be seen as approximate dynamic programming with data-driven value learning.
- Bellman consistency is the backbone.
- actor-critic is a learned control architecture.
- the critic is the learned long-horizon evaluator.
- the actor is the learned closed-loop controller.

# What to Never Forget

## 1. RL is sequential decision-making, not one-step prediction.

## 2. Policy and value are the two fundamental objects.

## 3. Bellman recursion is the backbone of optimality.

## 4. Dynamic programming is exact but crushed by dimensionality.

## 5. Q-learning is model-free and off-policy.

$$
Q(s,a) \leftarrow Q(s,a)+\alpha[r+\gamma\max_{a'}Q(s',a')-Q(s,a)]
$$

## 6. SARSA is on-policy.

$$
Q(s,a) \leftarrow Q(s,a)+\alpha[r+\gamma Q(s',a')-Q(s,a)]
$$

## 7. Tables die in large or continuous robotics problems.

## 8. Deep RL turns value learning into Bellman-style regression.

## 9. Replay buffer and target networks are stabilizers, not decoration.

## 10. Overestimation and variance are structural problems.

## 11. Actor-critic is the practical bridge to continuous control.

## 12. DDPG, TD3, and SAC are all attempts to make actor-critic actually behave.

## 13. SAC uses entropy to keep exploration alive.

## 14. Evaluation without seed averaging is suspicious.

## 15. In robotics, reward design is control design in disguise.

---

# Intuition Anchors by Topic

## Policy
The agent’s style of decision-making.

## Value
Long-horizon goodness, not instant gratification.

## Bellman equation
The future folded back into the present.

## Dynamic programming
Exact optimality by recursive decomposition.

## Curse of dimensionality
The reason beautiful exact methods stop paying rent.

## Q-learning
Learn how good actions are by bootstrapping from later estimates.

## SARSA
Learn the value of the policy you are actually living through.

## Replay buffer
Memory with shuffled receipts.

## Target network
A slowed-down teacher so the student stops chasing its own panic.

## Overestimation bias
The max operator getting drunk on noisy optimism.

## Variance
Critic uncertainty that makes learning wobble or explode.

## Actor
The policy network that actually decides.

## Critic
The long-horizon judge of the actor’s choices.

## DDPG
Deterministic actor-critic for continuous control.

## TD3
DDPG after therapy and trust issues.

## SAC
Actor-critic with entropy so exploration does not die young.

# 30-Years-Later Refresher

If future-me forgot almost everything, remember this skeleton:

1. RL learns a **policy** using experience.
2. A policy is judged by **long-term return**, not just immediate reward.
3. The **Bellman equation** expresses this recursive long-horizon logic.
4. **Dynamic programming** solves it exactly only in small problems.
5. **Q-learning** learns action values without needing a model.
6. **SARSA** differs because it is on-policy.
7. Real robotic problems need **function approximation**.
8. DQN learns Q-values with **replay** and **target networks**.
9. Deep value learning suffers from **overestimation** and **variance**.
10. Continuous control usually needs **actor-critic** methods.
11. **DDPG** uses a deterministic actor.
12. **TD3** fixes DDPG’s critic optimism.
13. **SAC** adds stochasticity and entropy regularization.
14. Proper RL work requires **multi-seed evaluation, learning curves, and reproducibility**.

If this skeleton is alive, the rest can be rebuilt.

# Final Robotics Takeaway

This course teaches one deep lesson:

A reinforcement learning algorithm is not just a reward-maximizing trick.

It is:
- an approximate dynamic programming method,
- a value-learning procedure,
- a control architecture,
- a policy optimization scheme,
- a bias-variance negotiation,
- and a practical engineering workflow.

The real jump from beginner to practitioner happens when you stop seeing RL as “agent gets reward” and start seeing it as a structured attempt to approximate optimal sequential control under uncertainty, approximation limits, and data constraints.

That is when RL stops being hype vocabulary and starts becoming control instinct.

---