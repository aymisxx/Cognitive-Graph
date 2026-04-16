# **Robot Vision and Perception**

### **From Camera Geometry to World Models to Actionable Robot Understanding**

**Focus:** durable robotics understanding: not just recognizing what is in an image, but turning sensing into estimation, mapping, interaction, and decision support.

---

# Why this course matters in robotics

Robot perception is the place where raw sensing becomes usable world understanding.

A robot does not get the world in clean symbolic form.

It gets:

- pixels,
- depth scans,
- noisy measurements,
- partial views,
- moving objects,
- ambiguous geometry,
- and latency-sensitive data streams.

This subject matters because robotics always needs answers to the same ruthless questions:

- Where am I?
- What am I looking at?
- Where is it in 3D?
- How uncertain is my estimate?
- What is safe to do next?

This course sits exactly at the intersection of:

- **geometry** - projection, cameras, stereo, 3D structure,
- **estimation** - Kalman filtering, uncertainty, tracking,
- **mapping** - SLAM, registration, world models,
- **recognition** - features, CNNs, segmentation, object understanding,
- **interaction** - grasping, affordances, language grounding,
- **systems** - ROS pipelines, sensor fusion, deployment.

If dynamics and control ask how the robot moves, perception asks what the robot actually knows while moving.

That makes this course core infrastructure, not side garnish.

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

1. robot vision framing and the sense-plan-act loop.
2. camera model and image formation.
3. features and learned visual representations.
4. stereo and epipolar geometry.
5. Kalman filtering and tracking.
6. SLAM.
7. point clouds, registration, voxels, octrees.
8. grasping as vision.
9. autonomous driving perception.
10. data fusion, language grounding, and deployment.

## Mode 3: Deep rebuild

Read everything and mentally reconnect:
- 3D world to 2D image,
- image evidence to geometric constraints,
- noisy sensing to state estimation,
- perception outputs to control and planning.

# Big Picture Map

At the highest level, this course answers ten giant questions.

## 1. What makes robot vision different from plain computer vision?

> Robotics cares not just about recognition, but about **actionable state**, **geometry**, **timing**, and **uncertainty**.

## 2. How does a camera turn 3D structure into an image?

> Through **projection**, **intrinsics**, **extrinsics**, and a **camera model**.

## 3. How do we extract useful information from images?

> With **features**, **descriptors**, **classifiers**, and later **deep networks**.

## 4. How do two views recover 3D structure?

> Through **epipolar geometry**, **stereo correspondence**, **disparity**, and **triangulation**.

## 5. How do we reason under noisy measurements over time?

> With **Kalman filtering** and state estimation.

## 6. How do we localize and map simultaneously?

> With **SLAM**.

## 7. How do we represent 3D sensed geometry?

> As **point clouds**, **meshes**, **voxels**, and **octrees**.

## 8. How does perception support manipulation?

> Through **pose estimation**, **grasp affordances**, and **visual grasp reasoning**.

## 9. What does perception look like in real autonomy stacks?

> Stereo, segmentation, tracking, depth completion, sensor fusion, and planning support.

## 10. Where is the field heading?

> Toward **vision-language fusion**, **foundation models**, **multimodal control**, and perception tightly coupled with action.

# Notation and foundational objects

We mostly work with standard robotics / vision quantities.

**3D world point:**

$$
P = \begin{bmatrix} X \\
Y \\
Z \end{bmatrix}
$$

**Homogeneous 3D point:**

$$
\tilde{P} = \begin{bmatrix} X \\
Y \\
Z \\
1 \end{bmatrix}
$$

**Image point:**

$$
p = \begin{bmatrix} u \\
v \end{bmatrix}
$$

**Camera intrinsics matrix:**

$$
K = \begin{bmatrix}
f_x & 0 & c_x \\
0 & f_y & c_y \\
0 & 0 & 1
\end{bmatrix}
$$

**Rigid transform from world to camera:**

$$
T = [R \mid t]
$$

**Linear state-space model for filtering:**

$$
x_{k+1} = A x_k + B u_k + w_k
$$

$$
z_k = H x_k + v_k
$$

**Point cloud point:**

$$
p_i = (x_i, y_i, z_i)
$$

**Robot pose estimate:**

$$
\hat{x}
$$

**Covariance:**

$$
P
$$

---

# Module 1: Robot Vision Framing - What the Robot Actually Needs

## 1.1 Robot vision is not just image classification

The first lecture makes the course objective very clear: the goal is to create roboticists who can integrate computer vision into robotics pipelines, with geometric understanding, object recognition / pose detection capability, and a sense-act-loop mindset. 

That is already a different philosophy from generic computer vision.

Plain CV may stop at:
- what object is this,
- where is it in the image,
- what category does this patch belong to.

Robot vision must continue to:
- where is the robot,
- where is the object relative to the robot,
- what is uncertain,
- what action becomes possible now.

## 1.2 Sense, plan, act is the real frame

The early lecture repeatedly anchors perception inside a broader robotics loop:

**Sense -> Plan -> Act**

That matters because perception is not a stand-alone scoreboard.
It is upstream of action.

If a perception output cannot support planning or control, it is incomplete from a robotics perspective.

## 1.3 Continuous observations vs symbols

Humans casually issue symbolic commands like:
- clean the kitchen,
- put the cheese grater away.

The robot, meanwhile, gets continuous measurements and must bridge that into actionable structure.

This is the deeper perception problem:
- grounding symbols into observations,
- grounding observations into geometry,
- grounding geometry into action.

## 1.4 The core questions never change

The first lecture reduces the field to a few giant questions:
- Where is the robot?
- Where are the objects?
- Which objects matter?
- How does the robot act given visual input?

That is the whole course in embryo.

## 1.5 Why the real world is harder than the internet

The notes explicitly point out the distribution gap:
- internet imagery is not robot imagery,
- cameras are noisy,
- views are partial,
- clutter and occlusion are normal,
- robotics must work under partial observability.

This is a huge maturity checkpoint.

A good vision model in clean benchmark settings is not automatically a good robot perception system.

## 1.6 Robotics meaning

Perception in robotics is not just recognition.
It is:
- state estimation,
- task-relevant scene understanding,
- geometric reconstruction,
- uncertainty-aware world modeling,
- and support for control.

That is why the course begins with framing, not formulas.

# Module 2: Camera Model - How 3D Becomes 2D

## 2.1 Why the camera model matters

Robot vision starts with image formation.
If you do not understand how a 3D point becomes a 2D pixel, the rest of the course becomes superstition.

The camera lecture centers the usual pinhole / perspective model.

## 2.2 Perspective projection

At the cleanest level, a 3D point in camera coordinates projects to normalized image coordinates by dividing by depth:

$$
x = X / Z, \qquad y = Y / Z
$$

This is the core reason vision is hard.
Depth is buried inside projection.

Lengths are not preserved.
Angles are not preserved.
Parallel lines can appear to meet.
Scale changes with distance.

## 2.3 Intrinsics and extrinsics

The practical camera map is usually written as:

$$
\lambda \tilde{p} = K [R \mid t] \tilde{P}
$$

where:
- $K$ is the intrinsic matrix,
- $R, t$ describe camera pose relative to the world,
- and $\lambda$ handles projective scale.

This separates two ideas cleanly:
- **extrinsics** tell where the camera is,
- **intrinsics** tell how the camera measures.

## 2.4 Principal point and focal lengths

The notes explicitly show the role of:
- focal length,
- image center / principal point,
- scaling from metric coordinates to pixels.

That means perception is never just “a point lands somewhere.”
It lands somewhere in a coordinate system created by hardware and calibration.

## 2.5 Why calibration is not optional

The autonomous-driving lecture highlights calibration challenges in stereo setups and even mentions calibration drift from thermal stress.

That is a very real robotics lesson:
- camera model errors become depth errors,
- depth errors become pose errors,
- pose errors become planning and control errors.

## 2.6 Robotics meaning

The camera model is the perception equivalent of forward kinematics.
It is the basic map between spaces.

Without it, you cannot do:
- pose estimation,
- stereo depth,
- camera-lidar alignment,
- calibration,
- triangulation,
- or disciplined vision debugging.

# Module 3: Features and Learned Visual Representations

## 3.1 Classical features: what to keep

The features lecture starts with linear classification intuition and then moves into hand-designed image features.

That structure matters.

The course is showing that visual understanding historically moved through stages:
1. choose informative features,
2. classify in feature space,
3. learn increasingly abstract features with deeper networks.

## 3.2 Keypoints

Keypoints are image locations with strong local structure, often large gradient variation in multiple directions.

Why they matter:
- they are repeatable,
- they survive viewpoint changes better than arbitrary pixels,
- and they can serve as landmarks.

The notes explicitly connect keypoints to the landmark question in SLAM.
That is exactly right.

## 3.3 SIFT and HOG

The lecture emphasizes two classical feature families.

### SIFT
- scale-invariant local descriptors,
- gradient histograms around a keypoint,
- canonical orientation handling,
- 128-dimensional descriptor structure.

### HOG
- histogram-of-oriented-gradients style representation,
- less tied to sparse keypoints,
- more distributed gradient statistics across image regions.

These matter because they teach what visual features are trying to do:
- compress local structure,
- preserve discriminative geometry,
- tolerate nuisance variation.

## 3.4 Neural nets and CNNs

The same lecture then makes the deep-learning jump:
- linear threshold units,
- stacked layers,
- universal approximation intuition,
- CNNs,
- ResNet,
- forward and backward passes.

The conceptual jump is this:

Instead of hand-designing descriptors, we let the network learn filters and hierarchical representations from data.

## 3.5 Why convolution matters

CNNs exploit two real facts about images:
- local patterns matter,
- the same pattern can appear in many places.

That gives parameter sharing, translation tolerance, and more efficient learning than fully connected flatten-everything chaos.

Pooling then compresses representation while keeping object-level meaning alive.

## 3.6 Robotics meaning

The big lesson is not “old features bad, deep nets good.”
It is:
- what makes a visual signal informative,
- how local structure becomes representation,
- why learned features scale better,
- and how perception pipelines moved from geometry-only to geometry-plus-learning.

That conceptual continuity matters for robotics because older ideas still survive inside matching, registration, SLAM, and correspondence.

# Module 4: Stereo and Epipolar Geometry - Turning Multiple Views into 3D

## 4.1 Why stereo matters

A single image hides depth.
A second view creates geometric constraints that make depth inference possible.

This is the heart of stereo vision.

## 4.2 Epipolar geometry

Two calibrated views impose a rigid constraint on where corresponding points can appear.

In compact form, epipolar geometry is captured by:

$$
\tilde{p}_2^T F \tilde{p}_1 = 0
$$

where $F$ is the fundamental matrix.

If intrinsics are known, the essential matrix captures the calibrated version of the relation.

The OCR from the epipolar lectures is messy, but the structure is clearly the standard line of development: epipoles, epipolar lines, and the algebra linking two views.

## 4.3 Why epipolar constraints are gold

Without the constraint, correspondence search is a 2D hunt.
With it, the candidate match collapses onto an epipolar line.

That reduces ambiguity massively.

So epipolar geometry is really a search-space reduction mechanism powered by rigid-view geometry.

## 4.4 Disparity and depth

For rectified stereo, depth is inversely related to disparity.
A simple memory form is:

$$
Z = f B / d
$$

where:
- $f$ is focal length,
- $B$ is baseline,
- $d$ is disparity.

The lecture OCR explicitly points toward the classic fact that disparity is inversely proportional to distance.

This one relationship is one of the highest-value facts in robot perception.

## 4.5 Correspondence is the real difficulty

Stereo is easy in principle and ugly in practice.
The lecture notes point toward stereo correspondence and candidate matching trouble.

This is the real issue:
- repeated textures,
- occlusion,
- low texture,
- reflective surfaces,
- calibration errors,
- thin structures.

Geometry gives constraints.
It does not magically give correct matches.

## 4.6 Robotics meaning

Stereo teaches one of the deepest robotics habits:

**Use structure to reduce uncertainty before using learning or brute force.**

That mindset survives everywhere:
- epipolar lines in stereo,
- motion models in Kalman filtering,
- correspondence constraints in SLAM,
- grasp priors in manipulation.

# Module 5: Kalman Filtering - Perception Under Uncertainty

## 5.1 Why filtering exists

Robots do not directly observe the true state.
They observe noisy measurements.

The Kalman lecture is basically about one central truth:

> perception is estimation, not direct access.

## 5.2 State, action, measurement

The lecture frames a robot model with:
- a latent state,
- an action / process model,
- a measurement model,
- uncertainty in both evolution and observation.

That is the correct mental frame for a huge amount of robotics.

## 5.3 Linear Gaussian model

A standard linear Kalman filter setup is:

$$
x_{k+1} = A x_k + B u_k + w_k
$$

$$
z_k = H x_k + v_k
$$

with process noise $w_k$ and measurement noise $v_k$ modeled as Gaussian.

## 5.4 Predict and update

The Kalman filter alternates between:

### Prediction
Use the process model to predict the next state and covariance.

$$
\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1} + B u_k
$$

$$
P_{k|k-1} = A P_{k-1|k-1} A^T + Q
$$

### Measurement update
Use the new observation to refine the estimate.

$$
K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}
$$

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H \hat{x}_{k|k-1})
$$

$$
P_{k|k} = (I - K_k H) P_{k|k-1}
$$

This is the basic estimation heartbeat of classical robot perception.

## 5.5 What the Kalman gain really means

The lecture OCR explicitly hints at the interpretation: trust the measurement more if the sensor is reliable, trust the process model more if the sensor is noisy.

That is what the gain does.
It is a principled compromise between:
- model-based prediction,
- measurement-based correction.

## 5.6 Markov assumption

The lecture also explicitly notes a Markov assumption.
That matters because filtering lives on the idea that the current state summarizes the relevant past for predicting the future.

That is what makes recursive estimation tractable.

## 5.7 Robotics meaning

Kalman filtering is not just a lecture topic.
It is everywhere:
- tracking,
- odometry fusion,
- lane tracking,
- localization,
- object motion estimation,
- inertial fusion,
- state estimation for controllers.

It is one of the most important bridges between perception and control.

# Module 6: SLAM - Localization and Mapping at the Same Time

## 6.1 Why SLAM is hard

SLAM sounds innocent:
- estimate robot pose,
- estimate map,
- do both simultaneously.

The problem is cursed because those two unknowns depend on each other.
You need the map to localize, and you need localization to build the map.

## 6.2 State design

The SLAM lecture clearly builds a joint state containing robot pose plus landmark variables.

At a conceptual level:

$$
x = \begin{bmatrix} x_r \\
m \end{bmatrix}
$$

where $x_r$ is robot pose and $m$ is the landmark map.

That is the core structural jump from filtering a robot state to filtering a robot-plus-world state.

## 6.3 Motion model and observation model

Like Kalman filtering, SLAM is organized around two ingredients:
- motion model,
- observation model.

Prediction moves the robot estimate forward.
Measurement update ties robot and landmark estimates back to sensory evidence.

## 6.4 Landmark features matter

The SLAM OCR explicitly points toward feature choices like SIFT / SURF / ORB style landmarks, data association, and loop closure.

That is exactly right.

SLAM depends on:
- stable landmarks,
- good matching,
- correct association,
- and revisiting old places without lying to yourself.

## 6.5 Data association and loop closure

These are the real gremlins.

### Data association
Which observed feature corresponds to which map landmark?

### Loop closure
How do we recognize that we have returned to a previously visited place and then use that fact to reduce accumulated drift?

If either goes wrong, the map becomes fiction.

## 6.6 Why covariance matters even more here

In SLAM, uncertainty is not a side variable.
It is the whole game.

Robot pose uncertainty and landmark uncertainty become coupled.
That coupling is what makes measurement updates globally informative and computationally painful.

## 6.7 Robotics meaning

SLAM is the canonical example of robot perception as joint estimation over time.

It fuses:
- geometry,
- features,
- motion models,
- probabilistic estimation,
- and map structure.

It is also the perfect reminder that robot perception is never just “run detector on frame.”

# Module 7: Point Clouds, Registration, Voxels, and Octrees

## 7.1 Point clouds as direct 3D evidence

The point-cloud lecture begins with the simplest definition:

a point cloud is a set of discrete 3D points, basically a list of $(X,Y,Z)$ values.

That is deceptively powerful.

A point cloud is already geometry, not just appearance.
It gives the robot direct spatial evidence about surfaces and shape.

## 7.2 Where point clouds come from

The notes mention several depth-generation sources:
- lidar,
- structured light,
- Kinect / RGB-D style sensing,
- time-of-flight methods,
- 3D scanners.

They also explicitly mention calibration between depth and RGB sensing.
That is a huge practical point.

## 7.3 Registration

Once we have multiple point clouds, we often need to align them.
That is registration.

The lecture emphasizes ICP, iterative closest point.

A standard ICP objective is:

$$
\min_{R,t} \sum_i \| R p_i + t - q_i \|^2
$$

for matched points $p_i$ and $q_i$.

This is a perfect example of perception becoming optimization.

## 7.4 Why ICP matters

ICP is not just a 3D graphics trick.
It supports:
- scan alignment,
- reconstruction,
- visual odometry,
- local map fusion,
- object pose registration.

## 7.5 Voxels and octrees

The lecture then moves from raw point sets to structured spatial representations.

### Voxels
Discretized volumetric cells in 3D.

### Octrees
Hierarchical recursive partitioning of 3D space.

Why they matter:
- efficient storage,
- occupancy representation,
- fast search,
- planning integration.

The notes explicitly tie voxelization to obstacle representation for planning.

## 7.6 Robotics meaning

This module teaches a broader lesson:

Perception is not only about estimating things.
It is also about choosing a world representation that planning and control can actually use.

A beautiful point cloud no planner can search is not yet a robotics solution.

# Module 8: Grasping as Vision

## 8.1 Why grasping belongs in a perception course

Because grasping starts with seeing the object in an action-relevant way.

The lecture makes this explicit by treating grasping as a perception problem about:
- object pose,
- contact geometry,
- affordances,
- thickness / orientation / accessibility,
- and partial observability.

## 8.2 Pose is the first gate

The lecture begins with SE(2) and SE(3) pose notions.
That matters because grasping is impossible without a working pose estimate or at least a good local geometric surrogate.

## 8.3 Why grasping is hard

The notes list exactly the right pain points:
- pose not easy to recover,
- where to grasp is the real challenge,
- dexterous manipulation makes it worse,
- bimanual coordination and object properties matter,
- partial observability never leaves.

## 8.4 Antipodal and prehensile reasoning

The lecture uses antipodal and prehensile grasp ideas to build intuition:
- opposing contacts,
- gripper width,
- object geometry,
- whether a flat plate or spoon-like object is actually graspable in the current configuration.

That is good robotics pedagogy because it grounds vision in mechanics.

## 8.5 From brittle rules to learned grasps

The notes explicitly move from hand-specified proposals toward learned grasping:
- grasp proposal,
- grasp learning,
- affordance densities,
- point-cloud based classification,
- Dex-Net,
- transporter-style ideas,
- RL for harder dexterous or extrinsic manipulation.

This is the same historical pattern as in vision more broadly:
- geometry and heuristics first,
- data-driven refinement later.

## 8.6 Robotics meaning

Grasping as vision is the clearest example of why perception cannot stop at semantic labels.

The robot does not merely need:
- cup,
- spoon,
- plate.

It needs:
- graspable where,
- from what approach direction,
- under what uncertainty,
- with what end-effector limitations.

That is a different standard of perception.

# Module 9: Autonomous Driving Perception

## 9.1 Why driving is a great stress test

Autonomous driving perception is perception under:
- real time constraints,
- large scale scenes,
- safety critical failure costs,
- heterogeneous sensors,
- dynamic objects,
- weather and lighting variation.

So it is basically robot perception with extra consequences.

## 9.2 Problem families the lecture highlights

The driving lecture organizes the field around several practical problem classes:
- depth estimation,
- depth completion,
- object detection in 2D and 3D,
- semantic segmentation,
- odometry,
- object tracking,
- lane detection,
- sensor fusion.

That is a very good systems view.

## 9.3 Datasets and benchmarking

The lecture explicitly mentions KITTI and other major datasets such as Waymo, NuScenes, Argo, Berkeley Deep Drive, and Oxford RobotCar.

The real lesson is that autonomy stacks live on large-scale benchmark ecosystems, but the datasets also reveal the hard cases:
- calibration trouble,
- incomplete lidar rasterization,
- lighting variation,
- difficult traffic-sign and traffic-light detection,
- emergency-vehicle recognition.

## 9.4 Tracking and Kalman filtering

The lecture directly connects object tracking and lane tracking to Kalman-filter style temporal estimation.

That is exactly what makes perception temporal instead of frame-by-frame amnesia.

## 9.5 Planning support

The notes explicitly tie segmentation and scene understanding to long-range planning, and then show the broader car stack with fast control, replanning, and mission-scale planning.

That is the robotics truth again:

Perception earns its keep by supporting downstream decisions.

## 9.6 What is still missing

The lecture ends with harder missing pieces:
- common sense reasoning,
- the changing world,
- weather,
- unstructured environments outside roads.

That is a strong reality check.
Benchmark success is not the same as world completeness.

# Module 10: Data Fusion, Vision-Language, and Multimodal Robot Perception

## 10.1 Why data fusion matters

No single sensing modality is enough.

Cameras are rich but ambiguous in depth.
Lidar is geometric but sparse and semantically thin.
Language is symbolic but detached from immediate sensory state.

The data-fusion lecture pushes exactly toward this multimodal view.

## 10.2 Vision and language

The lecture organizes a big chunk of the modern field around:
- language to programs,
- vision-and-language model learning,
- vision-and-language control,
- and useful fusion architectures.

This is the modern answer to the symbolic-grounding problem from Lecture 1.

## 10.3 Model families worth remembering

The notes explicitly list several families and systems:
- FiLM,
- Mask R-CNN,
- UNet,
- CLIP,
- PALM-E,
- SAM,
- CLIPort,
- SayCan,
- RT-1, RT-2, RT-X,
- OpenVLA.

The exact implementations matter less than the structural shift they represent:

Perception is moving from isolated vision modules to large multimodal representations that can support policy-level reasoning.

## 10.4 Mask R-CNN and UNet mental anchors

### Mask R-CNN
Bounding boxes plus classes plus masks.

### UNet
Encoder-decoder style segmentation architecture.

These are worth keeping because segmentation is one of the most action-relevant outputs in robotics.

## 10.5 Foundation-model lesson

The lecture also emphasizes the major challenges that remain even with VLMs:
- embodiment,
- robot-specific data scarcity,
- interaction,
- lifelong learning,
- common sense reasoning.

This is a great corrective against foundation-model hype.

A giant model without embodiment is not yet a robot perceptual system.

## 10.6 Robotics meaning

This module is where the course becomes unmistakably modern.

The field is not abandoning geometry and estimation.
It is trying to fuse them with learned representations, language grounding, and broader priors.

# Module 11: ROS and Perception as a System Pipeline

## 11.1 Why ROS appears in this course

Because robot perception is never just an algorithm on paper.
It is a data pipeline.

The ROS lecture emphasizes:
- modularity,
- hardware abstraction,
- communication tools,
- visualization,
- nodes,
- topics,
- services,
- actions,
- packages.

## 11.2 Why this matters for perception specifically

Perception stacks are inherently distributed.
You may have:
- camera node,
- lidar node,
- detector node,
- tracker node,
- SLAM node,
- planner node,
- visualizer.

ROS is the system glue that lets these components communicate in real time.

## 11.3 The mature lesson

A perception algorithm that ignores systems integration is still half academic.

Robots need:
- synchronized streams,
- reliable message passing,
- modular debugging,
- reproducible interfaces,
- deployment discipline.

That is why ROS belongs in a perception summary.

---

# The Full Concept Chain

This course is easiest to remember as a logical ladder.

## Ladder

1. Start with the **sense-plan-act** loop.
2. Ask what robot perception must provide beyond generic CV.
3. Learn how a 3D point becomes a 2D pixel through the **camera model**.
4. Learn how visual structure is represented through **features** and **descriptors**.
5. Replace hand-designed features with **learned CNN features** when scale demands it.
6. Use multiple views and **epipolar geometry** to recover 3D constraints.
7. Use **Kalman filtering** to track hidden state through noisy measurements.
8. Extend filtering into **SLAM** by estimating robot pose and map together.
9. Represent 3D geometry with **point clouds**, **registration**, **voxels**, and **octrees**.
10. Turn scene understanding into action via **grasping** and **affordance reasoning**.
11. Scale to real applications like **autonomous driving** with segmentation, tracking, and fusion.
12. Fuse vision with language and modern learned priors in **multimodal robot perception**.
13. Deploy the whole thing through **system pipelines** such as ROS.

That is the course arc.

# Robotics Connection Sheet

## A. Localization and navigation

- camera geometry connects observations to coordinates.
- Kalman filtering turns noisy sensing into a state estimate.
- SLAM builds map and pose together.
- landmarks, loop closure, and data association determine long-horizon reliability.

## B. Manipulation and grasping

- pose estimation determines where the object is.
- geometry and visibility determine if a grasp is even feasible.
- grasp affordances are perception outputs, not just mechanics outputs.
- partial observability is often the main enemy.

## C. Autonomous driving and mobile robots

- stereo depth, segmentation, tracking, and lane estimation support planning.
- lidar-camera fusion resolves limitations of individual sensors.
- temporal filtering matters because frame-wise detection is not enough.
- real-world robustness depends on calibration, noise handling, and changing conditions.

## D. 3D scene understanding

- point clouds and ICP support registration and map fusion.
- voxels and octrees convert raw geometry into searchable planning structures.
- representation choice changes what downstream algorithms can do.

## E. Multimodal robots

- language grounding maps commands to perceptual targets.
- foundation-style vision-language systems help with task-level abstraction.
- embodiment, interaction, and lifelong adaptation are still unresolved bottlenecks.

## F. Systems engineering

- perception lives inside nodes, topics, and real-time data pathways.
- latency, synchronization, and modularity are as important as algorithm choice.
- a great model inside a broken pipeline is still a broken robot.

# What to Never Forget

## 1. Robot perception is about actionable world understanding, not only recognition.

## 2. A camera is a projection device, not a truth device.

$$
\lambda \tilde{p} = K [R \mid t] \tilde{P}
$$

## 3. Depth is the hidden variable in monocular vision.

## 4. Stereo works because geometry constrains correspondence.

$$
\tilde{p}_2^T F \tilde{p}_1 = 0
$$

## 5. Disparity and depth are inversely related.

$$
Z = f B / d
$$

## 6. Perception is estimation under uncertainty.

## 7. Kalman filtering is prediction plus correction, not magic.

$$
\hat{x}^- = A\hat{x} + Bu, \qquad
\hat{x} = \hat{x}^- + K(z - H\hat{x}^-)
$$

## 8. SLAM is hard because map and pose are coupled unknowns.

## 9. Point clouds are geometry; voxels and octrees are usable structure.

## 10. Registration is an optimization problem.

$$
\min_{R,t} \sum_i \|R p_i + t - q_i\|^2
$$

## 11. Grasping needs action-relevant perception, not just object labels.

## 12. Segmentation, tracking, and fusion matter because real robots operate over time.

## 13. Vision-language systems are powerful, but embodiment is still the catch.

## 14. A perception algorithm that ignores ROS / systems integration is only half done.

---

# Intuition Anchors by Topic

## Camera model
A camera is a geometry compressor that hides depth inside projection.

## Intrinsics
The camera's measuring personality.

## Extrinsics
Where the camera is, and how it is oriented relative to the world.

## Keypoints
The image locations most worth remembering.

## Descriptor
A compact local signature for matching visual structure.

## CNN
A learnable hierarchy of local pattern detectors.

## Epipolar geometry
Two views whispering constraints to each other about where a match can exist.

## Disparity
Horizontal mismatch that secretly carries depth.

## Kalman filter
A negotiation between what the model predicted and what the sensor reported.

## SLAM
Trying to learn where you are while learning what the world looks like.

## Point cloud
A raw 3D scatter of where surfaces probably exist.

## ICP
Geometry alignment by repeatedly minimizing disagreement.

## Voxel
A 3D pixel.

## Octree
A smart hierarchical way to store 3D space without going broke.

## Grasp affordance
Not just what the object is, but where action is possible.

## Vision-language grounding
Turning words into perceptual targets the robot can actually use.

## ROS perception stack
Algorithms with plumbing, timestamps, and consequences.

# 30-Years-Later Refresher

If future-me forgot almost everything, remember this skeleton:

1. Robot perception sits inside **sense -> plan -> act**.
2. The camera model explains how **3D becomes 2D**.
3. Features and CNNs explain how images become **representations**.
4. Stereo and epipolar geometry explain how multiple views recover **depth and structure**.
5. Kalman filtering explains how noisy measurements become **state estimates**.
6. SLAM explains how robot pose and world map are estimated **together**.
7. Point clouds, voxels, and octrees explain how sensed geometry becomes a **usable 3D world model**.
8. Grasping as vision explains how perception must become **action-relevant**.
9. Autonomous-driving perception shows what happens when all of this must work in real time, at scale.
10. Vision-language and fusion methods push perception toward **task-level grounded intelligence**.
11. ROS reminds you that a perception system is also a **software system**.

If this skeleton is alive, the rest can be rebuilt.

# Final Robotics Takeaway

This course teaches one deep lesson:

Perception is not “the robot sees things.”

It is:
- geometry under projection,
- inference under noise,
- mapping under uncertainty,
- representation learning under scale,
- action-relevant scene understanding,
- and systems engineering under real-time constraints.

The real jump from beginner to roboticist happens when you stop seeing perception as a pile of vision algorithms and start seeing it as the process of converting messy sensory streams into world models and decisions that a robot can safely act on.

That is when robot vision stops being image processing and starts becoming robotic intelligence.

---

# Minimal Core Formula Sheet

$$
\lambda \tilde{p} = K [R \mid t] \tilde{P}
$$

$$
x = X / Z, \qquad y = Y / Z
$$

$$
\tilde{p}_2^T F \tilde{p}_1 = 0
$$

$$
Z = f B / d
$$

$$
x_{k+1} = A x_k + B u_k + w_k
$$

$$
z_k = H x_k + v_k
$$

$$
\hat{x}_{k|k-1} = A \hat{x}_{k-1|k-1} + B u_k
$$

$$
P_{k|k-1} = A P_{k-1|k-1} A^T + Q
$$

$$
K_k = P_{k|k-1} H^T (H P_{k|k-1} H^T + R)^{-1}
$$

$$
\hat{x}_{k|k} = \hat{x}_{k|k-1} + K_k (z_k - H \hat{x}_{k|k-1})
$$

$$
P_{k|k} = (I - K_k H) P_{k|k-1}
$$

$$
x_{slam} = \begin{bmatrix} x_r \\
m \end{bmatrix}
$$

$$
\min_{R,t} \sum_i \| R p_i + t - q_i \|^2
$$

$$
p_i = (x_i, y_i, z_i)
$$

---

# End Note

This is not "just computer vision."

It is the perceptual backbone of robotics.

---