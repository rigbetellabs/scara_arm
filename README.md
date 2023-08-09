# Install Dependencies

### Prerequisites:

If you want to use MoveIt with ROS Noetic, there are some prerequisites that you need to install first. Here's what you'll need:

- **ROS Noetic**: Make sure you have ROS Noetic installed on your system. You can find instructions on how to do that [here](http://wiki.ros.org/noetic/Installation).
- **MoveIt**: You'll obviously need MoveIt installed as well. You can install it by running the following command:

  ```
  sudo apt-get install ros-noetic-moveit

  ```

### Install Planning adapters    
Install CHOMP Optimizer Adapter: `sudo apt install ros-noetic-moveit-chomp-optimizer-adapter`



# Download and Build package:

Open a new terminal and run the following command to clone the repo:
```bash
cd ~/ && mkdir -p catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/rigbetellabs/scara_arm.git --recurse-submodules
cd scara_arm/
git submodule update --init --recursive
cd ~/catkin_ws
```

Run rosdep to update the binaries and install missing:
```bash
source /opt/ros/noetic/setup.bash
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build the packages:
```bash
cd ~/catkin_ws
# you can use build or make
catkin_make
```

# How to run?
To launch the robot base with no gripper in the simulated rviz env:
```bash
roslaunch scara_arm robot_bringup.launch
```
### Configurations 
1. `interface` types: Running the arm in a perticular env 
    - `rviz` : launch rviz and fake trajectory execution *(default)*
    - `gazebo` : launch with Gazebo simulation
    - `hardware` : connect to the robot arm
```bash
roslaunch scara_arm robot_bringup.launch interface:=hardware
```

2. `gripper_type` types: Attach Grippers
    - `default` : Only the base arm no gripper *(default)*
    - `vacuum` : Attach Vacuum Suction Gripper
    - `two_finger` : Attach Two Finger Gripper
```bash
roslaunch scara_arm robot_bringup.launch interface:=hardware gripper_type:=vacuum
```

3. `pipeline` types: Planning Pipelines for the arm
    - `ompl` : Open Motion Planning Library *(default)*
    - `chomp` : Covariant Hamiltonian Optimization for Motion Planning 
    - `stomp` : Stochastic Trajectory Optimization for Motion Planning
    - `ompl_stomp` : OMPL as a pre-processor for STOMP
    - `ompl_chomp` : OMPL as a pre-processor for CHOMP
    - `chomp_stomp` : STOMP as a post-processor for CHOMP
    - `stomp_chomp` : CHOMP as a post-processor for STOMP
```bash
roslaunch scara_arm robot_bringup.launch interface:=hardware gripper_type:=vacuum pipeline:=chomp
```

# Theory on Planners
## What are planners?

In MoveIt, planners are used to generate motion plans for robotic systems. There are several types of planners available, including OMPL, CHOMP, and STOMP. Each planner has its own strengths and weaknesses, and can be used in combination with other planners through the use of Planning Request Adapters. The Open Motion Planning Library (OMPL) is a collection of state-of-the-art sampling-based motion planning algorithms and is the default planner in MoveIt. CHOMP is a gradient-based trajectory optimization procedure that optimizes trajectories based on a cost function, while STOMP is a probabilistic optimization framework that produces smooth collision-free paths within reasonable times.

## Types of Planners

### 1. OMPL

The Open Motion Planning Library is a powerful collection of state-of-the-art sampling-based motion planning algorithms and is the default planner in MoveIt. 

Several planners that are part of the OMPL planning library are capable of optimizing for a specified optimization objective. This tutorial describes that steps that are needed to configure these objectives. The asymptotically (near-)optimal planners that are currently exposed to MoveIt are:

RRT*
PRM*
LazyPRM*
BFMT
FMT
Lower Bound Tree RRT (LBTRRT)
SPARS
SPARS2
Transition-based RRT (T-RRT)
Adaptively Informed Tree (AIT*)
Batched Informed Tree (BIT*)
Advanced Batched Informed Tree (ABIT*)

### 2. CHOMP

Covariant Hamiltonian optimization for motion planning (CHOMP) is a gradient-based trajectory optimization procedure that makes many everyday motion planning problems both simple and trainable. While most high-dimensional motion planners separate trajectory generation into distinct planning and optimization stages, this algorithm capitalizes on covariant gradient and functional gradient approaches to the optimization stage to design a motion planning algorithm based entirely on trajectory optimization. 

Given an infeasible naive trajectory, CHOMP reacts to the surrounding environment to quickly pull the trajectory out of collision while simultaneously optimizing dynamical quantities such as joint velocities and accelerations. It rapidly converges to a smooth collision-free trajectory that can be executed efficiently on the robot.

### 3. STOMP

Stochastic Trajectory Optimization for Motion Planning (STOMP) is a probabilistic optimization framework. STOMP produces smooth well behaved collision free paths within reasonable times. The approach relies on generating noisy trajectories to explore the space around an initial (possibly infeasible) trajectory which are then combined to produce an updated trajectory with lower cost. A cost function based on a combination of obstacle and smoothness cost is optimized in each iteration. No gradient information is required for the particular optimization algorithm that we use and so general costs for which derivatives may not be available (e.g. costs corresponding to constraints and motor torques) can be included in the cost function.

Some of the strengths of STOMP include:

- Incorporates additional objective functions such as torque limits, energy and tool constraints.
- Handles cost functions which do not need to be differentiable.
- Uses distance field and spherical approximations to quickly compute distance queries and collision costs.

## ****Planning Adapter:****

Planning Request Adapters is a concept in MoveIt which can be used to modify the trajectory (pre-processing and/or post-processing) for a motion planner. Some examples of existing planning adapters in MoveIt include AddTimeParameterization, FixWorkspaceBounds, FixStartBounds, FixStartStateCollision, FixStartStatePathConstraints, CHOMPOptimizerAdapter, etc. ! Using the concepts of Planning Adapters, multiple motion planning algorithms can be used in a pipeline to produce robust motion plans. For example, a sample pipeline of motion plans might include an initial plan produced by OMPL which can then be optimized by CHOMP to produce a motion plan which would likely be better than a path produced by OMPL or CHOMP alone. Similarly, using the concept of Planning Adapters, other motion planners can be mixed and matched depending on the environment the robot is operating in.

### OMPL as a pre-processor for CHOMP

OMPL can used as a base planner to produce an initial motion plan which can act as a initial guess for CHOMP. CHOMP can then produce optimized paths. In most cases, the quality of such a path produced should be better than that produced by OMPL alone or CHOMP alone.

### CHOMP as a post-processor for STOMP

CHOMP can be used to produce a path and then STOMP can be used to smoothen the path. This helps in getting rid of the jerky motion of the trajectories produced by CHOMP alone in the presence of obstacles.

### OMPL as a pre-processor for STOMP

As stomp can used as a smoothing algorithm, it can be used to smoothen the plans produced by other motion planners. OMPL first produces a path, STOMP can then generate a smoothened version of that path. Such a path in most cases should be better than a path produced by either just OMPL or STOMP alone.

### STOMP as a post-processor for CHOMP

For this case, a path can be initially produced by STOMP, CHOMP can then take this as an initial guess and produce an optimized version of the smoothened path produced by STOMP.
