---
sidebar_position: 3
---

# Chapter 11: Nav2 Navigation

## Introduction to Nav2

Nav2 is the ROS 2 navigation stack, providing a modular and flexible framework for robots to autonomously navigate complex environments. It is the successor to ROS 1's `navigation` stack, rebuilt from the ground up for ROS 2's distributed, real-time-capable architecture.

## Key Components of Nav2

Nav2 consists of several interconnected ROS 2 nodes, each responsible for a specific aspect of navigation.

```mermaid
graph TD
    A[Sensor Data (LiDAR, Camera)] --> B[Localization (AMCL, VSLAM)]
    B --> C[Global Planner]
    C --> D[Costmap 2D (Global)]
    D --> E[Local Planner]
            E --> F[Costmap 2D (Local)]
            E --> G[Controller (e.g., PID)]
    G --> H[Robot Base Controller (cmd_vel)]
    H --> I[Robot Actuators]
    J[BT Navigator (Behavior Tree)] --> K[Recovery Behaviors]
    J --> L[Goal Follower]
    J --> C
    B --> J
```

### 1. Localization

Estimating the robot's position and orientation within a map.
- **AMCL (Adaptive Monte Carlo Localization)**: Popular for 2D laser-based localization in a known map.
- **VSLAM (Visual SLAM)**: Used for 3D localization and mapping with cameras.

### 2. Global Planner

Generates a collision-free path from the robot's current location to the goal, considering the entire map.
- **NavFn**: Classic Dijkstra's or A* based planner.
- **SmacPlanner**: Highly optimized A* and Dijkstra's variants.

### 3. Local Planner (Controller)

Takes the global path and generates short-term velocity commands to guide the robot along the path while avoiding dynamic obstacles.
- **DWB (Dynamic Window Approach)**: Evaluates possible velocities in a dynamic window.
- **TEB (Timed-Elastic Band)**: Optimizes a robot trajectory for time-optimality, path length, and collision avoidance.

### 4. Costmaps

Represent the environment as a grid, assigning a "cost" to each cell based on proximity to obstacles.
- **Global Costmap**: Static or slowly updating map of the entire environment, used by the global planner.
- **Local Costmap**: Dynamic, smaller map around the robot, used by the local planner for immediate obstacle avoidance.

### 5. Behavior Tree (BT) Navigator

A flexible way to define complex navigation behaviors, including goal following, recovery behaviors, and complex decision-making.

```xml
<!-- Example Behavior Tree Node (simplified) -->
<BehaviorTree ID="NavigateToPose">
  <Sequence name="root">
    <Action ID="ComputePathToGoal" />
    <Action ID="FollowPath" />
    <Action ID="SpinAround" /> <!-- Example recovery behavior -->
  </Sequence>
</BehaviorTree>
```

## Configuring Nav2

Nav2 is configured primarily through YAML files.

```yaml
# Example snippet from a Nav2 configuration file
amcl:
  ros__parameters:
    use_sim_time: true
    set_initial_pose: false
    min_particles: 500
    max_particles: 2000
    initial_pose: [-0.5, 0.0, 0.0]

controller_server:
  ros__parameters:
    use_sim_time: true
    min_x_velocity_threshold: 0.001
    min_y_velocity_threshold: 0.001
    min_theta_velocity_threshold: 0.001
    
    controller_frequency: 20.0
    progress_checker_plugin: "progress_checker"
    goal_checker_plugin: "goal_checker"
    # ... more parameters ...
```

## Nav2 for Humanoid Robots

While Nav2 is typically used for wheeled mobile robots, its modularity allows for adaptation to humanoid navigation.
- **Footstep Planning**: Integrating a footstep planner (like `humanoid_nav_msgs` or custom solutions) with Nav2's global/local planners.
- **Balance Control**: Tying the navigation commands to the humanoid's balance and whole-body control systems.
- **Perception**: Utilizing advanced perception from Isaac ROS or other sensors for 3D obstacle avoidance.

## Launching Nav2

Nav2 is typically launched via a ROS 2 launch file, integrating all its nodes and configurations.

```xml
<!-- Example ROS 2 launch file for Nav2 (simplified) -->
<launch>
  <arg name="map" default="$(find nav2_bringup)/maps/turtlebot3_world.yaml"/>
  <arg name="params_file" default="$(find nav2_bringup)/params/nav2_params.yaml"/>

  <include file="$(find nav2_bringup)/launch/bringup_launch.py">
    <arg name="map" value="$(var map)"/>
    <arg name="params_file" value="$(var params_file)"/>
  </include>
</launch>
```

## Chapter Summary

- Nav2 is the modular ROS 2 navigation stack for autonomous robot movement.
- It comprises localization, global planning, local planning, and costmaps.
- Behavior Trees enable complex navigation logic and recovery.
- Configured via YAML files, allowing flexible parameter tuning.
- Adaptable for humanoid navigation by integrating footstep planning and balance control.

## Exercise

Outline the steps and potential challenges involved in integrating a custom 3D point cloud-based obstacle avoidance system into the Nav2 stack for a humanoid robot. Which Nav2 components would need modification or custom plugins?

---

**Next**: [Module 4: Vision-Language-Action →](chapter-12)
