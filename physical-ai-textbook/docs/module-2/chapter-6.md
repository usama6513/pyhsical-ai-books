---
sidebar_position: 1
---

# Chapter 6: Gazebo Simulation Environment

## Introduction to Gazebo

Gazebo is a powerful 3D robot simulator that is widely used in the robotics community. It allows for testing algorithms, designing robots, and performing complex scenarios in a virtual environment before deploying to real hardware.

## Key Features

- **Physics Engine**: Supports multiple high-performance physics engines (ODE, Bullet, Simbody, DART) for realistic rigid body dynamics.
- **Sensors**: Simulates a wide range of sensors including cameras, LiDAR, IMU, force-torque, and more.
- **Models**: Large collection of pre-built robot and object models.
- **Plugins**: Extend functionality with custom code for robot control, sensor processing, and environment interaction.
- **ROS Integration**: Deep integration with ROS/ROS 2 for seamless robot control and data exchange.

## Gazebo vs Real World

| Feature | Gazebo | Real World |
|---|---|---|
| Safety | High (no physical damage) | Low (can be dangerous) |
| Cost | Low (software only) | High (hardware, maintenance) |
| Reproducibility | High | Low (environmental variables) |
| Speed | Can be faster than real-time | Real-time |
| Fidelity | Limited by models and physics | Perfect |

## Gazebo World Files (`.world`)

World files define the simulation environment, including:
- **Physics properties**: Gravity, friction
- **Lighting**: Ambient light, directional lights
- **Ground plane**: Flat surface
- **Static models**: Buildings, furniture
- **Dynamic models**: Robots, movable objects
- **Sensors**: Cameras, LiDAR attached to models

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <light type="directional" name="sun">
      <cast_shadows>1</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <surface><friction><ode><mu>100</mu><mu2>50</mu></ode></friction></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>100 100</size></plane></geometry>
          <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Green</name></script></material>
        </visual>
      </link>
    </model>
    <include>
      <uri>model://my_robot_model</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
  </world>
</sdf>
```

## Integrating Robots into Gazebo

Robots are typically defined using URDF/SDF files and included in a Gazebo world.
- **URDF**: Unified Robot Description Format (primarily for robot kinematics)
- **SDF**: Simulation Description Format (Gazebo's native format, includes physics, sensors, plugins)

### Launching Gazebo with ROS 2

Gazebo is often launched using ROS 2 launch files.

```xml
<!-- Example ROS 2 launch file for Gazebo -->
<launch>
  <include file="$(find gazebo_ros)/launch/gazebo.launch">
    <arg name="world_name" value="$(find my_robot_pkg)/worlds/my_empty_world.world"/>
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="true"/>
    <arg name="gui" value="true"/>
    <arg name="headless" value="false"/>
    <arg name="debug" value="false"/>
  </include>

  <node name="spawn_entity" pkg="gazebo_ros" exec="spawn_entity.py" output="screen"
        arguments="-topic robot_description -entity my_humanoid"/>
</launch>
```

## Gazebo Plugins

Plugins extend Gazebo's functionality. Common plugins include:
- **ROS 2 control plugin**: Connects robot joints to ROS 2 controllers.
- **Sensor plugins**: Simulate various sensors (camera, LiDAR, IMU).
- **Model plugins**: Add custom behavior to models.

```xml
<gazebo>
  <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
    <robot_param>robot_description</robot_param>
    <robot_param_node>robot_state_publisher</robot_param_node>
    <parameters>$(find my_robot_pkg)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

## Chapter Summary

- Gazebo provides a rich 3D simulation environment for robotics.
- It features realistic physics, sensor simulation, and ROS/ROS 2 integration.
- World files define the simulation scene.
- Robots are integrated using URDF/SDF and spawned via launch files.
- Plugins extend Gazebo's capabilities.

## Exercise

Create a simple Gazebo world file with a ground plane and a single box model. Launch this world using a ROS 2 launch file. If you have a URDF from the previous chapter, try to spawn it into your new world.

---

**Next**: [Chapter 7: URDF vs SDF Formats →](chapter-7)
