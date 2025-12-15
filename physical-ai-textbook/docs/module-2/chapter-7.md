---
sidebar_position: 2
---

# Chapter 7: URDF vs SDF Formats

## Introduction

When working with robot models in ROS and Gazebo, you'll encounter two primary XML formats: URDF (Unified Robot Description Format) and SDF (Simulation Description Format). While both describe robots and environments, they have different design philosophies and use cases.

## URDF (Unified Robot Description Format)

### Purpose
- Primarily designed for ROS to describe the kinematic and dynamic properties of a single robot.
- Focuses on the robot's structure, joints, and sensors for motion planning, control, and visualization in ROS tools like RViz.

### Key Characteristics
- **Single Robot**: Describes one robot at a time.
- **Kinematics & Dynamics**: Strong focus on the robot's links, joints (types, limits, axes), and inertial properties.
- **Limited Environment**: Cannot describe the environment, only the robot itself.
- **No Physics Engine Tags**: Does not contain tags specific to physics engines (e.g., collision parameters, friction coefficients for a specific simulator).
- **Extensible**: Often extended with XACRO for modularity.

### Example URDF Element
```xml
<link name="base_link">
    <inertial>
        <mass value="1.0" />
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
    <visual>
        <geometry><box size="0.1 0.1 0.1" /></geometry>
    </visual>
    <collision>
        <geometry><box size="0.1 0.1 0.1" /></geometry>
    </collision>
</link>
```

## SDF (Simulation Description Format)

### Purpose
- The native XML format for Gazebo, designed to fully describe *everything* in a simulation environment.
- Can describe robots, static objects, dynamic objects, sensors, lights, and even the physics engine parameters.
- Comprehensive for simulation, including specific collision properties, friction, and sensor noise models.

### Key Characteristics
- **Multi-Object & Environment**: Can describe multiple robots and complex environments (worlds).
- **Full Simulation Properties**: Includes detailed physics properties (e.g., friction, damping, restitution), sensor noise models, and plugin configurations for Gazebo.
- **Self-Contained**: A single SDF file can define an entire simulation scenario.
- **Extensible**: Can use `include` tags to incorporate other models or SDF snippets.

### Example SDF Element
```xml
<model name="my_box">
  <static>true</static>
  <link name="link">
    <pose>0 0 0.5 0 0 0</pose>
    <collision name="collision">
      <geometry><box><size>1 1 1</size></box></geometry>
      <surface>
        <friction><ode><mu>0.5</mu><mu2>0.5</mu></ode></friction>
      </surface>
    </collision>
    <visual name="visual">
      <geometry><box><size>1 1 1</size></box></geometry>
      <material><script><uri>file://media/materials/scripts/gazebo.material</uri><name>Gazebo/Green</name></script></material>
    </visual>
  </link>
</model>
```

## Comparison: URDF vs SDF

| Feature              | URDF                               | SDF                                     |
|----------------------|------------------------------------|-----------------------------------------|
| **Primary Use**      | Robot description for ROS          | Full simulation environment for Gazebo |
| **Scope**            | Single robot                       | Multiple robots + environment           |
| **Physics Props**    | Basic inertial, kinematics         | Detailed physics (friction, damping)    |
| **Environment**      | No                                 | Yes (lights, ground, static objects)    |
| **Plugins**          | No direct support                  | Direct support for Gazebo plugins       |
| **Sensor Modeling**  | Basic (for ROS data types)         | Advanced (noise models, physics interaction) |
| **Extensibility**    | XACRO                              | `include` tags, more programmatic       |
| **Conversion**       | URDF can be converted to SDF       | SDF cannot be directly converted to URDF |

## Conversion Between URDF and SDF

- **URDF to SDF**: Gazebo can directly parse URDF files, or you can convert them to SDF using tools like `urdfdom_py`. When Gazebo loads a URDF, it essentially creates an SDF representation internally. This conversion adds default physics properties that were missing in the URDF.
- **SDF to URDF**: Not possible in general because SDF contains more information (environment, physics engine specifics, multiple models) that URDF cannot represent.

## Best Practices

- **Use URDF for your robot's core description**: Define the robot's kinematic and dynamic properties in URDF (or XACRO for complex robots). This keeps your robot definition clean and portable across ROS tools.
- **Use SDF for the Gazebo world**: Define your simulation environment, including static objects, lights, and the specific physics properties, in an SDF world file.
- **Embed URDF in SDF**: When launching a Gazebo simulation, you often specify your robot's URDF file, and Gazebo integrates it into the SDF world.

## Chapter Summary

- URDF describes robots for ROS (kinematics, dynamics).
- SDF describes entire simulation environments for Gazebo (robots, environment, physics).
- URDF is generally for a single robot, SDF for multiple models and worlds.
- Conversion from URDF to SDF is possible; the reverse is not.
- Use URDF for robot definition and SDF for world definition.

## Exercise

Given a simple mobile robot defined in URDF, identify which aspects would need to be added or modified if you were to represent it fully in SDF for a Gazebo simulation that includes custom ground friction and a specific camera noise model.

---

**Next**: [Chapter 8: Unity Integration →](chapter-8)
