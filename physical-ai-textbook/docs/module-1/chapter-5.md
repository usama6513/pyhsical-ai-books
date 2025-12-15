---
sidebar_position: 5
---

# Chapter 5: URDF Modeling for Humanoids

## What is URDF?

URDF (Unified Robot Description Format) is an XML format for describing robotic models. It's used in ROS for representing the robot's kinematics, dynamics, visual properties, and collision properties.

## Key Elements of URDF

### `<robot>`

The root element that encapsulates the entire robot description.

```xml
<robot name="my_humanoid">
    <!-- Links and Joints go here -->
</robot>
```

### `<link>`

Represents a rigid body of the robot, such as a limb, torso, or head. Each link has:
- **Visual properties**: `color`, `geometry` (box, cylinder, sphere, mesh)
- **Collision properties**: Simplified shapes for collision detection
- **Inertial properties**: `mass`, `inertia` matrix

```xml
<link name="base_link">
    <visual>
        <geometry><box size="0.1 0.1 0.1" /></geometry>
        <material name="blue"><color rgba="0 0 0.8 1" /></material>
    </visual>
    <collision>
        <geometry><box size="0.1 0.1 0.1" /></geometry>
    </collision>
    <inertial>
        <mass value="1.0" />
        <inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01" />
    </inertial>
</link>
```

### `<joint>`

Connects two links, defining their kinematic relationship and axis of motion.

#### Common Joint Types:
- **Revolute**: Rotational joint (e.g., elbow, knee)
- **Continuous**: Revolute joint with unlimited range (e.g., wheel)
- **Prismatic**: Linear joint (e.g., linear actuator)
- **Fixed**: No motion between links (e.g., camera attached to a head)

```xml
<joint name="joint_name" type="revolute">
    <parent link="parent_link_name" />
    <child link="child_link_name" />
    <origin xyz="X Y Z" rpy="ROLL PITCH YAW" /> <!-- Position and orientation -->
    <axis xyz="X Y Z" /> <!-- Axis of rotation for revolute/continuous/prismatic -->
    <limit lower="-1.57" upper="1.57" effort="100" velocity="10" /> <!-- Joint limits -->
</joint>
```

## Building a Simple Humanoid Leg (Example)

Let's model a basic two-segment leg.

```xml
<robot name="simple_leg">

    <link name="hip_link">
        <visual><geometry><cylinder radius="0.05" length="0.1" /></geometry></visual>
        <inertial><mass value="0.5" /><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001" /></inertial>
    </link>

    <joint name="hip_knee_joint" type="revolute">
        <parent link="hip_link" />
        <child link="thigh_link" />
        <origin xyz="0 0 -0.05" rpy="0 0 0" />
        <axis xyz="1 0 0" />
        <limit lower="-1.57" upper="1.57" effort="50" velocity="2.0" />
    </joint>

    <link name="thigh_link">
        <visual><geometry><box size="0.05 0.05 0.2" /></geometry></visual>
        <inertial><mass value="1.0" /><inertia ixx="0.005" ixy="0" ixz="0" iyy="0.005" iyz="0" izz="0.001" /></inertial>
    </link>

    <joint name="knee_ankle_joint" type="revolute">
        <parent link="thigh_link" />
        <child link="shin_link" />
        <origin xyz="0 0 -0.1" rpy="0 0 0" />
        <axis xyz="1 0 0" />
        <limit lower="-1.57" upper="0" effort="50" velocity="2.0" />
    </joint>

    <link name="shin_link">
        <visual><geometry><box size="0.05 0.05 0.2" /></geometry></visual>
        <inertial><mass value="0.7" /><inertia ixx="0.003" ixy="0" ixz="0" iyy="0.003" iyz="0" izz="0.001" /></inertial>
    </link>

</robot>
```

## XACRO (XML Macros for ROS)

For complex robots, URDF files can become very long and repetitive. XACRO extends URDF with macros and other programming constructs, making it more modular and readable.

### Why use XACRO?
- **Modularity**: Define common components once (e.g., a standard servo motor)
- **Parameters**: Pass arguments to macros (e.g., limb length, joint limits)
- **Mathematical expressions**: Calculate values dynamically

```xml
<!-- Example XACRO macro for a generic joint -->
<xacro:macro name="standard_joint" params="prefix parent child xyz rpy">
    <joint name="${prefix}_joint" type="revolute">
        <parent link="${parent}" />
        <child link="${child}" />
        <origin xyz="${xyz}" rpy="${rpy}" />
        <axis xyz="0 0 1" />
        <limit lower="-${pi/2}" upper="${pi/2}" effort="100" velocity="10" />
    </joint>
</xacro:macro>

<!-- Usage -->
<xacro:standard_joint prefix="shoulder" parent="torso_link" child="upper_arm_link" xyz="0 0 0.1" rpy="0 0 0" />
```

## Visualizing URDF Models

Tools like `rviz` (in ROS) can load and display URDF models, allowing for verification of kinematics, joint limits, and collision geometry.

```bash
# Example command to display a URDF in RViz
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```

## Chapter Summary

- URDF describes robot kinematics, dynamics, visual, and collision properties.
- Key elements are `<robot>`, `<link>`, and `<joint>`.
- XACRO simplifies complex URDF models using macros.
- Visualization tools like RViz are essential for debugging models.

## Exercise

Extend the `simple_leg` URDF example to include a foot link and an ankle joint. Define appropriate limits and inertial properties. Convert this URDF to a XACRO file, defining a macro for a standard leg segment.

---

**Next**: [Module 2: Digital Twin →](../module-2/chapter-6)
