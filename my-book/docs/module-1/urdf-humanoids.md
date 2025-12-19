---
sidebar_position: 3
title: "URDF for Humanoids"
description: "Understanding URDF fundamentals and humanoid robot structures for Physical AI & Humanoid Robotics"
---

# URDF for Humanoids: Robot Description in ROS 2

## Learning Objectives

After completing this chapter, you will be able to:
- Understand URDF (Unified Robot Description Format) fundamentals
- Create and interpret URDF files for humanoid robots
- Define links and joints for complex humanoid structures
- Map AI logic to physical robot components using URDF
- Integrate rclpy with URDF-based robot models

## Introduction to URDF

URDF (Unified Robot Description Format) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid bodies), joints (connections between links), and other properties like inertial characteristics and visual appearance.

For humanoid robotics, URDF is particularly important because it provides a standardized way to describe the complex kinematic structure of human-like robots with multiple degrees of freedom. This description is essential for simulation, visualization, motion planning, and control.

### Why URDF Matters for Humanoid Robotics

Humanoid robots have complex kinematic structures with many interconnected parts. URDF provides:

- **Kinematic Structure**: Defines how different body parts are connected
- **Physical Properties**: Mass, center of mass, and inertial properties for each link
- **Visual Representation**: How the robot appears in simulation and visualization tools
- **Collision Properties**: How the robot interacts with the environment in simulation
- **Sensor Mounting Points**: Where sensors are physically attached on the robot

## Links and Joints: The Building Blocks

### Links

A link represents a rigid body in the robot. In a humanoid robot, links might represent:
- Torso
- Head
- Upper arms
- Lower arms
- Hands
- Upper legs
- Lower legs
- Feet

Each link has properties like mass, visual appearance, and collision geometry:

```xml
<link name="upper_arm">
  <inertial>
    <mass value="2.0" />
    <origin xyz="0 0 0.1" />
    <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01" />
  </inertial>
  <visual>
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <geometry>
      <cylinder length="0.2" radius="0.05" />
    </geometry>
    <material name="blue">
      <color rgba="0 0 1 1" />
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0.1" rpy="0 0 0" />
    <geometry>
      <cylinder length="0.2" radius="0.05" />
    </geometry>
  </collision>
</link>
```

### Joints

A joint connects two links and defines their relative motion. In humanoid robots, joints typically represent:
- Shoulder joints (ball joints with multiple degrees of freedom)
- Elbow joints (revolute joints)
- Hip joints (ball joints)
- Knee joints (revolute joints)
- Ankle joints (revolute joints)

Joints can be of different types:
- **Fixed**: No movement between links
- **Revolute**: Single axis of rotation
- **Continuous**: Like revolute but unlimited rotation
- **Prismatic**: Single axis of translation
- **Floating**: 6 DOF with no constraints
- **Planar**: Movement on a plane

```xml
<joint name="elbow_joint" type="revolute">
  <parent link="upper_arm" />
  <child link="lower_arm" />
  <origin xyz="0 0 0.2" rpy="0 0 0" />
  <axis xyz="0 1 0" />
  <limit lower="-2.0" upper="2.0" effort="100" velocity="1.0" />
</joint>
```

## Complete Humanoid URDF Example

Here's a simplified example of a humanoid robot URDF:

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    </visual>
  </link>

  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.5"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1"/>
      </material>
    </visual>
  </link>

  <joint name="torso_joint" type="fixed">
    <parent link="base_link"/>
    <child link="torso"/>
    <origin xyz="0 0 0.3"/>
  </joint>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.2 0 0.1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <visual>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left Hand -->
  <link name="left_hand">
    <visual>
      <geometry>
        <box size="0.1 0.08 0.05"/>
      </geometry>
      <material name="skin">
        <color rgba="1 0.8 0.6 1"/>
      </material>
    </visual>
  </link>

  <joint name="left_wrist_joint" type="revolute">
    <parent link="left_lower_arm"/>
    <child link="left_hand"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-0.78" upper="0.78" effort="5" velocity="1"/>
  </joint>

  <!-- Similar definitions for right arm, legs, etc. -->
</robot>
```

## Mapping AI Logic to Robot Components

One of the key challenges in humanoid robotics is mapping high-level AI decisions to specific robot components defined in URDF. This is where rclpy comes in, as it allows AI agents to interact with the robot's control system.

### Using TF2 with URDF

The Transform Library (TF2) in ROS 2 uses the URDF to understand the kinematic relationships between robot components. This allows AI agents to:

- Determine the position and orientation of any part of the robot
- Plan movements relative to the robot's body parts
- Coordinate multiple parts of the robot for complex actions

```python
import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped

class RobotStateNode(Node):
    def __init__(self):
        super().__init__('robot_state_node')

        # Create TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Timer to periodically check transforms
        self.timer = self.create_timer(0.1, self.check_robot_state)

    def check_robot_state(self):
        try:
            # Get transform between torso and head
            trans = self.tf_buffer.lookup_transform(
                'torso', 'head',
                rclpy.time.Time())

            self.get_logger().info(
                f'Head position relative to torso: '
                f'x={trans.transform.translation.x}, '
                f'y={trans.transform.translation.y}, '
                f'z={trans.transform.translation.z}'
            )
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotStateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Working with URDF in Python

While URDF files are XML-based, ROS 2 provides tools to work with them in Python. You can load URDF models, access their properties, and use them in your AI agents:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import xml.etree.ElementTree as ET

class URDFControllerNode(Node):
    def __init__(self):
        super().__init__('urdf_controller_node')

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Publish joint commands
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10)

        # Load URDF information (simplified)
        self.joint_limits = self.load_joint_limits()

        self.get_logger().info('URDF Controller Node initialized')

    def load_joint_limits(self):
        # In a real implementation, you might load this from the URDF
        # or from a parameter server
        return {
            'left_shoulder_joint': (-1.57, 1.57),
            'left_elbow_joint': (-1.57, 1.57),
            'right_shoulder_joint': (-1.57, 1.57),
            'right_elbow_joint': (-1.57, 1.57),
        }

    def joint_state_callback(self, msg):
        # Process joint states and apply AI logic
        # This is where your AI agent would make decisions based on joint positions
        self.get_logger().info(f'Received joint states for {len(msg.name)} joints')

        # Example: Simple AI decision based on current joint states
        commands = self.make_ai_decision(msg)

        # Publish commands
        cmd_msg = Float64MultiArray()
        cmd_msg.data = commands
        self.joint_cmd_pub.publish(cmd_msg)

    def make_ai_decision(self, joint_state):
        # This is where your AI logic would go
        # For example, return target joint angles based on current state
        commands = []
        for i, name in enumerate(joint_state.name):
            if name in self.joint_limits:
                # Simple example: move toward center position
                current_pos = joint_state.position[i]
                target_pos = 0.0  # Center position
                commands.append(target_pos)
            else:
                commands.append(current_pos)  # No change for other joints

        return commands

def main(args=None):
    rclpy.init(args=args)
    node = URDFControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## URDF Best Practices for Humanoid Robots

### 1. Hierarchical Structure
Organize your URDF with a clear hierarchical structure that reflects the physical robot:
- Start with a base link
- Build up the body in a logical sequence
- Use meaningful names for links and joints

### 2. Proper Inertial Properties
Accurate inertial properties are crucial for simulation and control:
- Mass values should reflect real hardware
- Inertia tensors should be properly calculated
- Center of mass should be accurately positioned

### 3. Appropriate Joint Limits
Set realistic joint limits that match your physical robot:
- Prevents impossible movements
- Protects the robot from damage
- Makes simulation more realistic

### 4. Collision and Visual Separation
Keep collision and visual geometries appropriately defined:
- Visual geometry for rendering
- Collision geometry for physics simulation
- Often different shapes for performance vs. accuracy

## Summary

This chapter covered the fundamentals of URDF (Unified Robot Description Format) for humanoid robots. We explored how URDF defines the kinematic structure of robots through links and joints, provided examples of complete humanoid URDF files, and demonstrated how to integrate URDF with rclpy for AI-robot interaction. Understanding URDF is crucial for connecting AI decision-making to physical robot actions in humanoid robotics applications. The ability to map high-level AI concepts to specific robot components defined in URDF enables the creation of intelligent humanoid robots that can interact with their environment effectively.

## Next Steps

Congratulations! You've completed Module 1: The Robotic Nervous System (ROS 2). You now understand:
- The fundamentals of ROS 2 as middleware for robotics
- How to create Python AI agents using rclpy
- How to describe robot models using URDF

This foundation provides you with the essential knowledge to work with humanoid robots and integrate AI systems with robotic platforms.