---
title: "URDF for Humanoids: Describing Robot Structure"
sidebar_label: "URDF for Humanoids"
description: "Understanding Unified Robot Description Format (URDF) for describing humanoid robot structures and connecting AI logic to robot hardware"
slug: /module-1/urdf-humanoids
---

# URDF for Humanoids: Describing Robot Structure

## Learning Objectives

By the end of this chapter, you will be able to:
- Understand the fundamentals of Unified Robot Description Format (URDF)
- Create URDF files for humanoid robot structures
- Define links and joints for humanoid robots
- Connect AI agents with robot descriptions using URDF
- Apply URDF concepts to physical AI and humanoid robotics

## Introduction to URDF

Unified Robot Description Format (URDF) is an XML-based format used in ROS to describe robot models. It defines the physical and visual properties of a robot, including its links (rigid bodies), joints (connections between links), and other properties like inertia, visual representation, and collision properties.

URDF is fundamental to robotics simulation, visualization, and control. For humanoid robots, URDF provides the essential bridge between AI algorithms and the physical robot structure.

## URDF Fundamentals

### Links

Links represent rigid bodies in a robot. Each link has:
- Physical properties (mass, inertia)
- Visual properties (geometry, material, color)
- Collision properties (collision geometry)

Basic link structure:
```xml
<link name="link_name">
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="1.0"/>
    <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
  </inertial>
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
    <material name="blue">
      <color rgba="0 0 0.8 1"/>
    </material>
  </visual>
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="1 1 1"/>
    </geometry>
  </collision>
</link>
```

### Joints

Joints connect links and define how they can move relative to each other. Common joint types:
- **fixed**: No movement between links
- **revolute**: Rotational movement around an axis
- **continuous**: Continuous rotational movement
- **prismatic**: Linear sliding movement
- **floating**: 6-DOF movement
- **planar**: Movement in a plane

Basic joint structure:
```xml
<joint name="joint_name" type="revolute">
  <parent link="parent_link_name"/>
  <child link="child_link_name"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
</joint>
```

## Humanoid Structure: Links and Joints

Humanoid robots require specific link and joint configurations to mimic human-like movement. A typical humanoid structure includes:

### Body Structure
- **base_link**: The main body/chest of the robot
- **head**: Connected via neck joint
- **left_arm**: Shoulder, elbow, wrist joints
- **right_arm**: Shoulder, elbow, wrist joints
- **left_leg**: Hip, knee, ankle joints
- **right_leg**: Hip, knee, ankle joints

### Example Humanoid URDF Structure

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">
  <!-- Base Link -->
  <link name="base_link">
    <inertial>
      <mass value="10.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
      <material name="grey">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0.5"/>
      <geometry>
        <box size="0.3 0.3 1.0"/>
      </geometry>
    </collision>
  </link>

  <!-- Head -->
  <link name="head">
    <inertial>
      <mass value="2.0"/>
      <origin xyz="0 0 0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
    <visual>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
      <material name="white">
        <color rgba="1.0 1.0 1.0 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0"/>
      <geometry>
        <sphere radius="0.15"/>
      </geometry>
    </collision>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 1.0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.5" upper="0.5" effort="10" velocity="1"/>
  </joint>

  <!-- Left Arm -->
  <link name="left_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_arm"/>
    <origin xyz="0.15 0 0.7"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="left_lower_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_upper_arm"/>
    <child link="left_lower_arm"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Right Arm (similar structure) -->
  <link name="right_upper_arm">
    <inertial>
      <mass value="1.0"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_arm"/>
    <origin xyz="-0.15 0 0.7"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <link name="right_lower_arm">
    <inertial>
      <mass value="0.8"/>
      <origin xyz="0 0 -0.15"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0.2 0.2 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_upper_arm"/>
    <child link="right_lower_arm"/>
    <origin xyz="0 0 -0.3"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1"/>
  </joint>

  <!-- Left Leg -->
  <link name="left_upper_leg">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_upper_leg"/>
    <origin xyz="0.07 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="left_lower_leg">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_upper_leg"/>
    <child link="left_lower_leg"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <!-- Right Leg -->
  <link name="right_upper_leg">
    <inertial>
      <mass value="1.5"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.06"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_upper_leg"/>
    <origin xyz="-0.07 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="20" velocity="1"/>
  </joint>

  <link name="right_lower_leg">
    <inertial>
      <mass value="1.2"/>
      <origin xyz="0 0 -0.25"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0" iyy="0.02" iyz="0.0" izz="0.02"/>
    </inertial>
    <visual>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.25"/>
      <geometry>
        <cylinder length="0.5" radius="0.05"/>
      </geometry>
    </collision>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_upper_leg"/>
    <child link="right_lower_leg"/>
    <origin xyz="0 0 -0.5"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="1.57" effort="20" velocity="1"/>
  </joint>
</robot>
```

## Connecting AI Logic to ROS Nodes

### Using URDF with Python AI Agents

AI agents can access robot structure information through URDF to understand the robot's capabilities and constraints:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformListener
from geometry_msgs.msg import TransformStamped
import xml.etree.ElementTree as ET

class AIWithURDFNode(Node):
    def __init__(self):
        super().__init__('ai_with_urdf_node')

        # Subscribe to joint states
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Access URDF information
        self.urdf_string = self.get_parameter_or_set_default('robot_description', '')
        if self.urdf_string:
            self.parse_urdf()

        # Timer for AI processing
        self.ai_timer = self.create_timer(0.1, self.ai_processing_callback)

        # Store joint information from URDF
        self.joint_limits = {}
        self.link_masses = {}

    def get_parameter_or_set_default(self, param_name, default_value):
        """Helper to get parameter with default"""
        self.declare_parameter(param_name, default_value)
        return self.get_parameter(param_name).value

    def parse_urdf(self):
        """Parse URDF to extract joint limits and link properties"""
        try:
            root = ET.fromstring(self.urdf_string)

            # Extract joint information
            for joint in root.findall('joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')

                if joint_type in ['revolute', 'prismatic']:
                    limit = joint.find('limit')
                    if limit is not None:
                        self.joint_limits[joint_name] = {
                            'lower': float(limit.get('lower', -3.14)),
                            'upper': float(limit.get('upper', 3.14)),
                            'effort': float(limit.get('effort', 10.0)),
                            'velocity': float(limit.get('velocity', 1.0))
                        }

            # Extract link information
            for link in root.findall('link'):
                link_name = link.get('name')
                inertial = link.find('inertial')
                if inertial is not None:
                    mass = inertial.find('mass')
                    if mass is not None:
                        self.link_masses[link_name] = float(mass.get('value', 1.0))

            self.get_logger().info(f'Parsed URDF: {len(self.joint_limits)} joints, {len(self.link_masses)} links')

        except Exception as e:
            self.get_logger().error(f'Error parsing URDF: {e}')

    def joint_state_callback(self, msg):
        """Process joint state information"""
        for i, name in enumerate(msg.name):
            if name in self.joint_limits:
                # Check if joint is within limits
                position = msg.position[i]
                limits = self.joint_limits[name]

                if not (limits['lower'] <= position <= limits['upper']):
                    self.get_logger().warn(f'Joint {name} out of limits: {position}')

    def ai_processing_callback(self):
        """Main AI processing logic using URDF information"""
        # Example: Use URDF information to make decisions
        # Check if any joints are near their limits
        for joint_name, limits in self.joint_limits.items():
            # AI logic could use URDF information to plan movements
            # that respect joint limits and physical constraints
            pass

def main(args=None):
    rclpy.init(args=args)
    ai_node = AIWithURDFNode()
    rclpy.spin(ai_node)
    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Mapping AI Logic to ROS Nodes

### The Bridge Between AI and Hardware

In humanoid robotics, AI agents need to understand the robot's physical structure to make informed decisions. The connection between AI logic and robot hardware involves:

1. **URDF Parsing**: AI agents can parse URDF to understand the robot's kinematic structure
2. **Joint State Monitoring**: Using joint state information to track the robot's current configuration
3. **Inverse Kinematics**: Using the kinematic chain defined in URDF to calculate joint angles for desired end-effector positions
4. **Safety Constraints**: Respecting joint limits and physical constraints defined in URDF

### Example: Safe Movement Planning

```python
class SafeMovementPlanner(Node):
    def __init__(self):
        super().__init__('safe_movement_planner')

        # Publisher for joint commands
        self.joint_command_publisher = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )

        # Access robot description
        self.urdf_string = self.get_parameter_or_set_default('robot_description', '')
        if self.urdf_string:
            self.parse_urdf_for_constraints()

        # Timer for planning
        self.planning_timer = self.create_timer(0.1, self.planning_callback)

    def get_parameter_or_set_default(self, param_name, default_value):
        """Helper to get parameter with default"""
        self.declare_parameter(param_name, default_value)
        return self.get_parameter(param_name).value

    def parse_urdf_for_constraints(self):
        """Parse URDF specifically for safety constraints"""
        try:
            root = ET.fromstring(self.urdf_string)
            self.joint_constraints = {}

            for joint in root.findall('joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')

                if joint_type in ['revolute', 'prismatic']:
                    limit = joint.find('limit')
                    if limit is not None:
                        self.joint_constraints[joint_name] = {
                            'lower': float(limit.get('lower', -3.14)),
                            'upper': float(limit.get('upper', 3.14)),
                            'effort': float(limit.get('effort', 10.0)),
                            'velocity': float(limit.get('velocity', 1.0))
                        }

            self.get_logger().info('Loaded safety constraints from URDF')

        except Exception as e:
            self.get_logger().error(f'Error parsing URDF for constraints: {e}')

    def validate_joint_commands(self, joint_state_msg):
        """Validate joint commands against URDF constraints"""
        valid_commands = JointState()
        valid_commands.name = []
        valid_commands.position = []
        valid_commands.velocity = []
        valid_commands.effort = []

        for i, name in enumerate(joint_state_msg.name):
            if name in self.joint_constraints:
                constraints = self.joint_constraints[name]
                pos = joint_state_msg.position[i]

                # Clamp position to limits with safety margin
                safety_margin = 0.1  # 10% safety margin
                lower_safe = constraints['lower'] + safety_margin
                upper_safe = constraints['upper'] - safety_margin

                clamped_pos = max(lower_safe, min(upper_safe, pos))

                valid_commands.name.append(name)
                valid_commands.position.append(clamped_pos)

                # Also validate velocity and effort if provided
                if i < len(joint_state_msg.velocity):
                    vel = joint_state_msg.velocity[i]
                    max_vel = constraints['velocity']
                    clamped_vel = max(-max_vel, min(max_vel, vel))
                    valid_commands.velocity.append(clamped_vel)
                else:
                    valid_commands.velocity.append(0.0)

                if i < len(joint_state_msg.effort):
                    eff = joint_state_msg.effort[i]
                    max_eff = constraints['effort']
                    clamped_eff = max(-max_eff, min(max_eff, eff))
                    valid_commands.effort.append(clamped_eff)
                else:
                    valid_commands.effort.append(0.0)
            else:
                # If joint not in constraints, pass through (might be a sensor joint)
                valid_commands.name.append(name)
                if i < len(joint_state_msg.position):
                    valid_commands.position.append(joint_state_msg.position[i])
                else:
                    valid_commands.position.append(0.0)

                if i < len(joint_state_msg.velocity):
                    valid_commands.velocity.append(joint_state_msg.velocity[i])
                else:
                    valid_commands.velocity.append(0.0)

                if i < len(joint_state_msg.effort):
                    valid_commands.effort.append(joint_state_msg.effort[i])
                else:
                    valid_commands.effort.append(0.0)

        return valid_commands

    def planning_callback(self):
        """Main planning logic with safety validation"""
        # Example: Generate some joint commands
        desired_joints = JointState()
        desired_joints.name = ['left_shoulder_joint', 'right_shoulder_joint']
        desired_joints.position = [0.5, -0.5]  # Some desired positions
        desired_joints.velocity = [0.1, 0.1]   # Desired velocities
        desired_joints.effort = [5.0, 5.0]     # Desired efforts

        # Validate commands against URDF constraints
        safe_commands = self.validate_joint_commands(desired_joints)

        # Publish safe commands
        self.joint_command_publisher.publish(safe_commands)
        self.get_logger().info('Published safe joint commands')

def main(args=None):
    rclpy.init(args=args)
    planner_node = SafeMovementPlanner()
    rclpy.spin(planner_node)
    planner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## URDF Examples for Humanoid Robots

### Advanced URDF Features

For humanoid robots, URDF can include additional features:

- **Transmission elements**: Define how actuators connect to joints
- **Gazebo plugins**: Simulation-specific properties
- **Materials and colors**: Visual properties for simulation and visualization

```xml
<!-- Example with transmission -->
<transmission name="left_shoulder_trans">
  <type>transmission_interface/SimpleTransmission</type>
  <joint name="left_shoulder_joint">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
  </joint>
  <actuator name="left_shoulder_motor">
    <hardwareInterface>hardware_interface/EffortJointInterface</hardwareInterface>
    <mechanicalReduction>1</mechanicalReduction>
  </actuator>
</transmission>

<!-- Gazebo-specific properties -->
<gazebo reference="base_link">
  <material>Gazebo/Grey</material>
  <mu1>0.2</mu1>
  <mu2>0.2</mu2>
</gazebo>
```

## Summary

URDF is the fundamental format for describing robot structures in ROS, making it essential for humanoid robotics. By defining links and joints with appropriate physical properties, URDF enables simulation, visualization, and control of humanoid robots. AI agents can leverage URDF information to understand the robot's kinematic structure, respect physical constraints, and generate safe movements.

The connection between AI logic and robot hardware through URDF is crucial for developing intelligent humanoid robots that can safely interact with their environment. Understanding URDF fundamentals is key to bridging the gap between high-level AI algorithms and low-level robot control.

## Next Steps

- Review [ROS 2 Basics](./ros2-basics.md) for fundamental ROS 2 concepts
- Explore [Python Agents & rclpy](./python-agents-rclpy.md) to understand Python integration