---
sidebar_position: 2
title: "Physics Simulation with Gazebo"
description: "Using Gazebo for physics-based simulation in Physical AI & Humanoid Robotics"
---

# Physics Simulation with Gazebo

## Learning Objectives

After completing this chapter, you will be able to:
- Understand Gazebo's physics simulation capabilities
- Configure gravity, collision detection, and environmental physics
- Simulate humanoid robot interactions with environments
- Implement sensor simulation for LiDAR, depth cameras, and IMUs
- Integrate Gazebo with ROS 2 for realistic robot simulation

## Introduction to Gazebo Physics Simulation

Gazebo is a powerful robotics simulator that provides accurate physics simulation, realistic rendering, and convenient programmatic interfaces. For humanoid robotics, Gazebo offers sophisticated physics capabilities that enable realistic simulation of robot-environment interactions.

Gazebo uses the Open Dynamics Engine (ODE), Bullet Physics, or DART as its underlying physics engines, providing accurate simulation of rigid body dynamics, collisions, and contact forces. This makes it ideal for testing humanoid robots that must navigate complex environments and interact with objects.

## Simulating Physics, Gravity, and Collisions

### Physics Engine Configuration

Gazebo allows you to configure various physics parameters to match your simulation needs:

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
  <real_time_update_rate>1000.0</real_time_update_rate>
  <gravity>0 0 -9.8</gravity>
</physics>
```

### Gravity Simulation

Gravity is a fundamental force in robotics simulation. Gazebo accurately simulates gravitational forces, which is critical for humanoid robots that must maintain balance and locomotion. The default gravity vector is (0, 0, -9.8) m/s², representing Earth's gravitational acceleration.

For humanoid robots, proper gravity simulation is essential for:
- Balance control algorithms
- Walking gait development
- Fall detection and recovery
- Manipulation tasks involving weight

### Collision Detection

Gazebo provides multiple collision detection algorithms and allows for detailed collision geometry specification. For humanoid robots, collision detection is crucial for:

- Self-collision avoidance
- Environment interaction
- Contact force calculation
- Safety systems

Collision properties can be defined in SDF (Simulation Description Format) files:

```xml
<collision name="collision">
  <geometry>
    <box>
      <size>0.1 0.1 0.1</size>
    </box>
  </geometry>
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>
        <mu2>1.0</mu2>
      </ode>
    </friction>
  </surface>
</collision>
```

## Humanoid Robot Interaction with Environments

### Environment Modeling

Creating realistic environments is crucial for humanoid robot simulation. Gazebo provides tools to model various environments:

- Indoor spaces (offices, homes, laboratories)
- Outdoor terrains (grass, pavement, stairs)
- Obstacle courses for navigation testing
- Interaction objects (furniture, tools, doors)

### Terrain Simulation

Gazebo supports various terrain types:
- Flat surfaces for basic testing
- Uneven terrain for walking challenges
- Stairs and ramps for locomotion testing
- Dynamic obstacles for navigation

### Contact and Force Simulation

Humanoid robots interact with their environment through contact forces. Gazebo accurately simulates these forces, enabling:
- Balance control development
- Manipulation skill training
- Gait optimization
- Safety system validation

## Sensor Simulation: LiDAR, Depth Cameras, IMUs

### LiDAR Simulation

Gazebo provides realistic LiDAR simulation that models:
- Range accuracy and noise
- Angular resolution
- Field of view
- Maximum detection range

Example LiDAR sensor configuration:

```xml
<sensor name="lidar" type="ray">
  <ray>
    <scan>
      <horizontal>
        <samples>720</samples>
        <resolution>1</resolution>
        <min_angle>-1.570796</min_angle>
        <max_angle>1.570796</max_angle>
      </horizontal>
    </scan>
    <range>
      <min>0.1</min>
      <max>30.0</max>
      <resolution>0.01</resolution>
    </range>
  </ray>
  <plugin name="lidar_controller" filename="libgazebo_ros_ray_sensor.so">
    <ros>
      <namespace>lidar</namespace>
      <remapping>~/out:=scan</remapping>
    </ros>
    <output_type>sensor_msgs/LaserScan</output_type>
  </plugin>
</sensor>
```

### Depth Camera Simulation

Depth cameras in Gazebo simulate RGB-D sensors like the Intel RealSense or Microsoft Kinect. These sensors provide both color and depth information:

- RGB image data
- Depth information
- Point cloud generation
- Noise modeling

### IMU Simulation

Inertial Measurement Units (IMUs) are critical for humanoid robot balance and orientation. Gazebo simulates IMU data including:
- Accelerometer readings
- Gyroscope measurements
- Orientation estimates
- Noise characteristics

Example IMU configuration:

```xml
<sensor name="imu_sensor" type="imu">
  <always_on>true</always_on>
  <update_rate>100</update_rate>
  <topic>imu/data</topic>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
    <ros>
      <namespace>imu</namespace>
      <remapping>~/out:=data</remapping>
    </ros>
    <initial_orientation_as_reference>false</initial_orientation_as_reference>
  </plugin>
</sensor>
```

## Integrating Gazebo with ROS 2

### Gazebo ROS 2 Bridge

The Gazebo ROS 2 bridge provides seamless integration between Gazebo simulation and ROS 2 systems. This allows your ROS 2 nodes to interact with simulated robots as if they were controlling real hardware.

### Launch Files

Gazebo simulations are typically started using ROS 2 launch files that:
- Load robot models (URDF/SDF)
- Configure physics parameters
- Start sensor plugins
- Launch controller nodes

### Control Interfaces

Gazebo supports various control interfaces:
- Joint position/velocity/effort controllers
- Force/torque sensors
- Camera and lidar interfaces
- Custom plugin interfaces

## Best Practices for Gazebo Simulation

### Performance Optimization
- Use appropriate physics step sizes
- Limit simulation complexity when possible
- Optimize collision geometry (simpler shapes)
- Adjust rendering quality based on needs

### Accuracy Considerations
- Calibrate sensor noise parameters
- Validate simulation against real-world data
- Use realistic friction and damping values
- Consider sim-to-real transfer challenges

## Summary

Gazebo provides powerful physics simulation capabilities essential for humanoid robot development. Through accurate modeling of gravity, collisions, and environmental interactions, combined with realistic sensor simulation, Gazebo enables safe and cost-effective development of humanoid robots. The integration with ROS 2 makes it straightforward to test real robot control code in simulation before deployment to physical hardware.

## Next Steps

Now that you understand Gazebo physics simulation, continue to the next chapter to learn about [High-Fidelity Simulation with Unity](./unity-high-fidelity-interaction), where you'll discover how Unity complements Gazebo for visual realism and human-robot interaction studies.

This chapter builds upon the ROS 2 communication patterns you learned in [Module 1: Python Agents & rclpy](../module-1/python-agents-rclpy), showing how to integrate simulation environments with ROS 2 systems for comprehensive robot development.