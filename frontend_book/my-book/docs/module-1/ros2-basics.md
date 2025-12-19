---
title: "ROS 2 Basics: The Robotic Nervous System"
sidebar_label: "ROS 2 Basics"
description: "Understanding the fundamentals of ROS 2, the middleware that connects all parts of a robotic system"
slug: /module-1/ros2-basics
---

# ROS 2 Basics: The Robotic Nervous System

## Learning Objectives

By the end of this chapter, you will be able to:
- Explain the purpose of middleware in robotics and distributed systems
- Distinguish between ROS 2 and ROS 1, understanding their key differences
- Describe the architecture of ROS 2 and its core components
- Understand the concept of distributed robotic systems
- Identify when to use ROS 2 in physical AI and humanoid robotics projects

## Introduction to Middleware in Robotics

Middleware in robotics serves as the communication infrastructure that allows different components of a robotic system to interact seamlessly. Think of it as the nervous system of a robot - it connects sensors, actuators, processing units, and control systems, enabling them to coordinate and share information.

In traditional robotics, components were often tightly coupled, making it difficult to modify, extend, or replace individual parts. Middleware addresses this challenge by providing a standardized communication layer that abstracts the complexities of inter-component communication.

ROS 2 (Robot Operating System 2) is a middleware framework that provides services designed specifically for robotics applications, including hardware abstraction, device drivers, libraries, visualizers, message-passing, package management, and more.

## ROS 2 vs ROS 1: Key Differences

ROS 2 was developed to address limitations in ROS 1 and to bring robotics development into the modern era of distributed computing. Here are the key differences:

### 1. Communication Layer
- **ROS 1**: Uses a centralized master-slave architecture with a single master node that coordinates all communication
- **ROS 2**: Uses DDS (Data Distribution Service) for peer-to-peer communication, eliminating the single point of failure

### 2. Quality of Service (QoS)
- **ROS 1**: Limited QoS capabilities
- **ROS 2**: Rich QoS policies for reliability, durability, liveliness, and resource management

### 3. Real-time Support
- **ROS 1**: Limited real-time capabilities
- **ROS 2**: Better support for real-time systems with real-time safe code paths

### 4. Security
- **ROS 1**: No built-in security features
- **ROS 2**: Built-in security framework with authentication, access control, and encryption

### 5. Cross-platform Support
- **ROS 1**: Primarily Linux-focused
- **ROS 2**: Better Windows and macOS support

### 6. Lifecycle Management
- **ROS 1**: Simple node lifecycle (alive or dead)
- **ROS 2**: Rich node lifecycle with configurable states and transitions

## Distributed Robotic Systems

Distributed robotic systems consist of multiple computing nodes that work together to achieve a common goal. In such systems:

- Each node can run on different hardware (PCs, microcontrollers, embedded systems)
- Nodes can be physically separated and communicate over networks
- The system can continue operating even if individual nodes fail
- Tasks can be parallelized across multiple nodes for better performance

ROS 2's DDS-based communication enables true distributed robotics by:

1. **Decentralized Communication**: No single master node required; nodes discover each other automatically
2. **Network Transparency**: Communication works the same whether nodes are on the same machine or across the internet
3. **Fault Tolerance**: If one node fails, others can continue operating
4. **Scalability**: New nodes can be added to the system without reconfiguration

## ROS 2 Architecture Overview

The ROS 2 architecture consists of several layers:

### 1. Client Libraries
- **rclcpp**: C++ client library
- **rclpy**: Python client library (the focus of this book)
- **rcl**: The common client library layer
- **rmw**: ROS Middleware Interface that abstracts DDS implementations

### 2. DDS/RMW Layer
- **DDS Implementation**: Various DDS vendors (Fast DDS, Cyclone DDS, RTI Connext)
- **RMW**: ROS Middleware Interface that allows switching between DDS implementations

### 3. Core Components
- **Nodes**: Basic computational units that perform computation
- **Topics**: Named buses over which nodes exchange messages
- **Services**: Synchronous request/response communication
- **Actions**: Asynchronous goal-oriented communication with feedback
- **Parameters**: Configuration values that can be changed at runtime

### 4. Tools
- **ros2 command**: Command-line tools for introspection and control
- **rqt**: GUI tools for visualization and debugging
- **rviz**: 3D visualization tool for robot data

## Core Concepts

### Nodes
A node is an executable that uses ROS 2 to communicate with other nodes. In a robot system, you might have nodes for:
- Sensor drivers (camera, LIDAR, IMU)
- Control algorithms (path planning, motion control)
- Perception systems (object detection, SLAM)
- User interfaces

Here's a simple example of creating a node in Python using rclpy:

```python
import rclpy
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalNode()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topics and Messages
Topics are named buses over which nodes exchange messages. Communication is asynchronous and follows a publish/subscribe pattern:
- Publishers send messages to topics
- Subscribers receive messages from topics
- Multiple publishers and subscribers can exist for the same topic

Here's an example of a subscriber node:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Services
Services provide synchronous request/response communication:
- A client sends a request to a service
- A server processes the request and sends back a response
- Communication is synchronous - the client waits for the response

Example service server:

```python
from add_two_ints_srv.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Actions
Actions are used for long-running tasks that require feedback:
- Goal: Request to perform a task
- Feedback: Updates on task progress
- Result: Final outcome of the task

## Summary

ROS 2 represents a significant evolution in robotics middleware, addressing the limitations of ROS 1 while providing the robust, distributed communication capabilities needed for modern robotics applications. Its DDS-based architecture enables true distributed systems, making it ideal for complex robotic applications including humanoid robots and physical AI systems.

Understanding these fundamentals is crucial for effectively designing and implementing robotic systems with ROS 2. The next chapter will explore how to create Python-based agents that communicate using ROS 2 concepts.

## Next Steps

- Continue to [Python Agents & rclpy](./python-agents-rclpy.md) to learn how to implement ROS 2 communication patterns in Python
- Explore [URDF for Humanoids](./urdf-humanoids.md) to understand how to describe robot structures