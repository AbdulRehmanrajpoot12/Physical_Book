---
sidebar_position: 1
title: "ROS 2 Basics"
description: "Understanding the fundamentals of ROS 2 for Physical AI & Humanoid Robotics"
---

# ROS 2 Basics: The Robotic Nervous System

## Learning Objectives

After completing this chapter, you will be able to:
- Explain the purpose of middleware in robotics
- Distinguish between ROS 1 and ROS 2
- Understand distributed robotic systems
- Describe the ROS 2 architecture overview

## The Purpose of Middleware in Robotics

Middleware in robotics serves as the communication backbone that allows different software components to interact seamlessly. In complex robotic systems, multiple processes run simultaneously across different hardware components - sensors, actuators, control systems, and AI modules. Middleware abstracts the complexities of inter-process communication, allowing developers to focus on robot behavior rather than communication protocols.

In the context of humanoid robotics, middleware becomes even more critical. A humanoid robot typically has dozens of sensors (cameras, IMUs, force sensors) and actuators (motors in joints) that must coordinate precisely. The middleware ensures that sensor data flows to processing units, that AI decisions reach the appropriate actuators, and that all components operate in harmony.

## ROS 1 vs ROS 2: Evolution of the Robot Operating System

ROS (Robot Operating System) began as a research framework in 2007, evolving into a de facto standard for robotics development. However, ROS 1 had several limitations that became apparent as robotics moved from research labs to real-world applications:

### ROS 1 Limitations:
- **Single Master Architecture**: A single point of failure that could bring down the entire system
- **Limited Multi-Robot Support**: Difficult to coordinate multiple robots effectively
- **Security Concerns**: No built-in security or authentication mechanisms
- **Real-time Constraints**: Challenging to meet strict timing requirements
- **Communication Protocols**: Relied on TCPROS/UDPROS which were not suitable for all environments

### ROS 2 Solutions:
- **DDS-based Communication**: Uses Data Distribution Service (DDS) for robust, distributed communication
- **Masterless Architecture**: No single point of failure; nodes can discover each other directly
- **Enhanced Security**: Built-in security features for authentication and encryption
- **Real-time Support**: Better support for real-time applications and deterministic behavior
- **Cross-platform Compatibility**: Improved support across different operating systems

For humanoid robotics applications, these improvements are crucial. The masterless architecture allows for more robust multi-robot systems, while DDS provides the reliability needed for safety-critical humanoid control systems.

## Distributed Robotic Systems

Modern robotics, especially humanoid robotics, requires distributed systems where computation is spread across multiple processing units. This distribution happens at multiple levels:

### Hardware Distribution
- **On-board Processing**: Different computational units handle perception, planning, and control
- **Sensor Clustering**: Groups of sensors are processed locally before sharing with the system
- **Actuator Control**: Joint controllers operate independently while coordinating with the whole system

### Software Distribution
- **Functional Decomposition**: Different software packages handle specific robot functions
- **Modular Architecture**: Components can be developed, tested, and deployed independently
- **Scalable Communication**: System can grow by adding new components without redesigning everything

ROS 2's architecture naturally supports this distributed approach through its DDS-based communication layer, allowing nodes to be distributed across multiple machines while maintaining transparent communication.

## ROS 2 Architecture Overview

The ROS 2 architecture consists of several key components:

### Nodes
Nodes are the fundamental execution units in ROS 2. Each node typically represents a specific function or process in the robotic system. In a humanoid robot, you might have nodes for:
- Sensor data processing
- Motion planning
- Joint control
- Perception systems
- AI decision-making

Here's a simple example of creating a ROS 2 node in Python using rclpy:

```python
import rclpy
from rclpy.node import Node

class SimpleNode(Node):
    def __init__(self):
        super().__init__('simple_node')
        self.get_logger().info('Simple node created')

def main(args=None):
    rclpy.init(args=args)
    node = SimpleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Communication Primitives
ROS 2 provides several communication mechanisms:

#### Topics (Publish/Subscribe)
- Asynchronous, one-to-many communication
- Ideal for sensor data streams and status updates
- Publishers and subscribers are decoupled in time and space

Example of a simple publisher:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher = SimplePublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Services (Request/Response)
- Synchronous, one-to-one communication
- Suitable for operations that require a response
- Used for actions that need confirmation or return data

#### Actions
- Extended services with feedback and goal management
- Perfect for long-running operations with progress updates
- Essential for complex humanoid robot behaviors

### Client Libraries
ROS 2 provides client libraries for multiple programming languages:
- **rclcpp**: C++ client library
- **rclpy**: Python client library
- Other languages: Rust, Java, C#, etc.

For AI integration with humanoid robots, Python is often preferred due to the rich ecosystem of AI libraries, making rclpy particularly important.

## Summary

This chapter introduced the fundamental concepts of ROS 2 as middleware for robotics. We explored why middleware is essential in robotics, the evolution from ROS 1 to ROS 2, the importance of distributed systems in modern robotics, and the key architectural components of ROS 2. Understanding these fundamentals is crucial for effectively working with humanoid robots and integrating AI systems with robotic platforms.

## Next Steps

Now that you understand the basics of ROS 2, continue to the next chapter to learn about [Python Agents & rclpy](./python-agents-rclpy), where you'll discover how to create Python-based AI agents that interact with ROS 2 systems.