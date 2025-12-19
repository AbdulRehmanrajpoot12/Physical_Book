---
title: "Python Agents & rclpy: Connecting AI Logic to ROS 2"
sidebar_label: "Python Agents & rclpy"
description: "Understanding how to create Python-based agents that communicate with ROS 2 nodes using the rclpy client library"
slug: /module-1/python-agents-rclpy
---

# Python Agents & rclpy: Connecting AI Logic to ROS 2

## Learning Objectives

By the end of this chapter, you will be able to:
- Create Python nodes that communicate with ROS 2 using rclpy
- Implement publishers and subscribers in Python
- Design services and clients in Python
- Understand how to integrate AI agents with ROS 2 nodes
- Apply communication patterns to humanoid robot data flow

## Introduction to rclpy

rclpy is the Python client library for ROS 2, providing a Pythonic interface to ROS 2 concepts. It allows Python developers to create ROS 2 nodes, publish and subscribe to topics, provide and call services, and work with actions. This makes it ideal for implementing AI agents that need to interact with robotic systems.

Python is particularly well-suited for AI and machine learning applications, making rclpy a crucial bridge between AI logic and robotic hardware.

## Creating Python Nodes with rclpy

### Basic Node Structure

A basic ROS 2 node in Python follows this structure:

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('node_name')
        # Initialize node components here

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Node Lifecycle

ROS 2 nodes have a well-defined lifecycle:
1. Initialization: The node is created and configured
2. Execution: The node runs and processes callbacks
3. Shutdown: The node is properly cleaned up

## Topics and Publish/Subscribe Pattern

The publish/subscribe pattern is fundamental to ROS 2 communication. Publishers send messages to topics, and subscribers receive messages from topics. This enables asynchronous, decoupled communication between nodes.

### Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher_ = self.create_publisher(String, 'topic_name', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    publisher_node = PublisherNode()
    rclpy.spin(publisher_node)
    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(
            String,
            'topic_name',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Services and Request/Response Pattern

Services provide synchronous communication between nodes. A service client sends a request to a service server, which processes the request and returns a response.

### Creating a Service Server

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceServerNode(Node):
    def __init__(self):
        super().__init__('service_server_node')
        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(
            f'Incoming request: a={request.a}, b={request.b}, sum={response.sum}'
        )
        return response

def main(args=None):
    rclpy.init(args=args)
    service_server_node = ServiceServerNode()
    rclpy.spin(service_server_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Creating a Service Client

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node

class ServiceClientNode(Node):
    def __init__(self):
        super().__init__('service_client_node')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    service_client_node = ServiceClientNode()

    response = service_client_node.send_request(1, 2)
    service_client_node.get_logger().info(f'Result: {response.sum}')

    service_client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid Data Flow Example

In humanoid robotics, data flows between various components using ROS 2 communication patterns. Here's an example of how an AI agent might interact with a humanoid robot:

### AI Decision Node

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscribe to sensor data
        self.sensor_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_state_callback,
            10
        )

        # Subscribe to camera data
        self.camera_subscription = self.create_subscription(
            String,  # Simplified - in reality, this would be sensor_msgs/Image
            'camera_data',
            self.camera_callback,
            10
        )

        # Publish commands to robot
        self.cmd_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        # AI processing timer
        self.timer = self.create_timer(0.1, self.ai_processing_callback)

        # Internal state
        self.joint_states = None
        self.camera_data = None

    def joint_state_callback(self, msg):
        self.joint_states = msg
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')

    def camera_callback(self, msg):
        self.camera_data = msg
        self.get_logger().info('Received camera data')

    def ai_processing_callback(self):
        # Simple AI decision making
        if self.joint_states and self.camera_data:
            # Analyze sensor data and make decisions
            cmd_msg = Twist()
            cmd_msg.linear.x = 0.5  # Move forward
            cmd_msg.angular.z = 0.0  # No rotation

            # Publish command
            self.cmd_publisher.publish(cmd_msg)
            self.get_logger().info('Published movement command')

def main(args=None):
    rclpy.init(args=args)
    ai_node = AIDecisionNode()
    rclpy.spin(ai_node)
    ai_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Best Practices for Python AI Agents

### 1. Proper Error Handling

```python
import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException

class RobustAINode(Node):
    def __init__(self):
        super().__init__('robust_ai_node')

        # Declare parameters with defaults
        self.declare_parameter('ai_model_path', '/default/path/model.pkl')

        try:
            model_path = self.get_parameter('ai_model_path').value
            self.load_ai_model(model_path)
        except ParameterNotDeclaredException:
            self.get_logger().error('AI model path parameter not declared')

    def load_ai_model(self, path):
        try:
            # Load your AI model here
            self.get_logger().info(f'Loading AI model from {path}')
        except Exception as e:
            self.get_logger().error(f'Failed to load AI model: {e}')
```

### 2. Efficient Message Handling

```python
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy

class EfficientNode(Node):
    def __init__(self):
        super().__init__('efficient_node')

        # Configure QoS for different types of data
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.sensor_subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.sensor_callback,
            sensor_qos
        )
```

### 3. Resource Management

```python
class ResourceManagedNode(Node):
    def __init__(self):
        super().__init__('resource_managed_node')
        self.ai_model = None
        self.initialize_resources()

    def initialize_resources(self):
        try:
            # Initialize AI model and other resources
            self.get_logger().info('Initializing AI resources')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize resources: {e}')

    def destroy_node(self):
        # Clean up resources before destroying node
        if self.ai_model:
            # Clean up AI model
            self.ai_model = None
        super().destroy_node()
```

## Summary

Python agents using rclpy provide a powerful way to connect AI logic with ROS 2 systems. The publish/subscribe pattern enables asynchronous communication between components, while services provide synchronous request/response interactions. For humanoid robotics applications, these patterns allow AI agents to process sensor data and generate appropriate commands for robot control.

The examples in this chapter demonstrate how to create robust Python nodes that can serve as the bridge between sophisticated AI algorithms and the physical robot systems they control. The next chapter will explore how to describe robot structures using URDF, which is essential for humanoid robots.

## Next Steps

- Continue to [URDF for Humanoids](./urdf-humanoids.md) to learn how to describe robot structures
- Review [ROS 2 Basics](./ros2-basics.md) for fundamental concepts