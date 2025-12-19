---
sidebar_position: 2
title: "Python Agents & rclpy"
description: "Using Python AI agents with ROS 2 through rclpy for Physical AI & Humanoid Robotics"
---

# Python Agents & rclpy: Connecting AI to ROS 2

## Learning Objectives

After completing this chapter, you will be able to:
- Implement ROS 2 nodes using Python and rclpy
- Understand and use ROS 2 communication patterns (nodes, topics, services)
- Create Python-based AI agents that interact with ROS 2 systems
- Design appropriate communication architectures for humanoid robot systems

## Introduction to rclpy

rclpy is the official Python client library for ROS 2, providing a Python API to interact with the ROS 2 middleware. It allows Python developers to create ROS 2 nodes, publish and subscribe to topics, provide and call services, and work with actions. This is particularly important for AI integration since Python has the richest ecosystem of AI and machine learning libraries.

### Why Python for AI Integration

Python dominates the AI and machine learning landscape with libraries like TensorFlow, PyTorch, scikit-learn, and others. When building AI agents for humanoid robots, Python allows seamless integration between AI algorithms and robotic systems through rclpy.

## ROS 2 Nodes with rclpy

A ROS 2 node in Python is created by subclassing the `Node` class from `rclpy.node`. Here's the basic structure:

```python
import rclpy
from rclpy.node import Node

class MyRobotNode(Node):
    def __init__(self):
        super().__init__('my_robot_node')
        # Initialize publishers, subscribers, services, etc. here
        self.get_logger().info('Robot node initialized')

def main(args=None):
    rclpy.init(args=args)
    node = MyRobotNode()

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

### Node Best Practices

When creating nodes for humanoid robotics applications:
- Use descriptive node names that indicate their function
- Initialize all publishers, subscribers, and services in the `__init__` method
- Use appropriate Quality of Service (QoS) settings for your application
- Implement proper error handling and cleanup

## Topics and Publish/Subscribe Pattern

Topics enable asynchronous, one-to-many communication in ROS 2. They're ideal for sensor data streams, status updates, and other data that multiple components might need.

### Creating a Publisher

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64

class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(String, 'sensor_data', 10)
        self.timer = self.create_timer(0.1, self.publish_sensor_data)  # Publish at 10Hz
        self.sensor_value = 0

    def publish_sensor_data(self):
        msg = String()
        msg.data = f'Sensor reading: {self.sensor_value}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.sensor_value += 1

def main(args=None):
    rclpy.init(args=args)
    node = SensorPublisher()

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

### Creating a Subscriber

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class DataSubscriber(Node):
    def __init__(self):
        super().__init__('data_subscriber')
        self.subscription = self.create_subscription(
            String,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = DataSubscriber()

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

## Services and Request/Response Pattern

Services provide synchronous, one-to-one communication suitable for operations that require a response. They're ideal for actions that need confirmation or return data.

### Creating a Service Server

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Returning: {request.a} + {request.b} = {response.sum}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()

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

### Creating a Service Client

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class MinimalClient(Node):
    def __init__(self):
        super().__init__('minimal_client')
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
    client = MinimalClient()
    response = client.send_request(1, 2)
    client.get_logger().info(f'Result: {response.sum}')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Humanoid Data Flow Example

In humanoid robotics, the communication pattern often involves multiple layers:

1. **Sensor Layer**: Publishers for joint states, IMU data, camera feeds, etc.
2. **Processing Layer**: Nodes that process sensor data and generate commands
3. **AI Layer**: AI agents that make high-level decisions
4. **Actuator Layer**: Subscribers that send commands to robot joints

Here's a simplified example of how these layers might communicate:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from builtin_interfaces.msg import Duration

class HumanoidController(Node):
    def __init__(self):
        super().__init__('humanoid_controller')

        # Subscribe to joint states from the robot
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Publish commands to the robot
        self.joint_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/joint_commands',
            10)

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20Hz

        self.current_joint_states = None

    def joint_state_callback(self, msg):
        self.current_joint_states = msg

    def control_loop(self):
        if self.current_joint_states is not None:
            # This is where AI logic would determine commands
            cmd_msg = Float64MultiArray()
            # Example: simple PD controller
            cmd_msg.data = [0.0] * len(self.current_joint_states.position)  # Placeholder

            self.joint_cmd_pub.publish(cmd_msg)

def main(args=None):
    rclpy.init(args=args)
    controller = HumanoidController()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integrating AI Agents with ROS 2

Python AI agents can be seamlessly integrated with ROS 2 systems. Here's an example of how an AI decision-making node might look:

```python
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class AIDecisionNode(Node):
    def __init__(self):
        super().__init__('ai_decision_node')

        # Subscribe to sensor data
        self.sensor_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.sensor_callback,
            10)

        # Publish AI decisions
        self.decision_pub = self.create_publisher(
            String,
            '/ai_decisions',
            10)

        # AI model (simplified example)
        self.ai_model_initialized = False
        self.initialize_ai_model()

    def initialize_ai_model(self):
        # In a real implementation, you would load your AI model here
        # e.g., TensorFlow model, PyTorch model, etc.
        self.get_logger().info('AI model initialized')
        self.ai_model_initialized = True

    def sensor_callback(self, msg):
        if self.ai_model_initialized:
            # Process sensor data through AI model
            sensor_data = np.array(msg.position)  # Simplified example

            # Make AI decision based on sensor data
            decision = self.make_ai_decision(sensor_data)

            # Publish decision
            decision_msg = String()
            decision_msg.data = decision
            self.decision_pub.publish(decision_msg)

    def make_ai_decision(self, sensor_data):
        # This is where your AI logic would go
        # For this example, we'll just return a simple decision
        if np.mean(sensor_data) > 0.5:
            return "move_forward"
        else:
            return "move_backward"

def main(args=None):
    rclpy.init(args=args)
    ai_node = AIDecisionNode()

    try:
        rclpy.spin(ai_node)
    except KeyboardInterrupt:
        pass
    finally:
        ai_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Summary

This chapter covered the fundamentals of using Python and rclpy to create AI agents that interact with ROS 2 systems. We explored the core communication patterns (topics, services), demonstrated how to create publishers and subscribers, and showed how AI agents can be integrated with robotic systems. Understanding these patterns is crucial for building effective AI-robot interfaces in humanoid robotics applications.

## Next Steps

Now that you understand how to connect AI agents to ROS 2 using rclpy, continue to the next chapter to learn about [URDF for Humanoids](./urdf-humanoids), where you'll discover how to describe robot models and map AI logic to physical robot components.