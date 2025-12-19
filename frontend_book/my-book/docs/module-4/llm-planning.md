---
sidebar_position: 2
title: "Cognitive Planning with LLMs"
description: "Translating natural language into action sequences and mapping plans to ROS 2 actions for Physical AI & Humanoid Robotics"
---

# Cognitive Planning with LLMs: Natural Language to Action Sequences

## Table of Contents

- [Learning Objectives](#learning-objectives)
- [Introduction to Cognitive Planning](#introduction-to-cognitive-planning)
- [LLM Capabilities for Natural Language Understanding](#llm-capabilities-for-natural-language-understanding)
- [Translating Natural Language to Action Sequences](#translating-natural-language-to-action-sequences)
- [Mapping Plans to ROS 2 Actions](#mapping-plans-to-ros-2-actions)
- [Planning Architecture and Implementation](#planning-architecture-and-implementation)
- [Integration with Robotics Systems](#integration-with-robotics-systems)
- [Summary](#summary)
- [Additional Resources](#additional-resources)

## Learning Objectives

After completing this chapter, you will be able to:
- Explain LLM capabilities for natural language understanding and planning
- Translate natural language instructions into executable action sequences
- Map high-level plans to specific ROS 2 actions
- Implement cognitive planning systems for robotics applications
- Integrate LLM-based planning with existing robotics frameworks

## Introduction to Cognitive Planning

Cognitive planning represents the bridge between high-level human instructions and low-level robot actions. In robotics, cognitive planning involves interpreting natural language commands and generating executable action sequences that achieve the user's intent. This capability is essential for creating intuitive human-robot interaction systems.

Cognitive planning systems must address several key challenges:
- **Natural Language Understanding**: Interpreting the semantics of human commands
- **World Modeling**: Understanding the current state and environment
- **Action Sequencing**: Creating ordered sequences of robot behaviors
- **Constraint Handling**: Managing physical, temporal, and safety constraints
- **Feedback Integration**: Adapting plans based on execution results

This chapter explores how Large Language Models (LLMs) can enhance cognitive planning capabilities in robotics systems, enabling more sophisticated and natural human-robot interaction.

## LLM Capabilities for Natural Language Understanding

Large Language Models have revolutionized natural language processing, offering unprecedented capabilities for understanding and generating human language. In the context of robotics, LLMs can serve as powerful cognitive planners that translate natural language into executable robot behaviors.

### Key LLM Capabilities

LLMs offer several advantages for cognitive planning:

1. **Semantic Understanding**: LLMs can interpret the meaning behind natural language commands
2. **Context Awareness**: Models can maintain context across multiple interactions
3. **Reasoning**: LLMs can perform logical reasoning to plan complex sequences
4. **Generalization**: Models can handle novel commands not explicitly programmed
5. **Multimodal Integration**: Advanced models can incorporate visual and spatial information

### LLM Architecture for Planning

Modern LLMs use transformer-based architectures that excel at understanding complex relationships in text:

1. **Input Processing**: Natural language commands are tokenized and processed
2. **Attention Mechanisms**: Models focus on relevant parts of the command
3. **Knowledge Integration**: Models leverage learned knowledge about the world
4. **Output Generation**: Structured action sequences are generated

### Planning-Specific Considerations

When using LLMs for cognitive planning, several factors must be considered:

- **Reliability**: LLMs may generate incorrect plans that must be validated
- **Consistency**: Planning behavior should be predictable and safe
- **Efficiency**: Real-time applications require fast planning responses
- **Interpretability**: Users need to understand how plans are generated

## Translating Natural Language to Action Sequences

The core challenge in cognitive planning is translating high-level natural language commands into low-level executable actions. This translation process involves several key steps.

### Command Parsing and Interpretation

The first step in translation involves understanding the user's intent:

```python
def parse_command(llm_client, command, context):
    """
    Parse a natural language command using an LLM
    """
    prompt = f"""
    You are a robot command parser. Analyze the following user command and extract structured information:

    Command: "{command}"
    Context: {context}

    Extract the following information in JSON format:
    {{
        "intent": "primary action the user wants",
        "entities": [{{"type": "object_type", "value": "specific_value", "attributes": {{}}}}],
        "constraints": [{{"type": "constraint_type", "value": "constraint_value"}}],
        "expected_outcome": "what the user expects to happen"
    }}
    """

    response = llm_client.chat.completions.create(
        model="gpt-3.5-turbo",
        messages=[{"role": "user", "content": prompt}],
        temperature=0.1  # Low temperature for consistency
    )

    return json.loads(response.choices[0].message.content)
```

### Action Sequence Generation

Once the command is parsed, the system generates an appropriate sequence of actions:

```python
def generate_action_sequence(parsed_command, robot_capabilities, environment_state):
    """
    Generate a sequence of actions based on parsed command
    """
    intent = parsed_command['intent']
    entities = parsed_command['entities']
    constraints = parsed_command['constraints']

    # Select appropriate action template based on intent
    if intent == "navigation":
        return generate_navigation_sequence(entities, constraints)
    elif intent == "manipulation":
        return generate_manipulation_sequence(entities, constraints)
    elif intent == "perception":
        return generate_perception_sequence(entities, constraints)
    elif intent == "complex_task":
        return generate_complex_task_sequence(parsed_command, robot_capabilities)
    else:
        # Use LLM to generate custom action sequence
        return generate_custom_sequence(parsed_command, robot_capabilities)
```

### Hierarchical Planning

Complex tasks often require hierarchical planning, breaking high-level goals into subtasks:

```python
def generate_complex_task_sequence(parsed_command, robot_capabilities):
    """
    Generate action sequence for complex tasks using hierarchical planning
    """
    # Use LLM to break down complex task
    breakdown_prompt = f"""
    Break down the following complex task into simpler subtasks that can be executed by a robot:

    Task: {parsed_command['intent']}
    Entities: {parsed_command['entities']}
    Constraints: {parsed_command['constraints']}

    Provide the breakdown in the following JSON format:
    {{
        "subtasks": [
            {{
                "description": "what needs to be done",
                "type": "navigation|manipulation|perception|system",
                "parameters": {{"key": "value"}},
                "dependencies": ["other_subtask_id"]
            }}
        ]
    }}
    """

    # Execute subtasks in appropriate order considering dependencies
    return plan_subtask_execution(breakdown_result)
```

### Example Translation Process

Consider the command "Go to the kitchen and bring me a cup from the counter":
1. **Parsing**: Identify intent (fetching object), destination (kitchen), object (cup), location (counter)
2. **Sequence Generation**: Navigate to kitchen → locate counter → identify cup → grasp cup → return
3. **Validation**: Ensure each step is feasible with robot capabilities
4. **Refinement**: Adjust based on environment state and constraints

## Mapping Plans to ROS 2 Actions

Once action sequences are generated, they must be mapped to specific ROS 2 action interfaces for execution on the robot.

### ROS 2 Action Interface Mapping

ROS 2 provides standard action interfaces for common robot behaviors:

```python
import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from manipulation_msgs.action import GraspObject
from perception_msgs.action import DetectObjects

class PlanExecutor:
    def __init__(self, node):
        self.node = node

        # Initialize action clients
        self.nav_client = ActionClient(node, NavigateToPose, 'navigate_to_pose')
        self.manip_client = ActionClient(node, GraspObject, 'grasp_object')
        self.perception_client = ActionClient(node, DetectObjects, 'detect_objects')

    def execute_action_sequence(self, action_sequence):
        """
        Execute a sequence of actions on the robot
        """
        results = []

        for action in action_sequence:
            result = self.execute_single_action(action)
            results.append(result)

            # Check if action succeeded before proceeding
            if not result.success:
                return results  # Stop execution on failure

        return results

    def execute_single_action(self, action):
        """
        Execute a single action based on its type
        """
        action_type = action['type']
        parameters = action['parameters']

        if action_type == 'navigation':
            return self.execute_navigation(parameters)
        elif action_type == 'manipulation':
            return self.execute_manipulation(parameters)
        elif action_type == 'perception':
            return self.execute_perception(parameters)
        else:
            raise ValueError(f"Unknown action type: {action_type}")
```

### Action Parameter Mapping

Each action type requires specific parameters that must be mapped from the high-level plan:

```python
def execute_navigation(self, parameters):
    """
    Execute navigation action
    """
    goal = NavigateToPose.Goal()

    # Map high-level parameters to ROS 2 navigation goal
    goal.pose.header.frame_id = parameters.get('frame_id', 'map')
    goal.pose.pose.position.x = parameters['x']
    goal.pose.pose.position.y = parameters['y']
    goal.pose.pose.orientation.w = parameters.get('orientation_w', 1.0)

    # Send goal to navigation system
    future = self.nav_client.send_goal_async(goal)
    return future

def execute_manipulation(self, parameters):
    """
    Execute manipulation action
    """
    goal = GraspObject.Goal()

    # Map object parameters to manipulation goal
    goal.object_name = parameters['object_name']
    goal.object_pose = parameters['object_pose']
    goal.grasp_type = parameters.get('grasp_type', 'pinch')

    future = self.manip_client.send_goal_async(goal)
    return future
```

### Error Handling and Recovery

Robust planning systems must handle execution failures and adapt accordingly:

```python
def execute_with_recovery(self, action_sequence):
    """
    Execute action sequence with error handling and recovery
    """
    for i, action in enumerate(action_sequence):
        try:
            result = self.execute_single_action(action)

            if not result.success:
                # Attempt recovery based on failure type
                recovery_action = self.generate_recovery_action(action, result.error_code)

                if recovery_action:
                    recovery_result = self.execute_single_action(recovery_action)

                    if recovery_result.success:
                        continue  # Continue with next action
                    else:
                        # Recovery failed, stop execution
                        return PlanResult(success=False, completed_actions=i, error="Recovery failed")
                else:
                    # No recovery possible
                    return PlanResult(success=False, completed_actions=i, error=result.error_code)

        except Exception as e:
            return PlanResult(success=False, completed_actions=i, error=str(e))

    return PlanResult(success=True, completed_actions=len(action_sequence))
```

## Planning Architecture and Implementation

A well-designed cognitive planning system requires a robust architecture that integrates LLMs with robotics systems effectively.

### Modular Architecture

The planning system should be modular to enable independent development and testing:

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Command       │    │   Plan          │    │   Action        │
│   Parser        │───▶│   Generator     │───▶│   Executor      │
│   (LLM)         │    │   (LLM)         │    │   (ROS 2)       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Context       │    │   Validation    │    │   Feedback      │
│   Manager       │    │   Layer         │    │   Handler       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Context Management

Effective planning requires maintaining and updating relevant context:

```python
class ContextManager:
    def __init__(self):
        self.environment_state = {}
        self.robot_state = {}
        self.user_preferences = {}
        self.task_history = []

    def update_context(self, new_information):
        """
        Update context with new information from sensors or user
        """
        self.environment_state.update(new_information.get('environment', {}))
        self.robot_state.update(new_information.get('robot', {}))

        # Update context history
        self.task_history.append(new_information)

    def get_context_prompt(self):
        """
        Generate context prompt for LLM
        """
        return f"""
        Environment State: {self.environment_state}
        Robot State: {self.robot_state}
        User Preferences: {self.user_preferences}
        Recent Task History: {self.task_history[-5:]}
        """
```

### Validation and Safety

All generated plans must be validated for safety and feasibility:

```python
class PlanValidator:
    def __init__(self, robot_capabilities, environment_model):
        self.robot_capabilities = robot_capabilities
        self.environment_model = environment_model

    def validate_plan(self, action_sequence):
        """
        Validate a plan for safety and feasibility
        """
        validation_results = []

        for action in action_sequence:
            result = self.validate_single_action(action)
            validation_results.append(result)

            if not result.is_safe or not result.is_feasible:
                return PlanValidationResult(
                    is_valid=False,
                    errors=[result.error for result in validation_results if not result.is_safe or not result.is_feasible]
                )

        return PlanValidationResult(is_valid=True, errors=[])

    def validate_single_action(self, action):
        """
        Validate a single action
        """
        # Check robot capabilities
        if not self.check_robot_capability(action):
            return ValidationResult(is_safe=False, is_feasible=False, error="Robot cannot perform action")

        # Check environment constraints
        if not self.check_environment_constraints(action):
            return ValidationResult(is_safe=False, is_feasible=False, error="Environment constraints violated")

        # Check safety constraints
        if not self.check_safety_constraints(action):
            return ValidationResult(is_safe=False, is_feasible=True, error="Safety constraint violated")

        return ValidationResult(is_safe=True, is_feasible=True, error=None)
```

## Integration with Robotics Systems

Successful cognitive planning systems must integrate seamlessly with existing robotics frameworks and infrastructure.

### ROS 2 Integration Patterns

Common integration patterns for LLM-based planning in ROS 2:

```python
class CognitivePlannerNode(Node):
    def __init__(self):
        super().__init__('cognitive_planner_node')

        # Publishers and subscribers
        self.command_sub = self.create_subscription(String, 'user_command', self.command_callback, 10)
        self.plan_pub = self.create_publisher(String, 'generated_plan', 10)
        self.status_pub = self.create_publisher(String, 'planner_status', 10)

        # Action clients for execution
        self.plan_executor = PlanExecutor(self)

        # LLM client
        self.llm_client = self.initialize_llm_client()

        # Context manager
        self.context_manager = ContextManager()

        # Plan validator
        self.validator = PlanValidator(self.get_robot_capabilities(), self.get_environment_model())

    def command_callback(self, msg):
        """
        Handle incoming user commands
        """
        try:
            # Get current context
            context = self.context_manager.get_context_prompt()

            # Parse and plan
            parsed_command = self.parse_command(msg.data, context)
            action_sequence = self.generate_action_sequence(parsed_command)

            # Validate plan
            validation_result = self.validator.validate_plan(action_sequence)

            if validation_result.is_valid:
                # Execute plan
                execution_result = self.plan_executor.execute_action_sequence(action_sequence)

                # Publish results
                self.publish_execution_result(execution_result)
            else:
                self.get_logger().error(f"Plan validation failed: {validation_result.errors}")
                self.publish_error(f"Plan validation failed: {validation_result.errors}")

        except Exception as e:
            self.get_logger().error(f"Planning error: {str(e)}")
            self.publish_error(f"Planning error: {str(e)}")
```

### Multi-Robot Coordination

For systems with multiple robots, planning must consider coordination:

```python
class MultiRobotPlanner:
    def __init__(self, robot_ids):
        self.robot_ids = robot_ids
        self.robot_assignments = {}

    def assign_tasks(self, action_sequence):
        """
        Assign actions to appropriate robots based on capabilities and availability
        """
        assignments = {}

        for action in action_sequence:
            suitable_robots = self.find_suitable_robots(action)

            if suitable_robots:
                # Assign to most appropriate robot
                assigned_robot = self.select_best_robot(suitable_robots, action)
                assignments[action['id']] = assigned_robot
            else:
                raise ValueError(f"No suitable robot for action: {action}")

        return assignments
```

## Summary

Cognitive planning with LLMs represents a significant advancement in human-robot interaction, enabling robots to understand and execute complex natural language commands. The process involves translating high-level human instructions into executable action sequences through several key stages: command parsing, action sequence generation, plan validation, and execution.

The integration of LLMs with robotics systems requires careful consideration of reliability, safety, and real-time performance. Successful implementations use modular architectures that separate command parsing, plan generation, validation, and execution while maintaining context and handling errors effectively.

Key components of effective cognitive planning systems include:
- Natural language understanding capabilities using LLMs
- Hierarchical planning for complex tasks
- Mapping between high-level plans and ROS 2 action interfaces
- Validation and safety checks
- Context management and feedback integration

These systems enable more intuitive and flexible human-robot interaction, allowing users to express their intentions in natural language while robots handle the complexity of translating these intentions into executable behaviors.

## Additional Resources

For more information on related topics:

- Study [OpenAI API documentation](https://platform.openai.com/docs/) for LLM integration patterns
- Review [ROS 2 Navigation](https://navigation.ros.org/) for navigation action interfaces
- Explore [OpenVLA](https://github.com/openvla/openvla) for vision-language-action models
- Check the [Module 1: The Robotic Nervous System (ROS 2)](../module-1/ros2-basics) for foundational concepts
- Review the [Module 2: The Digital Twin (Gazebo & Unity)](../module-2/digital-twins-physical-ai) for simulation concepts
- Examine the [Module 3: The AI-Robot Brain (NVIDIA Isaac™)](../module-3/nvidia-isaac-sim) for AI integration patterns