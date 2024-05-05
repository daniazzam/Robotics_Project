# Project Overview: Automated Factory Order Fulfillment System

## Introduction
- **Objective**: Enhance factory order fulfillment efficiency through a robotic system.
- **Technology Stack**: Robotics, intelligent algorithms, ROS (Robot Operating System), user-centric design.
- **Tools Used**: MOVEIT motion planning, RViz, Gazebo Simulator.
- **Skills Demonstrated**: Robotic manipulation, algorithmic problem solving, system integration.

## System Description
- **Robotic Components**: Includes advanced robots, robotic arms, and mobile robots.
- **Functionality**:
  - Robotic arms work in tandem to retrieve and pack items.
  - Mobile robots deliver orders to designated destinations.
- **Highlight**: Integration of precision robotics with intelligent path planning and user-centric operational processes.

## ROS Architecture
### ROS Nodes
- **Arm1 ROS Node**
  - **Role**: Processes and prioritizes client orders using a priority-based scheduling algorithm.
  - **Capabilities**:
    - Utilizes MOVEIT for optimal motion planning, ensuring safety and efficiency.
    - Provides ROS services like `AddOrderService` for order submission and `GetOrderService` for order status updates.
- **Object Detector Node**
  - **Role**: Detects and classifies objects on the conveyor belt using computer vision.
  - **Functionality**:
    - Subscribes to raw image feeds, publishing processed outputs for real-time monitoring.
    - Coordinates with Arm2 for object pre-positioning and pickup.
- **Arm2 ROS Node**
  - **Role**: Acts as an action server to execute pickup and positioning tasks based on inputs from the Object Detector.
  - **Functionality**:
    - Receives and acts on pre-positioning and pickup requests.
    - Determines the placement of objects based on the priority of orders.

## Challenges and Solutions
- **Challenges**:
  - Difficulty in handling objects at varying conveyor belt speeds.
  - Limitations in simultaneous multiple object detection and specific object handling.
- **Proposed Improvements**:
  - Implementation of feedback mechanisms for error handling and better coordination between robotic arms.
  - Dynamic adjustment of robotic actions based on conveyor belt speed measurements.

## Future Directions
- **Reliability and Adaptability**: Enhance system reliability and adaptability to varying operational conditions.
- **Efficiency Improvements**: Implement advanced detection and motion planning algorithms for improved efficiency.

## Additional Resources
- **Project Demonstration Video**: [Watch Here](https://youtu.be/NoscmFj998E)


