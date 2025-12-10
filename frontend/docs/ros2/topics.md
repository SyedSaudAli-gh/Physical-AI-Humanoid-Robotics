---
sidebar_position: 3
---

# ROS 2 Topics

ROS 2 topics form the backbone of message-based communication in the Robot Operating System 2. They enable asynchronous communication between nodes through a publish-subscribe pattern, which is fundamental to creating distributed robotic systems.

## Learning Outcomes

By the end of this chapter, you will be able to:
- Understand the publish-subscribe communication model in ROS 2
- Create publishers and subscribers for custom message types
- Design message structures appropriate for your robotic application
- Use ROS 2 tools for debugging and monitoring topic communication

## Theoretical Foundations

Topics in ROS 2 implement a publish-subscribe communication pattern where nodes publish messages to topics and other nodes subscribe to those topics to receive messages. This decouples the nodes and enables flexible system architectures.

Topics are identified by names and can carry messages of specific types. The communication is asynchronous and one-to-many by default, meaning multiple subscribers can receive messages from a single publisher.

### Key Concepts

1. **Message Types**: All messages on a topic must be of the same type, defined in ROS message definition files (.msg).

2. **Quality of Service (QoS)**: ROS 2 allows configuring how messages are delivered, including reliability, durability, and history policies.

3. **Topic Remapping**: Topics can be remapped to different names at runtime, allowing for flexible system configurations.

## Practical Simulations

Let's examine how to create publishers and subscribers for custom message types in ROS 2.

### Creating a Custom Message

First, let's define a custom message for our robot's sensor data:

```
# SensorReading.msg
float64 range
string sensor_type
builtin_interfaces/Time timestamp
```

### Python Publisher Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from sensor_msgs.msg import Range  # Using standard message type for this example

class SensorPublisher(Node):

    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher_ = self.create_publisher(Range, 'sensor_data', 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.publish_sensor_data)
        self.sensor_value = 0.0

    def publish_sensor_data(self):
        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'sensor_frame'
        msg.radiation_type = Range.ULTRASOUND  # 0 for ultrasound
        msg.field_of_view = 0.1  # 0.1 radian field of view
        msg.min_range = 0.0
        msg.max_range = 10.0
        msg.range = self.sensor_value
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing sensor data: {msg.range}')
        
        # Simulate changing sensor values
        self.sensor_value = (self.sensor_value + 0.1) % 5.0

def main(args=None):
    rclpy.init(args=args)
    sensor_publisher = SensorPublisher()
    
    try:
        rclpy.spin(sensor_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Python Subscriber Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

class SensorSubscriber(Node):

    def __init__(self):
        super().__init__('sensor_subscriber')
        self.subscription = self.create_subscription(
            Range,
            'sensor_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(
            f'Received sensor data - Type: {msg.radiation_type}, '
            f'Range: {msg.range}, Time: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}'
        )

def main(args=None):
    rclpy.init(args=args)
    sensor_subscriber = SensorSubscriber()
    
    try:
        rclpy.spin(sensor_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Code Examples

Here's a complete example of a publisher-subscriber pair for a robot velocity command:

### Velocity Publisher

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_velocity)
        self.i = 0

    def publish_velocity(self):
        msg = Twist()
        msg.linear.x = 1.0  # Move forward at 1 m/s
        msg.angular.z = 0.5  # Rotate at 0.5 rad/s
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing velocity: linear={msg.linear.x}, angular={msg.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    velocity_publisher = VelocityPublisher()
    
    try:
        rclpy.spin(velocity_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Chapter Summary

This chapter explored the fundamental concept of topics in ROS 2, which enable publish-subscribe communication between nodes. We covered the theoretical aspects of the communication model and provided practical examples of creating publishers and subscribers. Understanding topics is crucial for building distributed robotic applications where different components need to communicate efficiently.