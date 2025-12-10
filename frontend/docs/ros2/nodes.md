---
sidebar_position: 2
---

# ROS 2 Nodes

In this chapter, we'll explore the fundamental concept of ROS 2 nodes, which are the basic execution units of a ROS program. A node is a process that performs computation, and it's the building block for developing complex robotic applications.

## Learning Outcomes

By the end of this chapter, you will be able to:
- Understand what ROS 2 nodes are and their role in the ROS ecosystem
- Create a simple ROS 2 node in both Python and C++
- Implement communication between nodes using topics and services
- Use ROS 2 tools to introspect and debug node behavior

## Theoretical Foundations

ROS 2 nodes represent the fundamental execution units in the Robot Operating System 2. They are processes that perform computation and are designed to work together in a distributed system. Each node typically performs a specific function in a robotic system, such as sensor data processing, motion planning, or control execution.

The node concept enables modularity and reusability in robotic software design. Nodes communicate with each other through topics, services, and actions, which allows for a flexible and decoupled architecture.

### Key Concepts

1. **Node Lifecycle**: Nodes follow a specific lifecycle from creation to destruction, with states including unconfigured, inactive, active, and finalized.

2. **Names and Namespaces**: Each node has a unique name within its namespace to avoid naming conflicts in large systems.

3. **Parameters**: Nodes can accept parameters that configure their behavior, making them reusable with different configurations.

## Practical Simulations

Now let's look at how to create a simple ROS 2 node. We'll start with a Python example that creates a node that publishes a simple message.

### Python Example

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
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
    minimal_publisher = MinimalPublisher()
    
    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        minimal_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ Example

```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclpy::init(argc, argv);
  rclpy::spin(std::make_shared<MinimalPublisher>());
  rclpy::shutdown();
  return 0;
}
```

## Chapter Summary

This chapter introduced the fundamental concept of ROS 2 nodes, which are essential for building distributed robotic systems. We explored both the theoretical foundations and practical implementations of ROS 2 nodes, including examples in Python and C++. Understanding nodes and their lifecycle is crucial for developing more complex robotic applications using ROS 2.