---
sidebar_position: 4
---

# Chapter 4: Python-ROS Integration with rclpy

## Introduction to rclpy

`rclpy` is the Python client library for ROS 2. It provides a simple and intuitive way to write ROS 2 applications in Python.

## Basic rclpy Node Structure

Every ROS 2 Python application starts with initializing `rclpy` and creating a node.

```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node_name')
        self.get_logger().info('My node has been started!')

def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node) # Keep node alive
    my_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Publishing Messages

To send data, a node creates a publisher and publishes messages to a topic.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Example message type

class MinimalPublisher(Node):
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
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Subscribing to Messages

To receive data, a node creates a subscriber and defines a callback function to process incoming messages.

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

## Creating a Service Client

A service client sends requests to a service server and waits for a response.

```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # Custom service type

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.client = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.request = AddTwoInts.Request()

    def send_request(self, a, b):
        self.request.a = a
        self.request.b = b
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)
    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(41, 1)
    minimal_client.get_logger().info(
        'Result of add_two_ints: for %d + %d = %d' %
        (response.sum, minimal_client.request.a, minimal_client.request.b))
    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Implementing a Service Server

A service server waits for requests and computes a response.

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
        self.get_logger().info('Incoming request: a: %d b: %d' % (request.a, request.b))
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Chapter Summary

- `rclpy` is the Python client library for ROS 2.
- Nodes are the fundamental building blocks.
- Publishers send messages, subscribers receive them.
- Services implement request/response patterns.
- ROS 2 provides robust communication for robotics applications.

## Exercise

Create a ROS 2 Python package with two nodes:
1. A "command_publisher" that publishes integer commands to a topic every second.
2. A "robot_controller" that subscribes to these commands and logs them.
Use custom message types for the commands.

---

**Next**: [Chapter 5: URDF Modeling for Humanoids →](chapter-5)
