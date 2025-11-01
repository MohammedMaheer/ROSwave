#!/usr/bin/env python3
"""
Demo Topics Generator - Creates 30 concurrent test ROS2 topics with realistic data
Publishes various message types at different rates to simulate real robot environment
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile
import threading
import time
from datetime import datetime
import random

# Standard message types
from std_msgs.msg import String, Float64, Int32, Bool
from geometry_msgs.msg import Twist, Point, Quaternion, Pose
from sensor_msgs.msg import LaserScan, Image, PointCloud2, Imu, CameraInfo
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage


class DemoTopicPublisher(Node):
    """Publisher for a single demo topic with realistic data"""
    
    def __init__(self, topic_name, message_type, publish_rate_hz):
        super().__init__(f'demo_{topic_name.replace("/", "_")}')
        self.topic_name = topic_name
        self.message_type = message_type
        self.publish_rate_hz = publish_rate_hz
        self.counter = 0
        
        # Create publisher with generic publisher interface
        self.publisher = self.create_publisher(
            message_type,
            topic_name,
            10
        )
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate_hz  # seconds
        self.timer = self.create_timer(timer_period, self.publish_callback)
        
        self.get_logger().info(
            f"Created publisher for '{topic_name}' ({message_type.__name__}) @ {publish_rate_hz}Hz"
        )
    
    def publish_callback(self):
        """Publish next message"""
        try:
            msg = self.create_message()
            self.publisher.publish(msg)
            self.counter += 1
        except Exception as e:
            self.get_logger().error(f"Error publishing: {e}")
    
    def create_message(self):
        """Create appropriate message based on type"""
        if self.message_type == String:
            msg = String()
            msg.data = f"Demo data from {self.topic_name} - {self.counter}"
            return msg
        
        elif self.message_type == Float64:
            msg = Float64()
            msg.data = random.uniform(0, 100) + self.counter * 0.1
            return msg
        
        elif self.message_type == Int32:
            msg = Int32()
            msg.data = self.counter % 1000
            return msg
        
        elif self.message_type == Bool:
            msg = Bool()
            msg.data = self.counter % 2 == 0
            return msg
        
        elif self.message_type == Twist:
            msg = Twist()
            msg.linear.x = random.uniform(-1.0, 1.0)
            msg.linear.y = random.uniform(-1.0, 1.0)
            msg.linear.z = random.uniform(-1.0, 1.0)
            msg.angular.x = random.uniform(-1.0, 1.0)
            msg.angular.y = random.uniform(-1.0, 1.0)
            msg.angular.z = random.uniform(-1.0, 1.0)
            return msg
        
        elif self.message_type == Pose:
            msg = Pose()
            msg.position.x = random.uniform(-10, 10)
            msg.position.y = random.uniform(-10, 10)
            msg.position.z = random.uniform(0, 5)
            msg.orientation.x = 0.0
            msg.orientation.y = 0.0
            msg.orientation.z = random.uniform(-1, 1)
            msg.orientation.w = 1.0
            return msg
        
        elif self.message_type == Odometry:
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            msg.pose.pose.position.x = random.uniform(-50, 50)
            msg.pose.pose.position.y = random.uniform(-50, 50)
            msg.pose.pose.position.z = 0.0
            msg.pose.pose.orientation.w = 1.0
            msg.twist.twist.linear.x = random.uniform(-2, 2)
            msg.twist.twist.linear.y = random.uniform(-2, 2)
            return msg
        
        elif self.message_type == Imu:
            msg = Imu()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "imu_link"
            msg.linear_acceleration.x = random.uniform(-10, 10)
            msg.linear_acceleration.y = random.uniform(-10, 10)
            msg.linear_acceleration.z = random.uniform(8, 12)
            msg.angular_velocity.x = random.uniform(-1, 1)
            msg.angular_velocity.y = random.uniform(-1, 1)
            msg.angular_velocity.z = random.uniform(-1, 1)
            msg.orientation.w = 1.0
            return msg
        
        elif self.message_type == LaserScan:
            msg = LaserScan()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "laser_frame"
            msg.angle_min = 0.0
            msg.angle_max = 6.28
            msg.angle_increment = 0.01
            msg.time_increment = 0.0
            msg.scan_time = 0.1
            msg.range_min = 0.1
            msg.range_max = 30.0
            msg.ranges = [random.uniform(0.5, 10.0) for _ in range(360)]
            msg.intensities = [random.uniform(0, 100) for _ in range(360)]
            return msg
        
        elif self.message_type == CameraInfo:
            msg = CameraInfo()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera"
            msg.height = 480
            msg.width = 640
            msg.distortion_model = "plumb_bob"
            return msg
        
        else:
            # Default String message
            msg = String()
            msg.data = f"Generic message {self.counter}"
            return msg


def main():
    """Main function - create 10 demo topics"""
    rclpy.init()
    
    print("=" * 80)
    print("ROS2 DEMO TOPICS GENERATOR - Starting 10 Concurrent Test Topics")
    print("=" * 80)
    
    # Define 10 demo topics with different message types and rates
    demo_topics = [
        # Sensor data (high frequency)
        ("/sensor/lidar", LaserScan, 10.0),
        ("/sensor/camera/info", CameraInfo, 5.0),
        ("/sensor/imu", Imu, 50.0),
        
        # Motor/Drive commands
        ("/motor/left_wheel", Float64, 20.0),
        ("/motor/right_wheel", Float64, 20.0),
        
        # Navigation/Odometry
        ("/odometry/filtered", Odometry, 30.0),
        
        # Control commands
        ("/cmd_vel", Twist, 10.0),
        
        # Status/Diagnostics
        ("/status/heartbeat", Bool, 5.0),
        ("/battery/voltage", Float64, 2.0),
        
        # Debug
        ("/debug/info", String, 5.0),
    ]
    
    # Create all publishers
    publishers = []
    executor = MultiThreadedExecutor(num_threads=10)
    
    for topic_name, msg_type, rate_hz in demo_topics:
        try:
            pub = DemoTopicPublisher(topic_name, msg_type, rate_hz)
            publishers.append(pub)
            executor.add_node(pub)
        except Exception as e:
            print(f"Error creating publisher for {topic_name}: {e}")
    
    print(f"\n✅ Created {len(publishers)} demo topics successfully!")
    print("\n📊 Demo Topics Summary:")
    print("-" * 80)
    for i, (topic_name, msg_type, rate_hz) in enumerate(demo_topics, 1):
        print(f"{i:2d}. {topic_name:40s} | {msg_type.__name__:20s} | {rate_hz:5.1f} Hz")
    
    print("\n" + "=" * 80)
    print("🚀 Publishing demo data... Press Ctrl+C to stop")
    print("=" * 80 + "\n")
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\n\n⏹️  Shutting down demo topics...")
    finally:
        for pub in publishers:
            pub.destroy_node()
        rclpy.shutdown()
        print("✅ Demo topics generator stopped")


if __name__ == '__main__':
    main()
