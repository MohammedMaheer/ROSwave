#!/bin/bash

# Test script to verify write speed metric calculation

echo "=== Write Speed Metric Test ==="
echo ""

# Check if ROS2 is available
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please source ROS2 workspace first."
    exit 1
fi

echo "✅ ROS2 found"
echo ""

# Start the dashboard in background
echo "Starting ROS2 Dashboard..."
cd /tmp/ros2_dashboard
python3 main.py &
DASHBOARD_PID=$!
echo "Dashboard started (PID: $DASHBOARD_PID)"
sleep 5

# Start a test publisher that generates significant data
echo ""
echo "Starting test data publisher (image messages for faster data generation)..."
echo "This will publish 640x480 RGB images at 10 Hz to generate ~10 MB/s"
echo ""

# Create a simple image publisher
python3 << 'EOF' &
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
import time

class TestImagePublisher(Node):
    def __init__(self):
        super().__init__('test_image_publisher')
        self.publisher = self.create_publisher(Image, '/test/camera/image_raw', 10)
        self.timer = self.create_timer(0.1, self.publish_image)  # 10 Hz
        self.counter = 0
        
    def publish_image(self):
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera'
        msg.height = 480
        msg.width = 640
        msg.encoding = 'rgb8'
        msg.step = 640 * 3
        # Generate random image data (~1 MB per message)
        msg.data = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8).tobytes()
        self.publisher.publish(msg)
        self.counter += 1
        if self.counter % 10 == 0:
            print(f"Published {self.counter} images (~{self.counter} MB)")

def main():
    rclpy.init()
    node = TestImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF

PUBLISHER_PID=$!
echo "Test publisher started (PID: $PUBLISHER_PID)"

echo ""
echo "=== Test Instructions ==="
echo "1. In the dashboard, go to 'Topic Selection' tab"
echo "2. Check the '/test/camera/image_raw' topic"
echo "3. Go to 'Recording Control' tab"
echo "4. Click 'Start Recording'"
echo "5. Wait for 10-15 seconds while watching the metrics"
echo "6. Observe the 'Write Speed' value in the metrics display"
echo "7. It should show 0.0 MB/s on first update, then ~1-10 MB/s on subsequent updates"
echo "8. Check the terminal for debug output showing calculations"
echo ""
echo "Expected output:"
echo "  [DEBUG] First update - write speed set to 0, size: 0.XXX MB"
echo "  [DEBUG] Write speed: X.XXX MB/s (delta: X.XXX MB / 1.00 s)"
echo "  [DEBUG] Write speed: X.XXX MB/s (delta: X.XXX MB / 1.00 s)"
echo ""
echo "Press Ctrl+C when done testing"

# Wait for user to finish testing
wait $DASHBOARD_PID

# Cleanup
echo ""
echo "Cleaning up..."
kill $PUBLISHER_PID 2>/dev/null
echo "Test complete!"
