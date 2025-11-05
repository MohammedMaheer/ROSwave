#!/usr/bin/env python3
"""
Test script to verify real-time monitoring enhancements
- Tests periodic Hz refresh functionality
- Tests recording state tracking
- Tests status column updates
"""

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from PyQt5.QtCore import QTimer, Qt
from gui.topic_monitor import TopicMonitorWidget


class MockRos2Manager:
    """Mock ROS2 manager for testing"""
    
    def __init__(self):
        self.hz_count = 0
    
    def get_topics_info(self):
        """Return mock topics"""
        return [
            {
                'name': '/sensor_data',
                'type': 'sensor_msgs/msg/LaserScan',
                'publisher_count': 1,
                'hz': 10.0 + self.hz_count
            },
            {
                'name': '/cmd_vel',
                'type': 'geometry_msgs/msg/Twist',
                'publisher_count': 1,
                'hz': 5.0 + self.hz_count
            },
            {
                'name': '/idle_topic',
                'type': 'std_msgs/msg/String',
                'publisher_count': 0,
                'hz': 0.0
            }
        ]
    
    def get_topics_hz_batch(self, topics, max_workers=4):
        """Return mock Hz values"""
        # Increment for each call to show real-time updates
        self.hz_count += 0.5
        return {
            '/sensor_data': 10.0 + self.hz_count,
            '/cmd_vel': 5.0 + self.hz_count,
            '/idle_topic': 0.0
        }


def test_topic_monitor():
    """Test topic monitor with real-time monitoring"""
    app = QApplication(sys.argv)
    
    # Create test window
    window = QWidget()
    window.setWindowTitle("Real-Time Topic Monitor Test")
    window.setGeometry(100, 100, 1000, 600)
    
    layout = QVBoxLayout()
    
    # Create mock manager
    mock_manager = MockRos2Manager()
    
    # Create topic monitor
    topic_monitor = TopicMonitorWidget(mock_manager)
    layout.addWidget(topic_monitor)
    
    window.setLayout(layout)
    
    # Test 1: Initial load
    print("\n✅ TEST 1: Initial topic loading")
    topic_monitor.refresh_topics()
    QTimer.singleShot(500, lambda: print(f"   Topics loaded: {topic_monitor.topics_table.rowCount()} rows"))
    
    # Test 2: Recording state enabled (should start periodic Hz refresh)
    print("\n✅ TEST 2: Recording state enabled")
    QTimer.singleShot(1000, lambda: test_recording_enabled(topic_monitor))
    
    # Test 3: Verify periodic refresh is running
    print("\n✅ TEST 3: Verify periodic Hz refresh timer")
    QTimer.singleShot(1500, lambda: verify_hz_timer(topic_monitor))
    
    # Test 4: Recording state disabled (should stop periodic Hz refresh)
    print("\n✅ TEST 4: Recording state disabled")
    QTimer.singleShot(2000, lambda: test_recording_disabled(topic_monitor))
    
    # Test 5: Verify status column
    print("\n✅ TEST 5: Verify status column updates")
    QTimer.singleShot(2500, lambda: verify_status_column(topic_monitor))
    
    # Exit after tests
    QTimer.singleShot(3000, app.quit)
    
    window.show()
    sys.exit(app.exec_())


def test_recording_enabled(topic_monitor):
    """Test recording state enabled"""
    topic_monitor.set_recording_state(True)
    is_running = topic_monitor._hz_refresh_timer.isActive()
    status = "✓ PASS" if is_running else "✗ FAIL"
    print(f"   {status}: Hz refresh timer is {'active' if is_running else 'inactive'}")


def verify_hz_timer(topic_monitor):
    """Verify Hz refresh timer is configured correctly"""
    interval = topic_monitor._hz_refresh_timer.interval()
    is_active = topic_monitor._hz_refresh_timer.isActive()
    status = "✓ PASS" if is_active and interval == 10000 else "✗ FAIL"
    print(f"   {status}: Timer interval={interval}ms, active={is_active}")


def test_recording_disabled(topic_monitor):
    """Test recording state disabled"""
    topic_monitor.set_recording_state(False)
    is_running = topic_monitor._hz_refresh_timer.isActive()
    status = "✓ PASS" if not is_running else "✗ FAIL"
    print(f"   {status}: Hz refresh timer is {'stopped' if not is_running else 'still running'}")


def verify_status_column(topic_monitor):
    """Verify status column exists and has values"""
    table = topic_monitor.topics_table
    col_count = table.columnCount()
    has_status_col = col_count == 6
    
    # Check if status column has data
    status_values = []
    for row in range(table.rowCount()):
        item = table.item(row, 5)
        if item:
            status_values.append(item.text())
    
    has_status_data = len(status_values) > 0
    status = "✓ PASS" if has_status_col and has_status_data else "✗ FAIL"
    print(f"   {status}: Status column exists={has_status_col}, has data={has_status_data}")
    if status_values:
        print(f"   Status values: {status_values}")


if __name__ == '__main__':
    print("=" * 60)
    print("REAL-TIME TOPIC MONITOR TESTS")
    print("=" * 60)
    test_topic_monitor()
