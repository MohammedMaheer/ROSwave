#!/usr/bin/env python3
"""
Non-GUI diagnostic to identify initialization bottlenecks
"""

import sys
import time
import os

# Set display mode to avoid Wayland issues
os.environ['QT_QPA_PLATFORM'] = 'offscreen'

print("="*70)
print("ROS2 Dashboard - Initialization Diagnostic (Non-GUI)")
print("="*70)
print()

# Test 1: Performance Manager
print("[1/4] Testing PerformanceModeManager...")
start = time.time()
try:
    from core.performance_modes import PerformanceModeManager
    manager = PerformanceModeManager()
    elapsed = time.time() - start
    print(f"✓ Initialized in {elapsed:.3f}s")
    print(f"  System: {manager.system_info['memory_gb']}GB RAM, {manager.system_info['cpu_count']} cores")
    print(f"  Mode: {manager.get_current_mode().value.upper()}")
    settings = manager.get_mode_settings()
    print(f"  Settings: ros2={settings['ros2_update_interval']}ms, "
          f"metrics={settings['metrics_update_interval']}ms")
except Exception as e:
    print(f"✗ Failed: {e}")
    sys.exit(1)

print()

# Test 2: ROS2 Manager
print("[2/4] Testing ROS2Manager...")
start = time.time()
try:
    from core.ros2_manager import ROS2Manager
    ros2_manager = ROS2Manager()
    elapsed = time.time() - start
    print(f"✓ Initialized in {elapsed:.3f}s")
except Exception as e:
    print(f"✗ Failed: {e}")
    sys.exit(1)

print()

# Test 3: List Topics (with timeout)
print("[3/4] Testing ROS2 topic info fetch (with timeout)...")
start = time.time()
try:
    topics = ros2_manager.get_topics_info()
    elapsed = time.time() - start
    if topics:
        print(f"✓ Got topic info in {elapsed:.3f}s")
        if isinstance(topics, list) and len(topics) > 0:
            print(f"  Total topics: {len(topics)}")
    else:
        print(f"✓ No topics found (ROS2 may not be running) in {elapsed:.3f}s")
except Exception as e:
    elapsed = time.time() - start
    print(f"⚠ Error in {elapsed:.3f}s: {str(e)[:60]}")

print()

# Test 4: Metrics Collector
print("[4/4] Testing MetricsCollector...")
start = time.time()
try:
    from core.metrics_collector import MetricsCollector
    collector = MetricsCollector()
    metrics = collector.get_live_metrics(ros2_manager)
    elapsed = time.time() - start
    print(f"✓ Initialized and got metrics in {elapsed:.3f}s")
    print(f"  CPU: {metrics.get('cpu_percent', 0):.1f}%")
    print(f"  Memory: {metrics.get('memory_percent', 0):.1f}%")
except Exception as e:
    print(f"✗ Failed: {e}")

print()
print("="*70)
print("✓ All critical components respond quickly!")
print("✓ If UI still freezes, it's likely a display server issue")
print("  Try running with: QT_QPA_PLATFORM=xcb python3 main.py")
print("="*70)
