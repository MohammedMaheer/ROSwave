#!/bin/bash
# Test script for blocking calls fix
# This script starts the dashboard and monitors for responsiveness

echo "=========================================="
echo "ROS2 Dashboard - Blocking Calls Fix Test"
echo "=========================================="
echo ""
echo "Starting dashboard with async fixes..."
echo ""
echo "IMPROVEMENTS:"
echo "✅ Fixed blocking get_topics_info() in recording_control.py"
echo "✅ Now uses async_ros2_manager for non-blocking updates"
echo "✅ Debouncing prevents excessive calls (max 1 per second)"
echo "✅ Callbacks process data without blocking UI thread"
echo ""
echo "TEST PROCEDURES:"
echo "1. Start recording with multiple topics"
echo "2. Verify dashboard remains responsive"
echo "3. Check for 'not responding' dialogs (should be GONE)"
echo "4. Monitor topic rates update smoothly"
echo "5. No UI freezing during rate calculations"
echo ""
echo "Starting application..."
echo ""

cd /home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main
python3 main.py
