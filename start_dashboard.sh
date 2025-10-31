#!/bin/bash
# ROS2 Dashboard Startup Script - Wayland Fallback Support
# This script handles display server issues automatically

echo "🚀 Starting ROS2 Dashboard..."
echo ""

# Check current display server
if [ -z "$DISPLAY" ] && [ -z "$WAYLAND_DISPLAY" ]; then
    echo "⚠️  No display server detected. Please ensure X11 or Wayland is running."
    exit 1
fi

# Try to use X11 first (more stable)
if [ -n "$DISPLAY" ]; then
    echo "✓ Using X11 display: $DISPLAY"
    python3 main.py "$@"
    exit $?
fi

# If X11 not available, try XWayland through Wayland
if [ -n "$WAYLAND_DISPLAY" ]; then
    echo "⚠️  X11 not available. Attempting Wayland with fallback..."
    echo "   (If Wayland crashes, the dashboard will restart automatically)"
    
    # Run with Wayland and catch crashes
    while true; do
        python3 main.py "$@"
        EXIT_CODE=$?
        
        if [ $EXIT_CODE -eq 0 ]; then
            echo "✓ Dashboard closed normally"
            exit 0
        fi
        
        if [ $EXIT_CODE -eq 1 ] || [ $EXIT_CODE -eq 139 ]; then
            echo ""
            echo "⚠️  Dashboard crashed (possibly Wayland issue)"
            echo "🔄 Attempting to restart in 2 seconds..."
            sleep 2
            continue
        fi
        
        # Other exit codes
        exit $EXIT_CODE
    done
fi

echo "❌ No suitable display server found"
exit 1
