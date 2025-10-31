#!/bin/bash
# ROS2 Dashboard - Display Backend Selector
# Tries different backends in order of stability

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔════════════════════════════════════════════╗"
echo "║   ROS2 Dashboard - Display Setup           ║"
echo "╚════════════════════════════════════════════╝"
echo ""

# Check if we can use X11 (most stable)
if command -v Xvfb &> /dev/null || [ -n "$DISPLAY" ]; then
    echo "🎯 Attempting X11 backend (most stable)..."
    export QT_QPA_PLATFORM=xcb
    python3 main.py "$@"
    exit $?
fi

# Try XCB (X11 protocol)
if [ -z "$DISPLAY" ]; then
    echo "📌 X11 display not found. Trying XCB protocol..."
    export QT_QPA_PLATFORM=xcb:1
    python3 main.py "$@" 2>/dev/null
    if [ $? -eq 0 ]; then
        exit 0
    fi
fi

# Try Wayland
if [ -n "$WAYLAND_DISPLAY" ]; then
    echo "🌊 Trying Wayland backend (with stability warning)..."
    echo "   Note: If you experience crashes, use X11 instead"
    echo ""
    
    # Don't set QT_QPA_PLATFORM, let it use default (which prefers Wayland)
    python3 main.py "$@"
    exit $?
fi

# Fallback: offscreen rendering (no GUI, but keeps app running)
echo "⚠️  No display server detected. Running in headless mode..."
echo "   (Dashboard will not show GUI, but will be functional)"
export QT_QPA_PLATFORM=offscreen
python3 main.py "$@"
exit $?
