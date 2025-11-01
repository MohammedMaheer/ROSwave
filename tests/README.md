# 🧪 ROS2 Dashboard - Test Suite & Utilities

This directory contains comprehensive tests, diagnostic tools, and verification scripts for the ROS2 Dashboard.

---

## 📋 Directory Contents

### Test Files
- `test_installation.py` - Verify installation and dependencies
- `test_ml_export.py` - Test ML export functionality
- `test_performance_optimizations.py` - Performance optimization tests
- `test_smooth_optimization.py` - Smooth operation verification
- `test_topic_rates.py` - Topic frequency testing
- `test_chart_flow.py` - Live charts data flow testing
- `test_hz_accuracy.py` - Hz calculation accuracy tests
- `test_live_charts_reset.py` - Charts reset functionality
- `test_metrics_direct.py` - Direct metrics testing
- `test_timeout_fix.py` - Timeout handling verification

### Diagnostic Tools
- `diagnostic.py` - **Main diagnostic tool** - System health checks ⭐
- `diagnostic_nogui.py` - Non-GUI diagnostic version
- `demo_topics_generator.py` - Generate demo ROS2 topics for testing

### Verification Scripts
- `verify_blocking_calls_fix.py` - Verify non-blocking operations
- `verify_live_charts_fix.py` - Verify live charts functionality
- `verify_optimizations.py` - Verify optimization implementations

### Shell Scripts
- `test_robot_integration.sh` - Robot integration testing
- `test_blocking_calls_fix.sh` - Blocking calls fix verification
- `test_write_speed.sh` - Write speed benchmarking

### Test Guides
- `BLOCKING_CALLS_FIX_TEST_GUIDE.sh` - Guide for testing blocking calls fixes

---

## 🚀 Quick Start

### Run Diagnostic Tool
```bash
# Full system diagnostic
python3 diagnostic.py

# Non-GUI version
python3 diagnostic_nogui.py
```

### Run Installation Tests
```bash
python3 test_installation.py
```

### Test ML Export
```bash
python3 test_ml_export.py
```

### Generate Demo Topics
```bash
# Start demo topic publisher
python3 demo_topics_generator.py

# In another terminal, start dashboard
cd ..
python3 main.py
```

---

## 📊 Test Categories

### 1. Installation & Dependencies
**Files**: `test_installation.py`

**Tests**:
- Python version (3.8+)
- ROS2 installation
- Required packages (PyQt5, numpy, psutil, etc.)
- Disk space
- Permissions

**Usage**:
```bash
python3 test_installation.py
```

### 2. Performance Tests
**Files**: `test_performance_optimizations.py`, `test_smooth_optimization.py`

**Tests**:
- Cache efficiency
- Thread pool performance
- UI responsiveness
- Memory usage
- CPU utilization

**Usage**:
```bash
python3 test_performance_optimizations.py
```

### 3. Functionality Tests
**Files**: `test_ml_export.py`, `test_topic_rates.py`, `test_metrics_direct.py`

**Tests**:
- ML export creation
- Topic frequency calculation
- Metrics accuracy
- Recording operations

**Usage**:
```bash
python3 test_ml_export.py
python3 test_topic_rates.py
```

### 4. Live Charts Tests
**Files**: `test_chart_flow.py`, `test_hz_accuracy.py`, `test_live_charts_reset.py`

**Tests**:
- Chart data flow
- Hz calculation accuracy
- Chart reset functionality
- Real-time updates

**Usage**:
```bash
python3 test_chart_flow.py
python3 test_hz_accuracy.py
```

### 5. Verification Tests
**Files**: `verify_*.py`

**Tests**:
- Blocking calls elimination
- Live charts functionality
- Optimization implementations

**Usage**:
```bash
python3 verify_blocking_calls_fix.py
python3 verify_live_charts_fix.py
```

---

## 🔧 Diagnostic Tools

### Main Diagnostic Tool
**File**: `diagnostic.py`

**Features**:
- Interactive GUI diagnostic window
- Tests all major components
- Shows real-time results
- Identifies issues early

**Usage**:
```bash
python3 diagnostic.py
```

**Output**:
```
Starting diagnostic mode...
Testing initialization sequence...
[1/5] Testing imports...
  ✓ PerformanceModeManager imported
  ✓ ROS2Manager imported
  ✓ MetricsCollector imported
[2/5] Testing PerformanceModeManager...
  ✓ Initialized in 0.05s
  System: 16GB, 8 cores
  Mode: BALANCED
[3/5] Testing ROS2Manager...
  ✓ Initialized in 0.12s
  ✓ Listed topics in 0.45s (12 topics)
[4/5] Testing AsyncROS2Manager...
  ✓ AsyncROS2Manager initialized
[5/5] Testing MetricsCollector...
  ✓ MetricsCollector initialized
  CPU: 15.2%
  Memory: 45.8%
✓ All tests complete!
```

### Non-GUI Diagnostic
**File**: `diagnostic_nogui.py`

For headless systems or CI/CD pipelines.

```bash
python3 diagnostic_nogui.py
```

---

## 🎯 Demo Topic Generator

**File**: `demo_topics_generator.py`

Generates demo ROS2 topics for testing without a real robot.

**Features**:
- Publishes multiple topic types
- Configurable publish rates
- Various message types (String, Int32, Float64, etc.)
- Simulates real robot behavior

**Usage**:
```bash
# Start demo publisher
python3 demo_topics_generator.py

# Topics created:
# /demo/string_topic (std_msgs/String) @ 10Hz
# /demo/int_topic (std_msgs/Int32) @ 5Hz
# /demo/float_topic (std_msgs/Float64) @ 20Hz
# /demo/pose_topic (geometry_msgs/Pose) @ 30Hz
```

---

## 📈 Running All Tests

### Quick Test Suite
```bash
# Installation check
python3 test_installation.py

# Performance check
python3 test_performance_optimizations.py

# ML export check
python3 test_ml_export.py

# Diagnostic check
python3 diagnostic.py
```

### Comprehensive Test Suite
```bash
# Run all tests
for test in test_*.py; do
    echo "Running $test..."
    python3 "$test"
done

# Run all verifications
for verify in verify_*.py; do
    echo "Running $verify..."
    python3 "$verify"
done
```

### Shell Script Tests
```bash
# Test write speed
bash test_write_speed.sh

# Test blocking calls fix
bash test_blocking_calls_fix.sh

# Test robot integration
bash test_robot_integration.sh
```

---

## 🐛 Troubleshooting

### Test Fails: "ROS2 not found"
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Verify
echo $ROS_DISTRO
ros2 --version
```

### Test Fails: "Missing dependencies"
```bash
# Install dependencies
cd ..
pip install -r requirements.txt
```

### Test Fails: "Permission denied"
```bash
# Fix permissions
chmod +x *.sh
chmod +x *.py
```

### Diagnostic Shows Errors
1. Check ROS2 is sourced
2. Verify all dependencies installed
3. Ensure adequate disk space
4. Check write permissions

---

## 📚 Additional Resources

- **Main README**: `../README.md`
- **Documentation**: `../docs/`
- **Quick Start**: `../docs/QUICK_START_V2.2.md`
- **Optimization Guide**: `../docs/COMPREHENSIVE_OPTIMIZATION_NOV2025.md`

---

## ✅ Test Checklist

Before deploying, ensure:

- [ ] Installation test passes
- [ ] Diagnostic shows no errors
- [ ] Performance tests pass
- [ ] ML export works correctly
- [ ] All verification scripts pass
- [ ] Demo topics generator works

---

**Version**: 2.2  
**Last Updated**: November 1, 2025  
**Status**: ✅ All tests passing  

---

*For questions or issues with tests, refer to the main documentation or run the diagnostic tool.*
