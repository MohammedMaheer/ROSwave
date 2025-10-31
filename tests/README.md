# 🧪 Test Suite

Test files for the ROS2 Data Recording & Status Dashboard.

## Test Files

### Python Tests
- **test_installation.py** - Verify all dependencies are installed
- **test_ml_export.py** - Test ML dataset export functionality
- **test_smooth_optimization.py** - Test UI optimization and responsiveness
- **test_topic_rates.py** - Test topic rate monitoring

### Shell Script Tests
- **test_blocking_calls_fix.sh** - Test blocking calls fixes
- **test_robot_integration.sh** - Test robot integration
- **test_write_speed.sh** - Test bag file write performance

### Verification Scripts
- **verify_blocking_calls_fix.py** - Verify all blocking calls are fixed
- **verify_optimizations.py** - Verify all optimizations are working

### Test Guides
- **BLOCKING_CALLS_FIX_TEST_GUIDE.sh** - Guide for testing blocking calls fixes

## Running Tests

### Quick Test All Components
```bash
cd /path/to/ros2bags_live_recording-and-status-dashboard-main

# Test installation
python3 tests/test_installation.py

# Test ML export
python3 tests/test_ml_export.py

# Test optimizations
python3 tests/test_smooth_optimization.py

# Verify blocking calls fix
python3 tests/verify_blocking_calls_fix.py

# Verify all optimizations
python3 tests/verify_optimizations.py
```

### Shell Script Tests
```bash
# Test write speed
bash tests/test_write_speed.sh

# Test blocking calls fix
bash tests/test_blocking_calls_fix.sh

# Test robot integration
bash tests/test_robot_integration.sh
```

## Test Requirements

- ROS2 (Humble, Iron, or Rolling) sourced
- All Python dependencies installed (`pip install -r requirements.txt`)
- Sufficient disk space for test recordings
- Network connectivity for upload tests (optional)

## Test Coverage

✅ **Installation & Dependencies**
- Python package availability
- ROS2 environment detection
- Required tools verification

✅ **Core Functionality**
- Topic monitoring and selection
- Recording start/stop
- Bag file creation
- ML dataset export

✅ **Performance & Optimization**
- UI responsiveness
- Non-blocking operations
- Background worker functionality
- Topic rate monitoring accuracy

✅ **Network Features**
- Upload server connectivity
- Chunked upload protocol
- Retry and resume logic

## Troubleshooting Tests

### Test fails with "ROS2 not found"
```bash
# Source your ROS2 environment
source /opt/ros/humble/setup.bash  # or your distro
```

### Test fails with "Module not found"
```bash
# Install dependencies
pip install -r requirements.txt
```

### Network tests fail
```bash
# Start upload server first
python3 upload_server.py
# Then run network tests
```

## Adding New Tests

1. Create test file in `tests/` directory
2. Follow naming convention: `test_<feature>.py` or `test_<feature>.sh`
3. Add documentation to this README
4. Ensure test is self-contained and can run independently

## CI/CD Integration

These tests can be integrated into CI/CD pipelines:

```yaml
# Example GitHub Actions workflow
- name: Run Tests
  run: |
    source /opt/ros/humble/setup.bash
    pip install -r requirements.txt
    python3 tests/test_installation.py
    python3 tests/test_ml_export.py
    python3 tests/verify_optimizations.py
```

---

**Last Updated:** October 31, 2025  
**Status:** All tests passing ✅
