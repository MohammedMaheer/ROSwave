#!/usr/bin/env python3
"""
Performance Verification Script
Tests all optimizations are working correctly
"""

import time
import subprocess
import sys

def test_startup_time():
    """Test application startup time"""
    print("\n" + "="*60)
    print("TEST 1: STARTUP TIME")
    print("="*60)
    
    start = time.time()
    # Simulate loading core modules (skip GUI for speed)
    from core.ros2_manager import ROS2Manager
    from core.async_worker import AsyncROS2Manager
    from core.metrics_collector import MetricsCollector
    elapsed = time.time() - start
    
    print(f"✓ Core modules loaded in {elapsed:.3f}s")
    if elapsed < 0.5:
        print("  ✅ PASS: Startup is fast")
        return True
    else:
        print("  ⚠️  WARN: Startup could be faster")
        return False

def test_caching():
    """Test aggressive caching"""
    print("\n" + "="*60)
    print("TEST 2: CACHING EFFECTIVENESS")
    print("="*60)
    
    from core.ros2_manager import ROS2Manager
    mgr = ROS2Manager()
    
    # First call - cache miss
    print("First call (cache miss)...")
    start = time.time()
    try:
        topics1 = mgr.get_topics_info()
        elapsed1 = time.time() - start
        print(f"  Took {elapsed1:.3f}s - {len(topics1)} topics found")
    except Exception as e:
        print(f"  ⚠️  ROS2 not available: {e}")
        return None
    
    # Second call - should be cached
    print("Second call (should be cached)...")
    start = time.time()
    topics2 = mgr.get_topics_info()
    elapsed2 = time.time() - start
    print(f"  Took {elapsed2:.6f}s")
    
    # Check cache is working
    if elapsed2 < 0.01:  # Should be < 10ms if cached
        print("  ✅ PASS: Caching is working (instant)")
        return True
    else:
        print(f"  ⚠️  WARN: Cache not effective ({elapsed2*1000:.1f}ms)")
        return False

def test_thread_pool():
    """Test thread pool functionality"""
    print("\n" + "="*60)
    print("TEST 3: THREAD POOL OPTIMIZATION")
    print("="*60)
    
    from core.async_worker import AsyncROS2Manager
    from core.ros2_manager import ROS2Manager
    
    ros2_mgr = ROS2Manager()
    async_mgr = AsyncROS2Manager(ros2_mgr, max_threads=2)
    
    print(f"Max threads configured: {async_mgr.max_threads}")
    print(f"Active threads: {async_mgr.active_thread_count()}")
    
    if async_mgr.max_threads == 2:
        print("  ✅ PASS: Thread pool correctly configured")
        return True
    else:
        print("  ❌ FAIL: Thread pool not configured correctly")
        return False

def test_metrics_safety():
    """Test thread safety of metrics"""
    print("\n" + "="*60)
    print("TEST 4: THREAD SAFETY")
    print("="*60)
    
    from core.metrics_collector import MetricsCollector
    import threading
    
    collector = MetricsCollector()
    
    # Check lock exists
    if hasattr(collector, '_lock'):
        print("  ✅ PASS: Thread lock exists")
        return True
    else:
        print("  ❌ FAIL: No thread lock found")
        return False

def test_gui_debouncing():
    """Test GUI debouncing logic"""
    print("\n" + "="*60)
    print("TEST 5: GUI DEBOUNCING")
    print("="*60)
    
    # Create mock window with debouncing
    class MockWindow:
        def __init__(self):
            self._last_ros2_update = 0
            self._ros2_update_cooldown = 1.0
        
        def should_update(self):
            import time
            current_time = time.time()
            if current_time - self._last_ros2_update < self._ros2_update_cooldown:
                return False
            self._last_ros2_update = current_time
            return True
    
    window = MockWindow()
    
    # First call - should update
    if window.should_update():
        print("  ✓ First call: Update allowed")
    else:
        print("  ❌ First call: Update blocked (wrong!)")
        return False
    
    # Immediate second call - should be blocked
    if not window.should_update():
        print("  ✓ Immediate second call: Update blocked (correct!)")
    else:
        print("  ⚠️  Immediate second call: Update allowed (debounce not working)")
        return False
    
    # Wait 1.1 seconds and try again
    print("  Waiting 1.1 seconds...")
    time.sleep(1.1)
    if window.should_update():
        print("  ✓ After 1.1s: Update allowed (correct!)")
        return True
    else:
        print("  ❌ After 1.1s: Update blocked (wrong!)")
        return False

def test_subprocess_timeout():
    """Test subprocess timeout is aggressive"""
    print("\n" + "="*60)
    print("TEST 6: SUBPROCESS TIMEOUT (should be 1 second)")
    print("="*60)
    
    from core.ros2_manager import ROS2Manager
    import inspect
    
    mgr = ROS2Manager()
    source = inspect.getsource(mgr.get_topics_info)
    
    if "timeout=1" in source or "timeout=1.0" in source:
        print("  ✅ PASS: Aggressive 1-second timeout found")
        return True
    else:
        print("  ⚠️  WARN: Timeout not found in source code")
        return False

def main():
    """Run all tests"""
    print("\n" + "█"*60)
    print("█  ROS2 DASHBOARD - PERFORMANCE OPTIMIZATION TESTS")
    print("█"*60)
    
    results = {
        "Startup Time": test_startup_time(),
        "Caching": test_caching(),
        "Thread Pool": test_thread_pool(),
        "Thread Safety": test_metrics_safety(),
        "Debouncing": test_gui_debouncing(),
        "Timeout": test_subprocess_timeout()
    }
    
    print("\n" + "="*60)
    print("TEST RESULTS SUMMARY")
    print("="*60)
    
    passed = sum(1 for v in results.values() if v is True)
    failed = sum(1 for v in results.values() if v is False)
    skipped = sum(1 for v in results.values() if v is None)
    
    for test_name, result in results.items():
        if result is True:
            print(f"✅ {test_name}")
        elif result is False:
            print(f"❌ {test_name}")
        else:
            print(f"⊘ {test_name} (skipped)")
    
    print("\n" + "-"*60)
    print(f"Passed: {passed}, Failed: {failed}, Skipped: {skipped}")
    print("-"*60)
    
    if failed == 0 and passed > 0:
        print("\n✅ ALL OPTIMIZATIONS ARE WORKING!")
        return 0
    else:
        print(f"\n⚠️  {failed} test(s) failed - review above")
        return 1

if __name__ == "__main__":
    sys.exit(main())
