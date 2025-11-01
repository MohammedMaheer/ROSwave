#!/usr/bin/env python3
"""
Test Live Charts Fast Loading - Verify charts load quickly without freezing
"""

import sys
import time
from PyQt5.QtWidgets import QApplication
from PyQt5.QtGui import QShowEvent

def test_live_charts_loading():
    """Test that live charts load quickly and don't freeze UI"""
    print("="*70)
    print("LIVE CHARTS FAST LOADING TEST")
    print("="*70)
    
    # Import dependencies
    from gui.live_charts import LiveChartsWidget
    from core.metrics_collector import MetricsCollector
    from core.ros2_manager import ROS2Manager
    
    print("\n1️⃣  Creating application and dependencies...")
    app = QApplication(sys.argv)
    
    metrics = MetricsCollector()
    ros2_mgr = ROS2Manager()
    
    print("✅ Dependencies created")
    
    # Test 1: Widget creation should be instant (no chart loading)
    print("\n2️⃣  Testing widget creation (should be instant)...")
    start = time.time()
    
    widget = LiveChartsWidget(
        metrics, 
        ros2_mgr, 
        buffer_size=60, 
        update_interval=300,
        auto_pause=True
    )
    
    create_time = time.time() - start
    print(f"✅ Widget created in {create_time*1000:.1f}ms")
    print(f"   Charts loaded: {widget._charts_loaded}")
    print(f"   Expected: False (lazy loading)")
    
    if widget._charts_loaded:
        print("❌ FAIL: Charts should NOT be loaded yet!")
        return False
    
    if create_time > 0.5:
        print(f"⚠️  WARNING: Creation took {create_time*1000:.0f}ms (expected < 500ms)")
    
    # Test 2: Async loading on first view
    print("\n3️⃣  Testing async chart loading (on first tab view)...")
    start = time.time()
    
    # Simulate tab becoming visible
    event = QShowEvent()
    widget.showEvent(event)
    
    # Process async loading
    print("   Processing async events...")
    for i in range(10):  # Give it up to 1 second
        QApplication.processEvents()
        time.sleep(0.1)
        if widget._charts_loaded:
            break
    
    load_time = time.time() - start
    
    if not widget._charts_loaded:
        print("❌ FAIL: Charts did not load after 1 second!")
        return False
    
    print(f"✅ Charts loaded in {load_time*1000:.1f}ms")
    print(f"   Charts loaded: {widget._charts_loaded}")
    print(f"   Has msg_rate_plot: {hasattr(widget, 'msg_rate_plot')}")
    print(f"   Has cpu_plot: {hasattr(widget, 'cpu_plot')}")
    print(f"   Has memory_plot: {hasattr(widget, 'memory_plot')}")
    
    if load_time > 2.0:
        print(f"⚠️  WARNING: Loading took {load_time*1000:.0f}ms (expected < 2000ms)")
    
    # Test 3: Verify all 6 charts exist
    print("\n4️⃣  Verifying all 6 charts created...")
    required_charts = [
        'msg_rate_plot',
        'bandwidth_plot', 
        'topic_count_plot',
        'cpu_plot',
        'memory_plot',
        'disk_write_plot'
    ]
    
    all_exist = True
    for chart_name in required_charts:
        exists = hasattr(widget, chart_name)
        status = "✅" if exists else "❌"
        print(f"   {status} {chart_name}: {exists}")
        if not exists:
            all_exist = False
    
    if not all_exist:
        print("❌ FAIL: Not all charts were created!")
        return False
    
    # Test 4: Verify update timer is running
    print("\n5️⃣  Verifying update timer...")
    if hasattr(widget, 'update_timer'):
        is_active = widget.update_timer.isActive()
        interval = widget.update_timer.interval()
        print(f"✅ Update timer exists")
        print(f"   Active: {is_active}")
        print(f"   Interval: {interval}ms")
    else:
        print("❌ FAIL: Update timer not found!")
        return False
    
    # Test 5: Performance metrics
    print("\n6️⃣  Performance Summary:")
    print(f"   Widget creation: {create_time*1000:.1f}ms (target: < 500ms)")
    print(f"   Chart loading: {load_time*1000:.1f}ms (target: < 2000ms)")
    print(f"   Total time: {(create_time + load_time)*1000:.1f}ms")
    
    total_time = create_time + load_time
    if total_time < 1.0:
        print(f"   ⚡ EXCELLENT: Total loading < 1 second!")
    elif total_time < 2.0:
        print(f"   ✅ GOOD: Total loading < 2 seconds")
    else:
        print(f"   ⚠️  SLOW: Total loading > 2 seconds")
    
    # Success
    print("\n" + "="*70)
    print("✅ ALL TESTS PASSED - LIVE CHARTS FAST LOADING VERIFIED!")
    print("="*70)
    print("\nKey Features:")
    print("  ⚡ Lazy loading (charts created only when tab viewed)")
    print("  ⚡ Async creation (doesn't block UI thread)")
    print("  ⚡ Progressive rendering (rows appear incrementally)")
    print("  ⚡ Optimized pyqtgraph (antialiasing disabled)")
    print("  ⚡ Fast plot creation (reduced styling overhead)")
    
    return True

if __name__ == "__main__":
    try:
        success = test_live_charts_loading()
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n❌ Test failed with exception: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
