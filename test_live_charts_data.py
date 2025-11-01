#!/usr/bin/env python3
"""
Diagnose Live Charts Data Issue
Tests if charts are receiving data from metrics collector
"""

import sys
import time
from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QTimer

def test_live_charts_data():
    """Test that live charts receive data from metrics collector"""
    print("="*70)
    print("LIVE CHARTS DATA DIAGNOSTIC")
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
    
    # Test metrics collector directly
    print("\n2️⃣  Testing MetricsCollector.get_live_metrics()...")
    test_metrics = metrics.get_live_metrics(None)
    print(f"   Metrics returned: {test_metrics is not None}")
    if test_metrics:
        print(f"   Keys: {list(test_metrics.keys())}")
        print(f"   CPU: {test_metrics.get('cpu_percent', 'N/A')}")
        print(f"   Memory: {test_metrics.get('memory_percent', 'N/A')}")
        print(f"   Message rate: {test_metrics.get('message_rate', 'N/A')}")
        print(f"   Topic count: {test_metrics.get('topic_count', 'N/A')}")
    else:
        print("   ❌ PROBLEM: get_live_metrics() returned None!")
    
    # Create widget
    print("\n3️⃣  Creating LiveChartsWidget...")
    widget = LiveChartsWidget(
        metrics, 
        ros2_mgr, 
        buffer_size=60, 
        update_interval=300,
        auto_pause=True
    )
    
    print(f"✅ Widget created")
    print(f"   Paused: {widget.paused}")
    print(f"   Update interval: {widget.update_interval}ms")
    print(f"   Charts loaded: {widget._charts_loaded}")
    
    # Trigger async loading
    print("\n4️⃣  Loading charts...")
    from PyQt5.QtGui import QShowEvent
    event = QShowEvent()
    widget.showEvent(event)
    
    # Process async loading
    for i in range(10):
        QApplication.processEvents()
        time.sleep(0.1)
        if widget._charts_loaded:
            break
    
    if not widget._charts_loaded:
        print("❌ PROBLEM: Charts did not load!")
        return False
    
    print("✅ Charts loaded")
    print(f"   Timer active: {widget.update_timer.isActive()}")
    print(f"   Timer interval: {widget.update_timer.interval()}ms")
    print(f"   Paused: {widget.paused}")
    
    # Force a few updates
    print("\n5️⃣  Testing chart updates...")
    print("   Forcing 5 update cycles...")
    
    for i in range(5):
        print(f"\n   Cycle {i+1}:")
        
        # Get metrics
        test_metrics = metrics.get_live_metrics(None)
        if test_metrics:
            print(f"     Metrics: CPU={test_metrics.get('cpu_percent', 0):.1f}%, "
                  f"MEM={test_metrics.get('memory_percent', 0):.1f}%, "
                  f"Topics={test_metrics.get('topic_count', 0)}")
        else:
            print(f"     ⚠️  Metrics returned None")
        
        # Call update_charts
        widget.update_charts()
        
        # Check data buffers
        print(f"     Buffer sizes: time={len(widget.time_data)}, "
              f"cpu={len(widget.cpu_data)}, "
              f"mem={len(widget.memory_data)}")
        
        if len(widget.time_data) > 0:
            print(f"     Latest data: time={widget.time_data[-1]:.1f}s, "
                  f"cpu={widget.cpu_data[-1]:.1f}%, "
                  f"mem={widget.memory_data[-1]:.1f}%")
        
        QApplication.processEvents()
        time.sleep(0.5)
    
    # Final check
    print("\n6️⃣  Final Status:")
    print(f"   Total data points collected: {len(widget.time_data)}")
    print(f"   Update counter: {widget.update_counter}")
    print(f"   Start time initialized: {widget.start_time is not None}")
    
    if len(widget.time_data) == 0:
        print("\n❌ PROBLEM IDENTIFIED: No data collected!")
        print("   Possible causes:")
        print("   1. metrics_collector.get_live_metrics() returns None")
        print("   2. Charts are paused (self.paused=True)")
        print("   3. Update timer not running")
        print("   4. Exception during data collection")
        
        # Debug
        print(f"\n   Debug info:")
        print(f"     self.paused = {widget.paused}")
        print(f"     timer.isActive() = {widget.update_timer.isActive()}")
        print(f"     self._charts_loaded = {widget._charts_loaded}")
        
        return False
    
    print("\n✅ Data collection working!")
    print(f"   CPU data range: {min(widget.cpu_data):.1f}% - {max(widget.cpu_data):.1f}%")
    print(f"   Memory data range: {min(widget.memory_data):.1f}% - {max(widget.memory_data):.1f}%")
    
    # Check if plots have data
    print("\n7️⃣  Checking plot data...")
    if hasattr(widget.cpu_plot, 'curve'):
        curve_data = widget.cpu_plot.curve.getData()
        if curve_data is not None and len(curve_data) == 2:
            x_data, y_data = curve_data
            print(f"   CPU plot has {len(x_data) if x_data is not None else 0} points")
        else:
            print(f"   ⚠️  CPU plot has no data!")
    
    return True

if __name__ == "__main__":
    try:
        success = test_live_charts_data()
        print("\n" + "="*70)
        if success:
            print("✅ DIAGNOSTIC PASSED - Charts should be working")
        else:
            print("❌ DIAGNOSTIC FAILED - See errors above")
        print("="*70)
        sys.exit(0 if success else 1)
    except Exception as e:
        print(f"\n❌ Diagnostic failed with exception: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
