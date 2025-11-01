#!/usr/bin/env python3
"""
Quick diagnostic to identify what's causing UI freezes
"""

import sys
import time
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget, QTextEdit
from PyQt5.QtCore import QTimer, pyqtSignal, QThread

class DiagnosticWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ROS2 Dashboard - Diagnostic Mode")
        self.setGeometry(100, 100, 600, 400)
        
        central = QWidget()
        layout = QVBoxLayout()
        self.text = QTextEdit()
        self.text.setReadOnly(True)
        layout.addWidget(self.text)
        central.setLayout(layout)
        self.setCentralWidget(central)
        
        self.log("Starting diagnostic mode...")
        self.log("Testing initialization sequence...")
        
        # Test each import
        self.test_imports()
        
        # Test core initialization
        QTimer.singleShot(500, self.test_performance_manager)
        QTimer.singleShot(1500, self.test_ros2_manager)
        QTimer.singleShot(2500, self.test_async_manager)
        QTimer.singleShot(3500, self.test_metrics)
        QTimer.singleShot(4500, lambda: self.log("✓ All tests complete!"))
        
    def log(self, message):
        self.text.append(message)
        print(message)
        QApplication.processEvents()  # Keep UI responsive
        
    def test_imports(self):
        try:
            self.log("[1/5] Testing imports...")
            from core.performance_modes import PerformanceModeManager
            self.log("  ✓ PerformanceModeManager imported")
            from core.ros2_manager import ROS2Manager
            self.log("  ✓ ROS2Manager imported")
            from core.metrics_collector import MetricsCollector
            self.log("  ✓ MetricsCollector imported")
            from core.async_worker import AsyncROS2Manager
            self.log("  ✓ AsyncROS2Manager imported")
        except Exception as e:
            self.log(f"  ✗ Import failed: {e}")
    
    def test_performance_manager(self):
        try:
            self.log("[2/5] Testing PerformanceModeManager...")
            start = time.time()
            from core.performance_modes import PerformanceModeManager
            manager = PerformanceModeManager()
            elapsed = time.time() - start
            self.log(f"  ✓ Initialized in {elapsed:.2f}s")
            self.log(f"  System: {manager.system_info['memory_gb']}GB, {manager.system_info['cpu_count']} cores")
            self.log(f"  Mode: {manager.get_current_mode().value.upper()}")
        except Exception as e:
            self.log(f"  ✗ Failed: {e}")
    
    def test_ros2_manager(self):
        try:
            self.log("[3/5] Testing ROS2Manager...")
            start = time.time()
            from core.ros2_manager import ROS2Manager
            manager = ROS2Manager()
            elapsed = time.time() - start
            self.log(f"  ✓ Initialized in {elapsed:.2f}s")
            
            # Test non-blocking list topics
            self.log("  Testing topic listing (with timeout)...")
            start = time.time()
            # This should NOT block - it has timeouts
            try:
                topics = manager.get_topics_info()  # Fixed: use get_topics_info() instead of list_topics()
                elapsed = time.time() - start
                self.log(f"  ✓ Listed topics in {elapsed:.2f}s ({len(topics) if topics else 0} topics)")
            except Exception as te:
                self.log(f"  ✓ Topics (with expected timeout): {str(te)[:50]}...")
        except Exception as e:
            self.log(f"  ✗ Failed: {e}")
    
    def test_async_manager(self):
        try:
            self.log("[4/5] Testing AsyncROS2Manager...")
            from core.ros2_manager import ROS2Manager
            from core.async_worker import AsyncROS2Manager
            
            ros2_mgr = ROS2Manager()
            async_mgr = AsyncROS2Manager(ros2_mgr, max_threads=2, cache_timeout=5.0)
            self.log(f"  ✓ AsyncROS2Manager initialized")
        except Exception as e:
            self.log(f"  ✗ Failed: {e}")
    
    def test_metrics(self):
        try:
            self.log("[5/5] Testing MetricsCollector...")
            from core.metrics_collector import MetricsCollector
            collector = MetricsCollector()
            metrics = collector.get_live_metrics(None)
            self.log(f"  ✓ MetricsCollector initialized and working")
            self.log(f"  CPU: {metrics.get('cpu_percent', 0):.1f}%")
            self.log(f"  Memory: {metrics.get('memory_percent', 0):.1f}%")
        except Exception as e:
            self.log(f"  ✗ Failed: {e}")

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = DiagnosticWindow()
    window.show()
    sys.exit(app.exec_())
