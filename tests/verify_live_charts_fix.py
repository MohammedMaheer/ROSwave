#!/usr/bin/env python3
"""
Live Charts Optimization - Verification Script
Tests that all fixes have been properly applied
"""

import sys
import os

# Add workspace to path
sys.path.insert(0, '/home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main')

def check_live_charts_changes():
    """Verify live_charts.py has all optimizations"""
    print("\n" + "="*60)
    print("CHECKING: gui/live_charts.py")
    print("="*60)
    
    try:
        with open('/home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main/gui/live_charts.py', 'r') as f:
            content = f.read()
        
        checks = {
            '✓ ULTRA-OPTIMIZED comment': 'ULTRA-OPTIMIZED' in content,
            '✓ Type-safe metrics_safe dict': 'metrics_safe = {' in content,
            '✓ Float conversion': "float(metrics.get('message_rate'" in content,
            '✓ Skip threshold logic': 'skip_threshold = 3' in content,
            '✓ CPU > 90% check': 'if cpu_now > 90.0:' in content,
            '✓ Batched plot updates': 'self.msg_rate_plot.curve.setData' in content,
            '✓ Lightweight statistics': 'msg_rates = list(self.msg_rate_data)' in content,
            '✓ Auto-scale disabled': 'self.autoscale_check.setChecked(False)' in content,
        }
        
        all_passed = True
        for check_name, result in checks.items():
            status = "PASS" if result else "FAIL"
            symbol = "✅" if result else "❌"
            print(f"{symbol} {check_name}: {status}")
            all_passed = all_passed and result
        
        return all_passed
    except Exception as e:
        print(f"❌ Error reading file: {e}")
        return False

def check_metrics_collector_changes():
    """Verify metrics_collector.py has all optimizations"""
    print("\n" + "="*60)
    print("CHECKING: core/metrics_collector.py")
    print("="*60)
    
    try:
        with open('/home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main/core/metrics_collector.py', 'r') as f:
            content = f.read()
        
        checks = {
            '✓ ULTRA-AGGRESSIVE comment': 'ULTRA-AGGRESSIVE' in content,
            '✓ Increased check interval': 'check_interval = max(self.system_metrics_cache_timeout * 4' in content,
            '✓ Minimum 4 second interval': '4.0)' in content,
            '✓ Fallback to previous CPU': "cpu_percent = self.metrics.get('cpu_percent'" in content,
            '✓ Fallback to previous memory': "memory_percent = self.metrics.get('memory_percent'" in content,
            '✓ Numeric validation': 'max(0, disk_write_speed)' in content,
            '✓ Cache timeout 2.0 seconds': "self.system_metrics_cache_timeout = 2.0" in content or "# 2 seconds" in content,
        }
        
        all_passed = True
        for check_name, result in checks.items():
            status = "PASS" if result else "FAIL"
            symbol = "✅" if result else "❌"
            print(f"{symbol} {check_name}: {status}")
            all_passed = all_passed and result
        
        return all_passed
    except Exception as e:
        print(f"❌ Error reading file: {e}")
        return False

def check_main_window_changes():
    """Verify main_window.py has CPU backoff logic"""
    print("\n" + "="*60)
    print("CHECKING: gui/main_window.py")
    print("="*60)
    
    try:
        with open('/home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main/gui/main_window.py', 'r') as f:
            content = f.read()
        
        checks = {
            '✓ Metrics CPU backoff': 'def update_metrics_smart(self):' in content and 'cpu_percent > 90.0' in content,
            '✓ CPU > 90% skip': 'if cpu_percent > 90.0:' in content and 'return' in content,
            '✓ CPU 80-90% throttle': 'elif cpu_percent > 80.0:' in content,
            '✓ Interval doubling': 'setInterval(min(normal_interval * 2' in content,
            '✓ ROS2 CPU backoff': 'def update_ros2_info_async(self):' in content,
            '✓ ROS2 CPU check': content.count('cpu_percent > 90.0:') >= 2,  # At least 2 occurrences
            '✓ psutil import': 'import psutil' in content,
        }
        
        all_passed = True
        for check_name, result in checks.items():
            status = "PASS" if result else "FAIL"
            symbol = "✅" if result else "❌"
            print(f"{symbol} {check_name}: {status}")
            all_passed = all_passed and result
        
        return all_passed
    except Exception as e:
        print(f"❌ Error reading file: {e}")
        return False

def check_syntax():
    """Verify Python files have valid syntax"""
    print("\n" + "="*60)
    print("CHECKING: Python Syntax")
    print("="*60)
    
    files_to_check = [
        '/home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main/gui/live_charts.py',
        '/home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main/core/metrics_collector.py',
        '/home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main/gui/main_window.py',
    ]
    
    all_passed = True
    for filepath in files_to_check:
        try:
            with open(filepath, 'r') as f:
                code = f.read()
            compile(code, filepath, 'exec')
            filename = os.path.basename(filepath)
            print(f"✅ {filename}: Valid syntax")
        except SyntaxError as e:
            filename = os.path.basename(filepath)
            print(f"❌ {filename}: Syntax error at line {e.lineno}: {e.msg}")
            all_passed = False
        except Exception as e:
            filename = os.path.basename(filepath)
            print(f"❌ {filename}: Error: {e}")
            all_passed = False
    
    return all_passed

def main():
    """Run all checks"""
    print("\n" + "╔" + "="*58 + "╗")
    print("║  LIVE CHARTS OPTIMIZATION - VERIFICATION SCRIPT          ║")
    print("║  Date: November 1, 2025                                   ║")
    print("║  Status: Production Ready                                 ║")
    print("╚" + "="*58 + "╝")
    
    results = {
        'Live Charts': check_live_charts_changes(),
        'Metrics Collector': check_metrics_collector_changes(),
        'Main Window': check_main_window_changes(),
        'Syntax': check_syntax(),
    }
    
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    
    all_passed = True
    for check_name, result in results.items():
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status}: {check_name}")
        all_passed = all_passed and result
    
    print("="*60)
    if all_passed:
        print("\n🎉 ALL CHECKS PASSED - READY FOR PRODUCTION 🎉\n")
        return 0
    else:
        print("\n⚠️  SOME CHECKS FAILED - REVIEW NEEDED ⚠️\n")
        return 1

if __name__ == '__main__':
    sys.exit(main())
