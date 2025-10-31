#!/usr/bin/env python3
"""
Quick test script to verify ROS2 Dashboard installation
"""

import sys

def test_dependencies():
    """Test all required dependencies"""
    print("=" * 60)
    print("TESTING DEPENDENCIES")
    print("=" * 60)
    
    dependencies = {
        'PyQt5': ['PyQt5.QtWidgets', 'PyQt5.QtCore', 'PyQt5.QtGui'],
        'System': ['psutil', 'yaml'],
        'Network': ['requests', 'flask', 'flask_cors']
    }
    
    all_ok = True
    for category, modules in dependencies.items():
        print(f"\n{category} Libraries:")
        for module in modules:
            try:
                __import__(module)
                print(f"  ✅ {module}")
            except ImportError as e:
                print(f"  ❌ {module}: {e}")
                all_ok = False
    
    return all_ok

def test_modules():
    """Test all application modules"""
    print("\n" + "=" * 60)
    print("TESTING APPLICATION MODULES")
    print("=" * 60)
    
    modules = {
        'Core': [
            'core.ros2_manager',
            'core.metrics_collector',
            'core.network_manager'
        ],
        'GUI': [
            'gui.main_window',
            'gui.topic_monitor',
            'gui.node_monitor',
            'gui.service_monitor',
            'gui.topic_echo',
            'gui.bag_playback',
            'gui.recording_control',
            'gui.metrics_display',
            'gui.advanced_stats',
            'gui.network_upload'
        ],
        'Entry Points': [
            'main',
            'upload_server'
        ]
    }
    
    all_ok = True
    for category, module_list in modules.items():
        print(f"\n{category} Modules:")
        for module in module_list:
            try:
                __import__(module)
                print(f"  ✅ {module}")
            except ImportError as e:
                print(f"  ❌ {module}: {e}")
                all_ok = False
    
    return all_ok

def test_instantiation():
    """Test core class instantiation"""
    print("\n" + "=" * 60)
    print("TESTING CORE CLASS INSTANTIATION")
    print("=" * 60)
    
    all_ok = True
    
    try:
        from core.ros2_manager import ROS2Manager
        ros2_mgr = ROS2Manager()
        print("  ✅ ROS2Manager")
    except Exception as e:
        print(f"  ❌ ROS2Manager: {e}")
        all_ok = False
    
    try:
        from core.metrics_collector import MetricsCollector
        metrics = MetricsCollector()
        metrics.reset()
        data = metrics.get_metrics()
        assert isinstance(data, dict)
        print("  ✅ MetricsCollector")
    except Exception as e:
        print(f"  ❌ MetricsCollector: {e}")
        all_ok = False
    
    try:
        from core.network_manager import NetworkManager
        net_mgr = NetworkManager()
        import os
        assert os.path.exists(net_mgr.db_path)
        print("  ✅ NetworkManager")
        print(f"     Database: {net_mgr.db_path}")
    except Exception as e:
        print(f"  ❌ NetworkManager: {e}")
        all_ok = False
    
    return all_ok

def main():
    """Run all tests"""
    print("\n🔧 ROS2 Dashboard Installation Test")
    print()
    
    results = []
    
    # Test dependencies
    results.append(("Dependencies", test_dependencies()))
    
    # Test modules
    results.append(("Modules", test_modules()))
    
    # Test instantiation
    results.append(("Instantiation", test_instantiation()))
    
    # Summary
    print("\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    
    all_passed = True
    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"{status} - {name}")
        if not passed:
            all_passed = False
    
    print("\n" + "=" * 60)
    if all_passed:
        print("✅ ALL TESTS PASSED - Installation is correct!")
        print("\nYou can now run the dashboard:")
        print("  python3 main.py")
        print("\nOptionally start the upload server:")
        print("  python3 upload_server.py")
        return 0
    else:
        print("❌ SOME TESTS FAILED - Check errors above")
        print("\nPlease install missing dependencies:")
        print("  pip3 install -r requirements.txt")
        return 1

if __name__ == "__main__":
    sys.exit(main())
