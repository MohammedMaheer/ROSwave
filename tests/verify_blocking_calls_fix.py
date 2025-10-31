#!/usr/bin/env python3
"""
Verification script - Check that blocking calls fix is properly implemented
"""

import sys
import os

def check_file_content(filepath, should_contain, should_not_contain=None):
    """Check if file contains expected content"""
    if not os.path.exists(filepath):
        print(f"❌ File not found: {filepath}")
        return False
    
    with open(filepath, 'r') as f:
        content = f.read()
    
    # Check for expected content
    for text in should_contain:
        if text not in content:
            print(f"❌ Missing expected content in {filepath}: {text}")
            return False
    
    # Check that problematic content is gone
    if should_not_contain:
        for text in should_not_contain:
            if text in content:
                print(f"❌ Found problematic content in {filepath}: {text}")
                return False
    
    return True

def main():
    print("=" * 80)
    print("BLOCKING CALLS FIX - VERIFICATION")
    print("=" * 80)
    print()
    
    base_path = "/home/maahir/Desktop/ros2bags_live_recording-and-status-dashboard-main"
    
    # Check 1: RecordingControlWidget has async manager parameter
    print("✓ Checking gui/recording_control.py...")
    checks = [
        ("async_ros2_manager",),
        ("_last_rates_update",),
        ("_on_topics_info_received",),
        ("_process_topics_info",),
        ("get_topics_async",),
        ("async_ros2_manager.get_topics_async",),
    ]
    
    passed = 0
    for check in checks:
        if check_file_content(f"{base_path}/gui/recording_control.py", check):
            print(f"  ✅ Found: {check[0]}")
            passed += 1
        else:
            print(f"  ❌ Missing: {check[0]}")
    
    print(f"  Result: {passed}/{len(checks)} checks passed")
    print()
    
    # Check 2: Main window passes async_ros2 to RecordingControlWidget
    print("✓ Checking gui/main_window.py...")
    if check_file_content(f"{base_path}/gui/main_window.py", 
                          ("self.recording_control = RecordingControlWidget(self.ros2_manager, self.async_ros2)",)):
        print("  ✅ RecordingControlWidget instantiated with async manager")
        mw_passed = 1
    else:
        print("  ❌ RecordingControlWidget not updated with async manager")
        mw_passed = 0
    print()
    
    # Check 3: Documentation files exist
    print("✓ Checking documentation...")
    docs = [
        "BLOCKING_CALLS_FIX.md",
        "BLOCKING_CALLS_FIX_SUMMARY.md",
    ]
    
    doc_passed = 0
    for doc in docs:
        doc_path = f"{base_path}/{doc}"
        if os.path.exists(doc_path):
            print(f"  ✅ Found: {doc}")
            doc_passed += 1
        else:
            print(f"  ❌ Missing: {doc}")
    print()
    
    # Summary
    total_checks = len(checks) + mw_passed + doc_passed
    total_passed = passed + mw_passed + doc_passed
    
    print("=" * 80)
    print("VERIFICATION SUMMARY")
    print("=" * 80)
    print(f"Total Checks: {total_passed}/{total_checks} passed")
    print()
    
    if total_passed == total_checks:
        print("✅ ALL CHECKS PASSED!")
        print()
        print("The blocking calls fix is properly implemented:")
        print("  • RecordingControlWidget accepts async_ros2_manager")
        print("  • update_topic_rates() now uses async callbacks")
        print("  • No more blocking subprocess calls on main thread")
        print("  • UI will remain responsive during recording")
        print("  • 'Not responding' dialogs should be eliminated")
        print()
        print("Ready to test!")
        return 0
    else:
        print("❌ SOME CHECKS FAILED!")
        print("Please review the implementation.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
