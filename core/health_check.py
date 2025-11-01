"""
Startup Health Checks - Validates system before launching dashboard
"""

import os
import sys
import subprocess
import shutil
from typing import Dict, List, Tuple


class HealthCheck:
    """Performs comprehensive health checks before application startup"""
    
    def __init__(self):
        self.checks_passed = []
        self.checks_failed = []
        self.warnings = []
        
    def run_all_checks(self) -> Tuple[bool, Dict]:
        """
        Run all health checks.
        
        Returns:
            Tuple of (all_passed: bool, results: dict)
        """
        print("\n" + "=" * 70)
        print("🏥 ROS2 DASHBOARD - SYSTEM HEALTH CHECK")
        print("=" * 70 + "\n")
        
        # Run checks
        self.check_python_version()
        self.check_ros2_installation()
        self.check_python_dependencies()
        self.check_disk_space()
        self.check_permissions()
        self.check_memory_available()
        
        # Summary
        print("\n" + "=" * 70)
        print("📊 HEALTH CHECK SUMMARY")
        print("=" * 70)
        print(f"✅ Passed: {len(self.checks_passed)}")
        print(f"❌ Failed: {len(self.checks_failed)}")
        print(f"⚠️  Warnings: {len(self.warnings)}")
        
        if self.checks_failed:
            print("\n❌ FAILED CHECKS:")
            for check in self.checks_failed:
                print(f"  • {check}")
                
        if self.warnings:
            print("\n⚠️  WARNINGS:")
            for warning in self.warnings:
                print(f"  • {warning}")
        
        print("=" * 70 + "\n")
        
        # Return results
        all_passed = len(self.checks_failed) == 0
        results = {
            'passed': all_passed,
            'checks_passed': self.checks_passed,
            'checks_failed': self.checks_failed,
            'warnings': self.warnings
        }
        
        return all_passed, results
        
    def check_python_version(self):
        """Check if Python version is 3.8+"""
        print("🐍 Checking Python version...")
        version = sys.version_info
        if version >= (3, 8):
            msg = f"Python {version.major}.{version.minor}.{version.micro}"
            print(f"  ✅ {msg}")
            self.checks_passed.append(msg)
        else:
            msg = f"Python {version.major}.{version.minor} < 3.8 (UNSUPPORTED)"
            print(f"  ❌ {msg}")
            self.checks_failed.append(msg)
            
    def check_ros2_installation(self):
        """Check if ROS2 is installed and sourced"""
        print("🤖 Checking ROS2 installation...")
        
        # Check ROS_DISTRO environment variable
        ros_distro = os.environ.get('ROS_DISTRO')
        if ros_distro:
            msg = f"ROS2 {ros_distro.upper()} found in environment"
            print(f"  ✅ {msg}")
            self.checks_passed.append(msg)
        else:
            msg = "ROS_DISTRO not set (ROS2 not sourced?)"
            print(f"  ⚠️  {msg}")
            self.warnings.append(msg)
        
        # Check if ros2 command exists
        ros2_cmd = shutil.which('ros2')
        if ros2_cmd:
            msg = f"ros2 command found at {ros2_cmd}"
            print(f"  ✅ {msg}")
            self.checks_passed.append(msg)
        else:
            msg = "ros2 command not found in PATH"
            print(f"  ❌ {msg}")
            self.checks_failed.append(msg)
            
    def check_python_dependencies(self):
        """Check if all required Python packages are installed"""
        print("📦 Checking Python dependencies...")
        
        required_packages = {
            'PyQt5': 'PyQt5',
            'pyqtgraph': 'pyqtgraph',
            'numpy': 'numpy',
            'psutil': 'psutil',
            'yaml': 'PyYAML',
            'requests': 'requests',
            'flask': 'Flask',
        }
        
        missing = []
        for module, package in required_packages.items():
            try:
                __import__(module)
                msg = f"{package} installed"
                print(f"  ✅ {msg}")
                self.checks_passed.append(msg)
            except ImportError:
                msg = f"{package} NOT installed"
                print(f"  ❌ {msg}")
                missing.append(package)
                self.checks_failed.append(msg)
        
        if missing:
            print(f"\n  💡 Install missing packages:")
            print(f"     pip install {' '.join(missing)}")
            
    def check_disk_space(self):
        """Check if sufficient disk space is available"""
        print("💾 Checking disk space...")
        
        home_dir = os.path.expanduser("~")
        try:
            import shutil
            total, used, free = shutil.disk_usage(home_dir)
            
            free_gb = free / (1024 ** 3)
            total_gb = total / (1024 ** 3)
            percent_free = (free / total) * 100
            
            if free_gb >= 5.0:  # At least 5GB free
                msg = f"{free_gb:.1f} GB free ({percent_free:.1f}%)"
                print(f"  ✅ {msg}")
                self.checks_passed.append(msg)
            elif free_gb >= 1.0:  # 1-5GB free
                msg = f"Low disk space: {free_gb:.1f} GB free ({percent_free:.1f}%)"
                print(f"  ⚠️  {msg}")
                self.warnings.append(msg)
            else:  # Less than 1GB free
                msg = f"CRITICAL: Only {free_gb:.1f} GB free ({percent_free:.1f}%)"
                print(f"  ❌ {msg}")
                self.checks_failed.append(msg)
                
        except Exception as e:
            msg = f"Could not check disk space: {e}"
            print(f"  ⚠️  {msg}")
            self.warnings.append(msg)
            
    def check_permissions(self):
        """Check write permissions for recordings directory"""
        print("🔐 Checking permissions...")
        
        recordings_dir = os.path.expanduser("~/ros2_recordings")
        ml_datasets_dir = os.path.expanduser("~/ros2_recordings/ml_datasets")
        
        # Check if we can create directories
        try:
            os.makedirs(recordings_dir, exist_ok=True)
            os.makedirs(ml_datasets_dir, exist_ok=True)
            
            # Try to create a test file
            test_file = os.path.join(recordings_dir, ".write_test")
            with open(test_file, 'w') as f:
                f.write("test")
            os.remove(test_file)
            
            msg = f"Write access to {recordings_dir}"
            print(f"  ✅ {msg}")
            self.checks_passed.append(msg)
            
        except Exception as e:
            msg = f"No write access to {recordings_dir}: {e}"
            print(f"  ❌ {msg}")
            self.checks_failed.append(msg)
            
    def check_memory_available(self):
        """Check if sufficient RAM is available"""
        print("🧠 Checking memory...")
        
        try:
            import psutil
            mem = psutil.virtual_memory()
            
            available_gb = mem.available / (1024 ** 3)
            total_gb = mem.total / (1024 ** 3)
            
            if mem.percent < 80:  # Less than 80% used
                msg = f"{available_gb:.1f} GB available ({100 - mem.percent:.1f}% free)"
                print(f"  ✅ {msg}")
                self.checks_passed.append(msg)
            elif mem.percent < 90:  # 80-90% used
                msg = f"High memory usage: {mem.percent:.1f}% used"
                print(f"  ⚠️  {msg}")
                self.warnings.append(msg)
            else:  # Over 90% used
                msg = f"CRITICAL memory usage: {mem.percent:.1f}% used"
                print(f"  ❌ {msg}")
                self.checks_failed.append(msg)
                
        except Exception as e:
            msg = f"Could not check memory: {e}"
            print(f"  ⚠️  {msg}")
            self.warnings.append(msg)


def run_health_check_and_continue() -> bool:
    """
    Run health check and determine if app should continue.
    
    Returns:
        True if app can continue, False if critical failures
    """
    checker = HealthCheck()
    passed, results = checker.run_all_checks()
    
    if not passed:
        print("\n⚠️  Some health checks failed!")
        print("The application may not work correctly.")
        
        # Only block if critical failures (not just warnings)
        critical_failures = [f for f in results['checks_failed'] 
                           if 'Python' in f or 'ros2 command' in f or 'CRITICAL' in f]
        
        if critical_failures:
            print("\n❌ CRITICAL FAILURES - Cannot continue:")
            for failure in critical_failures:
                print(f"  • {failure}")
            return False
        else:
            print("\n⚠️  Non-critical failures detected - continuing with degraded functionality")
            return True
    else:
        print("\n✅ All health checks passed! Starting dashboard...")
        return True
