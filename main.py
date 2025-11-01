#!/usr/bin/env python3
"""
ROS2 Bags Recording and Live Status Dashboard
Main entry point for the application with health checks
"""

import sys
from PyQt5.QtWidgets import QApplication  # type: ignore
from PyQt5.QtCore import Qt  # type: ignore


def main():
    """Main application entry point with startup health checks"""
    
    # Run health checks BEFORE initializing Qt
    print("Starting ROS2 Dashboard with system validation...\n")
    
    try:
        from core.health_check import run_health_check_and_continue
        
        if not run_health_check_and_continue():
            print("\n❌ Critical health check failures detected.")
            print("Please fix the issues above and try again.")
            sys.exit(1)
            
    except ImportError:
        print("⚠️  Health check module not available - continuing without validation")
    except Exception as e:
        print(f"⚠️  Health check failed: {e}")
        print("Continuing anyway...")
    
    # Enable High DPI scaling
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)  # type: ignore
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)  # type: ignore
    
    app = QApplication(sys.argv)
    app.setApplicationName("ROS2 Dashboard")
    app.setOrganizationName("ROS2 Monitoring")
    
    # Create and show main window
    from gui.main_window import MainWindow  # type: ignore
    window = MainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
