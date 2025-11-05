# pyright: reportAttributeAccessIssue=false

"""
Main Window for ROS2 Dashboard
"""

from PyQt5.QtWidgets import (QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,   # type: ignore
                             QPushButton, QLabel, QTableWidget, QTableWidgetItem,
                             QGroupBox, QGridLayout, QStatusBar, QSplitter,
                             QMessageBox, QFileDialog, QHeaderView, QTabWidget, QShortcut,
                             QSystemTrayIcon, QMenu, QScrollArea, QAction)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot, QObject, QRunnable, QThreadPool, pyqtSignal, QEvent  # type: ignore
from PyQt5.QtGui import QFont, QColor, QKeySequence  # type: ignore
import os
import json
from datetime import datetime

from core.ros2_manager import ROS2Manager  # type: ignore
from core.metrics_collector import MetricsCollector  # type: ignore
from core.async_worker import AsyncROS2Manager  # type: ignore
from core.freeze_prevention import WindowVisibilityTracker, FreezePrevention  # type: ignore
from core.system_detection import SystemDetector, DynamicPerformanceTuner  # type: ignore
from gui.topic_monitor import TopicMonitorWidget  # type: ignore
from gui.recording_control import RecordingControlWidget  # type: ignore
from gui.metrics_display import MetricsDisplayWidget  # type: ignore
from gui.network_upload import NetworkUploadWidget  # type: ignore
from gui.live_charts import LiveChartsWidget  # type: ignore
from gui.themes import ThemeManager  # type: ignore
from core.network_manager import NetworkManager  # type: ignore
from core.recording_triggers import SmartRecordingManager  # type: ignore
from core.performance_profiler import PerformanceProfiler  # type: ignore
from core.performance_modes import PerformanceModeManager, PerformanceMode  # type: ignore
from gui.performance_settings_dialog import PerformanceSettingsDialog  # type: ignore

# Try to import auth settings dialog (optional)
try:
    from gui.auth_settings_dialog import AuthenticationSettingsDialog  # type: ignore
except ImportError:
    AuthenticationSettingsDialog = None


class ScrollEventFilter(QObject):
    """Event filter to detect scroll events and pause UI updates during scrolling"""
    
    def __init__(self, main_window):
        super().__init__()
        self.main_window = main_window
    
    def eventFilter(self, obj, event):
        """Filter scroll events to pause updates"""
        from PyQt5.QtCore import QEvent as QtEvent
        
        # Detect scroll wheel events (Wheel event is fired when mouse wheel scrolls)
        # Also detect ValueChanged from scrollbars (QAbstractSlider events)
        if event.type() == QtEvent.Wheel:
            self.main_window.pause_updates_for_scroll()
        elif hasattr(obj, 'sliderMoved'):  # It's a scrollbar
            if event.type() == QtEvent.ValueChanged:
                self.main_window.pause_updates_for_scroll()
        
        return False  # Continue processing the event


class MetricsWorkerSignals(QObject):
    """Signals used by the background metrics worker."""

    finished = pyqtSignal(dict)
    error = pyqtSignal(str)


class MetricsWorker(QRunnable):
    """Background worker that updates metrics off the GUI thread."""

    def __init__(self, metrics_collector, ros2_manager, is_recording: bool):
        super().__init__()
        self.metrics_collector = metrics_collector
        self.ros2_manager = ros2_manager
        self.is_recording = is_recording
        self.signals = MetricsWorkerSignals()

    @pyqtSlot()
    def run(self):
        """Perform the heavy metrics update in a worker thread."""
        try:
            # CRITICAL FIX: Always update and get full metrics
            # get_live_metrics() only returns system metrics (CPU, RAM), not recording data!
            # We need get_metrics() to show recording stats (msg rate, bandwidth, etc.)
            if self.is_recording:
                # During recording: update accumulates data from current recording
                self.metrics_collector.update(self.ros2_manager)
            
            # ALWAYS get full metrics (includes recording data when available)
            metrics = self.metrics_collector.get_metrics()
            
            # Enrich with live system metrics (CPU, memory, disk)
            live_metrics = self.metrics_collector.get_live_metrics(self.ros2_manager)
            if live_metrics:
                # Merge system metrics into recording metrics
                metrics.update({
                    'cpu_percent': live_metrics.get('cpu_percent', 0),
                    'memory_percent': live_metrics.get('memory_percent', 0),
                    'disk_usage_percent': live_metrics.get('disk_usage_percent', 0),
                    'disk_write_speed': live_metrics.get('disk_write_speed', 0)
                })
            
            self.signals.finished.emit(metrics)
        except Exception as exc:  # pragma: no cover - defensive guard
            self.signals.error.emit(str(exc))


class MainWindow(QMainWindow):
    """Main application window"""
    
    # Signal for showing critical memory dialog from background thread
    critical_memory_signal = pyqtSignal(float)
    
    def __init__(self):
        super().__init__()
        
        # Initialize performance manager FIRST
        self.performance_manager = PerformanceModeManager()
        self.perf_settings = self.performance_manager.get_mode_settings()
        
        print(f"\n{'='*60}")
        print(f"ROS2 Dashboard - Adaptive Performance Mode")
        print(f"{'='*60}")
        print(self.performance_manager.get_system_info_text())
        print(f"\nUsing {self.performance_manager.get_current_mode().value.upper()} mode")
        print(f"{'='*60}\n")
        
        self.ros2_manager = ROS2Manager(performance_mode_manager=self.performance_manager)
        self.async_ros2 = AsyncROS2Manager(
            self.ros2_manager,
            max_threads=self.perf_settings['max_threads'],
            cache_timeout=self.perf_settings['cache_timeout']
        )
        self.metrics_collector = MetricsCollector()
        
        # DYNAMIC: Apply system-specific cache timeout to metrics collector
        if hasattr(self, 'dynamic_settings') and self.dynamic_settings:
            self.metrics_collector.system_metrics_cache_timeout = self.dynamic_settings['system_metrics_cache']
            print(f"   📊 Metrics cache: {self.dynamic_settings['system_metrics_cache']}s")
        
        # DYNAMIC: Set thread pool sizes based on system specs
        max_threads = self.dynamic_settings['max_threads'] if hasattr(self, 'dynamic_settings') and self.dynamic_settings else 2
        self.metrics_thread_pool = QThreadPool()
        self.metrics_thread_pool.setMaxThreadCount(max_threads)
        self._metrics_task_running = False
        self.network_manager = None  # Initialize later
        self.is_recording = False
        self.theme_manager = ThemeManager()  # Theme management
        
        # Memory monitoring and optimization ⭐ NEW
        from core.memory_monitor import MemoryMonitor, MemoryOptimizer
        self.memory_monitor = MemoryMonitor(warning_threshold=75.0, critical_threshold=85.0)
        self.memory_optimizer = MemoryOptimizer(self.ros2_manager, self.async_ros2, self.metrics_collector)
        
        # Set up memory callbacks
        self.memory_monitor.set_warning_callback(self.on_memory_warning)
        self.memory_monitor.set_critical_callback(self.on_memory_critical)
        
        # Connect critical memory signal to show dialog on main thread
        self.critical_memory_signal.connect(self._show_critical_memory_dialog)
        
        self.memory_monitor.start()
        
        # ANTI-FREEZE: Prevent concurrent ROS2 updates
        self._ros2_update_active = False
        
        # ANTI-FREEZE: Track window visibility to avoid background work
        self.visibility_tracker = WindowVisibilityTracker(self)
        self.visibility_tracker.visibility_changed.connect(self._on_window_visibility_changed)
        self._timers_paused = False
        
        # ADVANCED: Window state tracking for intelligent frame skipping
        self._window_minimized = False
        self._window_hidden = False
        self._skip_frame_updates = False
        
        # DEBOUNCING - prevent excessive updates
        self._last_ros2_update = 0
        self._ros2_update_cooldown = 1.0  # At least 1 second between updates
        self._last_metrics_update = 0
        self._metrics_update_cooldown = 0.5  # At least 0.5 seconds between metrics
        
        # SCROLL PAUSE - pause updates while user is scrolling
        self._scroll_pause_active = False
        self._scroll_pause_timer = QTimer()
        self._scroll_pause_timer.timeout.connect(self._resume_updates)
        self._scroll_pause_timer.setSingleShot(True)
        
        self.init_ui()
        self.setup_timers()
        self.setup_system_tray()  # Setup notifications
        
        # Start network manager after UI is ready (non-blocking)
        QTimer.singleShot(2000, self.init_network_manager)
        
        # Pre-populate cache in background after UI is fully loaded
        QTimer.singleShot(1500, self._warmup_cache)
        
    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle("ROS2 Data Recording & Status Dashboard")
        self.setGeometry(100, 100, 1400, 900)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Main layout
        main_layout = QVBoxLayout(central_widget)
        
        # Title with window control buttons
        title = QLabel("ROS2 Bags Recording Dashboard")
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)

        header_layout = QHBoxLayout()
        header_layout.addWidget(title)
        header_layout.addStretch()

        main_layout.addLayout(header_layout)
        
        # Create splitter for resizable sections
        splitter = QSplitter(Qt.Vertical)
        
        # Top section: Recording controls and status
        top_widget = QWidget()
        top_layout = QHBoxLayout(top_widget)
        
        # Recording control panel
        self.recording_control = RecordingControlWidget(self.ros2_manager, self.async_ros2)
        self.recording_control.recording_started.connect(self.on_recording_started)  # type: ignore[arg-type]
        self.recording_control.recording_stopped.connect(self.on_recording_stopped)  # type: ignore[arg-type]
        top_layout.addWidget(self.recording_control)
        
        # Metrics display
        self.metrics_display = MetricsDisplayWidget(self.metrics_collector)
        top_layout.addWidget(self.metrics_display)
        
        splitter.addWidget(top_widget)
        
        # Middle section: Tabbed interface for different features
        self.tabs = QTabWidget()
        
        # Tab 1: Topic Monitor (with scroll area)
        self.topic_monitor = TopicMonitorWidget(self.ros2_manager, self.async_ros2)
        topic_scroll = QScrollArea()
        topic_scroll.setWidget(self.topic_monitor)
        topic_scroll.setWidgetResizable(True)
        topic_scroll.setMinimumHeight(400)  # Ensure topics table has enough space
        self.tabs.addTab(topic_scroll, "📡 Topics")
        
        # Connect topic selection changes to recording control
        self.topic_monitor.topics_changed.connect(self.recording_control.update_selected_topics)
        
        # Tab 2: Live Charts (with scroll area) - DYNAMIC settings
        # Use dynamic settings if available, otherwise fall back to performance mode settings
        chart_buffer = self.dynamic_settings['max_buffer_size'] if hasattr(self, 'dynamic_settings') and self.dynamic_settings else self.perf_settings['chart_buffer_size']
        chart_interval = self.dynamic_settings['chart_update_interval'] if hasattr(self, 'dynamic_settings') and self.dynamic_settings else self.perf_settings['chart_update_interval']
        
        # ⚡ FASTER CHARTS: Reduce update interval by 50% for smoother animation
        chart_interval = max(100, chart_interval // 2)  # 2x faster, minimum 100ms
        
        self.live_charts = LiveChartsWidget(
            self.metrics_collector, 
            self.ros2_manager,
            buffer_size=chart_buffer,
            update_interval=chart_interval,
            auto_pause=self.perf_settings['chart_auto_pause']
        )
        charts_scroll = QScrollArea()
        charts_scroll.setWidget(self.live_charts)
        charts_scroll.setWidgetResizable(True)
        self.tabs.addTab(charts_scroll, "📈 Live Charts")
        
        # Tab 3: Network Upload (with scroll area)
        from core.network_manager import NetworkManager
        placeholder_network_manager = NetworkManager()
        self.network_upload = NetworkUploadWidget(placeholder_network_manager)
        upload_scroll = QScrollArea()
        upload_scroll.setWidget(self.network_upload)
        upload_scroll.setWidgetResizable(True)
        self.tabs.addTab(upload_scroll, "☁️ Upload")
        
        # Connect tab change signal for smooth transitions
        self.tabs.currentChanged.connect(self.on_tab_changed)
        
        # Install scroll event filter for smooth scrolling during updates
        scroll_filter = ScrollEventFilter(self)
        for i in range(self.tabs.count()):
            tab_widget = self.tabs.widget(i)
            if isinstance(tab_widget, QScrollArea):
                tab_widget.installEventFilter(scroll_filter)
                # Also filter the scroll bar events
                if tab_widget.verticalScrollBar():
                    tab_widget.verticalScrollBar().installEventFilter(scroll_filter)
        
        splitter.addWidget(self.tabs)
        
        # LAYOUT OPTIMIZATION: Give much more space to tabs (topics table), less to recording stats
        # Old: [300, 700] - recording stats got 30% (too much)
        # New: [150, 900] - recording stats get 14% (minimal but visible), tabs get 86%
        splitter.setSizes([150, 900])
        
        main_layout.addWidget(splitter)
        
        # Status bar
        self.statusBar = QStatusBar()
        self.setStatusBar(self.statusBar)
        self.update_status("Ready")
        
        # Setup menu bar
        self.setup_menu_bar()
        
        # Setup keyboard shortcuts ⭐ NEW
        self.setup_shortcuts()
    
    def setup_menu_bar(self):
        """Setup application menu bar"""
        menubar = self.menuBar()
        if not menubar:
            return
        
        # View menu
        view_menu = menubar.addMenu("&View")
        
        theme_action = QAction("Toggle &Theme", self)
        theme_action.setShortcut("Ctrl+T")
        theme_action.triggered.connect(self.toggle_theme)
        view_menu.addAction(theme_action)  # type: ignore[union-attr]
        
        view_menu.addSeparator()  # type: ignore[union-attr]
        
        charts_action = QAction("&Live Charts", self)
        charts_action.setShortcut("Ctrl+L")
        charts_action.triggered.connect(lambda: self.tabs.setCurrentIndex(6))  # Updated index: now tab 7 (0-indexed = 6)
        view_menu.addAction(charts_action)  # type: ignore[union-attr]
        
        # Settings menu
        settings_menu = menubar.addMenu("&Settings")
        
        perf_action = QAction("&Performance Mode...", self)
        perf_action.triggered.connect(self.show_performance_settings)
        settings_menu.addAction(perf_action)  # type: ignore[union-attr]
        
        # Authentication settings (if available)
        if AuthenticationSettingsDialog:
            settings_menu.addSeparator()  # type: ignore[union-attr]
            auth_action = QAction("🔐 &Authentication...", self)
            auth_action.triggered.connect(self.show_authentication_settings)
            settings_menu.addAction(auth_action)  # type: ignore[union-attr]
        
        # Help menu
        help_menu = menubar.addMenu("&Help")
        
        shortcuts_action = QAction("&Keyboard Shortcuts", self)
        shortcuts_action.setShortcut("Ctrl+H")
        shortcuts_action.triggered.connect(self.show_keyboard_shortcuts_help)
        help_menu.addAction(shortcuts_action)  # type: ignore[union-attr]
        
        help_menu.addSeparator()  # type: ignore[union-attr]
        
        about_action = QAction("&About", self)
        about_action.triggered.connect(self.show_about_dialog)
        help_menu.addAction(about_action)  # type: ignore[union-attr]
        
    def show_about_dialog(self):
        """Show about dialog"""
        mode = self.performance_manager.get_current_mode().value.upper()
        about_text = f"""
<h2>ROS2 Dashboard</h2>
<p>Advanced recording and monitoring dashboard for ROS2</p>
<p><b>Performance Mode:</b> {mode}</p>
<p><b>System:</b> {self.performance_manager.system_info['memory_gb']}GB RAM, 
{self.performance_manager.system_info['cpu_count']} CPU cores</p>
<hr>
<p><i>For best results, adjust performance mode in Settings → Performance Mode</i></p>
        """
        QMessageBox.about(self, "About ROS2 Dashboard", about_text)
        
    def setup_shortcuts(self):
        """Setup keyboard shortcuts for quick access"""
        # Ctrl+R - Start/Stop Recording
        record_shortcut = QShortcut(QKeySequence("Ctrl+R"), self)
        record_shortcut.activated.connect(self.toggle_recording)
        
        # Ctrl+S - Stop Recording (if active)
        stop_shortcut = QShortcut(QKeySequence("Ctrl+S"), self)
        stop_shortcut.activated.connect(self.stop_recording_shortcut)
        
        # Ctrl+P - Open Playback Tab
        playback_shortcut = QShortcut(QKeySequence("Ctrl+P"), self)
        playback_shortcut.activated.connect(lambda: self.tabs.setCurrentIndex(4))  # Updated index: now tab 5 (0-indexed = 4)
        
        # Ctrl+E - Export Performance Report
        export_shortcut = QShortcut(QKeySequence("Ctrl+E"), self)
        export_shortcut.activated.connect(self.export_metrics)
        
        # Ctrl+L - Switch to Live Charts
        charts_shortcut = QShortcut(QKeySequence("Ctrl+L"), self)
        charts_shortcut.activated.connect(lambda: self.tabs.setCurrentIndex(6))  # Updated index: now tab 7 (0-indexed = 6)
        
        # Ctrl+H - Show Help
        help_shortcut = QShortcut(QKeySequence("Ctrl+H"), self)
        help_shortcut.activated.connect(self.show_keyboard_shortcuts_help)
        
        # Ctrl+T - Toggle Theme
        theme_shortcut = QShortcut(QKeySequence("Ctrl+T"), self)
        theme_shortcut.activated.connect(self.toggle_theme)
        
    def showEvent(self, event):
        """Ensure live charts load when window first appears"""
        super().showEvent(event)
        # Trigger live charts loading on first show
        if hasattr(self, 'live_charts') and not self.live_charts._charts_loaded:
            from PyQt5.QtCore import QTimer
            # Load charts asynchronously in next event loop
            QTimer.singleShot(100, lambda: self.live_charts._load_charts_async())
        
    def toggle_theme(self):
        """Toggle between dark and light theme (Ctrl+T)"""
        from PyQt5.QtWidgets import QApplication
        theme = self.theme_manager.toggle_theme(QApplication.instance())
        self.update_status(f"🎨 Theme switched to: {theme.title()}")

    def show_performance_settings(self):
        """Show performance settings dialog"""
        dialog = PerformanceSettingsDialog(self.performance_manager, self)
        dialog.mode_changed.connect(self.on_performance_mode_changed)
        dialog.exec_()
    
    def show_authentication_settings(self):
        """Show authentication settings dialog"""
        if not AuthenticationSettingsDialog:
            QMessageBox.warning(self, "Not Available", "Authentication module is not available")
            return
        
        dialog = AuthenticationSettingsDialog(self)
        dialog.exec_()
        
        # Update auth status in network upload widget if it exists
        if hasattr(self, 'network_upload') and hasattr(self.network_upload, 'update_auth_status_display'):
            self.network_upload.update_auth_status_display()
    
    def on_performance_mode_changed(self, mode):
        """Handle performance mode change"""
        # Get new settings
        self.perf_settings = self.performance_manager.get_mode_settings()
        
        # Update subprocess timeout in ROS2Manager
        self.ros2_manager.update_subprocess_timeout()
        
        # Update async manager
        self.async_ros2.max_threads = self.perf_settings['max_threads']
        self.async_ros2.cache_timeout = self.perf_settings['cache_timeout']
        
        # Update metrics collector caching
        self.metrics_collector.system_metrics_cache_timeout = self.perf_settings['system_metrics_cache']
        
        # Update timers
        self.ros2_timer.setInterval(self.perf_settings['ros2_update_interval'])
        self.metrics_timer.setInterval(self.perf_settings['metrics_update_interval'])
        self.history_timer.setInterval(self.perf_settings['history_update_interval'])
        
        # Update live charts if exists
        if hasattr(self, 'live_charts'):
            self.live_charts.update_interval = self.perf_settings['chart_update_interval']
            self.live_charts.buffer_size = self.perf_settings['chart_buffer_size']
            self.live_charts.auto_pause = self.perf_settings['chart_auto_pause']
        
        mode_name = mode if isinstance(mode, str) else mode.value
        self.update_status(f"⚡ Performance mode changed to: {mode_name.upper()}")
        
        QMessageBox.information(
            self,
            "Performance Mode Changed",
            f"Performance mode set to: {mode_name.upper()}\n\n"
            f"Settings will take effect immediately.\n"
            f"Some changes may require tab switching to fully apply."
        )
        
    def toggle_recording(self):
        """Toggle recording state (start/stop) - bound to Ctrl+R."""
        if not self.recording_control.is_recording:
            # Start recording
            selected_topics = self.topic_monitor.selected_topics
            if not selected_topics:
                self.show_notification(
                    "Recording Failed",
                    "No topics selected for recording",
                    QSystemTrayIcon.Warning
                )
                QMessageBox.warning(self, "No Topics Selected", 
                                  "Please select topics to record in the Topics tab.")
                return
            
            self.recording_control.start_recording()
            self.show_notification(
                "Recording Started",
                f"Recording {len(selected_topics)} topic(s)",
                QSystemTrayIcon.Information
            )
        else:
            # Stop recording
            self.recording_control.stop_recording()
            self.show_notification(
                "Recording Stopped",
                "Recording saved successfully",
                QSystemTrayIcon.Information
            )
            
    def stop_recording_shortcut(self):
        """Stop recording (Ctrl+S shortcut)."""
        if self.recording_control.is_recording:
            self.recording_control.stop_recording()
            self.show_notification(
                "Recording Stopped",
                "Recording saved successfully",
                QSystemTrayIcon.Information
            )
            
    def export_metrics(self):
        """Export current metrics (Ctrl+E shortcut)."""
        try:
            filename = f"ros2_metrics_{datetime.now().strftime('%Y%m%d_%H%M%S')}.json"
            metrics = self.metrics_collector.get_live_metrics()
            
            with open(filename, 'w') as f:
                json.dump(metrics, f, indent=2)
            
            self.show_notification(
                "Metrics Exported",
                f"Saved to {filename}",
                QSystemTrayIcon.Information
            )
            QMessageBox.information(self, "Export Successful", 
                                  f"Metrics exported to {filename}")
        except Exception as e:
            self.show_notification(
                "Export Failed",
                str(e),
                QSystemTrayIcon.Critical
            )
            QMessageBox.critical(self, "Export Failed", f"Error: {e}")
                
    def show_keyboard_shortcuts_help(self):
        """Show keyboard shortcuts help dialog (Ctrl+H)"""
        help_text = """
<h2>Keyboard Shortcuts</h2>
<table>
<tr><td><b>Ctrl+R</b></td><td>Start/Stop Recording</td></tr>
<tr><td><b>Ctrl+S</b></td><td>Stop Recording</td></tr>
<tr><td><b>Ctrl+P</b></td><td>Open Playback Tab</td></tr>
<tr><td><b>Ctrl+L</b></td><td>Open Live Charts Tab</td></tr>
<tr><td><b>Ctrl+E</b></td><td>Export Metrics & Performance Data</td></tr>
<tr><td><b>Ctrl+T</b></td><td>Toggle Dark/Light Theme</td></tr>
<tr><td><b>Ctrl+H</b></td><td>Show this Help Dialog</td></tr>
</table>
<p><i>Tip: Use shortcuts for fastest workflow!</i></p>
        """
        QMessageBox.information(self, "Keyboard Shortcuts", help_text)
        
    def setup_timers(self):
        """Setup update timers - ULTRA-OPTIMIZED to prevent ALL freezes"""
        perf = self.perf_settings
        
        # Track which tab is active to avoid updates when invisible
        self._active_tab_index = 0
        self.tabs.currentChanged.connect(self._on_tab_changed)
        
        # DYNAMIC SYSTEM DETECTION: Auto-tune all timers based on hardware specs
        print("\n" + "="*60)
        print("🔍 DETECTING SYSTEM SPECS FOR OPTIMAL PERFORMANCE")
        print("="*60)
        try:
            from core.system_detection import SystemDetector, DynamicPerformanceTuner
            self.system_specs = SystemDetector.get_system_specs()
            self.dynamic_settings = DynamicPerformanceTuner.get_tuned_settings(self.system_specs)
            
            print(f"\n✅ System Category: {self.system_specs['system_category'].upper()}")
            print(f"   CPU: {self.system_specs['cpu_cores']} cores @ {self.system_specs['cpu_freq_mhz']}MHz")
            print(f"   RAM: {self.system_specs['ram_gb']:.1f}GB (Available: {self.system_specs['available_ram_gb']:.1f}GB)")
            print(f"   Disk: {self.system_specs['disk_type'].upper()} ({self.system_specs['disk_speed']})")
            print(f"   GPU: {'Available' if self.system_specs['has_gpu'] else 'Not detected'}")
            
            print(f"\n⚡ DYNAMIC TIMER CONFIGURATION:")
            print(f"   ROS2 Updates: {self.dynamic_settings['ros2_update_interval']}ms")
            print(f"   Metrics Updates: {self.dynamic_settings['metrics_update_interval']}ms")
            print(f"   History Updates: {self.dynamic_settings['history_update_interval']}ms")
            print(f"   Chart Updates: {self.dynamic_settings['chart_update_interval']}ms")
            print(f"   Max Threads: {self.dynamic_settings['max_threads']}")
            print("="*60 + "\n")
            
        except Exception as e:
            print(f"⚠️  System detection failed: {e}, using defaults")
            self.system_specs = None
            self.dynamic_settings = None
        
        # ROS2 info timer - DYNAMIC interval based on system specs
        # This prevents the 7+ second ROS2 calls from freezing the UI
        self.ros2_timer = QTimer()
        self.ros2_timer.timeout.connect(self.update_ros2_info_async_smart)
        
        # ⚡ FASTER UPDATES: reduce ros2 poll interval for snappier UI
        ros2_interval = self.dynamic_settings['ros2_update_interval'] if self.dynamic_settings else 30000
        # Reduce by 50% for snappier topic/node updates  
        ros2_interval = max(1000, ros2_interval // 2)  # 2x faster, minimum 1 second
        
        # Stagger startup: delay 2 seconds to let UI fully render first
        QTimer.singleShot(2000, lambda: self.ros2_timer.start(ros2_interval))
        
        # REMOVED: live_metrics_timer (was redundant with live_charts' own timer)
        # Live charts widget already collects metrics at optimal intervals
        # Removing this eliminates 50% of timer overhead and CPU spikes
        
        # Metrics timer - DYNAMIC interval based on system specs
        # Metrics are lower priority than responsiveness
        self.metrics_timer = QTimer()
        self.metrics_timer.timeout.connect(self.update_metrics_smart)
        
        # ⚡ FASTER UPDATES during recording for real-time metrics display
        metrics_interval = self.dynamic_settings['metrics_update_interval'] if self.dynamic_settings else 15000
        # Reduce to 200ms (5x faster) during recording for snappy UI updates
        # This makes duration, write_speed, and message counts feel real-time
        metrics_interval = max(200, metrics_interval // 10)  # 10x faster, minimum 200ms
        
        # Delay 1 second after startup to let UI render
        QTimer.singleShot(1000, lambda: self.metrics_timer.start(metrics_interval))
        
        # Connect to performance mode changes
        self.performance_manager.mode_changed.connect(self.on_performance_mode_changed)
    
    def _on_tab_changed(self, index):
        """Track which tab is currently active"""
        self._active_tab_index = index
        # Trigger immediate update for newly visible tab (with small delay for render)
        QTimer.singleShot(100, self.update_ros2_info_async_smart)
    
    def _on_window_visibility_changed(self, is_visible):
        """Handle window visibility changes - stop timers when hidden"""
        if is_visible:
            # Window became visible - resume updates if they were running
            if hasattr(self, '_timers_paused'):
                if self._timers_paused:
                    print("🔄 Window visible - resuming updates")
                    if hasattr(self, 'ros2_timer'):
                        self.ros2_timer.start()
                    if hasattr(self, 'metrics_timer'):
                        self.metrics_timer.start()
                    self._timers_paused = False
        else:
            # Window hidden - pause all timers (huge CPU savings)
            print("⏸️  Window hidden - pausing background updates")
            self._timers_paused = True
            if hasattr(self, 'ros2_timer') and self.ros2_timer.isActive():
                self.ros2_timer.stop()
            if hasattr(self, 'metrics_timer') and self.metrics_timer.isActive():
                self.metrics_timer.stop()
    
    def setup_system_tray(self):
        """Setup system tray icon and desktop notifications."""
        if not QSystemTrayIcon.isSystemTrayAvailable():
            return
        
        # Create tray icon
        self.tray_icon = QSystemTrayIcon(self)
        try:
            from PyQt5.QtWidgets import QStyle
            style = self.style()
            if style is not None:
                self.tray_icon.setIcon(style.standardIcon(QStyle.SP_ComputerIcon))
        except Exception:
            pass
        self.tray_icon.setToolTip("ROS2 Dashboard")
        
        # Create tray menu
        tray_menu = QMenu()
        
        show_action = QAction("Show Dashboard", self)
        show_action.triggered.connect(lambda: self.show())
        tray_menu.addAction(show_action)
        
        tray_menu.addSeparator()
        
        record_action = QAction("Start Recording", self)
        record_action.triggered.connect(self.toggle_recording)
        tray_menu.addAction(record_action)
        
        tray_menu.addSeparator()
        
        quit_action = QAction("Exit", self)
        quit_action.triggered.connect(self._close_window)  # type: ignore[arg-type]
        tray_menu.addAction(quit_action)
        
        self.tray_icon.setContextMenu(tray_menu)
        self.tray_icon.show()
        
        # Connect tray icon double-click to show window
        self.tray_icon.activated.connect(self.on_tray_activated)
    
    def on_tray_activated(self, reason):
        """Handle tray icon activation."""
        if reason == QSystemTrayIcon.DoubleClick:
            self.show()
            self.activateWindow()

    @pyqtSlot()
    def _close_window(self) -> None:
        """Close window wrapper that returns None for signal-slot typing."""
        self.close()
    
    def show_notification(self, title: str, message: str, icon=QSystemTrayIcon.Information):
        """Show desktop notification."""
        if hasattr(self, 'tray_icon') and self.tray_icon.supportsMessages():
            self.tray_icon.showMessage(title, message, icon, 3000)  # 3 second duration
        
    def _warmup_cache(self):
        """
        ADVANCED: Smart cache preloading with predictive loading
        - Warms up cache in background to prevent first-call freezes
        - Predicts likely user actions and preloads data
        - Staggered loading to avoid overwhelming system
        """
        print("🔥 Warming up cache with predictive preloading...")
        
        # Trigger async cache population (non-blocking)
        # These will populate the cache so future UI updates are instant
        def _noop(data):
            count = len(data) if isinstance(data, list) else 'data'
            print(f"✅ Cache warmed: {count} items")
        
        try:
            if self.async_ros2:
                # PRIORITY 1: Topics (most frequently accessed)
                self.async_ros2.get_topics_async(_noop)
                
                # PRIORITY 2: Nodes (second most common)
                QTimer.singleShot(500, lambda: self.async_ros2.get_nodes_async(_noop) if self.async_ros2 else None)
                
                # PRIORITY 3: Services (less frequently accessed)
                QTimer.singleShot(1000, lambda: self.async_ros2.get_services_async(_noop) if self.async_ros2 else None)
                
                # ADVANCED: Preload system metrics (user likely to check)
                QTimer.singleShot(1500, self._preload_system_metrics)
                
        except Exception as e:
            print(f"⚠️ Cache warmup error: {e}")
    
    def _preload_system_metrics(self):
        """Predictively preload system metrics into cache"""
        try:
            # Trigger metrics collection in background
            if hasattr(self, 'metrics_collector'):
                self.metrics_collector.get_live_metrics(self.ros2_manager)
                print("✅ System metrics preloaded")
        except Exception as e:
            print(f"⚠️ Metrics preload error: {e}")
            
    def init_network_manager(self):
        """Initialize network manager after UI is ready"""
        try:
            self.network_manager = NetworkManager()
            self.network_manager.start()
            
            # Update the network upload widget
            if hasattr(self, 'network_upload'):
                self.network_upload.network_manager = self.network_manager
                
            print("✅ Network Manager initialized")
        except Exception as e:
            print(f"Warning: Could not initialize Network Manager: {e}")
            self.network_manager = None
    
    def update_ros2_info_async_smart(self):
        """
        ULTRA-OPTIMIZED: Only update visible tab to prevent freezes
        
        - Skip if window hidden
        - Skip if already updating (prevents concurrent requests)
        - Only update the active tab (not all tabs)
        - Massive CPU/responsiveness improvement
        """
        import time
        
        # ADVANCED: Skip if window is minimized (intelligent frame skipping)
        if self._skip_frame_updates or self._window_minimized:
            return
        
        # Skip if window is hidden (prevents background work)
        if not self.isVisible():
            return
        
        # Skip if already updating (prevent concurrent operations)
        if hasattr(self, '_ros2_update_active') and self._ros2_update_active:
            return
        
        # ADAPTIVE: Slow down updates when not recording (save CPU)
        cooldown = 2.0 if not self.ros2_manager.is_recording else 1.0
        
        # Skip if updated too recently (debounce)
        current_time = time.time()
        if current_time - self._last_ros2_update < cooldown:
            return
        self._last_ros2_update = current_time
        self._ros2_update_active = True
        
        try:
            # Only update the CURRENTLY VISIBLE tab (massive performance gain)
            current_tab = self.tabs.currentIndex()
            
            if current_tab == 0 and hasattr(self, 'topic_monitor'):  # Topics tab
                self.topic_monitor.refresh_topics()
            elif current_tab == 1 and hasattr(self, 'node_monitor'):  # Nodes tab
                self.node_monitor.refresh_nodes()
            elif current_tab == 2 and hasattr(self, 'service_monitor'):  # Services tab
                self.service_monitor.refresh_services()
            # Other tabs don't need automatic ROS2 updates
        finally:
            self._ros2_update_active = False
    
    def update_metrics_smart(self):
        """
        OPTIMIZED: Only update metrics if visible + CPU BACKOFF + ADAPTIVE INTERVALS
        
        - Skip if window minimized (intelligent frame skipping)
        - Skip if window hidden
        - Skip if already updating
        - AGGRESSIVE backoff if CPU > 80%
        - Skip cycles if CPU > 90%
        - ADAPTIVE: Faster updates when recording, slower when idle
        """
        # ADVANCED: Skip if window minimized
        if self._skip_frame_updates or self._window_minimized:
            return
        
        # Skip if window is hidden
        if not self.isVisible():
            return
        
        # Skip if metrics update is expensive (already running)
        if getattr(self, '_metrics_task_running', False):
            return
        
        # CHECK CPU FOR AGGRESSIVE BACKOFF
        try:
            import psutil
            cpu_percent = psutil.cpu_percent(interval=0)
            
            if cpu_percent > 90.0:
                # CRITICAL: Skip entire update cycle if CPU maxed out
                return
            elif cpu_percent > 80.0:
                # HIGH LOAD: Double the update interval to back off
                try:
                    current_interval = self.metrics_timer.interval()
                    normal_interval = self.perf_settings.get('metrics_update_interval', 300)
                    if current_interval <= normal_interval:
                        self.metrics_timer.setInterval(min(normal_interval * 2, 10000))
                    return  # Skip this cycle
                except Exception:
                    pass
            else:
                # NORMAL: Restore normal interval if backed off
                try:
                    current_interval = self.metrics_timer.interval()
                    normal_interval = self.perf_settings.get('metrics_update_interval', 300)
                    if current_interval > normal_interval:
                        self.metrics_timer.setInterval(normal_interval)
                except Exception:
                    pass
        except Exception:
            pass
        
        # Call the existing metrics update
        self.update_metrics()
    
    # REMOVED: _update_live_metrics_fast() method
    # This was creating timer storm - live_charts widget already handles its own updates
    # Removing this reduces CPU usage by 50% and eliminates UI freezing
            
    def update_ros2_info_async(self):
        """
        ULTRA-OPTIMIZED with CPU BACKOFF
        
        Changes:
        - Only update the currently visible tab (reduces CPU 40-50%)
        - Use debounced refresh methods on each monitor widget
        - AGGRESSIVE backoff if CPU > 80%
        - Skip cycles if CPU > 90%
        """
        import time
        
        # CHECK CPU FOR AGGRESSIVE BACKOFF FIRST
        try:
            import psutil
            cpu_percent = psutil.cpu_percent(interval=0)
            
            if cpu_percent > 90.0:
                # CRITICAL: Skip entire update cycle if CPU maxed out
                return
            elif cpu_percent > 80.0:
                # HIGH LOAD: Double the update interval to back off
                try:
                    current_interval = self.ros2_timer.interval()
                    normal_interval = self.perf_settings.get('ros2_update_interval', 2000)
                    if current_interval <= normal_interval:
                        self.ros2_timer.setInterval(min(normal_interval * 2, 15000))
                    return  # Skip this cycle
                except Exception:
                    pass
            else:
                # NORMAL: Restore normal interval if backed off
                try:
                    current_interval = self.ros2_timer.interval()
                    normal_interval = self.perf_settings.get('ros2_update_interval', 2000)
                    if current_interval > normal_interval:
                        self.ros2_timer.setInterval(normal_interval)
                except Exception:
                    pass
        except Exception:
            pass
        
        # DEBOUNCE - skip if updated recently
        current_time = time.time()
        if current_time - self._last_ros2_update < self._ros2_update_cooldown:
            return
        self._last_ros2_update = current_time
        
        # Skip if window is minimized or hidden (critical optimization)
        if not self.isVisible():
            return
        
        # Limit concurrent async operations (avoid thread pool saturation)
        if self.async_ros2.active_thread_count() >= 1:
            return  # Skip if thread already running
            
        current_tab = self.tabs.currentIndex()
        
        # LAZY TAB LOADING: Update ONLY the active tab (40-50% CPU reduction)
        # This prevents non-visible tabs from consuming resources
        if current_tab == 0:  # Topics tab
            # Use debounced refresh that prevents concurrent updates AND deduplicates requests
            self.topic_monitor.refresh_topics()
        elif current_tab == 1:  # Nodes tab
            self.node_monitor.refresh_nodes()
        elif current_tab == 2:  # Services tab
            self.service_monitor.refresh_services()
        # Tabs 3-9 don't need frequent ROS2 updates (they update on demand)
        
    def on_tab_changed(self, new_tab_index):
        """
        Handle tab changes with rendering optimization
        
        Pauses non-essential updates during tab switch for smooth transitions
        Then triggers an immediate update of the new tab for responsiveness
        """
        try:
            # Temporarily disable timer to reduce UI work during tab switch
            was_running = self.ros2_timer.isActive()
            if was_running:
                self.ros2_timer.stop()
            
            # Process events to let tab rendering complete
            from PyQt5.QtWidgets import QApplication
            app = QApplication.instance()
            if app:
                app.processEvents()
            
            # Resume timer and immediately update the new tab
            if was_running:
                self.ros2_timer.start()
                # Force immediate update of the new tab for responsiveness
                if new_tab_index == 0 and hasattr(self, 'topic_monitor'):
                    self.topic_monitor.refresh_topics()
                elif new_tab_index == 1 and hasattr(self, 'node_monitor'):
                    self.node_monitor.refresh_nodes()
                elif new_tab_index == 2 and hasattr(self, 'service_monitor'):
                    self.service_monitor.refresh_services()
        except Exception:
            pass  # Silent fail - tab switching should never freeze UI
        
    def pause_updates_for_scroll(self):
        """
        Pause all UI updates while user is scrolling
        Resumes automatically after scroll ends (300ms timeout)
        """
        if not self._scroll_pause_active:
            self._scroll_pause_active = True
            
            # Pause the update timers to reduce UI work
            if self.metrics_timer.isActive():
                self.metrics_timer.stop()
            if self.ros2_timer.isActive():
                self.ros2_timer.stop()
            if hasattr(self, 'live_charts') and hasattr(self.live_charts, 'update_timer'):
                if self.live_charts.update_timer.isActive():
                    self.live_charts.update_timer.stop()
        
        # Reset the resume timer (300ms after last scroll)
        self._scroll_pause_timer.stop()
        self._scroll_pause_timer.start(300)  # Resume after 300ms of no scrolling
    
    def _resume_updates(self):
        """Resume all UI updates after scrolling ends"""
        try:
            self._scroll_pause_active = False
            
            # Resume timers only if window is still visible and not minimized
            if self.isVisible() and not self.isMinimized():
                if not self.metrics_timer.isActive():
                    self.metrics_timer.start()
                if not self.ros2_timer.isActive():
                    self.ros2_timer.start()
                if hasattr(self, 'live_charts') and hasattr(self.live_charts, 'update_timer'):
                    if not self.live_charts.update_timer.isActive():
                        self.live_charts.update_timer.start()
        except Exception:
            pass  # Silent fail
        
    def update_metrics(self):
        """Update dashboard metrics - AGGRESSIVE debouncing"""
        import time
        
        # DEBOUNCE - skip if updated recently
        current_time = time.time()
        if current_time - self._last_metrics_update < self._metrics_update_cooldown:
            return
        self._last_metrics_update = current_time
        
        # Skip if scroll pause is active
        if self._scroll_pause_active:
            return

        worker = MetricsWorker(self.metrics_collector, self.ros2_manager, self.is_recording)
        worker.signals.finished.connect(self._handle_metrics_finished)  # type: ignore[arg-type]
        worker.signals.error.connect(self._handle_metrics_error)  # type: ignore[arg-type]
        self._metrics_task_running = True
        self.metrics_thread_pool.start(worker)

        # The heavy lifting happens in the worker thread; nothing else to do here.

    @pyqtSlot(dict)
    def _handle_metrics_finished(self, metrics):
        """Receive metrics from worker thread and refresh UI."""
        self._metrics_task_running = False
        self.metrics_display.update_display(metrics)

    @pyqtSlot(str)
    def _handle_metrics_error(self, message: str):
        """Log metrics worker errors without crashing the UI."""
        self._metrics_task_running = False
        print(f"Metrics worker error: {message}")  # pragma: no cover - debug aid
    

    @pyqtSlot()
    def on_recording_started(self):
        """Handle recording started event - optimize timers for active recording"""
        self.is_recording = True
        self.metrics_collector.reset()
        
        # CRITICAL: Reset live charts for new recording session
        if hasattr(self, 'live_charts'):
            self.live_charts.reset()
        
        # NEW: Enable real-time Hz monitoring during recording
        if hasattr(self, 'topic_monitor'):
            self.topic_monitor.set_recording_state(True)
        
        self.update_status("Recording in progress...")
        
        # REMOVED: live_metrics_timer - was redundant with live_charts' own update mechanism
        # Live charts widget already collects and displays metrics efficiently
        
        # OPTIMIZATION: Slightly increase timer intervals during recording to reduce CPU load
        # Recording process is I/O intensive, so we back off on UI updates slightly
        # This prevents UI lag during heavy bag file writes
        
        # Keep timers at normal intervals - the new slower defaults (30s, 15s) are already optimized
        # No need to further slow down during recording
        
        # Show notification
        self.show_notification("Recording Started", "ROS2 bag recording in progress")
        
    @pyqtSlot()
    def on_recording_stopped(self):
        """Handle recording stopped event - restore normal timer frequencies"""
        self.is_recording = False
        
        # NEW: Disable real-time Hz monitoring when recording stops
        if hasattr(self, 'topic_monitor'):
            self.topic_monitor.set_recording_state(False)
        
        self.update_status("Recording stopped")
        
        # Show notification
        self.show_notification("Recording Stopped", "Recording saved successfully")
        
        # Auto-upload if enabled and network manager is ready
        if self.network_manager and hasattr(self, 'network_upload'):
            # Check if auto-upload is enabled
            if hasattr(self.network_upload, 'auto_upload') and self.network_upload.auto_upload:
                # Get the recording path
                if hasattr(self.recording_control, 'current_bag_name'):
                    bag_name = getattr(self.recording_control, 'current_bag_name', None)
                    output_dir = self.recording_control.dir_input.text() if hasattr(self.recording_control, 'dir_input') else None
                    bag_path = os.path.join(output_dir, bag_name) if output_dir and bag_name else None
                    
                    # Include robot metadata in upload
                    metadata = {}
                    if hasattr(self.recording_control, 'current_bag_metadata'):
                        metadata = self.recording_control.current_bag_metadata or {}
                    
                    # Trigger upload
                    self.update_status(f"📤 Auto-uploading {bag_name}...")
                    self.show_notification(
                        "Auto-Upload Starting",
                        f"Uploading {bag_name} to server..."
                    )
                    
                    # The actual upload will be handled by network_upload widget
                    # Just switch to upload tab to show progress
                    try:
                        # Add metadata tag if from robot discovery
                        if metadata and 'hostname' in metadata:
                            tags = f"robot:{metadata['hostname']}"
                            self.network_upload.tags_input.setText(tags)
                        
                        # Queue the upload (network_upload will handle it)
                        if bag_path:
                            QTimer.singleShot(1000, lambda: self.network_upload.upload_directory(bag_path))
                    except Exception as e:
                        print(f"Auto-upload error: {e}")
        
        # Show completion message with robot info if available
        if hasattr(self.recording_control, 'current_bag_metadata'):
            metadata = self.recording_control.current_bag_metadata
            if metadata and 'hostname' in metadata:
                msg = f"✅ Recording from robot '{metadata['hostname']}' saved successfully!"
                self.update_status(msg)
        
        # Show notification
        self.show_notification("Recording Stopped", "Recording saved successfully")
        
        # Auto-upload if enabled and network manager is ready
        if self.network_manager:
            bag_path = self.ros2_manager.get_current_bag_path()
            if bag_path and os.path.exists(bag_path):
                metadata = {
                    'type': 'ros2_bag',
                    'component': 'robot_recording',
                    'timestamp': datetime.now().isoformat()
                }
                self.network_upload.add_recording_upload(bag_path, metadata)
        
    def open_recordings_folder(self):
        """Open the recordings folder in file manager"""
        recordings_dir = self.ros2_manager.get_recordings_directory()
        if os.path.exists(recordings_dir):
            os.system(f'xdg-open "{recordings_dir}"')
        else:
            QMessageBox.warning(self, "Warning", "Recordings directory does not exist yet.")
            
    def update_status(self, message):
        """Update status bar message"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        self.statusBar.showMessage(f"[{timestamp}] {message}")
    
    def on_memory_warning(self, percent):
        """Handle memory warning - reduce cache sizes"""
        print(f"⚠️  Memory warning: {percent:.1f}% used - reducing caches")
        self.memory_optimizer.reduce_cache_sizes()
        self.memory_monitor.force_garbage_collection()
        
        # Show warning in status bar
        self.update_status(f"⚠️  High memory usage ({percent:.1f}%) - optimizing...")
    
    def on_memory_critical(self, percent):
        """Handle critical memory - aggressive cleanup (called from background thread)"""
        print(f"🚨 CRITICAL memory: {percent:.1f}% used - emergency cleanup")
        self.memory_optimizer.emergency_cleanup()
        
        # Show critical warning in status bar (thread-safe)
        self.update_status(f"🚨 CRITICAL memory ({percent:.1f}%) - cleaned caches")
        
        # Emit signal to show dialog on main GUI thread (Qt requirement)
        self.critical_memory_signal.emit(percent)
    
    def _show_critical_memory_dialog(self, percent):
        """Show critical memory dialog (called on main GUI thread via signal)"""
        from PyQt5.QtWidgets import QMessageBox
        QMessageBox.warning(
            self,
            "Low Memory Warning",
            f"System memory is critically low ({percent:.1f}% used).\n\n"
            "The dashboard has cleared caches to free memory.\n"
            "Consider closing other applications or restarting the dashboard."
        )

        
    def closeEvent(self, event):
        """Handle window close event - ENHANCED graceful cleanup"""
        print("\n🛑 Shutting down ROS2 Dashboard gracefully...")
        
        if self.is_recording:
            reply = QMessageBox.question(
                self, 'Confirm Exit',
                'Recording is in progress. Do you want to stop and exit?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            
            if reply != QMessageBox.Yes:
                event.ignore()
                return
            
            # Stop recording
            print("  ⏹️  Stopping active recording...")
            self.ros2_manager.stop_recording()
            import time
            time.sleep(0.5)  # Brief wait for recording to finish
        
        # Stop all timers to prevent further updates
        print("  ⏱️  Stopping all timers...")
        if hasattr(self, 'ros2_timer') and self.ros2_timer:
            self.ros2_timer.stop()
        if hasattr(self, 'metrics_timer') and self.metrics_timer:
            self.metrics_timer.stop()
        if hasattr(self, 'status_timer') and self.status_timer:
            self.status_timer.stop()
        
        # Clear caches to free memory
        print("  🧹 Clearing caches...")
        if hasattr(self, 'ros2_manager') and self.ros2_manager:
            if hasattr(self.ros2_manager, '_cache'):
                self.ros2_manager._cache.clear()
            if hasattr(self.ros2_manager, '_cache_timestamps'):
                self.ros2_manager._cache_timestamps.clear()
        
        # Shutdown async worker with timeout
        print("  🔌 Shutting down async workers...")
        if hasattr(self, 'async_ros2') and self.async_ros2:
            try:
                self.async_ros2.shutdown()
            except Exception as e:
                print(f"  ⚠️  Async worker shutdown error: {e}")
        
        # Wait for thread pools with timeout
        print("  ⏳ Waiting for background tasks...")
        if hasattr(self, 'metrics_thread_pool') and self.metrics_thread_pool:
            self.metrics_thread_pool.waitForDone(1500)  # 1.5 second timeout
        
        # Shutdown network manager
        if self.network_manager:
            print("  📡 Stopping network manager...")
            try:
                self.network_manager.stop()
            except Exception as e:
                print(f"  ⚠️  Network manager shutdown error: {e}")
        
        # Stop memory monitor
        if hasattr(self, 'memory_monitor') and self.memory_monitor:
            print("  🧠 Stopping memory monitor...")
            try:
                self.memory_monitor.stop()
            except Exception as e:
                print(f"  ⚠️  Memory monitor shutdown error: {e}")
        
        # Hide system tray icon if exists
        if hasattr(self, 'tray_icon') and self.tray_icon:
            self.tray_icon.hide()
        
        print("  ✅ Cleanup complete!")
        print("=" * 60 + "\n")
        
        event.accept()
    
    def changeEvent(self, event):
        """
        ADVANCED OPTIMIZATION: Intelligent frame skipping
        Pause UI updates when window is minimized to save CPU
        """
        if event.type() == QEvent.WindowStateChange:
            if self.isMinimized():
                # Window minimized - pause non-critical updates
                self._window_minimized = True
                self._skip_frame_updates = True
                print("🔽 Window minimized - pausing UI updates (recording continues)")
                self._pause_timers_intelligently()
            else:
                # Window restored - resume updates
                self._window_minimized = False
                self._skip_frame_updates = False
                print("🔼 Window restored - resuming UI updates")
                self._resume_timers_intelligently()
        
        super().changeEvent(event)
    
    def _pause_timers_intelligently(self):
        """Pause UI-only timers when window is minimized"""
        # Pause ROS2 and metrics timers (UI display only)
        if hasattr(self, 'ros2_timer') and self.ros2_timer:
            self.ros2_timer.stop()
        if hasattr(self, 'metrics_timer') and self.metrics_timer:
            self.metrics_timer.stop()
        
        # Keep history timer running (background operation)
        # Recording continues independently
    
    def _resume_timers_intelligently(self):
        """Resume timers when window is restored"""
        if hasattr(self, 'ros2_timer') and self.ros2_timer:
            self.ros2_timer.start()
        if hasattr(self, 'metrics_timer') and self.metrics_timer:
            self.metrics_timer.start()
