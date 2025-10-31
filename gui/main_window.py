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
from gui.topic_monitor import TopicMonitorWidget  # type: ignore
from gui.recording_control import RecordingControlWidget  # type: ignore
from gui.metrics_display import MetricsDisplayWidget  # type: ignore
from gui.node_monitor import NodeMonitorWidget  # type: ignore
from gui.service_monitor import ServiceMonitorWidget  # type: ignore
from gui.topic_echo import TopicEchoWidget  # type: ignore
from gui.advanced_stats import AdvancedStatsWidget  # type: ignore
from gui.network_upload import NetworkUploadWidget  # type: ignore
from gui.live_charts import LiveChartsWidget  # type: ignore
from gui.themes import ThemeManager  # type: ignore
from gui.recording_templates import RecordingTemplatesWidget  # type: ignore
from gui.network_robots import NetworkRobotsWidget  # type: ignore
from core.network_manager import NetworkManager  # type: ignore
from core.recording_triggers import SmartRecordingManager  # type: ignore
from core.performance_profiler import PerformanceProfiler  # type: ignore
from core.performance_modes import PerformanceModeManager, PerformanceMode  # type: ignore
from gui.performance_settings_dialog import PerformanceSettingsDialog  # type: ignore


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
            if self.is_recording:
                self.metrics_collector.update(self.ros2_manager)
                metrics = self.metrics_collector.get_metrics()
            else:
                metrics = self.metrics_collector.get_live_metrics(self.ros2_manager)
            self.signals.finished.emit(metrics)
        except Exception as exc:  # pragma: no cover - defensive guard
            self.signals.error.emit(str(exc))


class HistoryWorkerSignals(QObject):
    """Signals for background history refresh worker."""

    finished = pyqtSignal(list)
    error = pyqtSignal(str)


class HistoryWorker(QRunnable):
    """Scans recording directory and collects bag info off the GUI thread."""

    def __init__(self, ros2_manager):
        super().__init__()
        self.ros2_manager = ros2_manager
        self.signals = HistoryWorkerSignals()

    @pyqtSlot()
    def run(self):
        import os
        try:
            recordings_dir = self.ros2_manager.get_recordings_directory()
            if not recordings_dir or not os.path.exists(recordings_dir):
                self.signals.finished.emit([])
                return

            bag_paths = []
            for root, dirs, files in os.walk(recordings_dir):
                if 'metadata.yaml' in files:
                    bag_paths.append(root)

            rows = []
            for path in bag_paths:
                info = self.ros2_manager.get_bag_info(path)
                rows.append({'path': path, 'info': info})

            # Sort newest first by path name (timestamp usually in path)
            rows.sort(key=lambda r: r['path'], reverse=True)
            self.signals.finished.emit(rows)
        except Exception as exc:  # pragma: no cover
            self.signals.error.emit(str(exc))


class MainWindow(QMainWindow):
    """Main application window"""
    
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
        
        self.ros2_manager = ROS2Manager()
        self.async_ros2 = AsyncROS2Manager(
            self.ros2_manager,
            max_threads=self.perf_settings['max_threads'],
            cache_timeout=self.perf_settings['cache_timeout']
        )
        self.metrics_collector = MetricsCollector()
        self.metrics_thread_pool = QThreadPool()
        self._metrics_task_running = False
        self.history_thread_pool = QThreadPool()
        self._history_task_running = False
        self.network_manager = None  # Initialize later
        self.is_recording = False
        self.theme_manager = ThemeManager()  # Theme management
        
        # DEBOUNCING - prevent excessive updates
        self._last_ros2_update = 0
        self._ros2_update_cooldown = 1.0  # At least 1 second between updates
        self._last_metrics_update = 0
        self._metrics_update_cooldown = 0.5  # At least 0.5 seconds between metrics
        
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
        self.tabs.addTab(topic_scroll, "📡 Topics")
        
        # Connect topic selection changes to recording control
        self.topic_monitor.topics_changed.connect(self.recording_control.update_selected_topics)
        
        # Tab 2: Node Monitor (with scroll area)
        self.node_monitor = NodeMonitorWidget(self.ros2_manager, self.async_ros2)
        node_scroll = QScrollArea()
        node_scroll.setWidget(self.node_monitor)
        node_scroll.setWidgetResizable(True)
        self.tabs.addTab(node_scroll, "🔧 Nodes")
        
        # Tab 3: Service Monitor (with scroll area)
        self.service_monitor = ServiceMonitorWidget(self.ros2_manager, self.async_ros2)
        service_scroll = QScrollArea()
        service_scroll.setWidget(self.service_monitor)
        service_scroll.setWidgetResizable(True)
        self.tabs.addTab(service_scroll, "⚙️ Services")
        
        # Tab 4: Topic Echo (with scroll area)
        self.topic_echo = TopicEchoWidget(self.ros2_manager)
        echo_scroll = QScrollArea()
        echo_scroll.setWidget(self.topic_echo)
        echo_scroll.setWidgetResizable(True)
        self.tabs.addTab(echo_scroll, "👁️ Topic Echo")
        
        # Tab 5: Advanced Stats (with scroll area)
        self.advanced_stats = AdvancedStatsWidget(self.ros2_manager)
        stats_scroll = QScrollArea()
        stats_scroll.setWidget(self.advanced_stats)
        stats_scroll.setWidgetResizable(True)
        self.tabs.addTab(stats_scroll, "📊 Stats")
        
        # Tab 6: Live Charts (with scroll area)
        self.live_charts = LiveChartsWidget(
            self.metrics_collector, 
            self.ros2_manager,
            buffer_size=self.perf_settings['chart_buffer_size'],
            update_interval=self.perf_settings['chart_update_interval'],
            auto_pause=self.perf_settings['chart_auto_pause']
        )
        charts_scroll = QScrollArea()
        charts_scroll.setWidget(self.live_charts)
        charts_scroll.setWidgetResizable(True)
        self.tabs.addTab(charts_scroll, "📈 Live Charts")
        
        # Tab 7: Network Robots (with scroll area)
        self.network_robots = NetworkRobotsWidget()
        self.network_robots.robot_selected.connect(self.on_robot_selected)
        robots_scroll = QScrollArea()
        robots_scroll.setWidget(self.network_robots)
        robots_scroll.setWidgetResizable(True)
        self.tabs.addTab(robots_scroll, "🤖 Network Robots")
        
        # Tab 8: Templates (with scroll area)
        self.templates = RecordingTemplatesWidget(self.recording_control, self.topic_monitor)
        templates_scroll = QScrollArea()
        templates_scroll.setWidget(self.templates)
        templates_scroll.setWidgetResizable(True)
        self.tabs.addTab(templates_scroll, "📋 Templates")
        
        # Tab 9: Network Upload (with scroll area)
        from core.network_manager import NetworkManager
        placeholder_network_manager = NetworkManager()
        self.network_upload = NetworkUploadWidget(placeholder_network_manager)
        upload_scroll = QScrollArea()
        upload_scroll.setWidget(self.network_upload)
        upload_scroll.setWidgetResizable(True)
        self.tabs.addTab(upload_scroll, "☁️ Upload")
        
        # Tab 10: Recording History (with scroll area) - MOVED FROM BOTTOM SPLITTER
        self.recording_history = self.create_recording_history()
        history_scroll = QScrollArea()
        history_scroll.setWidget(self.recording_history)
        history_scroll.setWidgetResizable(True)
        self.tabs.addTab(history_scroll, "📁 History")
        
        splitter.addWidget(self.tabs)
        
        # Set splitter sizes - now only 2 sections (top + tabs, no bottom history)
        splitter.setSizes([300, 700])
        
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
        charts_action.triggered.connect(lambda: self.tabs.setCurrentIndex(6))
        view_menu.addAction(charts_action)  # type: ignore[union-attr]
        
        # Settings menu
        settings_menu = menubar.addMenu("&Settings")
        
        perf_action = QAction("&Performance Mode...", self)
        perf_action.triggered.connect(self.show_performance_settings)
        settings_menu.addAction(perf_action)  # type: ignore[union-attr]
        
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
        playback_shortcut.activated.connect(lambda: self.tabs.setCurrentIndex(4))
        
        # Ctrl+E - Export Performance Report
        export_shortcut = QShortcut(QKeySequence("Ctrl+E"), self)
        export_shortcut.activated.connect(self.export_metrics)
        
        # Ctrl+L - Switch to Live Charts
        charts_shortcut = QShortcut(QKeySequence("Ctrl+L"), self)
        charts_shortcut.activated.connect(lambda: self.tabs.setCurrentIndex(6))
        
        # Ctrl+H - Show Help
        help_shortcut = QShortcut(QKeySequence("Ctrl+H"), self)
        help_shortcut.activated.connect(self.show_keyboard_shortcuts_help)
        
        # Ctrl+T - Toggle Theme
        theme_shortcut = QShortcut(QKeySequence("Ctrl+T"), self)
        theme_shortcut.activated.connect(self.toggle_theme)
        
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
    
    def on_performance_mode_changed(self, mode):
        """Handle performance mode change"""
        # Get new settings
        self.perf_settings = self.performance_manager.get_mode_settings()
        
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
        
    def create_recording_history(self):
        """Create recording history widget - OPTIMIZED FOR SMOOTH SCROLLING"""
        group = QGroupBox("Recording History & File Information")
        layout = QVBoxLayout()
        
        # Table for recordings - OPTIMIZED FOR PERFORMANCE
        self.history_table = QTableWidget()
        self.history_table.setColumnCount(6)
        self.history_table.setHorizontalHeaderLabels([
            "Filename", "Size (MB)", "Duration", "Topics", "Start Time", "Status"
        ])
        
        # CRITICAL PERFORMANCE: Disable sorting and optimize rendering
        self.history_table.setSortingEnabled(False)
        
        vheader = self.history_table.verticalHeader()
        if vheader is not None:
            vheader.setDefaultSectionSize(25)
        
        self.history_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.history_table.setSelectionMode(QTableWidget.SingleSelection)
        
        header = self.history_table.horizontalHeader()
        if header is not None:
            header.setSectionResizeMode(QHeaderView.Stretch)
        
        layout.addWidget(self.history_table)
        
        # Buttons
        button_layout = QHBoxLayout()
        
        refresh_btn = QPushButton("Refresh List")
        refresh_btn.clicked.connect(self.refresh_recording_history)
        button_layout.addWidget(refresh_btn)
        
        open_folder_btn = QPushButton("Open Recordings Folder")
        open_folder_btn.clicked.connect(self.open_recordings_folder)
        button_layout.addWidget(open_folder_btn)
        
        layout.addLayout(button_layout)
        
        group.setLayout(layout)
        return group
        
    def setup_timers(self):
        """Setup update timers - DELAYED START to prevent startup freezes"""
        perf = self.perf_settings
        
        # ROS2 info timer - ULTRA SLOW interval (10 seconds) to prevent topic tab freezes
        # Topics tab was causing freezes from excessive checkbox recreation
        self.ros2_timer = QTimer()
        self.ros2_timer.timeout.connect(self.update_ros2_info_async)
        # DON'T START YET - delay 3 seconds to let UI load first
        QTimer.singleShot(3000, lambda: self.ros2_timer.start(10000))  # Changed from 5s to 10s
        
        # Metrics timer - VERY SLOW interval (5 seconds)
        # Metrics collection is async and non-blocking
        self.metrics_timer = QTimer()
        self.metrics_timer.timeout.connect(self.update_metrics)
        # Delay 5 seconds after startup
        QTimer.singleShot(5000, lambda: self.metrics_timer.start(5000))
        
        # Recording history timer - VERY SLOW interval (30 seconds)
        # History is low priority and doesn't need frequent updates
        self.history_timer = QTimer()
        self.history_timer.timeout.connect(self.refresh_recording_history)
        # Delay 10 seconds after startup
        QTimer.singleShot(10000, lambda: self.history_timer.start(30000))
        
        # Connect to performance mode changes
        self.performance_manager.mode_changed.connect(self.on_performance_mode_changed)
    
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
        """Warm up async cache in background - prevents first-call freezes"""
        print("🔥 Warming up cache in background...")
        
        # Trigger async cache population (non-blocking)
        # These will populate the cache so future UI updates are instant
        def _noop(data):
            print(f"✅ Cache warmed: {len(data) if isinstance(data, list) else 'data'} items")
        
        try:
            if self.async_ros2:
                # Queue async operations without blocking
                self.async_ros2.get_topics_async(_noop)
                # Stagger the calls to avoid overwhelming system
                QTimer.singleShot(500, lambda: self.async_ros2.get_nodes_async(_noop) if self.async_ros2 else None)
                QTimer.singleShot(1000, lambda: self.async_ros2.get_services_async(_noop) if self.async_ros2 else None)
        except Exception as e:
            print(f"⚠️ Cache warmup error: {e}")
            
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
            
    def update_ros2_info_async(self):
        """
        ULTRA-OPTIMIZED LAZY TAB LOADING with smart debouncing
        
        Changes:
        - Only update the currently visible tab (reduces CPU 40-50%)
        - Use debounced refresh methods on each monitor widget
        - Prevents non-visible tabs from consuming resources
        - Smart deduplication at async manager level prevents concurrent requests
        """
        import time
        
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
        
    def update_metrics(self):
        """Update dashboard metrics - AGGRESSIVE debouncing"""
        import time
        
        # DEBOUNCE - skip if updated recently
        current_time = time.time()
        if current_time - self._last_metrics_update < self._metrics_update_cooldown:
            return
        self._last_metrics_update = current_time
        
        if self._metrics_task_running:
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
    
    def on_robot_selected(self, hostname: str, topics: list):
        """Handle robot selection from network discovery - seamless workflow"""
        if not topics:
            self.update_status(f"⚠️ Robot {hostname} has no topics")
            return
        
        # Store selected robot info
        self.selected_robot = {
            'hostname': hostname,
            'topics': topics,
            'selected_at': datetime.now()
        }
        
        # Auto-select discovered topics in topic monitor
        if hasattr(self, 'topic_monitor'):
            # Refresh topic list first to ensure we have latest
            self.topic_monitor.refresh_topics()
            
            # Auto-select matching topics
            selected_count = 0
            for topic in topics:
                # Try to select the topic in the topic monitor
                for row in range(self.topic_monitor.topic_table.rowCount()):
                    table_topic = self.topic_monitor.topic_table.item(row, 0)
                    if table_topic and table_topic.text() == topic:
                        self.topic_monitor.topic_table.selectRow(row)
                        selected_count += 1
                        break
            
            # Switch to topics tab
            self.tabs.setCurrentIndex(0)
            
            self.update_status(f"🤖 Selected {selected_count}/{len(topics)} topics from robot: {hostname}")
            
            # Show workflow dialog
            from PyQt5.QtWidgets import QMessageBox
            msg = QMessageBox(self)
            msg.setIcon(QMessageBox.Information)
            msg.setWindowTitle("Robot Selected - Ready to Record")
            msg.setText(f"✅ Robot: {hostname}\n"
                       f"📊 Topics: {selected_count} selected\n\n"
                       f"Next Steps:")
            msg.setInformativeText(
                "1️⃣ Review selected topics in the Topics tab\n"
                "2️⃣ Click 'Start Recording' to begin\n"
                "3️⃣ Recording will auto-upload to server when complete\n\n"
                "💡 Tip: You can add/remove topics before recording"
            )
            msg.setStandardButtons(QMessageBox.Ok)
            
            # Add quick action buttons
            record_button = msg.addButton("🎬 Start Recording Now", QMessageBox.ActionRole)
            
            result = msg.exec_()
            
            # Check if user clicked record button
            if msg.clickedButton() == record_button:
                # Trigger recording immediately with robot metadata
                if selected_count > 0:
                    self.recording_control.start_recording(robot_metadata=self.selected_robot)
                else:
                    QMessageBox.warning(self, "No Topics", 
                                      "No matching topics found. Please select topics manually.")
        else:
            self.update_status(f"🤖 Robot {hostname} selected with {len(topics)} topics")
            
    @pyqtSlot()
    def on_recording_started(self):
        """Handle recording started event"""
        self.is_recording = True
        self.metrics_collector.reset()
        self.update_status("Recording in progress...")
        
        # Timers are already set to very infrequent (5-30 seconds)
        # No need to pause or modify them - they won't cause freezing
        
        # Show notification
        self.show_notification("Recording Started", "ROS2 bag recording in progress")
        
    @pyqtSlot()
    def on_recording_stopped(self):
        """Handle recording stopped event - with auto-upload support"""
        self.is_recording = False
        self.update_status("Recording stopped")
        
        # Timers stay at their slow intervals (5-30 seconds) - no changes needed
        # They were already set to prevent freezing
        
        self.refresh_recording_history()
        
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
        
    def refresh_recording_history(self):
        """Schedule background refresh of recording history (non-blocking)."""
        if self._history_task_running:
            return

        worker = HistoryWorker(self.ros2_manager)
        worker.signals.finished.connect(self._handle_history_finished)  # type: ignore[arg-type]
        worker.signals.error.connect(self._handle_history_error)  # type: ignore[arg-type]
        self._history_task_running = True
        self.history_thread_pool.start(worker)

    @pyqtSlot(list)
    def _handle_history_finished(self, rows):
        """Populate the history table with precomputed rows from worker."""
        try:
            self.history_table.setRowCount(len(rows))
            for idx, row in enumerate(rows):
                bag_path = row['path']
                bag_info = row['info'] or {}

                # Filename
                filename_item = QTableWidgetItem(os.path.basename(bag_path))
                self.history_table.setItem(idx, 0, filename_item)

                # Size
                size_mb = bag_info.get('size_mb', 0) or 0
                size_item = QTableWidgetItem(f"{size_mb:.2f}")
                size_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                self.history_table.setItem(idx, 1, size_item)

                # Duration
                duration = bag_info.get('duration', '0s')
                self.history_table.setItem(idx, 2, QTableWidgetItem(duration))

                # Topics
                topic_count = bag_info.get('topic_count', 0) or 0
                topics_item = QTableWidgetItem(str(topic_count))
                topics_item.setTextAlignment(Qt.AlignCenter)
                self.history_table.setItem(idx, 3, topics_item)

                # Start time
                start_time = bag_info.get('start_time', 'Unknown')
                self.history_table.setItem(idx, 4, QTableWidgetItem(start_time))

                # Status
                status = "Complete" if bag_info.get('is_complete', True) else "Incomplete"
                status_item = QTableWidgetItem(status)
                if status == "Complete":
                    status_item.setForeground(QColor('green'))
                else:
                    status_item.setForeground(QColor('orange'))
                self.history_table.setItem(idx, 5, status_item)
        finally:
            self._history_task_running = False

    @pyqtSlot(str)
    def _handle_history_error(self, message: str):
        self._history_task_running = False
        print(f"History worker error: {message}")  # pragma: no cover
            
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
        
    def closeEvent(self, event):
        """Handle window close event - proper cleanup"""
        if self.is_recording:
            reply = QMessageBox.question(
                self, 'Confirm Exit',
                'Recording is in progress. Do you want to stop and exit?',
                QMessageBox.Yes | QMessageBox.No, QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                self.ros2_manager.stop_recording()
                if self.network_manager:
                    self.network_manager.stop()
                
                # Shutdown async manager thread pools
                if hasattr(self, 'async_ros2'):
                    self.async_ros2.shutdown()
                
                event.accept()
            else:
                event.ignore()
        else:
            if self.network_manager:
                self.network_manager.stop()
            
            # Shutdown async manager thread pools
            if hasattr(self, 'async_ros2'):
                self.async_ros2.shutdown()
            
            event.accept()
