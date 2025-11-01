"""
Live Charts Widget - Real-time visualization of recording metrics and system stats
High-performance plotting using pyqtgraph for zero-latency updates
ULTRA-OPTIMIZED: Fast async loading, minimal overhead, progressive rendering
"""

import pyqtgraph as pg
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QComboBox, QGroupBox, QGridLayout, QCheckBox)
from PyQt5.QtCore import Qt, QTimer, QRunnable, QThreadPool, pyqtSignal  # type: ignore
import psutil
from PyQt5.QtGui import QFont
import numpy as np
from collections import deque
from datetime import datetime

# PERFORMANCE: Disable pyqtgraph features we don't need
pg.setConfigOptions(
    antialias=False,  # Faster rendering without antialiasing
    useOpenGL=False,  # Avoid OpenGL overhead (can cause issues on some systems)
    enableExperimental=False,  # Disable experimental features
    crashWarning=False  # Disable crash warnings for speed
)

# Import dynamic system detection
try:
    from core.system_detection import SystemDetector, DynamicPerformanceTuner
except ImportError:
    try:
        from system_detection import SystemDetector, DynamicPerformanceTuner
    except ImportError:
        SystemDetector = None
        DynamicPerformanceTuner = None


class LiveChartsWidget(QWidget):
    """Widget for real-time data visualization with multiple charts"""
    # Signal emitted when background topic count is ready
    topic_count_ready = pyqtSignal(int)
    
    class TopicCountWorker(QRunnable):
        """Background worker to fetch topic count without blocking UI"""
        def __init__(self, ros2_manager, signal):
            super().__init__()
            self.ros2_manager = ros2_manager
            self.signal = signal

        def run(self):
            try:
                topics = self.ros2_manager.get_topics_info()
                count = len(topics) if topics is not None else 0
            except Exception:
                count = 0
            try:
                # emit signal with topic count (safe from background thread)
                self.signal.emit(count)
            except Exception:
                pass
    
    def __init__(self, metrics_collector, ros2_manager, buffer_size=60, update_interval=1000, auto_pause=True):
        super().__init__()
        self.metrics_collector = metrics_collector
        self.ros2_manager = ros2_manager
        
        # DYNAMIC SETTINGS: Auto-tune based on system specs
        self.dynamic_settings = self._get_dynamic_settings()
        
        # Adaptive performance settings (can be overridden by dynamic settings)
        self.buffer_size = buffer_size or self.dynamic_settings.get('max_buffer_size', 600)
        self.update_interval = update_interval or self.dynamic_settings.get('chart_update_interval', 300)
        self.auto_pause = auto_pause
        
        # Dynamic parameters
        self.plot_skip_threshold = self.dynamic_settings.get('plot_skip_threshold', 5)
        self.stats_update_frequency = self.dynamic_settings.get('stats_update_frequency', 30)
        self.cpu_backoff_high = self.dynamic_settings.get('cpu_backoff_threshold_high', 80.0)
        self.cpu_backoff_critical = self.dynamic_settings.get('cpu_backoff_threshold_critical', 90.0)
        self.cpu_interval_multiplier = self.dynamic_settings.get('cpu_interval_multiplier', 2.0)
        
        # Data buffers (adaptive size based on system)
        self.max_points = self.buffer_size
        self.time_data = deque(maxlen=self.max_points)
        
        # Recording metrics buffers
        self.msg_rate_data = deque(maxlen=self.max_points)
        self.bandwidth_data = deque(maxlen=self.max_points)
        self.topic_count_data = deque(maxlen=self.max_points)
        
        # System metrics buffers
        self.cpu_data = deque(maxlen=self.max_points)
        self.memory_data = deque(maxlen=self.max_points)
        self.disk_write_data = deque(maxlen=self.max_points)
        
        self.start_time = None
        self.paused = not auto_pause  # Auto-pause if enabled
        self.update_counter = 0  # For optimization
        
        # Threadpool for background small tasks (topic count)
        self._threadpool = QThreadPool()
        max_threads = self.dynamic_settings.get('max_threads', 1)
        self._threadpool.setMaxThreadCount(max_threads)
        self._last_topic_async = 0
        # Connect signal to UI update
        self.topic_count_ready.connect(self._on_topic_count_ready)  # type: ignore

        # LAZY LOADING: Don't create plots immediately - create them on first show
        self._ui_initialized = False
        self._charts_loaded = False
        
        self.init_ui()
        self.setup_update_timer()
    
    def _get_dynamic_settings(self):
        """Get dynamic settings based on system specs"""
        try:
            if SystemDetector is not None and DynamicPerformanceTuner is not None:
                specs = SystemDetector.get_system_specs()
                settings = DynamicPerformanceTuner.get_tuned_settings(specs)
                print(f"✅ Auto-tuned for {specs['system_category']} system ({specs['cpu_cores']} cores, {specs['ram_gb']:.1f}GB RAM)")
                return settings
        except Exception as e:
            print(f"⚠️  Dynamic tuning failed: {e}, using defaults")
        
        # Return default settings if detection fails
        return {
            'max_buffer_size': 600,
            'chart_update_interval': 300,
            'plot_skip_threshold': 5,
            'stats_update_frequency': 30,
            'cpu_backoff_threshold_high': 80.0,
            'cpu_backoff_threshold_critical': 90.0,
            'cpu_interval_multiplier': 2.0,
            'max_threads': 2,
        }
        
    def init_ui(self):
        """Initialize UI components - FAST ASYNC: charts loaded on first view"""
        self.main_layout = QVBoxLayout()
        
        # Show loading message with animation
        loading_widget = QWidget()
        loading_layout = QVBoxLayout()
        
        loading_label = QLabel("⚡ Live Charts")
        loading_label.setAlignment(Qt.AlignCenter)  # type: ignore
        loading_label.setStyleSheet("color: #2196F3; font-size: 24px; font-weight: bold; padding: 20px;")
        loading_layout.addWidget(loading_label)
        
        info_label = QLabel("Charts load instantly when you view this tab")
        info_label.setAlignment(Qt.AlignCenter)  # type: ignore
        info_label.setStyleSheet("color: #666; font-size: 14px;")
        loading_layout.addWidget(info_label)
        
        tip_label = QLabel("💡 Tip: Charts update automatically in real-time")
        tip_label.setAlignment(Qt.AlignCenter)  # type: ignore
        tip_label.setStyleSheet("color: #888; font-size: 12px; padding: 10px;")
        loading_layout.addWidget(tip_label)
        
        loading_widget.setLayout(loading_layout)
        self.main_layout.addWidget(loading_widget)
        
        self.setLayout(self.main_layout)
    
    def _load_charts(self):
        """LAZY LOADING: Create charts only when tab is first viewed"""
        if self._charts_loaded:
            return
        
        print("📊 Lazy loading Live Charts widget...")
        
        # Clear placeholder
        while self.main_layout.count():
            item = self.main_layout.takeAt(0)
            if item is not None:
                widget = item.widget()
                if widget:
                    widget.deleteLater()
        
        # Control panel
        controls = self.create_controls()
        self.main_layout.addWidget(controls)
        
        # Create chart grid
        charts_layout = QGridLayout()
        charts_layout.setSpacing(10)  # Add spacing between charts
        
        # Recording Metrics Charts (Row 1)
        self.msg_rate_plot = self.create_plot(
            "Message Rate (msg/s)", 
            "Messages/sec", 
            color='#2196F3'
        )
        charts_layout.addWidget(self.msg_rate_plot, 0, 0)
        
        self.bandwidth_plot = self.create_plot(
            "Bandwidth (MB/s)", 
            "MB/s", 
            color='#4CAF50'
        )
        charts_layout.addWidget(self.bandwidth_plot, 0, 1)
        
        self.topic_count_plot = self.create_plot(
            "Active Topics", 
            "Count", 
            color='#FF9800'
        )
        charts_layout.addWidget(self.topic_count_plot, 0, 2)
        
        # System Metrics Charts (Row 2)
        self.cpu_plot = self.create_plot(
            "CPU Usage (%)", 
            "Percent", 
            color='#f44336'
        )
        charts_layout.addWidget(self.cpu_plot, 1, 0)
        
        self.memory_plot = self.create_plot(
            "Memory Usage (%)", 
            "Percent", 
            color='#9C27B0'
        )
        charts_layout.addWidget(self.memory_plot, 1, 1)
        
        self.disk_write_plot = self.create_plot(
            "Disk Write Speed (MB/s)", 
            "MB/s", 
            color='#00BCD4'
        )
        charts_layout.addWidget(self.disk_write_plot, 1, 2)
        
        # Set equal stretch for all rows and columns to distribute space evenly
        for row in range(2):
            charts_layout.setRowStretch(row, 1)
        for col in range(3):
            charts_layout.setColumnStretch(col, 1)
        
        charts_group = QGroupBox("Real-Time Performance Monitoring")
        charts_group.setLayout(charts_layout)
        self.main_layout.addWidget(charts_group, 1)  # Give charts more space in main layout
        
        # Statistics panel
        stats = self.create_statistics_panel()
        self.main_layout.addWidget(stats)
        
        self._charts_loaded = True
        print("✅ Live Charts loaded successfully")
    
    def _load_charts_async(self):
        """FAST ASYNC LOADING: Create charts without blocking UI thread"""
        if self._charts_loaded:
            return
        
        print("⚡ Fast async loading Live Charts widget...")
        
        try:
            # Clear placeholder with immediate processing
            while self.main_layout.count():
                item = self.main_layout.takeAt(0)
                if item is not None:
                    widget = item.widget()
                    if widget:
                        widget.deleteLater()
            
            # Force process pending events to update UI
            from PyQt5.QtWidgets import QApplication
            QApplication.processEvents()
            
            # Create controls (fast)
            controls = self.create_controls()
            self.main_layout.addWidget(controls)
            QApplication.processEvents()
            
            # Create chart grid with PROGRESSIVE LOADING
            charts_layout = QGridLayout()
            charts_layout.setSpacing(10)
            
            # ROW 1: Recording Metrics (create and show immediately)
            print("  📈 Creating recording metrics charts...")
            self.msg_rate_plot = self.create_plot_fast("Message Rate (msg/s)", "Messages/sec", color='#2196F3')
            charts_layout.addWidget(self.msg_rate_plot, 0, 0)
            
            self.bandwidth_plot = self.create_plot_fast("Bandwidth (MB/s)", "MB/s", color='#4CAF50')
            charts_layout.addWidget(self.bandwidth_plot, 0, 1)
            
            self.topic_count_plot = self.create_plot_fast("Active Topics", "Count", color='#FF9800')
            charts_layout.addWidget(self.topic_count_plot, 0, 2)
            
            # Process events to show first row
            QApplication.processEvents()
            
            # ROW 2: System Metrics
            print("  📊 Creating system metrics charts...")
            self.cpu_plot = self.create_plot_fast("CPU Usage (%)", "Percent", color='#f44336')
            charts_layout.addWidget(self.cpu_plot, 1, 0)
            
            self.memory_plot = self.create_plot_fast("Memory Usage (%)", "Percent", color='#9C27B0')
            charts_layout.addWidget(self.memory_plot, 1, 1)
            
            self.disk_write_plot = self.create_plot_fast("Disk Write Speed (MB/s)", "MB/s", color='#00BCD4')
            charts_layout.addWidget(self.disk_write_plot, 1, 2)
            
            # Process events to show second row
            QApplication.processEvents()
            
            # Set layout stretching
            for row in range(2):
                charts_layout.setRowStretch(row, 1)
            for col in range(3):
                charts_layout.setColumnStretch(col, 1)
            
            charts_group = QGroupBox("Real-Time Performance Monitoring")
            charts_group.setLayout(charts_layout)
            self.main_layout.addWidget(charts_group, 1)
            
            # Create statistics panel
            print("  📑 Creating statistics panel...")
            stats = self.create_statistics_panel()
            self.main_layout.addWidget(stats)
            
            # Final processing
            QApplication.processEvents()
            
            self._charts_loaded = True
            print("✅ Live Charts loaded successfully (async)")
            
            # Start the update timer now that charts are ready
            if hasattr(self, 'update_timer') and not self.paused:
                self.update_timer.start(self.update_interval)
                if hasattr(self, 'status_label'):
                    self.status_label.setText("📊 Monitoring Active")
            
        except Exception as e:
            print(f"❌ Error loading charts: {e}")
            import traceback
            traceback.print_exc()
        
    def create_controls(self):
        """Create control panel"""
        group = QGroupBox("Chart Controls")
        layout = QHBoxLayout()
        
        # Pause/Resume button
        self.pause_btn = QPushButton("⏸ Pause")
        self.pause_btn.clicked.connect(self.toggle_pause)
        self.pause_btn.setMaximumWidth(100)
        layout.addWidget(self.pause_btn)
        
        # Clear button
        clear_btn = QPushButton("🗑 Clear")
        clear_btn.clicked.connect(self.clear_charts)
        clear_btn.setMaximumWidth(100)
        layout.addWidget(clear_btn)
        
        # Export button
        export_btn = QPushButton("💾 Export Charts")
        export_btn.clicked.connect(self.export_charts)
        export_btn.setMaximumWidth(150)
        layout.addWidget(export_btn)
        
        # Time window selector
        layout.addWidget(QLabel("Time Window:"))
        self.time_window = QComboBox()
        self.time_window.addItems(["1 min", "5 min", "10 min", "30 min"])
        self.time_window.currentTextChanged.connect(self.change_time_window)
        self.time_window.setMaximumWidth(100)
        layout.addWidget(self.time_window)
        
        # Auto-scale checkbox
        self.autoscale_check = QCheckBox("Auto-scale Y")
        self.autoscale_check.setChecked(False)  # Disabled by default to reduce GPU load
        layout.addWidget(self.autoscale_check)
        
        layout.addStretch()
        
        # Status label
        self.status_label = QLabel("📊 Monitoring Active")
        self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
        layout.addWidget(self.status_label)
        
        group.setLayout(layout)
        return group
        
    def create_plot(self, title, y_label, color='#2196F3'):
        """Create a single plot widget with proper sizing"""
        plot_widget = pg.PlotWidget()
        plot_widget.setBackground('w')
        plot_widget.setTitle(title, color='#333', size='12pt')
        plot_widget.setLabel('left', y_label, color='#333')
        plot_widget.setLabel('bottom', 'Time (seconds)', color='#333')
        plot_widget.showGrid(x=True, y=True, alpha=0.3)
        
        # CRITICAL: Set minimum height so charts don't compress into thin lines
        plot_widget.setMinimumHeight(250)
        plot_widget.setMinimumWidth(300)
        
        # Create plot curve
        pen = pg.mkPen(color=color, width=2)
        curve = plot_widget.plot([], [], pen=pen, name=title)
        
        # Store curve reference
        plot_widget.curve = curve
        
        # Style
        plot_widget.getAxis('left').setPen(pg.mkPen(color='#333', width=1))
        plot_widget.getAxis('bottom').setPen(pg.mkPen(color='#333', width=1))
        
        return plot_widget
    
    def create_plot_fast(self, title, y_label, color='#2196F3'):
        """OPTIMIZED: Create plot widget with minimal overhead for fast loading"""
        plot_widget = pg.PlotWidget()
        
        # FAST: Minimal styling first, enhance later
        plot_widget.setBackground('w')
        plot_widget.setTitle(title, color='#333', size='11pt')  # Slightly smaller for speed
        
        # FAST: Skip detailed labels initially
        plot_widget.setLabel('left', y_label)
        plot_widget.setLabel('bottom', 'Time (s)')
        
        # FAST: Simplified grid
        plot_widget.showGrid(x=True, y=True, alpha=0.2)
        
        # FAST: Set size constraints
        plot_widget.setMinimumHeight(220)  # Slightly smaller initially
        plot_widget.setMinimumWidth(280)
        
        # FAST: Create curve with thinner pen for faster rendering
        pen = pg.mkPen(color=color, width=1.5)  # Thinner = faster
        curve = plot_widget.plot([], [], pen=pen)
        
        # Store curve reference
        plot_widget.curve = curve
        
        # OPTIMIZATION: Disable auto-range for faster updates
        plot_widget.setAutoVisible(y=True)
        plot_widget.enableAutoRange(axis='y', enable=True)
        plot_widget.enableAutoRange(axis='x', enable=False)  # Fixed X range for speed
        
        return plot_widget
        
    def create_statistics_panel(self):
        """Create statistics panel showing peaks, averages, etc."""
        group = QGroupBox("Statistics Summary")
        layout = QGridLayout()
        
        font = QFont()
        font.setBold(True)
        
        # Create stat labels
        self.stats_labels = {}
        
        stats = [
            ("Peak Message Rate:", "peak_msg_rate", 0, 0),
            ("Avg Message Rate:", "avg_msg_rate", 0, 1),
            ("Peak Bandwidth:", "peak_bandwidth", 0, 2),
            ("Avg Bandwidth:", "avg_bandwidth", 0, 3),
            ("Peak CPU:", "peak_cpu", 1, 0),
            ("Avg CPU:", "avg_cpu", 1, 1),
            ("Peak Memory:", "peak_memory", 1, 2),
            ("Avg Memory:", "avg_memory", 1, 3),
        ]
        
        for label_text, key, row, col in stats:
            label = QLabel(label_text)
            label.setFont(font)
            layout.addWidget(label, row, col * 2)
            
            value = QLabel("0")
            value.setStyleSheet("color: #2196F3;")
            self.stats_labels[key] = value
            layout.addWidget(value, row, col * 2 + 1)
        
        group.setLayout(layout)
        return group
        
    def setup_update_timer(self):
        """Setup timer for chart updates - adaptive based on performance mode"""
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_charts)
        self.update_timer.start(self.update_interval)
        
    def showEvent(self, event):
        """Resume updates when tab becomes visible - FAST ASYNC chart loading"""
        super().showEvent(event)
        
        # FAST ASYNC LOAD: Create charts asynchronously to prevent UI freeze
        if not self._charts_loaded:
            # Use QTimer.singleShot to load charts asynchronously (0ms = next event loop)
            # This prevents blocking the UI thread during chart creation
            from PyQt5.QtCore import QTimer
            QTimer.singleShot(0, self._load_charts_async)
            return  # Don't start timer yet, wait for charts to load
        
        if hasattr(self, 'update_timer') and self.auto_pause and not self.paused:
            # Only start if user hasn't manually paused
            self.update_timer.start(self.update_interval)
            if hasattr(self, 'pause_btn'):
                self.pause_btn.setText("⏸ Pause")
            if hasattr(self, 'status_label'):
                self.status_label.setText("📊 Monitoring Active")
            
    def hideEvent(self, event):
        """Pause updates when tab is hidden to save resources (if auto_pause enabled)"""
        super().hideEvent(event)
        if hasattr(self, 'update_timer') and self.auto_pause:
            # CRITICAL: Keep collecting metrics even when tab is not visible
            # This ensures live charts have data when user switches to the tab
            # Only stop if we're in minimal performance mode
            pass  # Don't actually stop the timer - keep it running
        
    def update_charts(self):
        """Update all charts with latest data - DYNAMIC OPTIMIZATION"""
        # SAFETY: Skip updates if charts not yet loaded (lazy loading)
        if not self._charts_loaded:
            return
        
        if self.paused:
            return
        
        self.update_counter += 1
        
        # CRITICAL OPTIMIZATION: Use DYNAMIC thresholds based on system specs
        # This adapts to the user's hardware automatically
        need_plot_update = (self.update_counter % self.plot_skip_threshold == 0)
        need_stats_update = (self.update_counter % self.stats_update_frequency == 0)
        
        if not need_plot_update and not need_stats_update:
            # Nothing to do this cycle - just append data and return early
            try:
                metrics = self.metrics_collector.get_live_metrics(None)
                if metrics is None:
                    metrics = {}
                
                metrics_safe = {
                    'message_rate': float(metrics.get('message_rate', 0) or 0),
                    'write_speed_mb_s': float(metrics.get('write_speed_mb_s', 0) or 0),
                    'topic_count': int(metrics.get('topic_count', 0) or 0),
                    'cpu_percent': float(metrics.get('cpu_percent', 0) or 0),
                    'memory_percent': float(metrics.get('memory_percent', 0) or 0),
                    'disk_write_speed': float(metrics.get('disk_write_speed', 0) or 0)
                }
                
                if self.start_time is None:
                    self.start_time = datetime.now()
                
                elapsed = (datetime.now() - self.start_time).total_seconds()
                self.time_data.append(elapsed)
                self.msg_rate_data.append(metrics_safe['message_rate'])
                self.bandwidth_data.append(metrics_safe['write_speed_mb_s'])
                self.topic_count_data.append(metrics_safe['topic_count'])
                self.cpu_data.append(metrics_safe['cpu_percent'])
                self.memory_data.append(metrics_safe['memory_percent'])
                self.disk_write_data.append(metrics_safe['disk_write_speed'])
            except Exception:
                pass
            return
        
        # COLLECT METRICS: Only when we actually need to do something
        try:
            metrics = self.metrics_collector.get_live_metrics(None)
            if metrics is None:
                metrics = {}
        except Exception:
            metrics = {}
        
        # Ensure all required metrics exist with numeric defaults
        metrics_safe = {
            'message_rate': float(metrics.get('message_rate', 0) or 0),
            'write_speed_mb_s': float(metrics.get('write_speed_mb_s', 0) or 0),
            'topic_count': int(metrics.get('topic_count', 0) or 0),
            'cpu_percent': float(metrics.get('cpu_percent', 0) or 0),
            'memory_percent': float(metrics.get('memory_percent', 0) or 0),
            'disk_write_speed': float(metrics.get('disk_write_speed', 0) or 0)
        }
        
        # Check CPU for adaptive throttling USING DYNAMIC THRESHOLDS
        cpu_now = metrics_safe['cpu_percent']
        if cpu_now > self.cpu_backoff_critical:
            # Skip entire cycle if CPU critically high (dynamic threshold)
            return
        elif cpu_now > self.cpu_backoff_high:
            # Increase the update interval during high CPU (dynamic threshold and multiplier)
            try:
                backoff_interval = int(self.update_interval * self.cpu_interval_multiplier)
                self.update_timer.setInterval(min(backoff_interval, 5000))
            except Exception:
                pass
            return
        else:
            # Restore normal interval
            try:
                if self.update_timer.interval() != self.update_interval:
                    self.update_timer.setInterval(self.update_interval)
            except Exception:
                pass
        
        # Initialize start time if first update
        if self.start_time is None:
            self.start_time = datetime.now()
        
        # Calculate elapsed time and append to buffers
        elapsed = (datetime.now() - self.start_time).total_seconds()
        self.time_data.append(elapsed)
        self.msg_rate_data.append(metrics_safe['message_rate'])
        self.bandwidth_data.append(metrics_safe['write_speed_mb_s'])
        self.topic_count_data.append(metrics_safe['topic_count'])
        self.cpu_data.append(metrics_safe['cpu_percent'])
        self.memory_data.append(metrics_safe['memory_percent'])
        self.disk_write_data.append(metrics_safe['disk_write_speed'])
        
        # Fetch topic count in background only if not under load
        try:
            now = datetime.now().timestamp()
            cpu_threshold = self.cpu_backoff_high - 10  # Back off earlier for async work
            if cpu_now < cpu_threshold and (now - getattr(self, '_last_topic_async', 0)) > 3.0:
                if self._threadpool.activeThreadCount() == 0:
                    worker = LiveChartsWidget.TopicCountWorker(self.ros2_manager, self.topic_count_ready)
                    self._threadpool.start(worker)
                    self._last_topic_async = now
        except Exception:
            pass
        
        # PLOT UPDATE
        if need_plot_update:
            if len(self.time_data) < 2:
                return
            
            try:
                # ULTRA-FAST: Use fromiter for zero-copy where possible
                time_array = np.fromiter(self.time_data, dtype=np.float32)
                msg_rate_array = np.fromiter(self.msg_rate_data, dtype=np.float32)
                bandwidth_array = np.fromiter(self.bandwidth_data, dtype=np.float32)
                topic_count_array = np.fromiter(self.topic_count_data, dtype=np.float32)
                cpu_array = np.fromiter(self.cpu_data, dtype=np.float32)
                memory_array = np.fromiter(self.memory_data, dtype=np.float32)
                disk_write_array = np.fromiter(self.disk_write_data, dtype=np.float32)
                
                # BATCH ALL PLOT UPDATES (no intermediate redraws)
                self.msg_rate_plot.curve.setData(time_array, msg_rate_array)
                self.bandwidth_plot.curve.setData(time_array, bandwidth_array)
                self.topic_count_plot.curve.setData(time_array, topic_count_array)
                self.cpu_plot.curve.setData(time_array, cpu_array)
                self.memory_plot.curve.setData(time_array, memory_array)
                self.disk_write_plot.curve.setData(time_array, disk_write_array)
                
                # NEVER auto-scale by default (huge GPU overhead)
                # Only scale if explicitly enabled AND we're not under CPU load AND charts visible
                if self.autoscale_check.isChecked() and cpu_now < 50.0:
                    # Batch autoscale operations (very infrequently)
                    if self.update_counter % 40 == 0:  # Only every 40 cycles (~6 seconds)
                        for plot in [self.msg_rate_plot, self.bandwidth_plot, self.topic_count_plot,
                                    self.cpu_plot, self.memory_plot, self.disk_write_plot]:
                            try:
                                plot.enableAutoRange(enable=True, recursive=False)
                            except Exception:
                                pass
            except Exception:
                # Silently fail plot updates to prevent UI freeze
                pass
        
        # STATISTICS UPDATE
        if need_stats_update:
            self.update_statistics()
        
    def update_statistics(self):
        """Update statistics panel - LIGHTWEIGHT"""
        if len(self.msg_rate_data) < 2:
            return
        
        try:
            # Calculate statistics with minimal allocations
            msg_rates = list(self.msg_rate_data)
            bandwidths = list(self.bandwidth_data)
            cpu = list(self.cpu_data)
            memory = list(self.memory_data)
            
            # Use Python built-in functions instead of numpy for lighter computation
            max_msg_rate = max(msg_rates) if msg_rates else 0
            avg_msg_rate = sum(msg_rates) / len(msg_rates) if msg_rates else 0
            
            max_bandwidth = max(bandwidths) if bandwidths else 0
            avg_bandwidth = sum(bandwidths) / len(bandwidths) if bandwidths else 0
            
            max_cpu = max(cpu) if cpu else 0
            avg_cpu = sum(cpu) / len(cpu) if cpu else 0
            
            max_memory = max(memory) if memory else 0
            avg_memory = sum(memory) / len(memory) if memory else 0
            
            # Update labels
            self.stats_labels['peak_msg_rate'].setText(f"{max_msg_rate:.1f} msg/s")
            self.stats_labels['avg_msg_rate'].setText(f"{avg_msg_rate:.1f} msg/s")
            self.stats_labels['peak_bandwidth'].setText(f"{max_bandwidth:.2f} MB/s")
            self.stats_labels['avg_bandwidth'].setText(f"{avg_bandwidth:.2f} MB/s")
            self.stats_labels['peak_cpu'].setText(f"{max_cpu:.1f}%")
            self.stats_labels['avg_cpu'].setText(f"{avg_cpu:.1f}%")
            self.stats_labels['peak_memory'].setText(f"{max_memory:.1f}%")
            self.stats_labels['avg_memory'].setText(f"{avg_memory:.1f}%")
        except Exception:
            # Silently fail to prevent UI freeze
            pass

    def _on_topic_count_ready(self, count: int):
        """Slot called when background topic-count fetch completes - LIGHTWEIGHT"""
        try:
            # Only update if CPU is not under heavy load
            if self.cpu_data and len(self.cpu_data) > 0:
                recent_cpu = float(list(self.cpu_data)[-1])
                if recent_cpu > 75.0:
                    # Skip updating topic count if CPU high to reduce load
                    return
            
            # Append latest topic count and keep buffer size
            self.topic_count_data.append(count)
        except Exception:
            pass
        
    def toggle_pause(self):
        """Toggle pause/resume"""
        self.paused = not self.paused
        if self.paused:
            self.pause_btn.setText("▶ Resume")
            self.status_label.setText("⏸ Paused")
            self.status_label.setStyleSheet("color: #FF9800; font-weight: bold;")
        else:
            self.pause_btn.setText("⏸ Pause")
            self.status_label.setText("📊 Monitoring Active")
            self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
            
    def reset(self):
        """Reset for new recording session - clears data and resets state"""
        # Clear data buffers (always safe)
        self.time_data.clear()
        self.msg_rate_data.clear()
        self.bandwidth_data.clear()
        self.topic_count_data.clear()
        self.cpu_data.clear()
        self.memory_data.clear()
        self.disk_write_data.clear()
        self.start_time = None
        self.update_counter = 0
        self.paused = not self.auto_pause  # Reset to default auto-pause state
        
        # Only update UI elements if they're already loaded (lazy loading)
        if self._charts_loaded:
            # Update plots with empty data
            self.msg_rate_plot.curve.setData([], [])
            self.bandwidth_plot.curve.setData([], [])
            self.topic_count_plot.curve.setData([], [])
            self.cpu_plot.curve.setData([], [])
            self.memory_plot.curve.setData([], [])
            self.disk_write_plot.curve.setData([], [])
            
            # Update button and labels
            self.pause_btn.setText("⏸ Pause")
            self.status_label.setText("📊 Monitoring Active")
            self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
            
            # Clear statistics
            for key in self.stats_labels:
                self.stats_labels[key].setText("0")
        
    def clear_charts(self):
        """Clear all chart data"""
        self.time_data.clear()
        self.msg_rate_data.clear()
        self.bandwidth_data.clear()
        self.topic_count_data.clear()
        self.cpu_data.clear()
        self.memory_data.clear()
        self.disk_write_data.clear()
        self.start_time = None
        
        # Update plots with empty data
        self.msg_rate_plot.curve.setData([], [])
        self.bandwidth_plot.curve.setData([], [])
        self.topic_count_plot.curve.setData([], [])
        self.cpu_plot.curve.setData([], [])
        self.memory_plot.curve.setData([], [])
        self.disk_write_plot.curve.setData([], [])
        
    def change_time_window(self, window):
        """Change time window for charts - with memory management limits"""
        windows = {
            "1 min": 60,
            "5 min": 300,
            "10 min": 600,
            "30 min": 1800
        }
        
        new_max = windows.get(window, 60)
        
        # OPTIMIZATION: Enforce reasonable limits for memory management
        # Different limits based on typical update intervals
        # At 300-500ms update intervals:
        # - 60 points = ~20-30 seconds
        # - 300 points = ~100-150 seconds (~2 minutes)
        # - 600 points = ~200-300 seconds (~4-5 minutes)
        # - 1800 points = ~600-900 seconds (~10-15 minutes)
        
        # Cap at reasonable maximum for memory efficiency
        MAX_BUFFER_SIZE = 2000  # Hard limit for memory safety
        if new_max > MAX_BUFFER_SIZE:
            new_max = MAX_BUFFER_SIZE
        
        # Update buffer sizes
        self.max_points = new_max
        
        # Recreate deques with new max length (preserves existing data up to new limit)
        self.time_data = deque(self.time_data, maxlen=new_max)
        self.msg_rate_data = deque(self.msg_rate_data, maxlen=new_max)
        self.bandwidth_data = deque(self.bandwidth_data, maxlen=new_max)
        self.topic_count_data = deque(self.topic_count_data, maxlen=new_max)
        self.cpu_data = deque(self.cpu_data, maxlen=new_max)
        self.memory_data = deque(self.memory_data, maxlen=new_max)
        self.disk_write_data = deque(self.disk_write_data, maxlen=new_max)
        
    def export_charts(self):
        """Export charts as images"""
        from PyQt5.QtWidgets import QFileDialog
        import os
        
        directory = QFileDialog.getExistingDirectory(
            self,
            "Select Directory to Save Charts",
            os.path.expanduser("~")
        )
        
        if directory:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            
            # Export each chart
            plots = [
                (self.msg_rate_plot, "message_rate"),
                (self.bandwidth_plot, "bandwidth"),
                (self.topic_count_plot, "topic_count"),
                (self.cpu_plot, "cpu_usage"),
                (self.memory_plot, "memory_usage"),
                (self.disk_write_plot, "disk_write")
            ]
            
            try:
                from pyqtgraph.exporters import ImageExporter
                for plot, name in plots:
                    filename = os.path.join(directory, f"{name}_{timestamp}.png")
                    exporter = ImageExporter(plot.plotItem)
                    exporter.parameters()['width'] = 1920
                    exporter.export(filename)
                
                self.status_label.setText(f"✅ Charts exported to {directory}")
                self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
            except ImportError:
                # Fallback: use Qt screenshot
                for plot, name in plots:
                    filename = os.path.join(directory, f"{name}_{timestamp}.png")
                    plot.grab().save(filename)
                    
                self.status_label.setText(f"✅ Charts saved to {directory}")
                self.status_label.setStyleSheet("color: #4CAF50; font-weight: bold;")
