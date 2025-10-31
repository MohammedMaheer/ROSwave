"""
Live Charts Widget - Real-time visualization of recording metrics and system stats
High-performance plotting using pyqtgraph for zero-latency updates
"""

import pyqtgraph as pg
from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,
                             QLabel, QComboBox, QGroupBox, QGridLayout, QCheckBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
import numpy as np
from collections import deque
from datetime import datetime


class LiveChartsWidget(QWidget):
    """Widget for real-time data visualization with multiple charts"""
    
    def __init__(self, metrics_collector, ros2_manager, buffer_size=60, update_interval=1000, auto_pause=True):
        super().__init__()
        self.metrics_collector = metrics_collector
        self.ros2_manager = ros2_manager
        
        # Adaptive performance settings
        self.buffer_size = buffer_size
        self.update_interval = update_interval
        self.auto_pause = auto_pause
        
        # Data buffers (adaptive size)
        self.max_points = buffer_size
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
        
        self.init_ui()
        self.setup_update_timer()
        
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()
        
        # Control panel
        controls = self.create_controls()
        layout.addWidget(controls)
        
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
        layout.addWidget(charts_group, 1)  # Give charts more space in main layout
        
        # Statistics panel
        stats = self.create_statistics_panel()
        layout.addWidget(stats)
        
        self.setLayout(layout)
        
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
        self.autoscale_check.setChecked(True)
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
        """Resume updates when tab becomes visible"""
        super().showEvent(event)
        if hasattr(self, 'update_timer') and self.auto_pause:
            self.update_timer.start(self.update_interval)
            self.paused = False
            self.pause_btn.setText("⏸ Pause")
            self.status_label.setText("📊 Monitoring Active")
            
    def hideEvent(self, event):
        """Pause updates when tab is hidden to save resources (if auto_pause enabled)"""
        super().hideEvent(event)
        if hasattr(self, 'update_timer') and self.auto_pause:
            self.update_timer.stop()
            self.paused = True
            self.pause_btn.setText("▶ Resume")
            self.status_label.setText("⏸ Paused (tab hidden)")
        
    def update_charts(self):
        """Update all charts with latest data - OPTIMIZED"""
        if self.paused:
            return
            
        self.update_counter += 1
            
        # Initialize start time if first update
        if self.start_time is None:
            self.start_time = datetime.now()
            
        # Calculate elapsed time
        elapsed = (datetime.now() - self.start_time).total_seconds()
        self.time_data.append(elapsed)
        
        # Get live metrics (includes system stats even when not recording)
        try:
            metrics = self.metrics_collector.get_live_metrics(self.ros2_manager)
        except:
            # If metrics fail, use zeros to keep charts updating
            metrics = {
                'message_rate': 0,
                'write_speed_mb_s': 0,
                'topic_count': 0,
                'cpu_percent': 0,
                'memory_percent': 0,
                'disk_write_speed': 0
            }
        
        # Recording metrics
        self.msg_rate_data.append(metrics.get('message_rate', 0))
        self.bandwidth_data.append(metrics.get('write_speed_mb_s', 0))
        self.topic_count_data.append(metrics.get('topic_count', 0))
        
        # System metrics
        self.cpu_data.append(metrics.get('cpu_percent', 0))
        self.memory_data.append(metrics.get('memory_percent', 0))
        self.disk_write_data.append(metrics.get('disk_write_speed', 0))
        
        # Update plots efficiently (only if we have data and every N updates for smoothness)
        if len(self.time_data) > 0 and self.update_counter % 2 == 0:  # Update display every 2 seconds
            time_array = np.array(list(self.time_data))
            
            self.msg_rate_plot.curve.setData(time_array, np.array(list(self.msg_rate_data)))
            self.bandwidth_plot.curve.setData(time_array, np.array(list(self.bandwidth_data)))
            self.topic_count_plot.curve.setData(time_array, np.array(list(self.topic_count_data)))
            self.cpu_plot.curve.setData(time_array, np.array(list(self.cpu_data)))
            self.memory_plot.curve.setData(time_array, np.array(list(self.memory_data)))
            self.disk_write_plot.curve.setData(time_array, np.array(list(self.disk_write_data)))
            
            # Auto-scale if enabled (batch operation)
            if self.autoscale_check.isChecked():
                for plot in [self.msg_rate_plot, self.bandwidth_plot, self.topic_count_plot,
                            self.cpu_plot, self.memory_plot, self.disk_write_plot]:
                    plot.enableAutoRange(enable=True)
        
        # Update statistics less frequently - every 10 updates (10 seconds)
        if self.update_counter % 10 == 0:
            self.update_statistics()
        
    def update_statistics(self):
        """Update statistics panel"""
        if len(self.msg_rate_data) == 0:
            return
            
        # Calculate statistics
        msg_rates = np.array(self.msg_rate_data)
        bandwidths = np.array(self.bandwidth_data)
        cpu = np.array(self.cpu_data)
        memory = np.array(self.memory_data)
        
        # Update labels
        self.stats_labels['peak_msg_rate'].setText(f"{np.max(msg_rates):.1f} msg/s")
        self.stats_labels['avg_msg_rate'].setText(f"{np.mean(msg_rates):.1f} msg/s")
        self.stats_labels['peak_bandwidth'].setText(f"{np.max(bandwidths):.2f} MB/s")
        self.stats_labels['avg_bandwidth'].setText(f"{np.mean(bandwidths):.2f} MB/s")
        self.stats_labels['peak_cpu'].setText(f"{np.max(cpu):.1f}%")
        self.stats_labels['avg_cpu'].setText(f"{np.mean(cpu):.1f}%")
        self.stats_labels['peak_memory'].setText(f"{np.max(memory):.1f}%")
        self.stats_labels['avg_memory'].setText(f"{np.mean(memory):.1f}%")
        
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
        """Change time window for charts"""
        windows = {
            "1 min": 60,
            "5 min": 300,
            "10 min": 600,
            "30 min": 1800
        }
        
        new_max = windows.get(window, 60)
        
        # Update buffer sizes
        self.max_points = new_max
        
        # Recreate deques with new max length
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
