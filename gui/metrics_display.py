"""
Metrics Display Widget - displays recording metrics and statistics
"""

from typing import Dict, Optional, Union

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,
                             QGroupBox, QGridLayout, QProgressBar)
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont


class MetricsDisplayWidget(QWidget):
    """Widget for displaying recording metrics"""
    
    def __init__(self, metrics_collector):
        super().__init__()
        self.metrics_collector = metrics_collector
        self.init_ui()
        
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()
        
        # Group box
        group = QGroupBox("Recording Metrics & Statistics")
        group_layout = QGridLayout()
        
        # Create metric labels
        label_font = QFont()
        label_font.setBold(True)
        
        value_font = QFont()
        value_font.setPointSize(11)
        
        row = 0
        
        # Recording duration
        duration_label = QLabel("Duration:")
        duration_label.setFont(label_font)
        group_layout.addWidget(duration_label, row, 0)
        
        self.duration_value = QLabel("00:00:00")
        self.duration_value.setFont(value_font)
        self.duration_value.setStyleSheet("color: #2196F3;")
        group_layout.addWidget(self.duration_value, row, 1)
        
        row += 1
        
        # Data size
        size_label = QLabel("Total Size:")
        size_label.setFont(label_font)
        group_layout.addWidget(size_label, row, 0)
        
        self.size_value = QLabel("0 MB")
        self.size_value.setFont(value_font)
        self.size_value.setStyleSheet("color: #4CAF50;")
        group_layout.addWidget(self.size_value, row, 1)
        
        row += 1
        
        # Write speed
        speed_label = QLabel("Write Speed:")
        speed_label.setFont(label_font)
        group_layout.addWidget(speed_label, row, 0)
        
        self.speed_value = QLabel("0 MB/s")
        self.speed_value.setFont(value_font)
        self.speed_value.setStyleSheet("color: #FF9800;")
        group_layout.addWidget(self.speed_value, row, 1)
        
        row += 1
        
        # Message count
        messages_label = QLabel("Messages:")
        messages_label.setFont(label_font)
        group_layout.addWidget(messages_label, row, 0)
        
        self.messages_value = QLabel("0")
        self.messages_value.setFont(value_font)
        group_layout.addWidget(self.messages_value, row, 1)
        
        row += 1
        
        # Topics being recorded
        topics_label = QLabel("Active Topics:")
        topics_label.setFont(label_font)
        group_layout.addWidget(topics_label, row, 0)
        
        self.topics_value = QLabel("0")
        self.topics_value.setFont(value_font)
        group_layout.addWidget(self.topics_value, row, 1)
        
        row += 1
        
        # Current message rate (real-time)
        rate_label = QLabel("Current Rate:")
        rate_label.setFont(label_font)
        group_layout.addWidget(rate_label, row, 0)
        
        self.rate_value = QLabel("0 msg/s")
        self.rate_value.setFont(value_font)
        group_layout.addWidget(self.rate_value, row, 1)
        
        row += 1
        
        # Disk usage indicator
        disk_label = QLabel("Disk Usage:")
        disk_label.setFont(label_font)
        group_layout.addWidget(disk_label, row, 0)
        
        self.disk_progress = QProgressBar()
        self.disk_progress.setMaximum(100)
        self.disk_progress.setValue(0)
        self.disk_progress.setTextVisible(True)
        self.disk_progress.setFormat("%v%")
        group_layout.addWidget(self.disk_progress, row, 1)
        
        group.setLayout(group_layout)
        layout.addWidget(group)
        self.setLayout(layout)
        
    def update_display(self, metrics: Optional[Dict[str, Union[int, float]]] = None):
        """Update the metrics display with provided metrics or fetch fresh copy."""
        if metrics is None:
            metrics = self.metrics_collector.get_metrics()

        metrics = dict(metrics or {})
        
        # Update duration
        duration = metrics.get('duration', 0)
        hours = int(duration // 3600)
        minutes = int((duration % 3600) // 60)
        seconds = int(duration % 60)
        self.duration_value.setText(f"{hours:02d}:{minutes:02d}:{seconds:02d}")
        
        # Update size
        size_mb = metrics.get('size_mb', 0)
        if size_mb >= 1024:
            self.size_value.setText(f"{size_mb/1024:.2f} GB")
        else:
            self.size_value.setText(f"{size_mb:.2f} MB")
            
        # Update write speed
        speed = metrics.get('write_speed_mb_s', 0)
        self.speed_value.setText(f"{speed:.2f} MB/s")
        
        # Update message count
        msg_count = metrics.get('message_count', 0)
        self.messages_value.setText(f"{msg_count:,}")
        
        # Update topic count
        topic_count = metrics.get('topic_count', 0)
        self.topics_value.setText(str(topic_count))
        
        # Update message rate
        msg_rate = metrics.get('message_rate', 0)
        self.rate_value.setText(f"{msg_rate:.1f} msg/s")
        
        # Update disk usage
        disk_usage = metrics.get('disk_usage_percent', 0)
        self.disk_progress.setValue(int(disk_usage))
        
        # Change color based on disk usage
        if disk_usage > 90:
            self.disk_progress.setStyleSheet("QProgressBar::chunk { background-color: #f44336; }")
        elif disk_usage > 75:
            self.disk_progress.setStyleSheet("QProgressBar::chunk { background-color: #FF9800; }")
        else:
            self.disk_progress.setStyleSheet("QProgressBar::chunk { background-color: #4CAF50; }")
