"""
Advanced Stats Widget - displays advanced ROS2 statistics
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QLabel,  # type: ignore
                             QGroupBox, QGridLayout, QPushButton)
from PyQt5.QtCore import Qt, QTimer  # type: ignore
from PyQt5.QtGui import QFont  # type: ignore
import psutil


class AdvancedStatsWidget(QWidget):
    """Widget for displaying advanced system and ROS2 statistics"""
    
    def __init__(self, ros2_manager, metrics_collector=None):
        super().__init__()
        self.ros2_manager = ros2_manager
        self.metrics_collector = metrics_collector
        self.init_ui()
        
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()
        
        # System Resources Group
        system_group = QGroupBox("System Resources")
        system_layout = QGridLayout()
        
        label_font = QFont()
        label_font.setBold(True)
        
        value_font = QFont()
        value_font.setPointSize(10)
        
        row = 0
        
        # CPU Usage
        cpu_label = QLabel("CPU Usage:")
        cpu_label.setFont(label_font)
        system_layout.addWidget(cpu_label, row, 0)
        
        self.cpu_value = QLabel("0%")
        self.cpu_value.setFont(value_font)
        system_layout.addWidget(self.cpu_value, row, 1)
        
        row += 1
        
        # Memory Usage
        mem_label = QLabel("Memory Usage:")
        mem_label.setFont(label_font)
        system_layout.addWidget(mem_label, row, 0)
        
        self.mem_value = QLabel("0 MB / 0 MB")
        self.mem_value.setFont(value_font)
        system_layout.addWidget(self.mem_value, row, 1)
        
        row += 1
        
        # Disk I/O
        disk_io_label = QLabel("Disk Write Speed:")
        disk_io_label.setFont(label_font)
        system_layout.addWidget(disk_io_label, row, 0)
        
        self.disk_io_value = QLabel("0 MB/s")
        self.disk_io_value.setFont(value_font)
        system_layout.addWidget(self.disk_io_value, row, 1)
        
        row += 1
        
        # Network I/O
        net_label = QLabel("Network I/O:")
        net_label.setFont(label_font)
        system_layout.addWidget(net_label, row, 0)
        
        self.net_value = QLabel("↑ 0 KB/s ↓ 0 KB/s")
        self.net_value.setFont(value_font)
        system_layout.addWidget(self.net_value, row, 1)
        
        system_group.setLayout(system_layout)
        layout.addWidget(system_group)
        
        # ROS2 Stats Group
        ros_group = QGroupBox("ROS2 Environment")
        ros_layout = QGridLayout()
        
        row = 0
        
        # ROS2 Distro
        distro_label = QLabel("ROS2 Distro:")
        distro_label.setFont(label_font)
        ros_layout.addWidget(distro_label, row, 0)
        
        self.distro_value = QLabel("Unknown")
        self.distro_value.setFont(value_font)
        ros_layout.addWidget(self.distro_value, row, 1)
        
        row += 1
        
        # Domain ID
        domain_label = QLabel("Domain ID:")
        domain_label.setFont(label_font)
        ros_layout.addWidget(domain_label, row, 0)
        
        self.domain_value = QLabel("0")
        self.domain_value.setFont(value_font)
        ros_layout.addWidget(self.domain_value, row, 1)
        
        row += 1
        
        # Total Topics
        topics_label = QLabel("Total Topics:")
        topics_label.setFont(label_font)
        ros_layout.addWidget(topics_label, row, 0)
        
        self.topics_value = QLabel("0")
        self.topics_value.setFont(value_font)
        ros_layout.addWidget(self.topics_value, row, 1)
        
        row += 1
        
        # Total Nodes
        nodes_label = QLabel("Total Nodes:")
        nodes_label.setFont(label_font)
        ros_layout.addWidget(nodes_label, row, 0)
        
        self.nodes_value = QLabel("0")
        self.nodes_value.setFont(value_font)
        ros_layout.addWidget(self.nodes_value, row, 1)
        
        ros_group.setLayout(ros_layout)
        layout.addWidget(ros_group)
        
        # Refresh button
        refresh_btn = QPushButton("Refresh Stats")
        refresh_btn.clicked.connect(self.refresh_stats)
        layout.addWidget(refresh_btn)
        
        layout.addStretch()
        self.setLayout(layout)
        
        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.refresh_stats)
        self.update_timer.start(2000)
        
        # Initial values
        self.last_disk_io = psutil.disk_io_counters()
        self.last_net_io = psutil.net_io_counters()
        
    def refresh_stats(self):
        """Refresh statistics"""
        try:
            # CPU Usage
            cpu_percent = psutil.cpu_percent(interval=0.1)
            self.cpu_value.setText(f"{cpu_percent:.1f}%")
            
            # Memory Usage
            mem = psutil.virtual_memory()
            mem_used_mb = mem.used / (1024 * 1024)
            mem_total_mb = mem.total / (1024 * 1024)
            self.mem_value.setText(f"{mem_used_mb:.0f} MB / {mem_total_mb:.0f} MB ({mem.percent:.1f}%)")
            
            # Disk I/O - show recording write speed when recording, otherwise system disk I/O
            if self.metrics_collector and self.ros2_manager.is_recording:
                # During recording: show actual bag write speed from metrics
                metrics = self.metrics_collector.get_metrics()
                write_speed = metrics.get('write_speed_mb_s', 0)
                self.disk_io_value.setText(f"{write_speed:.2f} MB/s")
                self.disk_io_value.setStyleSheet("color: #4CAF50; font-weight: bold;")  # Green when recording
            else:
                # When not recording: show system-wide disk writes
                disk_io = psutil.disk_io_counters()
                if self.last_disk_io:
                    write_bytes = disk_io.write_bytes - self.last_disk_io.write_bytes
                    write_mb_s = write_bytes / (1024 * 1024 * 2)  # 2 second interval
                    self.disk_io_value.setText(f"{write_mb_s:.2f} MB/s")
                    self.disk_io_value.setStyleSheet("")  # Reset style
                self.last_disk_io = disk_io
            
            # Network I/O
            net_io = psutil.net_io_counters()
            if self.last_net_io:
                sent_bytes = net_io.bytes_sent - self.last_net_io.bytes_sent
                recv_bytes = net_io.bytes_recv - self.last_net_io.bytes_recv
                sent_kb_s = sent_bytes / (1024 * 2)
                recv_kb_s = recv_bytes / (1024 * 2)
                self.net_value.setText(f"↑ {sent_kb_s:.1f} KB/s ↓ {recv_kb_s:.1f} KB/s")
            self.last_net_io = net_io
            
            # ROS2 Info
            import os
            ros_distro = os.environ.get('ROS_DISTRO', 'Not Set')
            self.distro_value.setText(ros_distro)
            
            domain_id = os.environ.get('ROS_DOMAIN_ID', '0')
            self.domain_value.setText(domain_id)
            
            # Topic and Node counts
            topics = self.ros2_manager.get_topics_info()
            self.topics_value.setText(str(len(topics)))
            
            nodes = self.ros2_manager.get_nodes_info()
            self.nodes_value.setText(str(len(nodes)))
            
        except Exception as e:
            print(f"Error refreshing stats: {e}")
