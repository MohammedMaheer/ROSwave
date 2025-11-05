"""
Network Upload Widget - UI for network upload monitoring and control
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget,  # type: ignore
                             QTableWidgetItem, QPushButton, QGroupBox, QHeaderView,
                             QLabel, QLineEdit, QCheckBox, QProgressBar, QSpinBox,
                             QMessageBox, QFileDialog)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal  # type: ignore
from PyQt5.QtGui import QColor, QFont, QTextCursor  # type: ignore
from datetime import datetime
import os
import time

# Import authentication manager
try:
    from core.auth_manager import AuthenticationManager
except ImportError:
    AuthenticationManager = None  # Fallback if not available


class NetworkUploadWidget(QWidget):
    """Widget for monitoring and controlling network uploads"""
    
    upload_requested = pyqtSignal(str, int, dict)  # file_path, priority, metadata
    
    def __init__(self, network_manager):
        super().__init__()
        self.network_manager = network_manager
        
        # Initialize authentication manager if available
        self.auth_manager = AuthenticationManager() if AuthenticationManager else None
        
        self.init_ui()
        
        # Setup update timer
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_display)  # type: ignore
        self.update_timer.start(2000)  # Update every 2 seconds
        
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()
        
        # Network Status Group
        status_group = QGroupBox("Network Status & Authentication")
        status_layout = QHBoxLayout()
        
        self.connection_label = QLabel("● Offline")
        self.connection_label.setStyleSheet("color: red; font-weight: bold;")
        status_layout.addWidget(self.connection_label)
        
        # Auth status indicator
        self.auth_status_label = QLabel("🔓 No Auth")
        self.auth_status_label.setStyleSheet("color: gray; font-weight: bold;")
        status_layout.addWidget(self.auth_status_label)
        
        status_layout.addStretch()
        
        self.server_url_input = QLineEdit()
        self.server_url_input.setPlaceholderText("http://localhost:8080/upload")
        self.server_url_input.setText(self.network_manager.upload_url)
        self.server_url_input.setMinimumWidth(300)
        status_layout.addWidget(QLabel("Server URL:"))
        status_layout.addWidget(self.server_url_input)
        
        # API token/key input
        status_layout.addWidget(QLabel("API Key:"))
        self.api_key_input = QLineEdit()
        self.api_key_input.setEchoMode(QLineEdit.Password)
        self.api_key_input.setPlaceholderText("Optional API token (if auth enabled)")
        self.api_key_input.setMaximumWidth(200)
        status_layout.addWidget(self.api_key_input)
        
        update_url_btn = QPushButton("Update")
        update_url_btn.clicked.connect(self.update_server_url)
        status_layout.addWidget(update_url_btn)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # Statistics Group
        stats_group = QGroupBox("Upload Statistics")
        stats_layout = QHBoxLayout()
        
        # Create stat labels
        self.uploaded_label = QLabel("Uploaded: 0")
        self.failed_label = QLabel("Failed: 0")
        self.pending_label = QLabel("Pending: 0")
        self.bytes_label = QLabel("Bytes: 0 MB")
        
        stats_layout.addWidget(self.uploaded_label)
        stats_layout.addWidget(QLabel("|"))
        stats_layout.addWidget(self.failed_label)
        stats_layout.addWidget(QLabel("|"))
        stats_layout.addWidget(self.pending_label)
        stats_layout.addWidget(QLabel("|"))
        stats_layout.addWidget(self.bytes_label)
        stats_layout.addStretch()
        
        stats_group.setLayout(stats_layout)
        layout.addWidget(stats_group)
        
        # Upload Control Group
        control_group = QGroupBox("Upload Control")
        control_layout = QVBoxLayout()
        
        # Auto-upload settings
        auto_layout = QHBoxLayout()
        self.auto_upload_check = QCheckBox("Auto-upload completed recordings")
        self.auto_upload_check.setChecked(True)
        auto_layout.addWidget(self.auto_upload_check)
        
        # Compression option
        self.compression_check = QCheckBox("Compress before upload")
        self.compression_check.setChecked(False)
        auto_layout.addWidget(self.compression_check)
        
        auto_layout.addWidget(QLabel("Priority:"))
        self.priority_spin = QSpinBox()
        self.priority_spin.setMinimum(1)
        self.priority_spin.setMaximum(10)
        self.priority_spin.setValue(5)
        auto_layout.addWidget(self.priority_spin)
        
        # Max retries
        auto_layout.addWidget(QLabel("Max Retries:"))
        self.retries_spin = QSpinBox()
        self.retries_spin.setMinimum(0)
        self.retries_spin.setMaximum(10)
        self.retries_spin.setValue(3)
        auto_layout.addWidget(self.retries_spin)
        
        auto_layout.addStretch()
        
        manual_upload_btn = QPushButton("Upload File...")
        manual_upload_btn.clicked.connect(self.manual_upload)
        auto_layout.addWidget(manual_upload_btn)
        
        control_layout.addLayout(auto_layout)
        
        # Batch upload settings
        batch_layout = QHBoxLayout()
        
        batch_layout.addWidget(QLabel("Batch Upload:"))
        self.batch_size_spin = QSpinBox()
        self.batch_size_spin.setMinimum(1)
        self.batch_size_spin.setMaximum(100)
        self.batch_size_spin.setValue(5)
        batch_layout.addWidget(self.batch_size_spin)
        batch_layout.addWidget(QLabel("bags at a time"))
        
        batch_upload_btn = QPushButton("📦 Batch Upload All")
        batch_upload_btn.clicked.connect(self.batch_upload_all)
        batch_layout.addWidget(batch_upload_btn)
        
        batch_layout.addStretch()
        
        control_layout.addLayout(batch_layout)
        
        # Metadata tags
        tags_layout = QHBoxLayout()
        tags_layout.addWidget(QLabel("Metadata Tags:"))
        self.tags_input = QLineEdit()
        self.tags_input.setPlaceholderText("e.g., robot:husky, env:warehouse")
        tags_layout.addWidget(self.tags_input)
        control_layout.addLayout(tags_layout)
        
        control_group.setLayout(control_layout)
        layout.addWidget(control_group)
        
        # Pending Uploads Table
        pending_group = QGroupBox("Pending Uploads")
        pending_layout = QVBoxLayout()
        
        self.pending_table = QTableWidget()
        self.pending_table.setColumnCount(6)
        self.pending_table.setHorizontalHeaderLabels([
            "File", "Priority", "Status", "Progress", "Size (MB)", "Retries"
        ])
        
        header = self.pending_table.horizontalHeader()
        if header:
            header.setSectionResizeMode(0, QHeaderView.Stretch)
            header.setSectionResizeMode(1, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(3, QHeaderView.Fixed)
            header.setMinimumSectionSize(120)
            header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(5, QHeaderView.ResizeToContents)
        
        pending_layout.addWidget(self.pending_table)
        
        # Control buttons
        btn_layout = QHBoxLayout()
        
        refresh_btn = QPushButton("Refresh")
        refresh_btn.clicked.connect(self.update_display)
        btn_layout.addWidget(refresh_btn)
        
        btn_layout.addStretch()
        
        retry_all_btn = QPushButton("Retry All Failed")
        retry_all_btn.clicked.connect(self.retry_all_failed)
        btn_layout.addWidget(retry_all_btn)
        
        clear_completed_btn = QPushButton("Clear Completed")
        clear_completed_btn.clicked.connect(self.clear_completed)
        btn_layout.addWidget(clear_completed_btn)
        
        pending_layout.addLayout(btn_layout)
        
        pending_group.setLayout(pending_layout)
        layout.addWidget(pending_group)
        
        # Upload History
        history_group = QGroupBox("Recent Upload History")
        history_layout = QVBoxLayout()
        
        self.history_table = QTableWidget()
        self.history_table.setColumnCount(5)
        self.history_table.setHorizontalHeaderLabels([
            "File", "Status", "Completed", "Size (MB)", "Duration (s)"
        ])
        self.history_table.setMaximumHeight(150)
        
        history_header = self.history_table.horizontalHeader()
        if history_header:
            history_header.setSectionResizeMode(0, QHeaderView.Stretch)
        
        history_layout.addWidget(self.history_table)
        
        history_group.setLayout(history_layout)
        layout.addWidget(history_group)
        
        self.setLayout(layout)
        
        # Initial update
        self.update_display()
        
    def update_display(self):
        """Update the display with current data"""
        try:
            # Check if network manager is available
            if not self.network_manager:
                self.connection_label.setText("● Not Initialized")
                self.connection_label.setStyleSheet("color: gray; font-weight: bold;")
                return
                
            # Update connection status
            if self.network_manager.is_online:
                self.connection_label.setText("● Online")
                self.connection_label.setStyleSheet("color: green; font-weight: bold;")
            else:
                self.connection_label.setText("● Offline")
                self.connection_label.setStyleSheet("color: red; font-weight: bold;")
                
            # Update statistics
            stats = self.network_manager.get_stats()
            self.uploaded_label.setText(f"Uploaded: {stats['total_uploaded']}")
            self.failed_label.setText(f"Failed: {stats['total_failed']}")
            self.pending_label.setText(f"Pending: {stats['queued_uploads']}")
            self.bytes_label.setText(f"Bytes: {stats['bytes_uploaded'] / (1024*1024):.2f} MB")
            
            # Update pending uploads table
            pending = self.network_manager.get_pending_uploads()
            self.pending_table.setRowCount(len(pending))
            
            for idx, upload in enumerate(pending):
                # File name
                filename = os.path.basename(upload['file_path'])
                file_item = QTableWidgetItem(filename)
                self.pending_table.setItem(idx, 0, file_item)
                
                # Priority
                priority_item = QTableWidgetItem(str(upload['priority']))
                priority_item.setTextAlignment(4)  # Qt.AlignCenter = 4
                self.pending_table.setItem(idx, 1, priority_item)
                
                # Status
                status_item = QTableWidgetItem(upload['status'].upper())
                status_item.setTextAlignment(4)  # Qt.AlignCenter = 4
                
                if upload['status'] == 'completed':
                    status_item.setForeground(QColor('green'))
                elif upload['status'] == 'failed':
                    status_item.setForeground(QColor('red'))
                elif upload['status'] == 'uploading':
                    status_item.setForeground(QColor('blue'))
                else:
                    status_item.setForeground(QColor('orange'))
                    
                self.pending_table.setItem(idx, 2, status_item)
                
                # Progress bar
                progress_bar = QProgressBar()
                progress_bar.setMinimum(0)
                progress_bar.setMaximum(100)
                progress_bar.setValue(int(upload['progress']))
                progress_bar.setFormat(f"{upload['progress']:.1f}%")
                self.pending_table.setCellWidget(idx, 3, progress_bar)
                
                # File size
                size_mb = upload['file_size'] / (1024*1024) if upload['file_size'] else 0
                size_item = QTableWidgetItem(f"{size_mb:.2f}")
                size_item.setTextAlignment(2 | 128)  # Qt.AlignRight | Qt.AlignVCenter = 2 | 128
                self.pending_table.setItem(idx, 4, size_item)
                
                # Retry count
                retry_item = QTableWidgetItem(str(upload['retry_count']))
                retry_item.setTextAlignment(4)  # Qt.AlignCenter = 4
                if upload['retry_count'] > 0:
                    retry_item.setForeground(QColor('orange'))
                self.pending_table.setItem(idx, 5, retry_item)
                
            # Update history table
            history = self.network_manager.get_upload_history(limit=10)
            self.history_table.setRowCount(len(history))
            
            for idx, item in enumerate(history):
                # File name
                filename = os.path.basename(item['file_path'])
                file_item = QTableWidgetItem(filename)
                self.history_table.setItem(idx, 0, file_item)
                
                # Status
                status_item = QTableWidgetItem(item['status'].upper())
                status_item.setTextAlignment(4)  # Qt.AlignCenter = 4
                if item['status'] == 'completed':
                    status_item.setForeground(QColor('green'))
                else:
                    status_item.setForeground(QColor('red'))
                self.history_table.setItem(idx, 1, status_item)
                
                # Completed time
                time_item = QTableWidgetItem(item['completed_at'])
                self.history_table.setItem(idx, 2, time_item)
                
                # Size
                size_mb = item['file_size'] / (1024*1024) if item['file_size'] else 0
                size_item = QTableWidgetItem(f"{size_mb:.2f}")
                size_item.setTextAlignment(2 | 128)  # Qt.AlignRight | Qt.AlignVCenter = 2 | 128
                self.history_table.setItem(idx, 3, size_item)
                
                # Duration
                duration = item.get('upload_duration', 0) or 0
                duration_item = QTableWidgetItem(f"{duration:.1f}")
                duration_item.setTextAlignment(2 | 128)  # Qt.AlignRight | Qt.AlignVCenter = 2 | 128
                self.history_table.setItem(idx, 4, duration_item)
                
        except Exception as e:
            print(f"Error updating network display: {e}")
            
    def update_server_url(self):
        """Update the server URL"""
        new_url = self.server_url_input.text()
        if new_url:
            self.network_manager.upload_url = new_url
            QMessageBox.information(self, "Success", f"Server URL updated to: {new_url}")
    
    def update_auth_status_display(self):
        """Update authentication status indicator"""
        if not self.auth_manager:
            self.auth_status_label.setText("🔓 No Auth Module")
            self.auth_status_label.setStyleSheet("color: gray; font-weight: bold;")
            return
        
        try:
            status = self.auth_manager.get_status()
            api_key = self.api_key_input.text().strip()
            
            if status['enabled']:
                if api_key:
                    # Verify the token
                    is_valid, message = self.auth_manager.verify_token(api_key)
                    if is_valid:
                        self.auth_status_label.setText("🔒 Auth Valid")
                        self.auth_status_label.setStyleSheet("color: green; font-weight: bold;")
                    else:
                        self.auth_status_label.setText("🔐 Auth Invalid")
                        self.auth_status_label.setStyleSheet("color: red; font-weight: bold;")
                else:
                    self.auth_status_label.setText("🔐 Auth Required")
                    self.auth_status_label.setStyleSheet("color: orange; font-weight: bold;")
            else:
                self.auth_status_label.setText("🔓 Auth Disabled")
                self.auth_status_label.setStyleSheet("color: gray; font-weight: bold;")
        except Exception as e:
            self.auth_status_label.setText(f"⚠️ Error: {str(e)[:20]}")
            self.auth_status_label.setStyleSheet("color: red; font-weight: bold;")
            
    def manual_upload(self):
        """Manually select a file to upload"""
        # Check authentication if enabled
        if self.auth_manager:
            status = self.auth_manager.get_status()
            if status['enabled']:
                api_key = self.api_key_input.text().strip()
                if not api_key:
                    QMessageBox.warning(self, "Authentication Required", 
                                      "Authentication is enabled. Please enter a valid API key.")
                    return
                
                is_valid, message = self.auth_manager.verify_token(api_key)
                if not is_valid:
                    QMessageBox.warning(self, "Invalid API Key", f"Authentication failed: {message}")
                    return
                
                # Check rate limit
                can_upload, limit_msg = self.auth_manager.check_rate_limit(api_key)
                if not can_upload:
                    QMessageBox.warning(self, "Rate Limit Exceeded", 
                                      f"Rate limit exceeded: {limit_msg}")
                    return
        
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Select File to Upload", "", "All Files (*.*)"
        )
        
        if file_path:
            priority = self.priority_spin.value()
            metadata = {
                'source': 'manual_upload',
                'timestamp': str(int(time.time()))
            }
            
            # Add API token to metadata if provided
            if self.auth_manager and self.api_key_input.text().strip():
                metadata['api_token'] = self.api_key_input.text().strip()
            
            self.network_manager.add_upload(file_path, priority, metadata)
            QMessageBox.information(self, "Success", f"File added to upload queue: {os.path.basename(file_path)}")
            self.update_display()
            
    def add_recording_upload(self, bag_path, metadata=None):
        """Add a recording to upload queue"""
        if not self.network_manager:
            print("Network manager not ready yet")
            return
            
        if self.auto_upload_check.isChecked():
            priority = self.priority_spin.value()
            upload_metadata = metadata or {}
            upload_metadata['source'] = 'ros2_recording'
            
            self.network_manager.add_upload(bag_path, priority, upload_metadata)
            
    def retry_all_failed(self):
        """Retry all failed uploads"""
        try:
            pending = self.network_manager.get_pending_uploads()
            failed_uploads = [u for u in pending if u['status'] == 'failed']
            
            if not failed_uploads:
                QMessageBox.information(self, "No Failed Uploads", "There are no failed uploads to retry")
                return
            
            retry_count = 0
            for upload in failed_uploads:
                # Reset status to pending so it will be retried
                self.network_manager.retry_upload(upload['file_path'])
                retry_count += 1
            
            QMessageBox.information(self, "Success", f"Queued {retry_count} failed upload(s) for retry")
            self.update_display()
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to retry uploads: {str(e)}")
        
    def clear_completed(self):
        """Clear completed uploads from database"""
        try:
            pending = self.network_manager.get_pending_uploads()
            completed_uploads = [u for u in pending if u['status'] == 'completed']
            
            if not completed_uploads:
                QMessageBox.information(self, "No Completed Uploads", "There are no completed uploads to clear")
                return
            
            cleared_count = 0
            for upload in completed_uploads:
                # Remove completed upload from tracking
                self.network_manager.clear_upload(upload['file_path'])
                cleared_count += 1
            
            QMessageBox.information(self, "Success", f"Cleared {cleared_count} completed upload(s) from the list")
            self.update_display()
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to clear completed uploads: {str(e)}")
    
    def batch_upload_all(self):
        """Upload all pending bags in batches"""
        # Check authentication if enabled
        if self.auth_manager:
            status = self.auth_manager.get_status()
            if status['enabled']:
                api_key = self.api_key_input.text().strip()
                if not api_key:
                    QMessageBox.warning(self, "Authentication Required", 
                                      "Authentication is enabled. Please enter a valid API key.")
                    return
                
                is_valid, message = self.auth_manager.verify_token(api_key)
                if not is_valid:
                    QMessageBox.warning(self, "Invalid API Key", f"Authentication failed: {message}")
                    return
                
                # Check rate limit
                can_upload, limit_msg = self.auth_manager.check_rate_limit(api_key)
                if not can_upload:
                    QMessageBox.warning(self, "Rate Limit Exceeded", 
                                      f"Rate limit exceeded: {limit_msg}")
                    return
        
        try:
            pending = self.network_manager.get_pending_uploads()
            pending_uploads = [u for u in pending if u['status'] in ['pending', 'failed']]
            
            if not pending_uploads:
                QMessageBox.information(self, "No Pending Uploads", "There are no pending uploads")
                return
            
            batch_size = self.batch_size_spin.value()
            max_retries = self.retries_spin.value()
            tags = self.tags_input.text()
            compress = self.compression_check.isChecked()
            
            # Group uploads into batches
            batches = [pending_uploads[i:i + batch_size] for i in range(0, len(pending_uploads), batch_size)]
            
            QMessageBox.information(
                self, 
                "Batch Upload Started", 
                f"Starting batch upload of {len(pending_uploads)} recording(s) in {len(batches)} batch(es)\n"
                f"Batch size: {batch_size}\n"
                f"Max retries: {max_retries}\n"
                f"Compression: {'Enabled' if compress else 'Disabled'}\n"
                f"Metadata tags: {tags if tags else 'None'}"
            )
            
            # Send all uploads with metadata
            for upload in pending_uploads:
                metadata = {
                    'tags': tags,
                    'compressed': compress,
                    'max_retries': max_retries,
                    'batch_upload': True,
                    'timestamp': str(int(time.time()))
                }
                
                # Add API token if auth is enabled
                if self.auth_manager and self.api_key_input.text().strip():
                    metadata['api_token'] = self.api_key_input.text().strip()
                
                self.network_manager.update_upload_metadata(upload['file_path'], metadata)
            
            # Update display
            self.update_display()
            
        except Exception as e:
            QMessageBox.warning(self, "Error", f"Failed to start batch upload: {str(e)}")
