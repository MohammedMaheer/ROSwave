# pyright: reportAttributeAccessIssue=false
"""
Recording Control Widget - controls for starting/stopping ROS2 bag recording
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QPushButton,  # type: ignore
                             QGroupBox, QLabel, QLineEdit, QFileDialog, QCheckBox,
                             QTableWidget, QTableWidgetItem, QHeaderView, QMessageBox)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer, QThread  # type: ignore
from PyQt5.QtGui import QFont, QColor
import os
from datetime import datetime
import threading
import time
import subprocess

# Import dynamic scaling for 1000+ topics support
try:
    from core.dynamic_hz_scaling import DynamicHzScaler
except ImportError:
    DynamicHzScaler = None  # Fallback if not available


class HzMonitorThread(QThread):
    """Background thread that monitors topic publishing rates"""
    
    hz_updated = pyqtSignal(dict)  # Emits {topic_name: hz_rate}
    
    def __init__(self, ros2_manager):
        super().__init__()
        self.ros2_manager = ros2_manager
        self.is_running = False
        self.topics_to_monitor = set()
        self._lock = threading.Lock()
    
    def set_topics(self, topics):
        """Set list of topics to monitor"""
        with self._lock:
            self.topics_to_monitor = set(topics)
    
    def run(self):
        """Monitor Hz of topics in background"""
        self.is_running = True
        consecutive_errors = 0
        # Use a ThreadPoolExecutor to parallelize subprocess calls and reduce overall latency
        from concurrent.futures import ThreadPoolExecutor, as_completed

        while self.is_running:
            try:
                with self._lock:
                    topics = list(self.topics_to_monitor)

                if not topics:
                    time.sleep(0.5)
                    continue

                hz_rates = {}

                # ⚡ DYNAMIC SCALING: Use DynamicHzScaler for 1000+ topic support
                if DynamicHzScaler:
                    config = DynamicHzScaler.calculate_optimal_workers(len(topics))
                    max_workers = config.get('max_workers', 8)
                else:
                    max_workers = min(8, max(1, len(topics)))
                
                with ThreadPoolExecutor(max_workers=max_workers) as executor:
                    future_map = {executor.submit(self._get_hz_for_topic, t): t for t in topics}
                    for future in as_completed(future_map, timeout=None):
                        topic_name = future_map[future]
                        try:
                            hz = future.result()
                        except Exception:
                            hz = 0.0
                        hz_rates[topic_name] = hz

                # Emit results
                if hz_rates:
                    self.hz_updated.emit(hz_rates)

                consecutive_errors = 0
                time.sleep(1.0)  # Update every second

            except Exception as e:
                consecutive_errors += 1
                if consecutive_errors > 5:
                    print(f"Hz monitor error: {e}")
                    consecutive_errors = 0
                time.sleep(0.5)
    
    def _get_hz_for_topic(self, topic_name):
        """Get Hz for a single topic with timeout"""
        try:
            # Use the advanced hz_monitor if available (much more accurate)
            if hasattr(self.ros2_manager, 'hz_monitor') and self.ros2_manager.hz_monitor:
                result = self.ros2_manager.hz_monitor.get_hz_smart(topic_name, use_cache=True)
                # get_hz_smart returns (hz, confidence) tuple - extract just the hz value
                if isinstance(result, tuple):
                    hz = result[0]
                else:
                    hz = result
                return hz
            
            # Fallback to old method if advanced monitor not available
            result = subprocess.run(
                ['ros2', 'topic', 'hz', topic_name],
                capture_output=True,
                text=True,
                timeout=0.15  # Very short timeout
            )
            
            # Parse output for Hz value
            lines = result.stdout.strip().split('\n')
            for line in reversed(lines):
                if 'average:' in line.lower():
                    try:
                        hz_str = line.split(':')[-1].replace('Hz', '').strip()
                        hz = float(hz_str)
                        return max(0, hz)
                    except (ValueError, IndexError):
                        pass
        except subprocess.TimeoutExpired:
            # Timeout means it's publishing, return 1+ Hz
            return 1.0
        except Exception:
            pass
        
        return 0.0
    
    def stop(self):
        """Stop the monitoring thread"""
        self.is_running = False
        self.wait()



class RecordingControlWidget(QWidget):
    """Widget for controlling ROS2 bag recording"""
    
    recording_started = pyqtSignal()
    recording_stopped = pyqtSignal()
    
    def __init__(self, ros2_manager, async_ros2_manager=None):
        super().__init__()
        self.ros2_manager = ros2_manager
        self.async_ros2_manager = async_ros2_manager
        self.is_recording = False
        self.current_bag_name = None
        self.current_bag_metadata = None
        self._last_rates_update = 0
        self._rates_update_cooldown = 5.0  # 5 seconds minimum - VERY infrequent updates to prevent blocking
        
        # Initialize Hz monitor thread (runs continuously in background)
        self.hz_monitor_thread = HzMonitorThread(ros2_manager)
        self.hz_monitor_thread.hz_updated.connect(self._on_hz_updated)
        self.hz_monitor_thread.start()
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()
        
        # Group box
        group = QGroupBox("Recording Control")
        group_layout = QVBoxLayout()
        
        # Output directory selection
        dir_layout = QHBoxLayout()
        dir_layout.addWidget(QLabel("Output Directory:"))
        
        self.dir_input = QLineEdit()
        default_dir = os.path.expanduser("~/ros2_recordings")
        self.dir_input.setText(default_dir)
        self.ros2_manager.set_output_directory(default_dir)
        dir_layout.addWidget(self.dir_input)
        
        browse_btn = QPushButton("Browse...")
        browse_btn.clicked.connect(self.browse_directory)
        dir_layout.addWidget(browse_btn)
        
        group_layout.addLayout(dir_layout)
        
        # Bag name
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Bag Name Prefix:"))
        
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("recording")
        self.name_input.setText("recording")
        name_layout.addWidget(self.name_input)
        
        group_layout.addLayout(name_layout)
        
        # Recording status
        self.status_label = QLabel("Status: Ready")
        status_font = QFont()
        status_font.setPointSize(12)
        status_font.setBold(True)
        self.status_label.setFont(status_font)
        self.status_label.setAlignment(Qt.AlignCenter)
        group_layout.addWidget(self.status_label)
        
        # Recording controls
        control_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("Start Recording")
        self.start_btn.setMinimumHeight(50)
        self.start_btn.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-size: 14px; font-weight: bold; }")
        self.start_btn.clicked.connect(self.start_recording)
        control_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("Stop Recording")
        self.stop_btn.setMinimumHeight(50)
        self.stop_btn.setStyleSheet("QPushButton { background-color: #f44336; color: white; font-size: 14px; font-weight: bold; }")
        self.stop_btn.clicked.connect(self.stop_recording)
        self.stop_btn.setEnabled(False)
        control_layout.addWidget(self.stop_btn)
        
        group_layout.addLayout(control_layout)
        
        # Current recording info
        self.recording_info = QLabel("")
        self.recording_info.setWordWrap(True)
        self.recording_info.setAlignment(Qt.AlignCenter)
        group_layout.addWidget(self.recording_info)
        
        group.setLayout(group_layout)
        layout.addWidget(group)
        
        # Selected Topics Monitor - LARGER
        topics_group = QGroupBox("📊 Selected Topics (Live Rates & Data Status)")
        topics_layout = QVBoxLayout()
        
        # Topics table with rates and message type
        self.selected_topics_table = QTableWidget()
        self.selected_topics_table.setColumnCount(5)
        self.selected_topics_table.setHorizontalHeaderLabels([
            'Topic Name', 'Message Type', 'Rate (Hz)', 'Data Status', 'Alert'
        ])
        header = self.selected_topics_table.horizontalHeader()
        if header is not None:
            header.setSectionResizeMode(0, QHeaderView.Stretch)
            header.setSectionResizeMode(1, QHeaderView.Stretch)
            header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
        
        # MUCH LARGER - 350 pixels instead of 200
        self.selected_topics_table.setMinimumHeight(350)
        self.selected_topics_table.setMaximumHeight(500)
        self.selected_topics_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self.selected_topics_table.setSelectionBehavior(QTableWidget.SelectRows)
        topics_layout.addWidget(self.selected_topics_table)
        
        # Info label
        self.topics_info_label = QLabel("No topics selected yet")
        self.topics_info_label.setStyleSheet("color: #666; font-style: italic;")
        topics_layout.addWidget(self.topics_info_label)
        
        topics_group.setLayout(topics_layout)
        layout.addWidget(topics_group)
        
        # Initialize topics tracking with message type and data presence
        self.selected_topics_data = {}  # {topic_name: {'rate': 0.0, 'msg_type': '', 'has_data': False, ...}}
        self.topic_rates_timer = QTimer()
        self.topic_rates_timer.timeout.connect(self.update_topic_rates)
        
        self.setLayout(layout)
        
    def browse_directory(self):
        """Open directory browser"""
        directory = QFileDialog.getExistingDirectory(
            self, "Select Output Directory", self.dir_input.text()
        )
        if directory:
            self.dir_input.setText(directory)
            self.ros2_manager.set_output_directory(directory)
            
    def start_recording(self, robot_metadata=None):
        """
        Start bag recording - OPTIMIZED for speed
        
        Args:
            robot_metadata: Optional dict with robot info (hostname, topics, etc.)
        """
        output_dir = self.dir_input.text()
        if not output_dir:
            QMessageBox.warning(self, "Warning", "Please specify an output directory")
            return
            
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)
        self.ros2_manager.set_output_directory(output_dir)
        
        # Generate bag name with timestamp
        prefix = self.name_input.text() or "recording"
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # If recording from discovered robot, include robot name
        if robot_metadata and 'hostname' in robot_metadata:
            robot_name = robot_metadata['hostname'].split('.')[0]  # Remove domain
            bag_name = f"{prefix}_{robot_name}_{timestamp}"
        else:
            bag_name = f"{prefix}_{timestamp}"
        
        # Save robot metadata to JSON if provided (async to avoid blocking)
        if robot_metadata:
            def _save_metadata():
                metadata_file = os.path.join(output_dir, f"{bag_name}_robot_info.json")
                try:
                    import json
                    with open(metadata_file, 'w') as f:
                        json.dump(robot_metadata, f, indent=2)
                    print(f"✅ Saved robot metadata to {metadata_file}")
                except Exception as e:
                    print(f"⚠️ Could not save robot metadata: {e}")
            
            # Run in background thread to avoid UI blocking
            import threading
            threading.Thread(target=_save_metadata, daemon=True).start()
        
        # Start recording - CRITICAL: This is the main operation
        success = self.ros2_manager.start_recording(bag_name)
        
        if success:
            self.is_recording = True
            self.current_bag_name = bag_name
            self.current_bag_metadata = robot_metadata
            self.start_btn.setEnabled(False)
            self.stop_btn.setEnabled(True)
            self.dir_input.setEnabled(False)
            self.name_input.setEnabled(False)
            
            self.status_label.setText("Status: Recording")
            self.status_label.setStyleSheet("color: red;")
            
            bag_path = os.path.join(output_dir, bag_name)
            if robot_metadata:
                robot_info = f" (Robot: {robot_metadata.get('hostname', 'Unknown')})"
            else:
                robot_info = ""
            self.recording_info.setText(f"Recording to: {bag_path}{robot_info}")
            
            # Start rate monitoring (async)
            self.start_rate_monitoring()
            
            # Tell Hz monitor thread to watch these topics
            self.hz_monitor_thread.set_topics(list(self.selected_topics_data.keys()))
            
            # ⚡ FAST EMIT - signal listeners immediately
            self.recording_started.emit()
        else:
            QMessageBox.critical(self, "Error", "Failed to start recording. Make sure ROS2 is running.")
            
    def stop_recording(self):
        """Stop bag recording - OPTIMIZED for speed"""
        # Stop immediately on UI
        self.is_recording = False
        self.start_btn.setEnabled(True)
        self.stop_btn.setEnabled(False)
        self.dir_input.setEnabled(True)
        self.name_input.setEnabled(True)
        
        self.status_label.setText("Status: Stopped")
        self.status_label.setStyleSheet("color: green;")
        
        # ⚡ EMIT SIGNAL FIRST - listeners can start preparing
        self.recording_stopped.emit()
        
        # THEN stop in background to avoid UI delay
        # Use threading to avoid blocking UI
        def _do_stop():
            try:
                self.ros2_manager.stop_recording()
            except Exception as e:
                print(f"Stop recording error: {e}")
        
        import threading
        threading.Thread(target=_do_stop, daemon=True).start()
        self.status_label.setStyleSheet("color: orange;")
        
        self.recording_info.setText("Recording stopped successfully")
        
        # Stop rate monitoring
        self.stop_rate_monitoring()
        
        # Tell Hz monitor to stop monitoring
        self.hz_monitor_thread.set_topics([])
        
        self.recording_stopped.emit()
    
    def update_selected_topics(self, selected_topics):
        """Update the list of selected topics for recording"""
        # Initialize tracking for new topics
        for topic in selected_topics:
            if topic not in self.selected_topics_data:
                self.selected_topics_data[topic] = {
                    'rate': 0.0,
                    'last_rate': 0.0,
                    'stalled': False,
                    'stalled_count': 0,
                    'msg_type': 'Unknown',
                    'has_data': False
                }
        
        # Remove topics no longer selected
        removed_topics = [t for t in self.selected_topics_data if t not in selected_topics]
        for topic in removed_topics:
            del self.selected_topics_data[topic]
        
        # Update table
        self.refresh_selected_topics_table()
        
        # Tell Hz monitor about topics (even if not recording yet)
        self.hz_monitor_thread.set_topics(selected_topics)
        
        # Start rate update timer if recording
        if self.is_recording and not self.topic_rates_timer.isActive():
            self.topic_rates_timer.start(1000)  # Update every second
    
    def refresh_selected_topics_table(self):
        """Refresh the selected topics table display with data status and message type - OPTIMIZED"""
        # BATCH UPDATES - Disable updates during bulk changes (smoother rendering)
        self.selected_topics_table.setUpdatesEnabled(False)
        
        self.selected_topics_table.setRowCount(len(self.selected_topics_data))
        
        if not self.selected_topics_data:
            self.topics_info_label.setText("No topics selected yet")
            self.selected_topics_table.setUpdatesEnabled(True)
            return
        
        self.topics_info_label.setText(f"Monitoring {len(self.selected_topics_data)} topic(s)")
        
        for row, (topic_name, data) in enumerate(self.selected_topics_data.items()):
            # Topic name
            topic_item = QTableWidgetItem(topic_name)
            topic_item.setFont(QFont("Monospace", 9))
            self.selected_topics_table.setItem(row, 0, topic_item)
            
            # Message Type
            msg_type = data.get('msg_type', 'Unknown')
            type_item = QTableWidgetItem(msg_type)
            type_item.setFont(QFont("Monospace", 8))
            self.selected_topics_table.setItem(row, 1, type_item)
            
            # Rate
            rate = data['rate']
            rate_item = QTableWidgetItem(f"{rate:.2f} Hz")
            self.selected_topics_table.setItem(row, 2, rate_item)
            
            # Data Status with COLOR CODING (GREEN = has data, RED = no data)
            has_data = data.get('has_data', False)
            if has_data:
                status = "🟢 DATA OK"
                status_item = QTableWidgetItem(status)
                status_item.setForeground(QColor('#00c853'))  # Bright Green
                status_item.setFont(QFont("", -1, QFont.Bold))
            else:
                status = "🔴 NO DATA"
                status_item = QTableWidgetItem(status)
                status_item.setForeground(QColor('#d32f2f'))  # Bright Red
                status_item.setFont(QFont("", -1, QFont.Bold))
            
            self.selected_topics_table.setItem(row, 3, status_item)
            
            # Alert/Warning System
            if data['stalled']:
                alert = "⚠️ STALLED"
                alert_item = QTableWidgetItem(alert)
                alert_item.setForeground(QColor('#ff6f00'))  # Orange
                alert_item.setFont(QFont("", -1, QFont.Bold))
            elif rate > 0:
                alert = "✅ ACTIVE"
                alert_item = QTableWidgetItem(alert)
                alert_item.setForeground(QColor('#4CAF50'))  # Green
            else:
                alert = "⏸️ IDLE"
                alert_item = QTableWidgetItem(alert)
                alert_item.setForeground(QColor('#ff9800'))  # Orange
            
            self.selected_topics_table.setItem(row, 4, alert_item)
        
        # RE-ENABLE UPDATES - All changes rendered at once (smoother, faster)
        self.selected_topics_table.setUpdatesEnabled(True)
    
    def update_topic_rates(self):
        """Update topic rates (called periodically during recording) - NON-BLOCKING"""
        if not self.is_recording:
            self.topic_rates_timer.stop()
            return
        
        # DEBOUNCE - prevent excessive calls (AGGRESSIVE - 2 second minimum)
        current_time = time.time()
        if current_time - self._last_rates_update < self._rates_update_cooldown:
            return
        self._last_rates_update = current_time
        
        # Only use async manager - NEVER call synchronously during recording
        if self.async_ros2_manager:
            # Check if a previous async call is still pending
            if self.async_ros2_manager.active_thread_count() > 0:
                # Skip this update - wait for previous one to complete
                return
            
            # Call async to prevent UI blocking
            self.async_ros2_manager.get_topics_async(self._on_topics_info_received)
        # NOTE: Removed fallback sync call - it was causing freezes!
    
    def _on_topics_info_received(self, topics_info):
        """Callback when topics info is received from async worker"""
        self._process_topics_info(topics_info)
    
    def _process_topics_info(self, topics_info):
        """Process topics info and update display - EXTRACTED to prevent blocking"""
        try:
            # Build a lookup for fast access
            topics_by_name = {t['name']: t for t in topics_info}
            
            # Update rates for each selected topic
            for topic_name, data in self.selected_topics_data.items():
                topic_data = topics_by_name.get(topic_name)
                
                if topic_data:
                    # Get current rate and message type
                    rate = topic_data.get('hz', 0.0)
                    msg_type = topic_data.get('type', 'Unknown')
                    
                    # Store previous rate for stall detection
                    data['last_rate'] = data['rate']
                    data['rate'] = rate
                    
                    # Set has_data based on rate (if publishing, has_data = True)
                    data['has_data'] = rate > 0
                    
                    # Detect stalling: rate dropped to 0 from non-zero
                    if data['last_rate'] > 0 and rate == 0:
                        data['stalled'] = True
                        data['stalled_count'] += 1
                    elif rate > 0:
                        data['stalled'] = False
                        data['stalled_count'] = 0
                    
                    # Update message type
                    if msg_type and msg_type != "Unknown":
                        data['msg_type'] = msg_type
                else:
                    # Topic disappeared
                    data['rate'] = 0.0
                    data['has_data'] = False
                    data['stalled'] = True
                    data['stalled_count'] += 1
            
            # Refresh table display
            self.refresh_selected_topics_table()
            
        except Exception as e:
            print(f"Error processing topic rates: {e}")
    
    def _on_hz_updated(self, hz_rates):
        """Callback when Hz monitor thread updates publishing rates"""
        try:
            # Update rates for topics from background Hz monitor
            for topic_name, hz_value in hz_rates.items():
                if topic_name in self.selected_topics_data:
                    # Handle tuple returns from advanced monitor (hz, confidence)
                    if isinstance(hz_value, tuple):
                        hz = hz_value[0]
                    else:
                        hz = hz_value
                    
                    data = self.selected_topics_data[topic_name]
                    data['last_rate'] = data['rate']
                    data['rate'] = hz
                    
                    # Set has_data based on rate
                    data['has_data'] = hz > 0
                    
                    # Detect stalling
                    if data['last_rate'] > 0 and hz == 0:
                        data['stalled'] = True
                        data['stalled_count'] += 1
                    elif hz > 0:
                        data['stalled'] = False
                        data['stalled_count'] = 0
            
            # Refresh table display (batched updates)
            self.refresh_selected_topics_table()
            
        except Exception as e:
            print(f"Error in Hz update callback: {e}")
    
    def start_rate_monitoring(self):
        """Start monitoring topic rates - VERY INFREQUENT to eliminate freezing"""
        if not self.topic_rates_timer.isActive() and self.selected_topics_data:
            # 5 seconds = almost no subprocess calls, use cache instead
            self.topic_rates_timer.start(5000)  # Update every 5 seconds (was 2 seconds)
    
    def stop_rate_monitoring(self):
        """Stop monitoring topic rates"""
        if self.topic_rates_timer.isActive():
            self.topic_rates_timer.stop()
