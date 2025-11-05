# pyright: reportAttributeAccessIssue=false
"""
Topic Monitor Widget - displays available ROS2 topics
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget,  # type: ignore
                             QTableWidgetItem, QPushButton, QGroupBox, QHeaderView,
                             QLabel, QCheckBox)
from PyQt5.QtCore import Qt, pyqtSignal, QTimer  # type: ignore
from PyQt5.QtGui import QColor  # type: ignore


class TopicMonitorWidget(QWidget):
    """Widget for monitoring ROS2 topics"""
    
    topic_selected = pyqtSignal(str, bool)  # topic_name, is_selected
    topics_changed = pyqtSignal(list)  # emitted when selected topics change
    
    def __init__(self, ros2_manager, async_ros2_manager=None):
        super().__init__()
        self.ros2_manager = ros2_manager
        self.async_ros2_manager = async_ros2_manager  # NEW: optional async manager
        self.selected_topics = set()
        
        # Debounce timer to coalesce rapid checkbox events into a single topics_changed emit
        self._topics_changed_timer = QTimer()
        self._topics_changed_timer.setSingleShot(True)
        self._topics_changed_timer.timeout.connect(self._emit_topics_changed)
        
        # CRITICAL: Prevent concurrent updates from causing freezes
        self._is_updating = False
        self._pending_update = False
        
        # Debounce timer for refresh calls (prevent spam)
        self._refresh_debounce_timer = QTimer()
        self._refresh_debounce_timer.setSingleShot(True)
        self._refresh_debounce_timer.timeout.connect(self._do_refresh)
        
        # NEW: Periodic Hz refresh timer (every 10 seconds for real-time monitoring)
        self._hz_refresh_timer = QTimer()
        self._hz_refresh_timer.setSingleShot(False)
        self._hz_refresh_timer.timeout.connect(self._periodic_hz_refresh)
        self._current_topics_info = []  # Cache for periodic refresh
        self._is_recording = False  # Track recording state
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()
        
        # Group box
        group = QGroupBox("Available ROS2 Topics")
        group_layout = QVBoxLayout()
        
        # Info label
        info_layout = QHBoxLayout()
        self.topic_count_label = QLabel("Topics: 0")
        info_layout.addWidget(self.topic_count_label)
        info_layout.addStretch()
        
        # Selection buttons
        select_all_btn = QPushButton("Select All")
        select_all_btn.clicked.connect(self.select_all_topics)
        info_layout.addWidget(select_all_btn)
        
        deselect_all_btn = QPushButton("Deselect All")
        deselect_all_btn.clicked.connect(self.deselect_all_topics)
        info_layout.addWidget(deselect_all_btn)
        
        group_layout.addLayout(info_layout)
        
        # Topics table - OPTIMIZED FOR SMOOTH SCROLLING
        self.topics_table = QTableWidget()
        self.topics_table.setColumnCount(6)
        self.topics_table.setHorizontalHeaderLabels([
            "Record", "Topic Name", "Message Type", "Publishers", "Hz", "Status"
        ])
        
        # CRITICAL PERFORMANCE: Disable sorting to prevent layout recalculation
        self.topics_table.setSortingEnabled(False)
        
        # Optimize row height for faster rendering
        vheader = self.topics_table.verticalHeader()
        if vheader is not None:
            vheader.setDefaultSectionSize(25)
        
        # Reduce selection overhead
        self.topics_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.topics_table.setSelectionMode(QTableWidget.NoSelection)
        
        # Set column widths
        header = self.topics_table.horizontalHeader()
        if header is not None:
            header.setSectionResizeMode(0, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(1, QHeaderView.Stretch)
            header.setSectionResizeMode(2, QHeaderView.Stretch)
            header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(4, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(5, QHeaderView.ResizeToContents)  # NEW: Status column
        
        group_layout.addWidget(self.topics_table)
        
        # Refresh button
        refresh_btn = QPushButton("Refresh Topics")
        refresh_btn.clicked.connect(self.refresh_topics)
        group_layout.addWidget(refresh_btn)
        
        group.setLayout(group_layout)
        layout.addWidget(group)
        self.setLayout(layout)
        
        # Don't do initial refresh - let timer handle it
        
    def refresh_topics(self):
        """Refresh the list of available topics - NON-BLOCKING using async with debounce"""
        # Debounce rapid refresh calls (prevents multiple concurrent updates)
        self._refresh_debounce_timer.stop()
        self._refresh_debounce_timer.start(100)  # Wait 100ms before actually refreshing
    
    def _do_refresh(self):
        """Actual refresh logic after debounce period"""
        # If already updating, mark as pending and return (don't queue multiple)
        if self._is_updating:
            self._pending_update = True
            return
        
        self._is_updating = True
        
        if self.async_ros2_manager:
            # Use async manager - callback when data ready
            self.async_ros2_manager.get_topics_async(self._on_topics_ready)
        else:
            # Fallback: don't do synchronous call! Queue in thread pool instead
            # This prevents UI freezing for 6+ seconds
            from PyQt5.QtCore import QThreadPool, QRunnable, pyqtSlot  # type: ignore
            
            class SyncRefreshWorker(QRunnable):
                def __init__(worker_self, parent_widget):
                    super().__init__()
                    worker_self.parent = parent_widget
                
                @pyqtSlot()
                def run(worker_self):
                    try:
                        # Perform blocking call in thread
                        topics_info = worker_self.parent.ros2_manager.get_topics_info()
                        # Call callback on main thread via Qt signal
                        worker_self.parent._on_topics_ready(topics_info)
                    except Exception as e:
                        print(f"Error refreshing topics (async fallback): {e}")
                        worker_self.parent.topic_count_label.setText(f"Topics: 0 (Error: ROS2 may not be running)")
                        worker_self.parent._is_updating = False
            
            # Queue in thread pool (non-blocking)
            if not hasattr(self, '_refresh_threadpool'):
                self._refresh_threadpool = QThreadPool()
                self._refresh_threadpool.setMaxThreadCount(1)  # Single-threaded
            
            worker = SyncRefreshWorker(self)
            self._refresh_threadpool.start(worker)
    
    def _on_topics_ready(self, topics_info):
        """Callback when topics data is ready from async operation"""
        try:
            # Cache topics for periodic refresh
            self._current_topics_info = topics_info
            
            self.update_topics_data(topics_info)
            
            # AFTER displaying topics with 0.0 Hz, fetch actual Hz values in background
            # This keeps UI responsive while still showing accurate Hz data
            self._start_background_hz_fetch(topics_info)
        finally:
            self._is_updating = False
            # If there was a pending update, do it now
            if self._pending_update:
                self._pending_update = False
                self._do_refresh()
    
    def set_recording_state(self, is_recording):
        """Track recording state to enable/disable real-time Hz monitoring"""
        self._is_recording = is_recording
        
        # Start periodic Hz refresh when recording starts
        if is_recording:
            if not self._hz_refresh_timer.isActive():
                self._hz_refresh_timer.start(10000)  # Refresh every 10 seconds
        else:
            # Stop periodic Hz refresh when recording stops
            if self._hz_refresh_timer.isActive():
                self._hz_refresh_timer.stop()
    
    def _periodic_hz_refresh(self):
        """Periodically refresh Hz values during recording (real-time monitoring)"""
        if self._current_topics_info and self._is_recording:
            # Fetch fresh Hz values in background without blocking UI
            self._start_background_hz_fetch(self._current_topics_info)
            
    def _start_background_hz_fetch(self, topics_info):
        """Start background task to fetch Hz values without blocking UI"""
        # Extract just the topic names
        topic_names = [t['name'] for t in topics_info]
        
        if not topic_names:
            return
        
        # Queue Hz fetching in a background thread (max 4 concurrent workers)
        from PyQt5.QtCore import QThreadPool, QRunnable, pyqtSlot  # type: ignore
        
        # Define worker class - use instance variables instead of closure
        parent_widget = self
        
        class HzFetchWorker(QRunnable):
            def __init__(self, parent, topics):
                super().__init__()
                self.parent = parent
                self.topics = topics
            
            @pyqtSlot()
            def run(self):
                try:
                    # Fetch Hz values for all topics in parallel (non-blocking)
                    hz_dict = self.parent.ros2_manager.get_topics_hz_batch(
                        self.topics, 
                        max_workers=4
                    )
                    # Update UI with fetched Hz values
                    self.parent._update_hz_values(hz_dict)
                except Exception:
                    # Silently ignore - Hz values are just a nice-to-have
                    pass
        
        # Use a persistent thread pool for Hz fetching
        if not hasattr(self, '_hz_threadpool'):
            self._hz_threadpool = QThreadPool()
            self._hz_threadpool.setMaxThreadCount(1)  # Don't spawn too many threads
        
        worker = HzFetchWorker(self, topic_names)
        self._hz_threadpool.start(worker)
    
    def _update_hz_values(self, hz_dict):
        """Update Hz column with fetched values (called from background thread via Qt)"""
        try:
            self.topics_table.setUpdatesEnabled(False)
            
            # Update each row with fetched Hz value
            for row in range(self.topics_table.rowCount()):
                name_item = self.topics_table.item(row, 1)
                if name_item:
                    topic_name = name_item.text()
                    if topic_name in hz_dict:
                        hz_value = hz_dict[topic_name]
                        hz_text = f"{hz_value:.1f}"
                        hz_item = self.topics_table.item(row, 4)
                        if hz_item and hz_item.text() != hz_text:
                            hz_item.setText(hz_text)
        finally:
            self.topics_table.setUpdatesEnabled(True)
            
    def on_topic_selected(self, topic_name, state):
        """Handle topic selection change"""
        if state == Qt.Checked:
            self.selected_topics.add(topic_name)
        else:
            self.selected_topics.discard(topic_name)
        self.topic_selected.emit(topic_name, state == Qt.Checked)
        # Debounce emitting the full selected-topics list to avoid flooding callers
        self._topics_changed_timer.start(50)
        
    def select_all_topics(self):
        """Select all topics for recording"""
        # Build selected set and update UI in batch without repeated signals
        topics = []
        for row in range(self.topics_table.rowCount()):
            name_item = self.topics_table.item(row, 1)
            if name_item:
                topics.append(name_item.text())

        self.selected_topics = set(topics)

        # Update checkbox widgets visually without emitting stateChanged
        for row in range(self.topics_table.rowCount()):
            checkbox_widget = self.topics_table.cellWidget(row, 0)
            if checkbox_widget:
                checkbox = checkbox_widget.findChild(QCheckBox)
                if checkbox:
                    checkbox.blockSignals(True)
                    checkbox.setChecked(True)
                    checkbox.blockSignals(False)

        # Emit single consolidated change
        self.topics_changed.emit(list(self.selected_topics))
                    
    def deselect_all_topics(self):
        """Deselect all topics"""
        self.selected_topics.clear()

        for row in range(self.topics_table.rowCount()):
            checkbox_widget = self.topics_table.cellWidget(row, 0)
            if checkbox_widget:
                checkbox = checkbox_widget.findChild(QCheckBox)
                if checkbox:
                    checkbox.blockSignals(True)
                    checkbox.setChecked(False)
                    checkbox.blockSignals(False)

        # Emit single consolidated change
        self.topics_changed.emit([])

    def _emit_topics_changed(self):
        """Emit consolidated topics_changed signal (debounced)."""
        self.topics_changed.emit(list(self.selected_topics))
                    
    def get_selected_topics(self):
        """Get list of selected topics"""
        return list(self.selected_topics)
        
    def set_selected_topics(self, topics):
        """Set selected topics from a list"""
        self.selected_topics = set(topics)
        self.refresh_topics()
        
    def clear_selection(self):
        """Clear all topic selections"""
        self.selected_topics.clear()
        self.refresh_topics()
    
    def update_topics_data(self, topics_info):
        """
        Update topics from async data - ULTRA-OPTIMIZED INCREMENTAL UPDATE.
        This method is called from background thread with pre-fetched data.
        
        CRITICAL OPTIMIZATION: Reuse existing widgets instead of recreating them.
        This eliminates the freeze caused by creating hundreds of checkboxes.
        """
        try:
            # Disable updates during batch operation (MASSIVE performance gain)
            self.topics_table.setUpdatesEnabled(False)
            
            # Build lookup of existing topics by row
            existing_topics = {}
            for row in range(self.topics_table.rowCount()):
                name_item = self.topics_table.item(row, 1)
                if name_item:
                    existing_topics[name_item.text()] = row
            
            # Build new topics lookup
            new_topics = {t['name']: t for t in topics_info}
            
            # Only resize if row count changed significantly (avoid flicker)
            if abs(len(topics_info) - self.topics_table.rowCount()) > 0:
                self.topics_table.setRowCount(len(topics_info))
            
            self.topic_count_label.setText(f"Topics: {len(topics_info)}")
            
            for idx, topic_info in enumerate(topics_info):
                topic_name = topic_info['name']
                
                # REUSE existing checkbox if topic already exists at same position
                existing_row = existing_topics.get(topic_name, -1)
                checkbox_widget = self.topics_table.cellWidget(idx, 0)
                
                if checkbox_widget is None or existing_row != idx:
                    # Create NEW checkbox only if needed
                    checkbox = QCheckBox()
                    checkbox.setChecked(topic_name in self.selected_topics)
                    checkbox.stateChanged.connect(
                        lambda state, name=topic_name: self.on_topic_selected(name, state)
                    )
                    
                    checkbox_widget = QWidget()
                    checkbox_layout = QHBoxLayout(checkbox_widget)
                    checkbox_layout.addWidget(checkbox)
                    checkbox_layout.setAlignment(Qt.AlignCenter)
                    checkbox_layout.setContentsMargins(0, 0, 0, 0)
                    
                    self.topics_table.setCellWidget(idx, 0, checkbox_widget)
                else:
                    # REUSE existing checkbox - just update checked state
                    checkbox = checkbox_widget.findChild(QCheckBox)
                    if checkbox:
                        # Block signals to prevent triggering on_topic_selected
                        checkbox.blockSignals(True)
                        checkbox.setChecked(topic_name in self.selected_topics)
                        checkbox.blockSignals(False)
                
                # Update or create topic name (cheap operation)
                name_item = self.topics_table.item(idx, 1)
                if name_item is None:
                    name_item = QTableWidgetItem(topic_name)
                    self.topics_table.setItem(idx, 1, name_item)
                elif name_item.text() != topic_name:
                    name_item.setText(topic_name)
                
                # Update or create message type
                msg_type = topic_info.get('type', 'Unknown')
                msg_type_item = self.topics_table.item(idx, 2)
                if msg_type_item is None:
                    msg_type_item = QTableWidgetItem(msg_type)
                    self.topics_table.setItem(idx, 2, msg_type_item)
                elif msg_type_item.text() != msg_type:
                    msg_type_item.setText(msg_type)
                
                # Update or create publisher count
                pub_count = topic_info.get('publisher_count', 0)
                pub_item = self.topics_table.item(idx, 3)
                if pub_item is None:
                    pub_item = QTableWidgetItem(str(pub_count))
                    pub_item.setTextAlignment(Qt.AlignCenter)
                    self.topics_table.setItem(idx, 3, pub_item)
                else:
                    pub_item.setText(str(pub_count))
                
                # Color code based on publisher count
                if pub_count > 0:
                    pub_item.setForeground(QColor('green'))
                else:
                    pub_item.setForeground(QColor('gray'))
                
                # Update or create frequency
                hz = topic_info.get('hz', 0.0)
                hz_text = f"{hz:.1f}"
                hz_item = self.topics_table.item(idx, 4)
                if hz_item is None:
                    hz_item = QTableWidgetItem(hz_text)
                    hz_item.setTextAlignment(Qt.AlignRight | Qt.AlignVCenter)
                    self.topics_table.setItem(idx, 4, hz_item)
                elif hz_item.text() != hz_text:
                    hz_item.setText(hz_text)
                
                # NEW: Update or create status column
                pub_count = topic_info.get('publisher_count', 0)
                status_text = "Publishing" if pub_count > 0 else "Idle"
                status_color = QColor('green') if pub_count > 0 else QColor('orange')
                
                status_item = self.topics_table.item(idx, 5)
                if status_item is None:
                    status_item = QTableWidgetItem(status_text)
                    status_item.setTextAlignment(Qt.AlignCenter)
                    self.topics_table.setItem(idx, 5, status_item)
                else:
                    status_item.setText(status_text)
                
                status_item.setForeground(status_color)
            
            # Re-enable updates and force single repaint (instead of incremental)
            self.topics_table.setUpdatesEnabled(True)
            self.topics_table.repaint()  # Immediate repaint for smooth scrolling
            
        except Exception as e:
            print(f"Error updating topics data: {e}")
            self.topic_count_label.setText(f"Topics: 0 (Error)")
            self.topics_table.setUpdatesEnabled(True)  # Ensure updates re-enabled

