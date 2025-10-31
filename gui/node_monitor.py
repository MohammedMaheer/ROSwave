# pyright: reportAttributeAccessIssue=false
"""
Node Monitor Widget - displays ROS2 nodes and their information
OPTIMIZED: Incremental updates, debouncing, reuse widgets
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget,  # type: ignore
                             QTableWidgetItem, QPushButton, QGroupBox, QHeaderView,
                             QLabel)
from PyQt5.QtCore import Qt, QTimer  # type: ignore
from PyQt5.QtGui import QColor  # type: ignore


class NodeMonitorWidget(QWidget):
    """Widget for monitoring ROS2 nodes with ULTRA-FAST incremental updates"""
    
    def __init__(self, ros2_manager, async_ros2_manager=None):
        super().__init__()
        self.ros2_manager = ros2_manager
        self.async_ros2_manager = async_ros2_manager  # NEW: optional async manager
        
        # CRITICAL: Prevent concurrent updates
        self._is_updating = False
        self._pending_update = False
        
        # Debounce timer for refresh calls
        self._refresh_debounce_timer = QTimer()
        self._refresh_debounce_timer.setSingleShot(True)
        self._refresh_debounce_timer.timeout.connect(self._do_refresh)
        
        self.init_ui()
        
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()
        
        # Group box
        group = QGroupBox("ROS2 Nodes")
        group_layout = QVBoxLayout()
        
        # Info label
        info_layout = QHBoxLayout()
        self.node_count_label = QLabel("Nodes: 0")
        info_layout.addWidget(self.node_count_label)
        info_layout.addStretch()
        
        refresh_btn = QPushButton("Refresh Nodes")
        refresh_btn.clicked.connect(self.refresh_nodes)
        info_layout.addWidget(refresh_btn)
        
        group_layout.addLayout(info_layout)
        
        # Nodes table - OPTIMIZED FOR SMOOTH SCROLLING
        self.nodes_table = QTableWidget()
        self.nodes_table.setColumnCount(4)
        self.nodes_table.setHorizontalHeaderLabels([
            "Node Name", "Namespace", "Publishers", "Subscribers"
        ])
        
        # CRITICAL PERFORMANCE: Disable sorting and optimize rendering
        self.nodes_table.setSortingEnabled(False)
        
        vheader = self.nodes_table.verticalHeader()
        if vheader is not None:
            vheader.setDefaultSectionSize(25)
        
        self.nodes_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.nodes_table.setSelectionMode(QTableWidget.NoSelection)
        
        header = self.nodes_table.horizontalHeader()
        if header is not None:
            header.setSectionResizeMode(0, QHeaderView.Stretch)
            header.setSectionResizeMode(1, QHeaderView.Stretch)
            header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
            header.setSectionResizeMode(3, QHeaderView.ResizeToContents)
        
        group_layout.addWidget(self.nodes_table)
        
        group.setLayout(group_layout)
        layout.addWidget(group)
        self.setLayout(layout)
        
    def refresh_nodes(self):
        """Refresh the list of ROS2 nodes - NON-BLOCKING with debounce"""
        # Debounce rapid refresh calls
        self._refresh_debounce_timer.stop()
        self._refresh_debounce_timer.start(100)
    
    def _do_refresh(self):
        """Actual refresh logic after debounce period"""
        if self._is_updating:
            self._pending_update = True
            return
        
        self._is_updating = True
        
        if self.async_ros2_manager:
            self.async_ros2_manager.get_nodes_async(self._on_nodes_ready)
        else:
            try:
                nodes_info = self.ros2_manager.get_nodes_info()
                self._on_nodes_ready(nodes_info)
            except Exception as e:
                print(f"Error refreshing nodes: {e}")
                self.node_count_label.setText("Nodes: 0 (Error)")
                self._is_updating = False
    
    def _on_nodes_ready(self, nodes_info):
        """Callback when nodes data is ready"""
        try:
            self.update_nodes_data(nodes_info)
        finally:
            self._is_updating = False
            if self._pending_update:
                self._pending_update = False
                self._do_refresh()

    def update_nodes_data(self, nodes_info):
        """Update nodes from async data - ULTRA-FAST INCREMENTAL UPDATES"""
        try:
            # Disable updates during batch operation
            self.nodes_table.setUpdatesEnabled(False)
            
            # Only resize if needed
            if len(nodes_info) != self.nodes_table.rowCount():
                self.nodes_table.setRowCount(len(nodes_info))
            
            self.node_count_label.setText(f"Nodes: {len(nodes_info)}")
            
            for idx, node_info in enumerate(nodes_info):
                # Reuse or create name item
                name_item = self.nodes_table.item(idx, 0)
                if name_item is None:
                    name_item = QTableWidgetItem(node_info['name'])
                    self.nodes_table.setItem(idx, 0, name_item)
                elif name_item.text() != node_info['name']:
                    name_item.setText(node_info['name'])
                
                # Reuse or create namespace item
                namespace_item = self.nodes_table.item(idx, 1)
                if namespace_item is None:
                    namespace_item = QTableWidgetItem(node_info['namespace'])
                    self.nodes_table.setItem(idx, 1, namespace_item)
                elif namespace_item.text() != node_info['namespace']:
                    namespace_item.setText(node_info['namespace'])
                
                # Reuse or create publishers item
                pub_count = node_info.get('publishers', 0)
                pub_item = self.nodes_table.item(idx, 2)
                if pub_item is None:
                    pub_item = QTableWidgetItem(str(pub_count))
                    pub_item.setTextAlignment(Qt.AlignCenter)
                    self.nodes_table.setItem(idx, 2, pub_item)
                elif pub_item.text() != str(pub_count):
                    pub_item.setText(str(pub_count))
                
                # Reuse or create subscribers item
                sub_count = node_info.get('subscribers', 0)
                sub_item = self.nodes_table.item(idx, 3)
                if sub_item is None:
                    sub_item = QTableWidgetItem(str(sub_count))
                    sub_item.setTextAlignment(Qt.AlignCenter)
                    self.nodes_table.setItem(idx, 3, sub_item)
                elif sub_item.text() != str(sub_count):
                    sub_item.setText(str(sub_count))
            
            # Re-enable updates and repaint
            self.nodes_table.setUpdatesEnabled(True)
            self.nodes_table.repaint()  # Immediate repaint for smooth scrolling
                
        except Exception as e:
            print(f"Error updating nodes data: {e}")
            self.nodes_table.setUpdatesEnabled(True)
