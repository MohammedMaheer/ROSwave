# pyright: reportAttributeAccessIssue=false
"""
Service Monitor Widget - displays ROS2 services
OPTIMIZED: Incremental updates, debouncing, reuse widgets
"""

from PyQt5.QtWidgets import (QWidget, QVBoxLayout, QHBoxLayout, QTableWidget,  # type: ignore
                             QTableWidgetItem, QPushButton, QGroupBox, QHeaderView,
                             QLabel)
from PyQt5.QtCore import Qt, QTimer  # type: ignore
from PyQt5.QtGui import QColor  # type: ignore


class ServiceMonitorWidget(QWidget):
    """Widget for monitoring ROS2 services with ULTRA-FAST incremental updates"""
    
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
        group = QGroupBox("ROS2 Services")
        group_layout = QVBoxLayout()
        
        # Info label
        info_layout = QHBoxLayout()
        self.service_count_label = QLabel("Services: 0")
        info_layout.addWidget(self.service_count_label)
        info_layout.addStretch()
        
        refresh_btn = QPushButton("Refresh Services")
        refresh_btn.clicked.connect(self.refresh_services)
        info_layout.addWidget(refresh_btn)
        
        group_layout.addLayout(info_layout)
        
        # Services table - OPTIMIZED FOR SMOOTH SCROLLING
        self.services_table = QTableWidget()
        self.services_table.setColumnCount(3)
        self.services_table.setHorizontalHeaderLabels([
            "Service Name", "Service Type", "Servers"
        ])
        
        # CRITICAL PERFORMANCE: Disable sorting and optimize rendering
        self.services_table.setSortingEnabled(False)
        
        vheader = self.services_table.verticalHeader()
        if vheader is not None:
            vheader.setDefaultSectionSize(25)
        
        self.services_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.services_table.setSelectionMode(QTableWidget.NoSelection)
        
        header = self.services_table.horizontalHeader()
        if header is not None:
            header.setSectionResizeMode(0, QHeaderView.Stretch)
            header.setSectionResizeMode(1, QHeaderView.Stretch)
            header.setSectionResizeMode(2, QHeaderView.ResizeToContents)
        
        group_layout.addWidget(self.services_table)
        
        group.setLayout(group_layout)
        layout.addWidget(group)
        self.setLayout(layout)
        
    def refresh_services(self):
        """Refresh the list of ROS2 services - NON-BLOCKING with debounce"""
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
            self.async_ros2_manager.get_services_async(self._on_services_ready)
        else:
            try:
                services_info = self.ros2_manager.get_services_info()
                self._on_services_ready(services_info)
            except Exception as e:
                print(f"Error refreshing services: {e}")
                self.service_count_label.setText("Services: 0 (Error)")
                self._is_updating = False
    
    def _on_services_ready(self, services_info):
        """Callback when services data is ready"""
        try:
            self.update_services_data(services_info)
        finally:
            self._is_updating = False
            if self._pending_update:
                self._pending_update = False
                self._do_refresh()

    def update_services_data(self, services_info):
        """Update services from async data - ULTRA-FAST INCREMENTAL UPDATES"""
        try:
            # Disable updates during batch operation
            self.services_table.setUpdatesEnabled(False)
            
            # Only resize if needed
            if len(services_info) != self.services_table.rowCount():
                self.services_table.setRowCount(len(services_info))
            
            self.service_count_label.setText(f"Services: {len(services_info)}")
            
            for idx, service_info in enumerate(services_info):
                # Reuse or create name item
                name_item = self.services_table.item(idx, 0)
                if name_item is None:
                    name_item = QTableWidgetItem(service_info['name'])
                    self.services_table.setItem(idx, 0, name_item)
                elif name_item.text() != service_info['name']:
                    name_item.setText(service_info['name'])
                
                # Reuse or create type item
                service_type = service_info.get('type', 'Unknown')
                type_item = self.services_table.item(idx, 1)
                if type_item is None:
                    type_item = QTableWidgetItem(service_type)
                    self.services_table.setItem(idx, 1, type_item)
                elif type_item.text() != service_type:
                    type_item.setText(service_type)
                
                # Reuse or create server count item
                server_count = service_info.get('server_count', 1)
                server_item = self.services_table.item(idx, 2)
                if server_item is None:
                    server_item = QTableWidgetItem(str(server_count))
                    server_item.setTextAlignment(Qt.AlignCenter)
                    self.services_table.setItem(idx, 2, server_item)
                elif server_item.text() != str(server_count):
                    server_item.setText(str(server_count))
            
            # Re-enable updates and repaint
            self.services_table.setUpdatesEnabled(True)
            self.services_table.repaint()  # Immediate repaint for smooth scrolling
                
        except Exception as e:
            print(f"Error updating services data: {e}")
            self.services_table.setUpdatesEnabled(True)
