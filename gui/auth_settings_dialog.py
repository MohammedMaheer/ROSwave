"""
Authentication Settings Dialog - Admin panel for API authentication
"""

from PyQt5.QtWidgets import (QDialog, QVBoxLayout, QHBoxLayout, QPushButton,  # type: ignore
                             QLabel, QLineEdit, QCheckBox, QTableWidget, QTableWidgetItem,
                             QMessageBox, QSpinBox, QGroupBox, QHeaderView, QComboBox, QTabWidget)
from PyQt5.QtCore import Qt, pyqtSignal  # type: ignore
from PyQt5.QtGui import QColor, QFont  # type: ignore

from core.auth_manager import AuthenticationManager


class AuthenticationSettingsDialog(QDialog):
    """Admin dialog for managing API authentication"""
    
    auth_changed = pyqtSignal(bool)  # Emitted when auth state changes
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("API Authentication Settings")
        self.setGeometry(100, 100, 900, 600)
        
        self.auth_manager = AuthenticationManager()
        self.init_ui()
        self.update_display()
    
    def init_ui(self):
        """Initialize UI components"""
        layout = QVBoxLayout()
        
        # Create tabs for different sections
        tabs = QTabWidget()
        
        # Tab 1: Authentication Status & Toggle
        status_tab = self.create_status_tab()
        tabs.addTab(status_tab, "🔐 Authentication")
        
        # Tab 2: Token Management
        tokens_tab = self.create_tokens_tab()
        tabs.addTab(tokens_tab, "🔑 Tokens")
        
        # Tab 3: Admin Settings
        admin_tab = self.create_admin_tab()
        tabs.addTab(admin_tab, "👤 Admin")
        
        layout.addWidget(tabs)
        
        # Bottom buttons
        button_layout = QHBoxLayout()
        button_layout.addStretch()
        
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.accept)
        button_layout.addWidget(close_btn)
        
        layout.addLayout(button_layout)
        self.setLayout(layout)
    
    def create_status_tab(self):
        """Create authentication status tab"""
        widget = QGroupBox("Authentication Status")
        layout = QVBoxLayout()
        
        # Status display
        self.status_label = QLabel()
        self.status_label.setFont(QFont("Courier", 10))
        layout.addWidget(self.status_label)
        
        layout.addSpacing(10)
        
        # Toggle button
        toggle_layout = QHBoxLayout()
        
        self.toggle_check = QCheckBox("Enable API Authentication")
        self.toggle_check.stateChanged.connect(self.toggle_authentication)
        toggle_layout.addWidget(self.toggle_check)
        
        toggle_layout.addStretch()
        
        widget.setLayout(layout)
        return widget
    
    def create_tokens_tab(self):
        """Create token management tab"""
        widget = QGroupBox("API Token Management")
        layout = QVBoxLayout()
        
        # Token generation section
        gen_layout = QHBoxLayout()
        
        gen_layout.addWidget(QLabel("Token Name:"))
        self.token_name_input = QLineEdit()
        self.token_name_input.setPlaceholderText("e.g., Upload Client 1")
        gen_layout.addWidget(self.token_name_input)
        
        gen_layout.addWidget(QLabel("Expires (days):"))
        self.expires_spin = QSpinBox()
        self.expires_spin.setMinimum(0)
        self.expires_spin.setMaximum(365)
        self.expires_spin.setValue(90)
        self.expires_spin.setSpecialValueText("Never")
        gen_layout.addWidget(self.expires_spin)
        
        gen_layout.addWidget(QLabel("Rate Limit (/min):"))
        self.rate_limit_spin = QSpinBox()
        self.rate_limit_spin.setMinimum(1)
        self.rate_limit_spin.setMaximum(10000)
        self.rate_limit_spin.setValue(100)
        gen_layout.addWidget(self.rate_limit_spin)
        
        gen_btn = QPushButton("➕ Generate Token")
        gen_btn.clicked.connect(self.generate_new_token)
        gen_layout.addWidget(gen_btn)
        
        layout.addLayout(gen_layout)
        
        layout.addSpacing(10)
        
        # Tokens table
        self.tokens_table = QTableWidget()
        self.tokens_table.setColumnCount(8)
        self.tokens_table.setHorizontalHeaderLabels([
            "Token Preview", "Name", "Created", "Expires", "Valid", "Last Used", "Requests", "Rate Limit"
        ])
        
        header = self.tokens_table.horizontalHeader()
        if header:
            header.setSectionResizeMode(0, QHeaderView.Stretch)
            header.setSectionResizeMode(1, QHeaderView.Stretch)
        
        layout.addWidget(self.tokens_table)
        
        # Token actions
        action_layout = QHBoxLayout()
        
        refresh_btn = QPushButton("🔄 Refresh")
        refresh_btn.clicked.connect(self.update_tokens_table)
        action_layout.addWidget(refresh_btn)
        
        revoke_btn = QPushButton("❌ Revoke Selected")
        revoke_btn.clicked.connect(self.revoke_selected_token)
        action_layout.addWidget(revoke_btn)
        
        action_layout.addStretch()
        
        layout.addLayout(action_layout)
        
        widget.setLayout(layout)
        return widget
    
    def create_admin_tab(self):
        """Create admin settings tab"""
        widget = QGroupBox("Admin Settings")
        layout = QVBoxLayout()
        
        # Admin key section
        key_layout = QHBoxLayout()
        key_layout.addWidget(QLabel("Admin Key:"))
        
        self.admin_key_input = QLineEdit()
        self.admin_key_input.setEchoMode(QLineEdit.Password)
        self.admin_key_input.setPlaceholderText("Enter strong admin password")
        key_layout.addWidget(self.admin_key_input)
        
        set_key_btn = QPushButton("🔑 Set Admin Key")
        set_key_btn.clicked.connect(self.set_admin_key)
        key_layout.addWidget(set_key_btn)
        
        layout.addLayout(key_layout)
        
        layout.addSpacing(10)
        
        # Info text
        info_label = QLabel(
            "The admin key is used to:\n"
            "• Generate new API tokens\n"
            "• Revoke existing tokens\n"
            "• Configure authentication settings\n\n"
            "Keep this key secure and private!"
        )
        info_label.setStyleSheet("color: #666; font-size: 10pt;")
        layout.addWidget(info_label)
        
        layout.addStretch()
        
        widget.setLayout(layout)
        return widget
    
    def toggle_authentication(self, state):
        """Toggle authentication on/off"""
        try:
            if state:
                # Enable authentication
                if not self.auth_manager.admin_key:
                    QMessageBox.warning(
                        self,
                        "Admin Key Required",
                        "Please set an admin key before enabling authentication"
                    )
                    self.toggle_check.setChecked(False)
                    return
                
                self.auth_manager.enable_authentication()
                self.auth_changed.emit(True)
                self.update_display()
                QMessageBox.information(self, "Success", "✅ API Authentication enabled")
            else:
                # Disable authentication
                reply = QMessageBox.warning(
                    self,
                    "Disable Authentication?",
                    "Are you sure? This will allow unauthenticated uploads.",
                    QMessageBox.Yes | QMessageBox.No
                )
                if reply == QMessageBox.Yes:
                    self.auth_manager.disable_authentication()
                    self.auth_changed.emit(False)
                    self.update_display()
                else:
                    self.toggle_check.setChecked(True)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to toggle authentication: {str(e)}")
    
    def generate_new_token(self):
        """Generate a new API token"""
        try:
            if not self.auth_manager.enabled:
                QMessageBox.warning(self, "Authentication Disabled", "Enable authentication first")
                return
            
            name = self.token_name_input.text().strip()
            if not name:
                QMessageBox.warning(self, "Invalid Input", "Please enter a token name")
                return
            
            expires_days = self.expires_spin.value()
            rate_limit = self.rate_limit_spin.value()
            
            token = self.auth_manager.generate_token(
                name=name,
                expires_in_days=expires_days if expires_days > 0 else None,
                rate_limit=rate_limit
            )
            
            # Show token to user (only once!)
            QMessageBox.information(
                self,
                "Token Generated",
                f"Token created successfully:\n\n{token}\n\nStore this securely - you won't see it again!"
            )
            
            self.token_name_input.clear()
            self.update_tokens_table()
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to generate token: {str(e)}")
    
    def revoke_selected_token(self):
        """Revoke selected token"""
        try:
            current_row = self.tokens_table.currentRow()
            if current_row < 0:
                QMessageBox.warning(self, "No Selection", "Please select a token to revoke")
                return
            
            item0 = self.tokens_table.item(current_row, 0)
            item1 = self.tokens_table.item(current_row, 1)
            if not item0 or not item1:
                return
            
            token_preview = item0.text()
            token_name = item1.text()
            
            reply = QMessageBox.warning(
                self,
                "Revoke Token?",
                f"Revoke token '{token_name}'?\n\nThis cannot be undone.",
                QMessageBox.Yes | QMessageBox.No
            )
            
            if reply == QMessageBox.Yes:
                # Find and revoke the token
                for token, token_obj in self.auth_manager.tokens.items():
                    if token_obj.name == token_name:
                        self.auth_manager.revoke_token(token)
                        break
                
                self.update_tokens_table()
                QMessageBox.information(self, "Success", "✅ Token revoked")
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to revoke token: {str(e)}")
    
    def set_admin_key(self):
        """Set admin key"""
        try:
            admin_key = self.admin_key_input.text()
            if not admin_key:
                QMessageBox.warning(self, "Invalid Input", "Please enter an admin key")
                return
            
            if len(admin_key) < 8:
                QMessageBox.warning(self, "Weak Key", "Admin key must be at least 8 characters")
                return
            
            self.auth_manager.enable_authentication(admin_key)
            self.admin_key_input.clear()
            self.update_display()
            QMessageBox.information(self, "Success", "✅ Admin key set and authentication enabled")
            
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to set admin key: {str(e)}")
    
    def update_display(self):
        """Update all displays"""
        status = self.auth_manager.get_status()
        
        status_text = f"""
🔐 Authentication Status
{'━' * 40}
Enabled: {'✅ Yes' if status['enabled'] else '❌ No'}
Total Tokens: {status['total_tokens']}
Valid Tokens: {status['valid_tokens']}
Total Requests: {status['total_requests']}
Admin Key Set: {'✅ Yes' if status['admin_key_set'] else '❌ No'}
"""
        self.status_label.setText(status_text)
        self.toggle_check.setChecked(status['enabled'])
        
        self.update_tokens_table()
    
    def update_tokens_table(self):
        """Update tokens table"""
        tokens_list = self.auth_manager.list_tokens()
        self.tokens_table.setRowCount(len(tokens_list))
        
        for idx, token_info in enumerate(tokens_list):
            # Token preview
            item = QTableWidgetItem(token_info['token_preview'])
            self.tokens_table.setItem(idx, 0, item)
            
            # Name
            item = QTableWidgetItem(token_info['name'])
            self.tokens_table.setItem(idx, 1, item)
            
            # Created
            created = token_info['created_at'].split('T')[0]
            item = QTableWidgetItem(created)
            self.tokens_table.setItem(idx, 2, item)
            
            # Expires
            expires = token_info['expires_at'].split('T')[0] if token_info['expires_at'] != 'Never' else 'Never'
            item = QTableWidgetItem(expires)
            self.tokens_table.setItem(idx, 3, item)
            
            # Valid
            valid_text = "✅ Yes" if token_info['valid'] else "❌ No"
            item = QTableWidgetItem(valid_text)
            item.setForeground(QColor('green') if token_info['valid'] else QColor('red'))
            self.tokens_table.setItem(idx, 4, item)
            
            # Last used
            last_used = token_info['last_used'].split('T')[0] if token_info['last_used'] != 'Never' else 'Never'
            item = QTableWidgetItem(last_used)
            self.tokens_table.setItem(idx, 5, item)
            
            # Request count
            item = QTableWidgetItem(str(token_info['request_count']))
            item.setTextAlignment(4)  # Qt.AlignCenter = 4
            self.tokens_table.setItem(idx, 6, item)
            
            # Rate limit
            item = QTableWidgetItem(str(token_info['rate_limit']))
            item.setTextAlignment(4)  # Qt.AlignCenter = 4
            self.tokens_table.setItem(idx, 7, item)
