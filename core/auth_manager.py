"""
API Authentication Module - Optional security layer for network uploads
"""

import hashlib
import secrets
import json
import os
from datetime import datetime, timedelta
from typing import Dict, Optional, Tuple


class APIToken:
    """Represents an API token with metadata"""
    
    def __init__(self, token: str, name: str, created_at: datetime, expires_at: Optional[datetime] = None):
        self.token: str = token
        self.name: str = name
        self.created_at: datetime = created_at
        self.expires_at: Optional[datetime] = expires_at
        self.last_used: Optional[datetime] = None
        self.request_count: int = 0
    
    def is_valid(self) -> bool:
        """Check if token is still valid"""
        if self.expires_at is None:
            return True
        return datetime.now() < self.expires_at
    
    def to_dict(self) -> dict:
        """Serialize token to dict"""
        return {
            'token': self.token,
            'name': self.name,
            'created_at': self.created_at.isoformat(),
            'expires_at': self.expires_at.isoformat() if self.expires_at else None,
            'last_used': self.last_used.isoformat() if self.last_used else None,
            'request_count': self.request_count
        }


class AuthenticationManager:
    """Manages API authentication, tokens, and access control"""
    
    def __init__(self, config_file: str = "auth_config.json"):
        self.config_file = config_file
        self.enabled = False
        self.tokens: Dict[str, APIToken] = {}
        self.api_key = None
        self.admin_key = None
        self.rate_limits = {}  # token -> requests_per_minute
        self.load_config()
    
    def load_config(self):
        """Load authentication configuration from file"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    config = json.load(f)
                    self.enabled = config.get('enabled', False)
                    self.api_key = config.get('api_key')
                    self.admin_key = config.get('admin_key')
                    
                    # Load tokens
                    for token_data in config.get('tokens', []):
                        token_obj = APIToken(
                            token=token_data['token'],
                            name=token_data['name'],
                            created_at=datetime.fromisoformat(token_data['created_at']),
                            expires_at=datetime.fromisoformat(token_data['expires_at']) if token_data.get('expires_at') else None
                        )
                        token_obj.last_used = datetime.fromisoformat(token_data['last_used']) if token_data.get('last_used') else None
                        token_obj.request_count = token_data.get('request_count', 0)
                        self.tokens[token_obj.token] = token_obj
                    
                    # Load rate limits
                    self.rate_limits = config.get('rate_limits', {})
                    
                    print(f"✅ Authentication config loaded. Enabled: {self.enabled}")
            except Exception as e:
                print(f"⚠️  Could not load auth config: {e}")
        else:
            print(f"ℹ️  No auth config found. Create one to enable authentication.")
    
    def save_config(self):
        """Save authentication configuration to file"""
        try:
            config = {
                'enabled': self.enabled,
                'api_key': self.api_key,
                'admin_key': self.admin_key,
                'tokens': [token.to_dict() for token in self.tokens.values()],
                'rate_limits': self.rate_limits
            }
            with open(self.config_file, 'w') as f:
                json.dump(config, f, indent=2)
            print(f"✅ Auth config saved to {self.config_file}")
        except Exception as e:
            print(f"❌ Could not save auth config: {e}")
    
    def enable_authentication(self, admin_key: Optional[str] = None):
        """Enable authentication mode"""
        self.enabled = True
        if admin_key:
            self.admin_key = hashlib.sha256(admin_key.encode()).hexdigest()
        else:
            self.admin_key = hashlib.sha256(secrets.token_bytes(32)).hexdigest()
        self.save_config()
        print(f"✅ Authentication enabled. Admin key hash: {self.admin_key[:8]}...")
    
    def disable_authentication(self):
        """Disable authentication mode"""
        self.enabled = False
        self.save_config()
        print("⚠️  Authentication disabled - uploads will be unauthenticated")
    
    def generate_token(self, name: str, expires_in_days: Optional[int] = None, rate_limit: int = 100) -> str:
        """
        Generate a new API token
        
        Args:
            name: Human-readable token name
            expires_in_days: Optional expiration (None = never expires)
            rate_limit: Max requests per minute
            
        Returns:
            str: New API token
        """
        token = secrets.token_urlsafe(32)
        expires_at = datetime.now() + timedelta(days=expires_in_days) if expires_in_days else None
        
        token_obj = APIToken(token=token, name=name, created_at=datetime.now(), expires_at=expires_at)
        self.tokens[token] = token_obj
        self.rate_limits[token] = rate_limit
        
        self.save_config()
        print(f"✅ Generated token '{name}': {token[:20]}...")
        return token
    
    def revoke_token(self, token: str) -> bool:
        """Revoke an API token"""
        if token in self.tokens:
            del self.tokens[token]
            if token in self.rate_limits:
                del self.rate_limits[token]
            self.save_config()
            print(f"✅ Token revoked")
            return True
        return False
    
    def verify_admin_key(self, admin_key: str) -> bool:
        """Verify admin authentication key"""
        if not self.enabled or not self.admin_key:
            return False
        admin_hash = hashlib.sha256(admin_key.encode()).hexdigest()
        return admin_hash == self.admin_key
    
    def verify_token(self, token: str) -> Tuple[bool, Optional[str]]:
        """
        Verify an API token
        
        Returns:
            Tuple[bool, str]: (is_valid, error_message or None)
        """
        if not self.enabled:
            return True, None  # Auth disabled, allow all
        
        if token not in self.tokens:
            return False, "Invalid token"
        
        token_obj = self.tokens[token]
        
        if not token_obj.is_valid():
            return False, "Token expired"
        
        # Update usage
        token_obj.last_used = datetime.now()
        token_obj.request_count += 1
        self.save_config()
        
        return True, None
    
    def check_rate_limit(self, token: str) -> Tuple[bool, Optional[str]]:
        """
        Check if token has exceeded rate limit
        
        Returns:
            Tuple[bool, str]: (is_allowed, error_message or None)
        """
        if not self.enabled or token not in self.tokens:
            return True, None
        
        rate_limit = self.rate_limits.get(token, 100)
        token_obj = self.tokens[token]
        
        if token_obj.request_count >= rate_limit:
            return False, f"Rate limit exceeded ({rate_limit} requests/min)"
        
        return True, None
    
    def list_tokens(self) -> list:
        """List all active tokens (for admin panel)"""
        tokens_list = []
        for token, token_obj in self.tokens.items():
            tokens_list.append({
                'token_preview': f"{token[:20]}...",
                'name': token_obj.name,
                'created_at': token_obj.created_at.isoformat(),
                'expires_at': token_obj.expires_at.isoformat() if token_obj.expires_at else 'Never',
                'valid': token_obj.is_valid(),
                'last_used': token_obj.last_used.isoformat() if token_obj.last_used else 'Never',
                'request_count': token_obj.request_count,
                'rate_limit': self.rate_limits.get(token, 100)
            })
        return tokens_list
    
    def get_status(self) -> dict:
        """Get authentication status"""
        return {
            'enabled': self.enabled,
            'total_tokens': len(self.tokens),
            'valid_tokens': sum(1 for t in self.tokens.values() if t.is_valid()),
            'total_requests': sum(t.request_count for t in self.tokens.values()),
            'admin_key_set': self.admin_key is not None
        }
