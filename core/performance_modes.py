"""
Adaptive Performance Modes for ROS2 Dashboard
Automatically adjusts settings based on system specifications
"""

import psutil
import platform
from enum import Enum
from typing import Dict, Any, Optional
from PyQt5.QtCore import QObject, pyqtSignal  # type: ignore


class PerformanceMode(Enum):
    """Performance mode levels"""
    HIGH = "high"           # High-end systems (16GB+ RAM, 8+ cores)
    BALANCED = "balanced"   # Mid-range systems (8-16GB RAM, 4-8 cores)
    LOW = "low"            # Low-end systems (<8GB RAM, <4 cores)
    CUSTOM = "custom"      # User-defined settings


class PerformanceModeManager(QObject):
    """Manages adaptive performance modes based on system specs"""
    
    mode_changed = pyqtSignal(str)  # Emits new mode name
    
    def __init__(self):
        super().__init__()
        self._current_mode: PerformanceMode = PerformanceMode.BALANCED
        self._custom_settings = None
        
        # Detect system specs
        self.system_info = self._detect_system_specs()
        
        # Auto-detect optimal mode
        self._current_mode = self._auto_detect_mode()
        
    def _detect_system_specs(self) -> Dict[str, Any]:
        """Detect system hardware specifications"""
        try:
            memory_gb = psutil.virtual_memory().total / (1024 ** 3)
            cpu_count = psutil.cpu_count(logical=False) or psutil.cpu_count()
            cpu_freq = psutil.cpu_freq()
            cpu_freq_mhz = cpu_freq.max if cpu_freq else 0
            
            # Get disk info
            disk = psutil.disk_usage('/')
            disk_total_gb = disk.total / (1024 ** 3)
            
            # Check if SSD (heuristic: very fast disk)
            try:
                disk_io_start = psutil.disk_io_counters()
                import time
                time.sleep(0.1)
                disk_io_end = psutil.disk_io_counters()
                if disk_io_start and disk_io_end:
                    read_speed = (disk_io_end.read_bytes - disk_io_start.read_bytes) / 0.1
                    is_ssd = read_speed > 100 * 1024 * 1024  # >100MB/s suggests SSD
                else:
                    is_ssd = False
            except Exception:
                is_ssd = False
            
            return {
                'memory_gb': round(memory_gb, 1),
                'cpu_count': cpu_count,
                'cpu_freq_mhz': round(cpu_freq_mhz, 0),
                'disk_gb': round(disk_total_gb, 1),
                'is_ssd': is_ssd,
                'platform': platform.system(),
                'architecture': platform.machine()
            }
        except Exception as e:
            print(f"Error detecting system specs: {e}")
            return {
                'memory_gb': 8.0,
                'cpu_count': 4,
                'cpu_freq_mhz': 2000,
                'disk_gb': 256,
                'is_ssd': False,
                'platform': 'Unknown',
                'architecture': 'Unknown'
            }
    
    def _auto_detect_mode(self) -> PerformanceMode:
        """Auto-detect optimal performance mode based on specs"""
        memory_gb = self.system_info['memory_gb']
        cpu_count = self.system_info['cpu_count']
        
        # High-end: 16GB+ RAM AND 8+ cores
        if memory_gb >= 16 and cpu_count >= 8:
            return PerformanceMode.HIGH
        
        # Low-end: <8GB RAM OR <4 cores
        elif memory_gb < 8 or cpu_count < 4:
            return PerformanceMode.LOW
        
        # Balanced: Everything else (8-16GB RAM, 4-8 cores)
        else:
            return PerformanceMode.BALANCED
    
    def get_mode_settings(self, mode: Optional[PerformanceMode] = None) -> Dict[str, Any]:
        """Get settings for specified mode (or current mode)"""
        if mode is None:
            mode = self._current_mode
        
        if mode == PerformanceMode.CUSTOM and self._custom_settings:
            return self._custom_settings
        
        # HIGH PERFORMANCE MODE (16GB+ RAM, 8+ cores) - ULTRA-SMOOTH
        if mode == PerformanceMode.HIGH:
            return {
                # Timer intervals (ms) - FASTEST FOR SMOOTHNESS
                'ros2_update_interval': 1500,      # 1.5 seconds - minimal blocking
                'metrics_update_interval': 200,    # 200ms - ultra-responsive
                'history_update_interval': 5000,   # 5 seconds - quick updates
                'chart_update_interval': 300,      # 300ms - very smooth charts
                
                # Thread pool settings - MAXIMUM PARALLELISM
                'max_threads': 8,                  # Many parallel operations
                'max_concurrent_threads': 5,       # Allow many concurrent ops
                
                # Cache settings (seconds) - BALANCED FOR SPEED
                'cache_timeout': 3,                # 3 seconds - balance freshness/speed
                'system_metrics_cache': 0.3,       # 300ms - very fresh metrics
                'topic_check_interval': 3,         # 3 seconds - frequent but not excessive
                
                # Chart settings - ULTRA-SMOOTH
                'chart_buffer_size': 200,          # 1 minute at 300ms intervals
                'chart_auto_pause': False,         # Always running on high-end
                
                # Memory settings
                'history_max_entries': 2000,       # Large history
                'enable_profiler': True,           # Always profiling
                'enable_advanced_features': True,  # All features
                
                # UI settings - ULTRA-SMOOTH
                'lazy_load_widgets': False,        # Load everything for maximum responsiveness
                'process_priority': 'high',        # High priority
                'debounce_interval': 100,          # Minimal debounce
                'batch_updates': False,            # Update immediately for smoothness
            }
        
        # BALANCED MODE (8-16GB RAM, 4-8 cores) - OPTIMIZED FOR SMOOTHNESS
        elif mode == PerformanceMode.BALANCED:
            return {
                # Timer intervals (ms) - OPTIMIZED FOR RESPONSIVENESS
                'ros2_update_interval': 2000,      # 2 seconds - Less blocking, faster UI
                'metrics_update_interval': 300,    # 300ms - Responsive during recording
                'history_update_interval': 10000,  # 10 seconds - Lower priority
                'chart_update_interval': 500,      # 500ms - Smooth charts without jank
                
                # Thread pool settings - OPTIMIZED FOR SPEED
                'max_threads': 4,                  # More parallelism = faster
                'max_concurrent_threads': 3,       # Allow 3 concurrent operations
                
                # Cache settings (seconds) - AGGRESSIVE CACHING = NO BLOCKING
                'cache_timeout': 5,                # 5 seconds - Less subprocess calls
                'system_metrics_cache': 0.5,       # 500ms - Fresh but not excessive
                'topic_check_interval': 5,         # 5 seconds - Reduced blocking
                
                # Chart settings - OPTIMIZED FOR SMOOTH RENDERING
                'chart_buffer_size': 100,          # ~2 minutes at 500ms intervals
                'chart_auto_pause': False,         # Keep running for smooth feel
                
                # Memory settings
                'history_max_entries': 1000,       # Keep history for analysis
                'enable_profiler': True,           # Monitor performance
                'enable_advanced_features': True,  # All features enabled
                
                # UI settings - ULTRA-SMOOTH RENDERING
                'lazy_load_widgets': True,         # Lazy load = faster startup
                'process_priority': 'high',        # Higher priority = smoother execution
                'debounce_interval': 300,          # Min 300ms between updates
                'batch_updates': True,             # Batch table updates = smoother
            }
        
        # LOW PERFORMANCE MODE (<8GB RAM, <4 cores) - OPTIMIZE FOR SMOOTHNESS
        else:  # PerformanceMode.LOW
            return {
                # Timer intervals (ms) - SLOWER BUT SMOOTH
                'ros2_update_interval': 4000,      # 4 seconds - minimal blocking
                'metrics_update_interval': 800,    # 800ms - reasonable during recording
                'history_update_interval': 20000,  # 20 seconds - minimal updates
                'chart_update_interval': 1500,     # 1.5 seconds - acceptable smoothness
                
                # Thread pool settings - MINIMAL CONTENTION
                'max_threads': 2,                  # Minimal threads
                'max_concurrent_threads': 1,       # One at a time to avoid CPU thrashing
                
                # Cache settings (seconds) - MAXIMUM CACHING
                'cache_timeout': 8,                # 8 seconds - aggressive caching
                'system_metrics_cache': 2,         # 2 seconds - reduce system calls
                'topic_check_interval': 8,         # 8 seconds - minimal checks
                
                # Chart settings - OPTIMIZED FOR LOW-END
                'chart_buffer_size': 40,           # 40 seconds of data
                'chart_auto_pause': True,          # Auto-pause to save resources
                
                # Memory settings - CONSERVATIVE
                'history_max_entries': 300,        # Minimal history
                'enable_profiler': False,          # Disable to save resources
                'enable_advanced_features': True,  # Keep features but optimize
                
                # UI settings - SMOOTH ON LOW-END
                'lazy_load_widgets': True,         # Lazy load everything
                'process_priority': 'below_normal', # Lower priority
                'debounce_interval': 500,          # Aggressive debounce
                'batch_updates': True,             # Batch updates = fewer redraws
            }
    
    def set_mode(self, mode: PerformanceMode):
        """Set performance mode"""
        if mode != self._current_mode:
            self._current_mode = mode
            self.mode_changed.emit(mode.value)
    
    def set_custom_settings(self, settings: Dict[str, Any]):
        """Set custom performance settings"""
        self._custom_settings = settings
        self._current_mode = PerformanceMode.CUSTOM
        self.mode_changed.emit('custom')
    
    def get_current_mode(self) -> PerformanceMode:
        """Get current performance mode"""
        return self._current_mode
    
    def get_mode_description(self, mode: Optional[PerformanceMode] = None) -> str:
        """Get human-readable description of mode"""
        if mode is None:
            mode = self._current_mode
        
        descriptions = {
            PerformanceMode.HIGH: (
                "High Performance Mode\n"
                f"Optimized for: 16GB+ RAM, 8+ CPU cores\n"
                f"Your system: {self.system_info['memory_gb']}GB RAM, "
                f"{self.system_info['cpu_count']} cores\n"
                "Features: Ultra-responsive updates, all features enabled, "
                "maximum parallelism"
            ),
            PerformanceMode.BALANCED: (
                "Balanced Mode\n"
                f"Optimized for: 8-16GB RAM, 4-8 CPU cores\n"
                f"Your system: {self.system_info['memory_gb']}GB RAM, "
                f"{self.system_info['cpu_count']} cores\n"
                "Features: Good responsiveness, smart caching, "
                "all features enabled"
            ),
            PerformanceMode.LOW: (
                "Low Performance Mode\n"
                f"Optimized for: <8GB RAM, <4 CPU cores\n"
                f"Your system: {self.system_info['memory_gb']}GB RAM, "
                f"{self.system_info['cpu_count']} cores\n"
                "Features: Resource-efficient, aggressive caching, "
                "features optimized for performance"
            ),
            PerformanceMode.CUSTOM: (
                "Custom Mode\n"
                "User-defined performance settings"
            )
        }
        
        return descriptions.get(mode, "Unknown mode")
    
    def get_system_info_text(self) -> str:
        """Get formatted system information text"""
        info = self.system_info
        return (
            f"System Information:\n"
            f"  Memory: {info['memory_gb']} GB\n"
            f"  CPU Cores: {info['cpu_count']}\n"
            f"  CPU Frequency: {info['cpu_freq_mhz']} MHz\n"
            f"  Disk: {info['disk_gb']} GB ({'SSD' if info['is_ssd'] else 'HDD'})\n"
            f"  Platform: {info['platform']} ({info['architecture']})\n"
            f"\n"
            f"Auto-Detected Mode: {self._current_mode.value.upper()}"
        )
    
    def should_use_feature(self, feature_name: str) -> bool:
        """Check if a feature should be enabled in current mode"""
        settings = self.get_mode_settings()
        
        # Features that can be disabled in low-end mode
        low_end_optional_features = {
            'performance_profiler': settings.get('enable_profiler', True),
            'live_charts': True,  # Always enabled but auto-paused
            'recording_triggers': settings.get('enable_advanced_features', True),
            'export_functionality': True,  # Always available
        }
        
        return low_end_optional_features.get(feature_name, True)
