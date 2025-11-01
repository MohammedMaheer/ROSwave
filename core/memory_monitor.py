"""
Memory Monitor - Prevents OOM kills by monitoring and managing memory usage
"""

import psutil
import threading
import time
from typing import Callable, Optional


class MemoryMonitor:
    """
    Monitors system memory and triggers callbacks when memory usage is high.
    Helps prevent OOM killer (exit code 137) by proactively reducing memory usage.
    """
    
    def __init__(self, warning_threshold=75.0, critical_threshold=85.0):
        """
        Initialize memory monitor.
        
        Args:
            warning_threshold: Memory usage % to trigger warning callback (default 75%)
            critical_threshold: Memory usage % to trigger critical callback (default 85%)
        """
        self.warning_threshold = warning_threshold
        self.critical_threshold = critical_threshold
        
        self.warning_callback: Optional[Callable] = None
        self.critical_callback: Optional[Callable] = None
        
        self._running = False
        self._monitor_thread = None
        self._check_interval = 5.0  # Check every 5 seconds
        
        self._last_warning = 0
        self._last_critical = 0
        self._cooldown = 30.0  # Don't spam callbacks more than once per 30 seconds
        
    def set_warning_callback(self, callback: Callable):
        """Set callback for warning threshold (75% memory)"""
        self.warning_callback = callback
        
    def set_critical_callback(self, callback: Callable):
        """Set callback for critical threshold (85% memory)"""
        self.critical_callback = callback
        
    def start(self):
        """Start monitoring memory in background thread"""
        if self._running:
            return
        
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        print("🔍 Memory monitor started")
        
    def stop(self):
        """Stop monitoring"""
        self._running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
        print("🛑 Memory monitor stopped")
        
    def get_memory_info(self):
        """Get current memory usage information"""
        mem = psutil.virtual_memory()
        return {
            'percent': mem.percent,
            'available_mb': mem.available / (1024 * 1024),
            'used_mb': mem.used / (1024 * 1024),
            'total_mb': mem.total / (1024 * 1024),
            'status': self._get_status(mem.percent)
        }
        
    def _get_status(self, percent):
        """Get memory status based on usage"""
        if percent >= self.critical_threshold:
            return 'critical'
        elif percent >= self.warning_threshold:
            return 'warning'
        else:
            return 'ok'
        
    def _monitor_loop(self):
        """Background monitoring loop"""
        while self._running:
            try:
                mem = psutil.virtual_memory()
                current_time = time.time()
                
                # Check critical threshold
                if mem.percent >= self.critical_threshold:
                    if (current_time - self._last_critical) > self._cooldown:
                        self._last_critical = current_time
                        if self.critical_callback:
                            print(f"⚠️  CRITICAL: Memory usage at {mem.percent:.1f}%")
                            self.critical_callback(mem.percent)
                
                # Check warning threshold
                elif mem.percent >= self.warning_threshold:
                    if (current_time - self._last_warning) > self._cooldown:
                        self._last_warning = current_time
                        if self.warning_callback:
                            print(f"⚠️  WARNING: Memory usage at {mem.percent:.1f}%")
                            self.warning_callback(mem.percent)
                
                time.sleep(self._check_interval)
                
            except Exception as e:
                print(f"Memory monitor error: {e}")
                time.sleep(self._check_interval)
                
    def force_garbage_collection(self):
        """Force Python garbage collection to free memory"""
        import gc
        before = psutil.virtual_memory().percent
        collected = gc.collect()
        after = psutil.virtual_memory().percent
        freed = before - after
        print(f"🗑️  Garbage collection: freed {freed:.1f}% memory ({collected} objects)")
        return freed


class MemoryOptimizer:
    """
    Optimizes memory usage by reducing cache sizes and clearing unused data.
    Works in conjunction with MemoryMonitor.
    """
    
    def __init__(self, ros2_manager=None, async_manager=None, metrics_collector=None):
        self.ros2_manager = ros2_manager
        self.async_manager = async_manager
        self.metrics_collector = metrics_collector
        
    def reduce_cache_sizes(self):
        """Reduce cache sizes to save memory"""
        print("📦 Reducing cache sizes...")
        
        # Reduce ROS2 manager cache
        if self.ros2_manager and hasattr(self.ros2_manager, '_cache'):
            cache_len = len(self.ros2_manager._cache)
            # Keep only most recent entries
            if cache_len > 10:
                with self.ros2_manager._cache_lock:
                    # Keep only last 5 entries
                    keys = list(self.ros2_manager._cache.keys())
                    for key in keys[:-5]:
                        self.ros2_manager._cache.pop(key, None)
                        self.ros2_manager._cache_timestamps.pop(key, None)
                print(f"  Reduced ROS2 cache from {cache_len} to 5 entries")
        
        # Reduce async manager cache
        if self.async_manager and hasattr(self.async_manager, '_cache'):
            cache_len = len(self.async_manager._cache)
            if cache_len > 10:
                with self.async_manager._lock:
                    keys = list(self.async_manager._cache.keys())
                    for key in keys[:-5]:
                        self.async_manager._cache.pop(key, None)
                        self.async_manager._cache_timestamps.pop(key, None)
                print(f"  Reduced async cache from {cache_len} to 5 entries")
        
    def clear_old_data(self):
        """Clear old cached data that's no longer needed"""
        print("🧹 Clearing old cached data...")
        
        # Clear ROS2 manager cache
        if self.ros2_manager and hasattr(self.ros2_manager, '_cache'):
            with self.ros2_manager._cache_lock:
                self.ros2_manager._cache.clear()
                self.ros2_manager._cache_timestamps.clear()
            print("  Cleared ROS2 manager cache")
        
        # Clear async manager cache
        if self.async_manager and hasattr(self.async_manager, '_cache'):
            with self.async_manager._lock:
                self.async_manager._cache.clear()
                self.async_manager._cache_timestamps.clear()
            print("  Cleared async manager cache")
        
    def emergency_cleanup(self):
        """Emergency memory cleanup - called when memory is critically low"""
        print("🚨 EMERGENCY MEMORY CLEANUP")
        
        # Clear all caches
        self.clear_old_data()
        
        # Force garbage collection
        import gc
        collected = gc.collect()
        print(f"  Garbage collected {collected} objects")
        
        # Get memory status
        mem = psutil.virtual_memory()
        print(f"  Memory after cleanup: {mem.percent:.1f}% used")
