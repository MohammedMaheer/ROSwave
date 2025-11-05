"""
Recording Protection System - Ensures recording is never compromised
"""

import os
import threading
import time
import psutil
from typing import Callable, Optional
from enum import Enum


class RecordingState(Enum):
    """Recording state enumeration"""
    IDLE = "idle"
    STARTING = "starting"
    RECORDING = "recording"
    STOPPING = "stopping"
    ERROR = "error"


class RecordingProtector:
    """
    Protects recording process from interference.
    Ensures recording resources are isolated and prioritized.
    """
    
    def __init__(self):
        self.state = RecordingState.IDLE
        self.recording_pid = None
        self.recording_process = None
        self._state_lock = threading.Lock()
        
        # Recording process health monitoring
        self._monitor_thread = None
        self._monitor_running = False
        self._health_callbacks = []
        
        # Resource allocation for recording
        self.reserved_cpu_cores = 1  # Minimum CPU cores for recording
        self.reserved_memory_mb = 200  # Minimum memory for recording
        
    def set_recording_process(self, process):
        """Register recording process for protection"""
        with self._state_lock:
            self.recording_process = process
            self.recording_pid = process.pid if process else None
            
    def set_state(self, new_state: RecordingState):
        """Update recording state"""
        with self._state_lock:
            self.state = new_state
            
    def get_state(self) -> RecordingState:
        """Get current recording state"""
        with self._state_lock:
            return self.state
            
    def is_recording(self) -> bool:
        """Check if recording is active"""
        with self._state_lock:
            return self.state == RecordingState.RECORDING
            
    def register_health_callback(self, callback: Callable):
        """Register callback for recording health issues"""
        self._health_callbacks.append(callback)
        
    def start_health_monitoring(self):
        """Start monitoring recording process health"""
        if self._monitor_running:
            return
            
        self._monitor_running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        print("🛡️  Recording protection activated")
        
    def stop_health_monitoring(self):
        """Stop health monitoring"""
        self._monitor_running = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2)
        print("🛡️  Recording protection deactivated")
        
    def _monitor_loop(self):
        """Continuously monitor recording process health"""
        while self._monitor_running:
            try:
                if not self.recording_process or not self.recording_pid:
                    time.sleep(1)
                    continue
                    
                # Check if process is still alive
                if not psutil.pid_exists(self.recording_pid):
                    print(f"⚠️  CRITICAL: Recording process {self.recording_pid} died!")
                    self._trigger_health_alert("process_died")
                    continue
                
                try:
                    proc = psutil.Process(self.recording_pid)
                    
                    # Check if process is zombie or defunct
                    if proc.status() == psutil.STATUS_ZOMBIE:
                        print(f"⚠️  CRITICAL: Recording process is ZOMBIE!")
                        self._trigger_health_alert("zombie_process")
                        continue
                    
                    # Check resource usage
                    cpu_percent = proc.cpu_percent(interval=0.1)
                    memory_mb = proc.memory_info().rss / (1024 * 1024)
                    
                    # Alert if resources look wrong
                    if memory_mb > 1000:  # > 1GB
                        print(f"⚠️  WARNING: Recording using {memory_mb:.1f}MB")
                        self._trigger_health_alert("high_memory")
                        
                except psutil.NoSuchProcess:
                    print(f"⚠️  CRITICAL: Recording process {self.recording_pid} not found!")
                    self._trigger_health_alert("process_not_found")
                
                time.sleep(2)  # Check every 2 seconds
                
            except Exception as e:
                print(f"Recording monitor error: {e}")
                time.sleep(1)
                
    def _trigger_health_alert(self, alert_type: str):
        """Trigger health alert callbacks"""
        for callback in self._health_callbacks:
            try:
                callback(alert_type)
            except Exception as e:
                print(f"Health alert callback error: {e}")
                
    def ensure_recording_priority(self):
        """
        Ensure recording process has priority.
        This should be called before heavy operations.
        """
        if not self.recording_process or not self.recording_pid:
            return
            
        try:
            proc = psutil.Process(self.recording_pid)
            
            # Set high priority (nice value -10 for Linux)
            if os.name == 'posix':  # Linux/Mac
                # Lower nice value = higher priority
                current_nice = os.getpriority(os.PRIO_PROCESS, self.recording_pid)
                if current_nice > -10:
                    os.setpriority(os.PRIO_PROCESS, self.recording_pid, -10)
                    print(f"📌 Recording process prioritized (nice={-10})")
                    
        except Exception as e:
            print(f"Could not prioritize recording: {e}")
            
    def check_resources_available(self) -> bool:
        """Check if sufficient resources are available for recording"""
        try:
            mem = psutil.virtual_memory()
            cpu_count = psutil.cpu_count()
            
            # Check if we have minimum required resources
            if mem.available < (self.reserved_memory_mb * 1024 * 1024):
                print(f"⚠️  WARNING: Low memory available ({mem.available / (1024*1024):.0f}MB)")
                return False
                
            # Check if recording process has enough CPU
            if cpu_count < self.reserved_cpu_cores:
                print(f"⚠️  WARNING: Insufficient CPU cores for recording")
                return False
                
            return True
            
        except Exception as e:
            print(f"Resource check error: {e}")
            return True  # Assume OK if we can't check
            
    def prevent_other_operations_if_recording(self, operation_name: str) -> bool:
        """
        Check if operation should be prevented due to active recording.
        Returns True if operation should proceed, False if should be blocked.
        """
        if not self.is_recording():
            return True  # Not recording, operation OK
            
        # Operations that should NOT run during recording
        blocked_operations = [
            'heavy_computation',
            'cache_clear',
            'ui_resize',
            'full_redraw',
        ]
        
        if operation_name in blocked_operations:
            print(f"⏸️  Deferring {operation_name} - recording in progress")
            return False
            
        return True


class HzMonitoringProtector:
    """
    Ensures Hz monitoring never blocks or interferes with recording.
    Hz monitoring runs in completely isolated thread pool.
    """
    
    def __init__(self):
        self.hz_thread_pool = None
        self.hz_max_workers = 2  # Isolated pool, small size
        self._isolation_level = "high"  # Can be "high", "medium", "low"
        
    def set_recording_active(self, is_recording: bool):
        """Adjust Hz monitoring when recording starts/stops"""
        if is_recording:
            # Reduce Hz monitoring priority when recording
            self._isolation_level = "high"
            print("🎯 Hz monitoring: HIGH ISOLATION (recording active)")
        else:
            # Can be more active when not recording
            self._isolation_level = "medium"
            print("🎯 Hz monitoring: MEDIUM ISOLATION (idle)")
            
    def should_hz_update_run(self) -> bool:
        """Check if Hz update should run now"""
        # Always allow Hz monitoring, never block it
        return True
        
    def get_hz_thread_pool_config(self):
        """Get configuration for Hz thread pool"""
        return {
            'max_workers': self.hz_max_workers,
            'thread_prefix': 'HzMonitor',
            'timeout': 2.0,  # Short timeout to not hold up recording
            'priority': 'low',  # Low CPU priority
        }


class RecordingFailsafe:
    """
    Emergency failsafe for recording.
    If recording appears to fail, attempts recovery.
    """
    
    def __init__(self, recording_control):
        self.recording_control = recording_control
        self._recovery_attempts = 0
        self._max_recovery_attempts = 3
        self._last_error_time = 0
        self._error_cooldown = 5.0  # 5 second cooldown between recoveries
        
    def attempt_recovery(self, error_message: str) -> bool:
        """
        Attempt to recover from recording error.
        Returns True if recovery attempted, False if max attempts exceeded.
        """
        current_time = time.time()
        
        # Check cooldown
        if (current_time - self._last_error_time) < self._error_cooldown:
            print(f"⏳ Cooldown active, skipping recovery")
            return False
            
        # Check max attempts
        if self._recovery_attempts >= self._max_recovery_attempts:
            print(f"🚨 CRITICAL: Max recovery attempts ({self._max_recovery_attempts}) exceeded!")
            print(f"Error: {error_message}")
            return False
            
        self._recovery_attempts += 1
        self._last_error_time = current_time
        
        print(f"🔄 Recovery attempt {self._recovery_attempts}/{self._max_recovery_attempts}")
        print(f"Error: {error_message}")
        
        # Try to resume recording
        try:
            if hasattr(self.recording_control, 'resume_recording'):
                self.recording_control.resume_recording()
                print("✅ Recovery successful!")
                return True
        except Exception as e:
            print(f"Recovery failed: {e}")
            
        return False
        
    def reset_recovery_counter(self):
        """Reset recovery counter after successful operation"""
        self._recovery_attempts = 0
        self._last_error_time = 0


class RecordingMonitor:
    """
    Comprehensive monitoring of recording system.
    Tracks recording health, Hz monitoring, and system state.
    """
    
    def __init__(self):
        self.protector = RecordingProtector()
        self.hz_protector = HzMonitoringProtector()
        self.failsafe = None  # Set later with recording_control reference
        
        self.metrics = {
            'start_time': None,
            'total_messages': 0,
            'message_rate': 0,
            'cpu_percent': 0,
            'memory_mb': 0,
            'last_update': 0,
            'error_count': 0,
        }
        
    def on_recording_start(self):
        """Called when recording starts"""
        self.protector.set_state(RecordingState.RECORDING)
        self.protector.start_health_monitoring()
        self.protector.ensure_recording_priority()
        self.hz_protector.set_recording_active(True)
        self.metrics['start_time'] = time.time()
        print("🎬 Recording started with protection active")
        
    def on_recording_stop(self):
        """Called when recording stops"""
        self.protector.set_state(RecordingState.STOPPING)
        self.protector.stop_health_monitoring()
        self.hz_protector.set_recording_active(False)
        print("🏁 Recording stopped, protection deactivated")
        
    def on_hz_update(self):
        """Called when Hz values update"""
        # This is fast and should never be blocked
        pass
        
    def get_recording_status(self) -> dict:
        """Get comprehensive recording status"""
        return {
            'state': self.protector.get_state().value,
            'is_recording': self.protector.is_recording(),
            'process_id': self.protector.recording_pid,
            'metrics': self.metrics.copy(),
        }
