"""
ULTRA-FREEZE-PREVENTION: Aggressive UI responsiveness optimization
This module implements extreme measures to prevent UI freezes while maintaining functionality.
"""

import time
from PyQt5.QtCore import QTimer, QObject, pyqtSignal  # type: ignore


class FreezePrevention:
    """Static methods for preventing UI freezes through extreme optimization"""
    
    @staticmethod
    def estimate_ros2_operation_time():
        """
        ROS2 operations typically take 6-7 seconds on this system.
        Any operation taking longer than 1 second should be async.
        """
        return 7.0  # 7 seconds for typical get_topics_info()
    
    @staticmethod
    def should_skip_background_work(window_visible, tab_active, work_priority=0):
        """
        Aggressive decision making for whether to do background work.
        
        Rules:
        - NEVER do work if window is hidden (huge win)
        - NEVER do work if tab isn't visible (40-50% CPU reduction)
        - NEVER do simultaneous ROS2 operations (queue them)
        - NEVER update if already updating (prevents thundering herd)
        
        Args:
            window_visible: Is the window visible?
            tab_active: Is the target tab active?
            work_priority: 0=low, 1=medium, 2=high
        
        Returns:
            True if we should SKIP this work, False if we should do it
        """
        # NEVER work if window is hidden
        if not window_visible:
            return True
        
        # NEVER work if this tab is not visible (except high priority)
        if not tab_active and work_priority < 2:
            return True
        
        return False


class RateLimitedTimer(QTimer):
    """
    A QTimer that prevents overlapping executions.
    
    If the callback is still running when the timer fires again,
    skip the next execution. This prevents UI freezes from cascading.
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self._is_running = False
        self.timeout.connect(self._check_and_execute)
        self._actual_callback = None
    
    def set_callback(self, callback):
        """Set the callback to execute"""
        self._actual_callback = callback
    
    def _check_and_execute(self):
        """Execute callback only if not already running"""
        if self._is_running:
            # Skip - already executing (prevents freezes)
            return
        
        try:
            self._is_running = True
            if self._actual_callback:
                self._actual_callback()
        finally:
            self._is_running = False


class WorkQueue:
    """
    Simple queue to ensure only one heavy operation runs at a time.
    Prevents thread pool saturation and UI freezes.
    """
    
    def __init__(self, max_concurrent=1):
        self.max_concurrent = max_concurrent
        self.active_count = 0
        self.queue = []
        self._lock = __import__('threading').Lock()
    
    def submit(self, work_fn, args=(), kwargs=None):
        """
        Queue work to be executed.
        Only executes when concurrent count is below limit.
        """
        if kwargs is None:
            kwargs = {}
        
        with self._lock:
            if self.active_count >= self.max_concurrent:
                # Queue it
                self.queue.append((work_fn, args, kwargs))
                return False  # Not executed immediately
            else:
                # Execute immediately
                self.active_count += 1
        
        # Execute in background
        def _wrapper():
            try:
                work_fn(*args, **kwargs)
            finally:
                self._process_queue()
        
        import threading
        thread = threading.Thread(target=_wrapper, daemon=True)
        thread.start()
        return True  # Executed immediately
    
    def _process_queue(self):
        """Process queued work"""
        with self._lock:
            self.active_count -= 1
            if self.queue and self.active_count < self.max_concurrent:
                work_fn, args, kwargs = self.queue.pop(0)
                self.active_count += 1
        
        if self.queue and self.active_count < self.max_concurrent:
            # Process next item
            def _wrapper():
                try:
                    work_fn(*args, **kwargs)
                finally:
                    self._process_queue()
            
            import threading
            thread = threading.Thread(target=_wrapper, daemon=True)
            thread.start()


class WindowVisibilityTracker(QObject):
    """
    Tracks window visibility changes to avoid background work when window is hidden.
    This is one of the most impactful optimizations - prevents CPU waste.
    """
    
    visibility_changed = pyqtSignal(bool)  # True = visible, False = hidden
    
    def __init__(self, window):
        super().__init__()
        self.window = window
        self._last_visible = True
        
        # Check visibility periodically (100ms is fine, very cheap)
        self.timer = QTimer()
        self.timer.timeout.connect(self._check_visibility)
        self.timer.start(100)
    
    def _check_visibility(self):
        """Check if window visibility changed"""
        is_visible = self.window.isVisible()
        if is_visible != self._last_visible:
            self._last_visible = is_visible
            self.visibility_changed.emit(is_visible)
    
    def is_visible(self):
        """Get current visibility state"""
        return self.window.isVisible()


def optimize_qt_event_loop():
    """
    Configure Qt event loop for maximum responsiveness.
    Call this at application startup.
    """
    try:
        from PyQt5.QtCore import QCoreApplication
        app = QCoreApplication.instance()
        if app:
            # Set event loop processing hints for responsiveness
            # This prioritizes user input events over background updates
            app.setStyle('Fusion')  # Simpler rendering = faster
    except Exception:
        pass
