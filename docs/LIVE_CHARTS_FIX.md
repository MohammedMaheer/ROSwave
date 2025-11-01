# Live Charts Fix - Comprehensive Diagnosis

## Problem Statement
Live Charts tab shows empty charts with all 0 values, even during recording.

## Root Causes Identified

### 1. **Metrics Collection Flow Broken**
- `metrics_collector.update()` is called but data isn't flowing to charts
- The fast live metrics timer (`live_metrics_timer`) was not being started until recording starts
- Chart update timer might be paused by hideEvent

### 2. **Data Type Issues**
- Metrics dict could contain None or non-numeric values
- Charts expect numeric arrays but were getting zeros

### 3. **Chart Tab Visibility Issue**
- Live Charts tab is not visible on startup
- hideEvent was stopping the update_timer
- Charts weren't collecting data in the background

## Fixes Applied

### Fix 1: Guaranteed Numeric Metrics (metrics_collector.py)
```python
def get_live_metrics(self, ros2_manager=None):
    """Get live metrics including system stats (lightweight topic checks) - GUARANTEED NUMERIC"""
    # All values are converted to numeric types
    # Handles None, strings, missing keys
    safe_metrics = {
        'duration': float(metrics_copy.get('duration', 0.0) or 0.0),
        'cpu_percent': float(metrics_copy.get('cpu_percent', 0.0) or 0.0),
        # ... all metrics guaranteed numeric
    }
    return safe_metrics
```

### Fix 2: Fast Live Metrics Timer (main_window.py)
```python
# Added live_metrics_timer that runs at 500ms during recording
self.live_metrics_timer = QTimer()
self.live_metrics_timer.timeout.connect(self._update_live_metrics_fast)

# Started when recording begins
def on_recording_started(self):
    if not self.live_metrics_timer.isActive():
        self.live_metrics_timer.start(500)  # 500ms = 2 Hz
```

### Fix 3: Keep Charts Running in Background (live_charts.py)
```python
def hideEvent(self, event):
    """Don't stop the timer even when tab is hidden"""
    super().hideEvent(event)
    if hasattr(self, 'update_timer') and self.auto_pause:
        pass  # Don't stop - keep running!
```

## Testing Checklist

- [ ] Start recording
- [ ] Navigate to Live Charts tab
- [ ] Verify charts show non-zero values
- [ ] Check that values increase over time
- [ ] Verify no UI freezing occurs
- [ ] Check CPU usage is low

## Files Modified

1. `core/metrics_collector.py` - Added type-safe metrics return
2. `gui/main_window.py` - Added live_metrics_timer for fast updates
3. `gui/live_charts.py` - Fixed hideEvent to keep timer running

## Expected Behavior After Fix

1. Recording starts → metrics_collector.reset() called
2. live_metrics_timer starts at 500ms interval
3. Timer calls _update_live_metrics_fast() which calls metrics_collector.update()
4. metrics_collector updates system metrics (CPU, memory)
5. Live charts calls get_live_metrics() every 300-400ms
6. Charts receive numeric data and display it
7. Charts render without freezing
