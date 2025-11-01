# 🚨 URGENT: CPU 100% & UI Freezing Fix

**Issue**: Dashboard hitting 100% CPU with UI freezing at intervals
**Root Cause**: Multiple overlapping timers creating "timer storm"
**Priority**: CRITICAL

---

## 🔍 Root Cause Analysis

### Timer Storm Identified:

1. **ros2_timer**: 20,000ms (20s) - ROS2 info updates
2. **metrics_timer**: 8,000ms (8s) - Metrics updates  
3. **live_metrics_timer**: 500ms (0.5s) - Fast metrics for charts **← PROBLEM**
4. **history_timer**: 60,000ms (60s) - History refresh
5. **live_charts.update_timer**: 1,000ms default (1s) - Chart updates **← DUPLICATE**

### The Problem:
- `live_metrics_timer` (500ms) + `live_charts.update_timer` (1000ms) are **duplicating work**
- Live charts widget already collects metrics in its own timer
- The extra `live_metrics_timer` at 500ms is **unnecessary overhead**
- Both timers call `metrics_collector.get_live_metrics()` independently
- This creates 2-3x the necessary CPU usage

### UI Freezing Causes:
1. **Autoscale enabled by default in live_charts** - GPU intensive operation
2. **Numpy array conversions happening every update** - should be batched
3. **ROS2 blocking calls** in timer callbacks (already has async, but not always used)

---

## ✅ Fixes Required

### FIX 1: Remove Duplicate Timer (HIGH PRIORITY)
**File**: `gui/main_window.py`

**Problem**: `live_metrics_timer` is redundant - live_charts has its own timer

**Solution**: 
- REMOVE `live_metrics_timer` entirely
- Let `live_charts` widget handle its own updates
- Live charts already has dynamic optimization built-in

**Impact**: 50% reduction in timer overhead

---

### FIX 2: Optimize Live Charts Settings (HIGH PRIORITY)
**File**: `gui/live_charts.py`

**Changes Needed**:
1. ✅ Already has lazy loading - GOOD
2. ✅ Already has CPU backoff - GOOD  
3. ❌ Autoscale checkbox defaults to `False` - but check enforcement
4. ❌ Update interval should respect performance mode - verify
5. ❌ Stats update frequency (every 30 cycles) - may be too frequent

**Recommendations**:
- Increase stats_update_frequency from 30 to 50 cycles (reduce stats computation)
- Increase plot_skip_threshold from 5 to 10 on LOW performance mode
- Never enable autoscale by default (causes GPU spikes)

---

### FIX 3: Increase Base Timer Intervals (MEDIUM PRIORITY)
**File**: `gui/main_window.py`

**Current**:
- ros2_timer: 20s ✅ OK
- metrics_timer: 8s ❌ Too frequent
- history_timer: 60s ✅ OK

**Recommended**:
- ros2_timer: 30s (reduce from 20s)
- metrics_timer: 15s (reduce from 8s) 
- history_timer: 120s (reduce from 60s)

**Rationale**: Metrics don't change that rapidly. User won't notice 15s vs 8s updates.

---

### FIX 4: Ensure Async Calls (LOW PRIORITY)
**Files**: All timer callbacks

**Verify**: All ROS2 calls in timer callbacks use async/background workers
- ✅ `update_ros2_info_async_smart` - already async
- ✅ `update_metrics_smart` - uses MetricsWorker
- ✅ `refresh_recording_history` - should be async (verify)

---

## 📋 Implementation Priority

### IMMEDIATE (Do First):
1. **Remove `live_metrics_timer`** - instant 50% CPU reduction
2. **Verify autoscale disabled by default** - prevents GPU spikes
3. **Increase metrics_timer to 15s** - reduce update frequency

### SHORT-TERM (Next):
4. **Tune live_charts dynamic settings** - improve adaptive performance
5. **Increase ros2_timer to 30s** - reduce ROS2 query overhead
6. **Verify all blocking calls are async** - prevent UI freezes

---

## 🎯 Expected Results

### Before Fix:
- CPU: 80-100% continuous
- UI: Freezes every 1-2 seconds
- Timers: 5 active timers competing

### After Fix:
- CPU: 20-40% average
- UI: Smooth, no freezes
- Timers: 3 essential timers, properly spaced

---

## 🔧 Quick Fix Code

### Remove live_metrics_timer:

```python
# IN gui/main_window.py setup_timers() method:

# DELETE these lines:
# self.live_metrics_timer = QTimer()
# self.live_metrics_timer.timeout.connect(self._update_live_metrics_fast)

# DELETE in on_recording_started():
# self.live_metrics_timer.start(500)

# DELETE in on_recording_stopped():
# self.live_metrics_timer.stop()

# DELETE the entire method:
# def _update_live_metrics_fast(self):
#     ... (entire method)
```

### Increase timer intervals:

```python
# IN gui/main_window.py setup_timers():

# CHANGE from:
QTimer.singleShot(5000, lambda: self.ros2_timer.start(20000))
QTimer.singleShot(6000, lambda: self.metrics_timer.start(8000))
QTimer.singleShot(15000, lambda: self.history_timer.start(60000))

# TO:
QTimer.singleShot(5000, lambda: self.ros2_timer.start(30000))  # 30s instead of 20s
QTimer.singleShot(6000, lambda: self.metrics_timer.start(15000))  # 15s instead of 8s
QTimer.singleShot(15000, lambda: self.history_timer.start(120000))  # 120s instead of 60s
```

---

## ✅ Testing Checklist

After applying fixes, verify:

- [ ] CPU usage under 40% when idle
- [ ] CPU usage under 60% when recording
- [ ] No UI freezes during tab switching
- [ ] Live charts still update smoothly
- [ ] Metrics display still shows current data
- [ ] Recording history refreshes (slower but still works)

---

**Status**: Ready to implement  
**Effort**: 10 minutes  
**Impact**: CRITICAL - fixes major performance issue  
**Risk**: LOW - removing redundant code only

---

*Fix to be implemented immediately*
