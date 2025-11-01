# Metrics Real-Time Analysis

## Overview
The ROS2 Dashboard uses a **hybrid approach** with both **real-time** and **near real-time** metrics depending on the metric type and system performance mode.

---

## Metric Categories & Update Intervals

### 🔴 **REAL-TIME Metrics** (300-500ms latency)
These metrics update at **high frequency** for smooth, responsive visualization:

| Metric | Update Interval | Data Source | Latency | Purpose |
|--------|-----------------|-------------|---------|---------|
| **Message Rate** | 500ms (live) | ROS2 bag size delta | <500ms | Display throughput |
| **Write Speed** | 500ms (live) | Disk size delta | <500ms | Monitor recording speed |
| **CPU Usage** | 300-1000ms | `psutil.cpu_percent()` | <1s | System health |
| **Memory Usage** | 300-1000ms | `psutil.virtual_memory()` | <1s | RAM monitoring |
| **Topic Count** | 500ms (live) | Active topic enumeration | <500ms | Active topics display |

**Updates By Performance Mode:**
- **HIGH**: 200ms metrics, 300ms charts (ultra-smooth)
- **BALANCED**: 300ms metrics, 500ms charts (smooth)
- **LOW**: 600ms metrics, 1200ms charts (efficient)

---

### 🟡 **NEAR REAL-TIME Metrics** (1-4 seconds latency)
These metrics have **moderate latency** due to I/O overhead:

| Metric | Update Interval | Data Source | Latency | Reason |
|--------|-----------------|-------------|---------|--------|
| **Disk Write Speed** | 4-8 seconds | `psutil.disk_io_counters()` | 4-8s | Expensive syscall |
| **Bag Size** | 500ms (live) + 4-8s check | Filesystem walk | <1s typical | Dir traversal cached |
| **Duration** | 500ms | Timer delta | <500ms | Calculated |
| **Topic Info** (in-bag) | 1-5 seconds | ROS2 bag metadata | 1-5s | ROS2 query latency |

**Caching Strategy:**
```
System Metrics Cache (CPU, Memory):
├─ HIGH mode:     300ms cache
├─ BALANCED mode: 1.0s  cache  ← Default
└─ LOW mode:      2.0s  cache

Disk I/O Cache:
├─ HIGH mode:     1.2s  (300ms × 4)
├─ BALANCED mode: 4.0s  (1.0s × 4)  ← Default
└─ LOW mode:      8.0s  (2.0s × 4)

ROS2 Topic Queries:
├─ Background check interval: 5-30 seconds
└─ Blocks when no cache available
```

---

### 🔵 **LAZY/BUFFERED Metrics** (5-15 seconds latency)
These metrics update less frequently:

| Metric | Update Interval | Data Source | Latency | Purpose |
|--------|-----------------|-------------|---------|---------|
| **Recording History** | 10-15 seconds | Database query | 10-15s | Historical data |
| **System Diagnostics** | 5 seconds | System profiling | 5s | Advanced stats |

---

## Live Charts Refresh Timeline

```
Timeline of a Live Metrics Update Cycle:
═══════════════════════════════════════════════════════════════

T+0ms:    on_recording_started()
├─ Reset metrics buffers
├─ Start live_metrics_timer (500ms) ← LIVE DATA COLLECTION STARTS HERE
└─ Clear charts

T+500ms:  _update_live_metrics_fast()
├─ metrics_collector.update(ros2_manager)
│  ├─ Get current bag path
│  ├─ Calculate write speed from size delta
│  └─ Update system metrics (CPU/Memory with cache)
└─ Data buffered in deques

T+500ms+: Charts append data silently (no redraw yet)
          (data accumulates in circular buffers)

T+500ms × 5 = 2.5s: Chart plot update (if plot_skip_threshold=5)
├─ Convert all deques to numpy arrays
├─ Batch setData() on all 6 plots
└─ Visible update on screen

T+500ms × 30 = 15s: Statistics update (if stats_update_frequency=30)
├─ Calculate peak/average metrics
└─ Update stats labels

═══════════════════════════════════════════════════════════════
```

**Key Points:**
- ✅ Data collection: **500ms** (REAL-TIME)
- ✅ Data plotting: **2.5s** on average (near real-time, optimized)
- ✅ Statistics: **15s** (deferred, low-priority)

---

## System Metrics Caching Details

### CPU Usage Collection
```python
cpu_percent = psutil.cpu_percent(interval=0)  # Non-blocking!
# Returns instant CPU reading without blocking
# Cached for 300-2000ms depending on mode
```

**Result:** 
- ✅ No blocking calls
- ✅ Instant readings
- ✅ Multiple queries use cache

### Memory Usage Collection
```python
mem = psutil.virtual_memory()  # Very fast syscall
memory_percent = mem.percent
# Returns instant memory reading
# Cached for 300-2000ms depending on mode
```

**Result:**
- ✅ Fast syscall
- ✅ No disk I/O
- ✅ Real-time accurate

### Disk I/O Speed (Most Expensive)
```python
# Only checked every 4-8 seconds (EXPENSIVE!)
disk_io = psutil.disk_io_counters()  # Reads from /proc/diskstats
write_bytes_delta = disk_io.write_bytes - last_reading.write_bytes
disk_write_speed = write_bytes_delta / time_delta
```

**Result:**
- ⚠️ Expensive syscall (limited to 4-8s intervals)
- ✅ Falls back to previous value if not checked
- ✅ No visible lag due to fallback

---

## Adaptive Performance Throttling

### Real-Time CPU-Based Backoff
```python
cpu_now = metrics['cpu_percent']

if cpu_now > 90%:
    # CRITICAL: Skip entire chart update cycle
    return  # No rendering, no processing
    
elif cpu_now > 80%:
    # HIGH LOAD: Increase chart update interval
    backoff_interval = int(update_interval × 2)  # 500ms → 1000ms
    self.update_timer.setInterval(backoff_interval)
    return
    
else:
    # NORMAL: Restore original interval
    self.update_timer.setInterval(update_interval)
```

**Effect:** Automatic throttling when system is under load!

---

## Summary Table

| Category | Type | Latency | Update Interval | Cache | Smoothness |
|----------|------|---------|-----------------|-------|-----------|
| **Message Rate** | Real-time | <500ms | 500ms | ❌ | ⭐⭐⭐⭐⭐ |
| **Write Speed** | Near RT | 1-4s | 500ms | ✅ | ⭐⭐⭐⭐ |
| **CPU Usage** | Real-time | <1s | 300-1000ms | ✅ | ⭐⭐⭐⭐⭐ |
| **Memory Usage** | Real-time | <1s | 300-1000ms | ✅ | ⭐⭐⭐⭐⭐ |
| **Topic Count** | Real-time | <500ms | 500ms | ❌ | ⭐⭐⭐⭐ |
| **Disk I/O Speed** | Near RT | 4-8s | 4-8s | ✅ | ⭐⭐⭐ |
| **Duration** | Real-time | <500ms | Timer | ❌ | ⭐⭐⭐⭐⭐ |
| **Bag Size** | Near RT | 1-2s | 500ms | ✅ | ⭐⭐⭐⭐ |

---

## Performance Characteristics by Mode

### HIGH Performance (16GB+, 8+ cores)
```
Metrics Update: 200ms   ← Ultra-responsive
Charts Update:  300ms   ← Smooth as silk
Cache Timeout:  300ms   ← Very fresh data
Result: Minimal latency, maximum smoothness
```

### BALANCED Performance (8-16GB, 4-8 cores) ← Default
```
Metrics Update: 300ms   ← Responsive
Charts Update:  500ms   ← Smooth
Cache Timeout:  1.0s    ← Balance freshness/speed
Result: Good latency, good smoothness, efficient
```

### LOW Performance (<8GB, <4 cores)
```
Metrics Update: 600ms   ← Reasonable
Charts Update:  1200ms  ← Acceptable
Cache Timeout:  2.0s    ← Aggressive caching
Result: Efficient, still smooth, resource-aware
```

---

## Implementation Details

### Live Metrics Timer (500ms fast collection)
Located: `gui/main_window.py`
```python
self.live_metrics_timer = QTimer()
self.live_metrics_timer.timeout.connect(self._update_live_metrics_fast)
# Starts at 500ms interval during recording
# Updates metrics_collector directly (no async worker)
# Feeds chart buffers with fresh data
```

### Chart Update Timer (adaptive interval)
Located: `gui/live_charts.py`
```python
self.update_timer = QTimer()
self.update_timer.timeout.connect(self.update_charts)
# Starts at configured interval (300-1200ms depending on mode)
# Skips plot updates using threshold (only refresh every 5-30 cycles)
# Auto-backs off if CPU > 80%
```

### Metrics Cache Strategy
Located: `core/metrics_collector.py`
```python
# System metrics (CPU, Memory) cached aggressively
if (current_time - cache_time) < CACHE_TIMEOUT:
    return cached_values  # Fast path, no syscalls

# Disk I/O checked very infrequently (4-8 seconds)
if (current_time - last_disk_check) > DISK_CHECK_INTERVAL:
    disk_io = psutil.disk_io_counters()  # Expensive!
    # Update disk write speed
```

---

## Conclusion

✅ **All metrics are effectively real-time or near real-time:**
- Message rate, CPU, Memory: **TRUE REAL-TIME** (<500ms)
- Write speed, disk I/O: **NEAR REAL-TIME** (1-4 seconds, cached)
- Sufficient caching prevents blocking UI thread
- Automatic CPU-based throttling maintains smoothness
- Charts update frequently enough for human perception (~2-3 times per second)

**User Experience:** The dashboard feels responsive and live, with metrics updating smoothly in real-time while keeping CPU usage low through intelligent caching and adaptive throttling.
