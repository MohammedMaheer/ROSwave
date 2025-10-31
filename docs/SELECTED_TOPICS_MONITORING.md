# 📊 Selected Topics Monitoring Feature

## Overview
The ROS2 Dashboard now includes a **Selected Topics Monitor** in the Recording Control tab that displays real-time information about the topics you're recording, including live message rates and connection status alerts.

## Features

### 1. **Live Topic Display**
When you select topics in the Topics tab, they automatically appear in the "Selected Topics (Live Rates)" section in the Recording Control tab.

**Display includes:**
- ✅ Topic Name (monospace for clarity)
- 📊 Current Message Rate (Hz)
- 🟢 Connection Status (OK, STALLED, NO DATA)

### 2. **Real-Time Rate Updates**
During active recording:
- Topic message rates update every 1 second
- Rates show as "Hz" (messages per second)
- Helps you verify topics are actively publishing

### 3. **Status Alerts**
The system monitors each topic and provides visual feedback:

| Status | Color | Meaning |
|--------|-------|---------|
| ✅ OK | Green | Topic is actively publishing (rate > 0) |
| ⏸️ NO DATA | Orange | Topic exists but no messages received |
| ⚠️ STALLED | Red | Topic stopped publishing or rate dropped to 0 |

### 4. **Stall Detection**
The system automatically detects when a previously publishing topic stops sending messages:
- Tracks previous rate for each topic
- Flags stalling topics in RED
- Counts consecutive stalls per topic
- Great for debugging connection issues

## How to Use

### Step 1: Select Topics
1. Go to the **📡 Topics** tab
2. Check the topics you want to record
3. Use "Select All" button for quick selection

### Step 2: Start Recording
1. Go back to the **🎙️ Recording** tab
2. Verify your selected topics appear in the table
3. Click "Start Recording"

### Step 3: Monitor Rates
- Watch the table update in real-time during recording
- Green = healthy topic
- Orange = no data (might be pause)
- Red = potential issue (stalled or disconnected)

### Step 4: Take Action
If you see RED stalled topics:
1. Check if the ROS2 node is still running
2. Verify network connectivity to the robot
3. Click "Retry Failed" in the Network Robots tab if using remote robots
4. Adjust recording filters if needed

## Implementation Details

### Files Modified
- `gui/recording_control.py` - Added Selected Topics Monitor UI and rate tracking
- `gui/topic_monitor.py` - Added `topics_changed` signal emission
- `gui/main_window.py` - Connected topic selection to recording control

### Methods Added

#### RecordingControlWidget
- `update_selected_topics(selected_topics)` - Update the list of topics to monitor
- `refresh_selected_topics_table()` - Refresh the UI table display
- `update_topic_rates()` - Update rates periodically (called every 1 second)
- `start_rate_monitoring()` - Start the update timer
- `stop_rate_monitoring()` - Stop the update timer

#### TopicMonitorWidget
- New signal: `topics_changed(list)` - Emitted when selected topics change

### Data Structure
```python
self.selected_topics_data = {
    'topic_name': {
        'rate': 0.0,           # Current rate
        'last_rate': 0.0,      # Previous rate
        'stalled': False,      # Is stalled?
        'stalled_count': 0     # How many times stalled
    },
    ...
}
```

## Performance Impact
- **CPU**: Minimal - only queries ROS2 every 1 second during recording
- **Memory**: ~100 bytes per monitored topic
- **Network**: Depends on ROS2 infrastructure (same as before)

## Troubleshooting

### Topics show "NO DATA" 
- Check if nodes are publishing
- Verify ROS2 domain is correct
- Check firewall settings

### Topics suddenly "STALLED"
- Could indicate node crash
- Check node logs: `ros2 node list` + `ros2 node info <node_name>`
- Verify network connectivity

### Rates don't update
- Recording might not be active
- Check "Status: Recording" label
- Verify at least one topic is selected

### Why no live rates before recording starts?
- Rates are only tracked during active recording to save CPU
- Rates are cached for 5 seconds (optimization)
- Exact rate might vary within ±0.5 Hz due to caching

## Future Enhancements

Potential improvements:
- [ ] Custom rate thresholds for stall detection
- [ ] Graph view of rate over time
- [ ] Alert notifications when topics stall
- [ ] Export rate statistics with recording
- [ ] Per-topic message count statistics
- [ ] Bandwidth calculation for each topic
- [ ] Automatic reconnection for failed topics

## Tips & Tricks

**Pro Tips:**
1. **Green = Trust**: If all topics are green, your recording is solid ✅
2. **Watch for Red**: Red topics are warnings - investigate before continuing
3. **Pattern Recognition**: Multiple red topics = network issue, single red = node issue
4. **Rate Validation**: Compare rates with what you expect - helps catch misconfiguration

**Common Rates:**
- Cameras: 20-30 Hz
- IMU: 100+ Hz
- LiDAR: 10-20 Hz
- Battery: 1 Hz
- GPS: 10 Hz

---

**Status**: ✅ Complete and Ready to Use  
**Last Updated**: 2025-10-28  
**Version**: 1.0
