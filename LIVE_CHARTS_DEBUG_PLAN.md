# Live Charts and Recording Metrics - Diagnostic and Fix

## Issues Identified from Screenshot

### 1. Recording Active But Showing "NO DATA" for Topics
- Topics `/parameter_events` and `/rosout` show "NO DATA ⚠️ STALLED"
- These are system topics that ARE publishing, so this is incorrect
- **Root Cause**: Topic monitoring may have stale rate data or incorrect stalleddetection

### 2. Statistics Summary Showing All Zeros
- Peak Message Rate: 0
- Avg Message Rate: 0  
- Peak Bandwidth: 0
- Avg Bandwidth: 0
- Peak CPU: 0
- Avg CPU: 0
- Peak Memory: 0
- Avg Memory: 0
- **Root Cause**: Live charts data buffers are empty or not being populated

### 3. Live Charts Tab Empty/Not Showing Data
- System metrics (CPU, Memory) SHOULD be visible even during recording
- Charts should show at least CPU and Memory data
- **Root Cause**: Charts may be paused or data collection not working

## Fixes to Apply

### Fix 1: Ensure System Metrics Always Collected in Live Charts
The `get_live_metrics()` correctly returns system metrics (tested: CPU=50%, Memory=76%).
But the live charts widget may not be calling `update_system_metrics()` before collecting.

### Fix 2: Make Sure Charts Are Not Paused on Tab Switch
The `auto_pause` feature should NOT stop data collection, only display updates.

### Fix 3: Ensure Statistics Update Immediately
The statistics panel should show values even with just a few data points.

## Implementation

### Changes to gui/live_charts.py:
1. Ensure `update_charts()` always populates buffers, even when paused
2. Reduce minimum data points for statistics from 1 to 0 (show zeros explicitly)
3. Add debug logging to verify data flow

### Changes to core/metrics_collector.py:
1. Ensure `update_system_metrics()` is called in `get_live_metrics()`
2. Verify system metrics cache is working properly

### Changes to recording topic monitoring:
1. Fix "NO DATA" issue - topics should show actual Hz, not "STALLED"
2. Check rate monitoring logic in recording_control.py

