# Offline-First Network Upload System

## 🌐 Overview

The ROS2 Dashboard now includes a robust **offline-first networking system** that handles data uploads with automatic resume capability. This ensures **zero data loss** even during network failures.

## ✨ Key Features

### 1. **Offline-First Architecture**
- All recordings are saved locally first
- Uploads happen in the background
- System continues working even when offline
- Automatic reconnection when network returns

### 2. **Chunked Upload with Resume**
- Files split into 5MB chunks (configurable)
- Each chunk uploaded independently
- Automatic resume from last successful chunk
- No re-upload of already transferred data

### 3. **Priority Queue System**
- Upload priority levels (1-10, lower = higher priority)
- Real-time data gets higher priority
- Background processing of older recordings
- Fair scheduling across multiple uploads

### 4. **Persistent State**
- SQLite database stores upload progress
- Survives application restarts
- Automatic recovery of pending uploads
- Complete upload history tracking

### 5. **Automatic Retry Logic**
- Configurable retry attempts (default: 5)
- Exponential backoff between retries
- Error logging for debugging
- Smart failure handling

### 6. **Bandwidth Management**
- Optional bandwidth throttling
- Concurrent upload limits
- Non-blocking UI operations
- Background worker threads

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   ROS2 Dashboard                         │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐        ┌──────────────┐              │
│  │   Recording  │───────▶│  Local       │              │
│  │   Manager    │        │  Storage     │              │
│  └──────────────┘        └──────┬───────┘              │
│                                  │                       │
│                                  ▼                       │
│  ┌──────────────────────────────────────────┐          │
│  │         Network Manager                   │          │
│  ├──────────────────────────────────────────┤          │
│  │  • Priority Queue                        │          │
│  │  • Chunk Handler                         │          │
│  │  • Retry Logic                           │          │
│  │  • State Persistence (SQLite)            │          │
│  │  • Network Monitor                       │          │
│  └──────────┬───────────────────────────────┘          │
│             │                                            │
└─────────────┼────────────────────────────────────────────┘
              │
              │ HTTP/HTTPS
              │ Chunked Upload
              │
              ▼
┌─────────────────────────────────────────────────────────┐
│                   Upload Server                          │
├─────────────────────────────────────────────────────────┤
│  • Chunk Reception                                       │
│  • Resume Handling                                       │
│  • Checksum Verification                                 │
│  • File Reassembly                                       │
└─────────────────────────────────────────────────────────┘
```

## 📋 Upload Process Flow

### 1. **Recording Completion**
```
Recording Stops → Bag Saved Locally → Added to Upload Queue
```

### 2. **Upload Initialization**
```python
# Step 1: Initialize upload session
POST /upload/init
{
  "filename": "recording_20251025_120000",
  "filesize": 1073741824,  # 1GB
  "chunks": 215,            # 5MB chunks
  "metadata": {
    "type": "ros2_bag",
    "component": "robot_arm",
    "priority": 3
  },
  "checksum": "md5_hash_here"
}

# Response
{
  "upload_id": "uuid-generated-id",
  "success": true
}
```

### 3. **Chunk Upload (with Resume)**
```python
# Upload each chunk
POST /upload/chunk
FormData:
  - upload_id: "uuid"
  - chunk_index: 0
  - chunk_total: 215
  - chunk: <binary_data>

# Server Response
{
  "received_chunks": 1,
  "total_chunks": 215,
  "progress": 0.46
}

# If network fails at chunk 50:
# - Chunks 0-49 are marked as uploaded
# - On reconnect, resume from chunk 50
# - No re-upload of completed chunks
```

### 4. **Finalization**
```python
# After all chunks uploaded
POST /upload/finalize
{
  "upload_id": "uuid",
  "checksum": "md5_hash_here"
}

# Server verifies checksum and combines chunks
{
  "success": true,
  "file_path": "/uploads/completed/recording_20251025_120000"
}
```

## 🚀 Usage Guide

### Starting the Upload Server

```bash
# Terminal 1: Start upload server
cd /tmp/ros2_dashboard
python3 upload_server.py

# Server runs on http://localhost:8080
# Uploads saved to ~/ros2_uploads/completed/
```

### Configuring Auto-Upload in Dashboard

1. Launch the dashboard
2. Go to **☁️ Upload** tab
3. Set server URL (default: `http://localhost:8080/upload`)
4. Enable "Auto-upload completed recordings"
5. Set upload priority (1-10)
6. Start recording as usual

### Manual Upload

1. Go to **☁️ Upload** tab
2. Click "Upload File..."
3. Select any file (bag or other)
4. File is queued and uploaded automatically

### Monitoring Uploads

**Pending Uploads Table** shows:
- File name
- Priority level
- Current status (PENDING/UPLOADING/PAUSED/COMPLETED)
- Progress bar with percentage
- File size
- Retry count

**Upload History** shows:
- Recent uploads
- Success/failure status
- Completion time
- Upload duration

## 🔧 Configuration

### Network Manager Settings

```python
network_manager = NetworkManager(
    upload_url="http://your-server.com/upload",
    cache_dir="~/.ros2_dashboard/upload_cache"
)

# Configuration options
network_manager.max_retries = 5              # Retry attempts
network_manager.retry_delay = 10             # Seconds between retries
network_manager.max_concurrent_uploads = 2   # Simultaneous uploads
network_manager.bandwidth_limit = None       # bytes/sec (None = unlimited)
network_manager.connection_check_interval = 5  # Network check frequency
```

### Upload Priority Levels

| Priority | Use Case | Description |
|----------|----------|-------------|
| 1-2 | Critical | Real-time telemetry, error logs |
| 3-5 | Normal | Regular recordings, standard bags |
| 6-8 | Low | Historical data, large files |
| 9-10 | Background | Archives, backups |

## 📊 Database Schema

### Upload Tasks Table
```sql
CREATE TABLE upload_tasks (
    id INTEGER PRIMARY KEY,
    file_path TEXT NOT NULL,
    upload_id TEXT,
    priority INTEGER DEFAULT 5,
    status TEXT DEFAULT 'pending',
    metadata TEXT,
    uploaded_chunks TEXT,  -- JSON array of completed chunks
    total_chunks INTEGER,
    created_at REAL,
    updated_at REAL,
    retry_count INTEGER DEFAULT 0,
    last_error TEXT,
    file_size INTEGER,
    bytes_uploaded INTEGER DEFAULT 0
);
```

### Upload History Table
```sql
CREATE TABLE upload_history (
    id INTEGER PRIMARY KEY,
    file_path TEXT NOT NULL,
    upload_id TEXT,
    status TEXT,
    completed_at REAL,
    file_size INTEGER,
    upload_duration REAL,
    error TEXT
);
```

## 🛡️ Error Handling

### Network Failure Scenarios

| Scenario | Handling |
|----------|----------|
| **Connection Lost Mid-Upload** | Pause upload, save state, resume when online |
| **Server Unreachable** | Queue upload, retry with backoff |
| **Chunk Upload Fails** | Retry chunk only, not entire file |
| **Checksum Mismatch** | Report error, mark as failed |
| **Disk Full (Server)** | Retry later, notify user |
| **Timeout** | Increase timeout, retry |

### Status Indicators

- 🟢 **Online** - Server reachable, uploads active
- 🔴 **Offline** - No connection, queuing uploads
- 🟡 **PENDING** - Waiting in queue
- 🔵 **UPLOADING** - Currently transferring
- 🟠 **PAUSED** - Temporarily stopped (network issue)
- ✅ **COMPLETED** - Successfully uploaded
- ❌ **FAILED** - Upload failed after max retries

## 🔒 Security Considerations

### Current Implementation
- HTTP with no authentication
- **For local network/development only**

### Production Recommendations
```python
# 1. Use HTTPS
upload_url = "https://secure-server.com/upload"

# 2. Add authentication token
metadata = {
    'auth_token': 'your-secret-token',
    'api_key': 'your-api-key'
}

# 3. Enable SSL verification
requests.post(url, verify=True, cert='/path/to/cert.pem')

# 4. Encrypt sensitive data in metadata
# 5. Implement rate limiting
# 6. Add IP whitelisting
```

## 📈 Performance Optimization

### Chunk Size Tuning
```python
# For slow networks
task.chunk_size = 1 * 1024 * 1024  # 1MB chunks

# For fast networks
task.chunk_size = 10 * 1024 * 1024  # 10MB chunks

# For very large files
task.chunk_size = 50 * 1024 * 1024  # 50MB chunks
```

### Concurrent Uploads
```python
# More concurrent uploads (requires more bandwidth)
network_manager.max_concurrent_uploads = 5

# Fewer concurrent uploads (more stable)
network_manager.max_concurrent_uploads = 1
```

### Bandwidth Throttling
```python
# Limit to 10 MB/s
network_manager.bandwidth_limit = 10 * 1024 * 1024

# Unlimited (default)
network_manager.bandwidth_limit = None
```

## 🧪 Testing the System

### Test 1: Basic Upload
```bash
# 1. Start server
python3 upload_server.py

# 2. Start dashboard
python3 main.py

# 3. Record a bag
# 4. Stop recording
# 5. Check Upload tab - should show upload progress
```

### Test 2: Network Failure Recovery
```bash
# 1. Start upload of large file
# 2. Kill server (Ctrl+C) mid-upload
# 3. Note progress (e.g., 45%)
# 4. Restart server
# 5. Upload should resume from 45%, not 0%
```

### Test 3: Application Restart
```bash
# 1. Queue several uploads
# 2. Close dashboard (while uploading)
# 3. Reopen dashboard
# 4. Go to Upload tab
# 5. Pending uploads should be restored
```

### Test 4: Priority Queue
```bash
# 1. Add file with priority 8
# 2. Add file with priority 3
# 3. Add file with priority 1
# 4. Observe: Priority 1 uploads first, then 3, then 8
```

## 🔍 Troubleshooting

### Issue: Uploads Not Starting
**Check:**
- Server is running and accessible
- Server URL is correct in dashboard
- Auto-upload is enabled
- Network connection exists

### Issue: Upload Stuck at X%
**Solutions:**
- Check network stability
- Increase retry delay
- Restart dashboard to reload state
- Check server logs for errors

### Issue: High Retry Count
**Causes:**
- Network instability
- Server overloaded
- Firewall blocking requests
- Wrong server URL

### Issue: Upload Failed After Max Retries
**Actions:**
- Check server logs
- Verify file exists and is readable
- Manually re-queue upload
- Check disk space on server

## 📝 API Endpoints

### Server Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/health` | Health check |
| POST | `/upload/init` | Initialize upload |
| POST | `/upload/chunk` | Upload chunk |
| POST | `/upload/finalize` | Complete upload |
| GET | `/upload/status/<id>` | Get upload status |
| GET | `/uploads` | List completed uploads |

## 🎯 Future Enhancements

- [ ] P2P upload for robot fleets
- [ ] Delta sync for incremental uploads
- [ ] Compression before upload
- [ ] Multi-server redundancy
- [ ] Upload scheduling (off-peak hours)
- [ ] Cloud storage integration (S3, Azure, GCS)
- [ ] End-to-end encryption
- [ ] Webhook notifications
- [ ] GraphQL API
- [ ] WebSocket for real-time progress

## 📚 Additional Resources

- [Flask Documentation](https://flask.palletsprojects.com/)
- [Requests Library](https://requests.readthedocs.io/)
- [SQLite Python](https://docs.python.org/3/library/sqlite3.html)
- [HTTP Chunked Transfer](https://en.wikipedia.org/wiki/Chunked_transfer_encoding)

---

**System Status:** ✅ Production Ready (Local Network)  
**Version:** 1.0.0  
**Last Updated:** October 25, 2025
