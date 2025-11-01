# ⚙️ Configuration Files

This directory contains configuration files for the ROS2 Dashboard.

---

## 📋 Contents

### Robot Configuration Files
- `robot_*.json` - Robot-specific configuration files

These JSON files store robot-specific settings and metadata for the dashboard.

---

## 📄 Configuration File Format

### Robot Configuration (`robot_*.json`)

**Example Structure**:
```json
{
  "robot_name": "robot_maahir-Aspire-A514-54",
  "timestamp": "20251028_074017",
  "settings": {
    "recording_path": "/path/to/recordings",
    "default_topics": [
      "/camera/image_raw",
      "/odom",
      "/cmd_vel"
    ],
    "auto_record": false,
    "compression": "zstd"
  },
  "metadata": {
    "created_at": "2025-10-28T07:40:17",
    "last_modified": "2025-10-28T07:40:17",
    "version": "2.2"
  }
}
```

**Fields**:
- `robot_name`: Unique identifier for the robot/system
- `timestamp`: Configuration creation timestamp
- `settings`: Robot-specific dashboard settings
  - `recording_path`: Default path for bag file recordings
  - `default_topics`: List of topics to auto-select for recording
  - `auto_record`: Auto-start recording on dashboard launch
  - `compression`: Bag file compression format (zstd, lz4, none)
- `metadata`: Configuration metadata
  - `created_at`: ISO 8601 timestamp
  - `last_modified`: ISO 8601 timestamp
  - `version`: Dashboard version

---

## 🚀 Usage

### Load Configuration
The dashboard automatically loads robot configurations at startup from this directory.

### Create New Configuration
```python
from core.config_manager import ConfigManager

config = ConfigManager()
config.create_robot_config(
    robot_name="my_robot",
    recording_path="/home/user/bags",
    default_topics=["/scan", "/odom"]
)
```

### Update Configuration
```python
config.update_robot_config(
    robot_name="my_robot",
    settings={"auto_record": True}
)
```

### Load Specific Configuration
```python
config.load_robot_config("robot_my_robot_20251101_120000.json")
```

---

## 📝 Naming Convention

Configuration files follow this naming pattern:
```
robot_<hostname>_<timestamp>.json
```

**Example**:
```
robot_maahir-Aspire-A514-54_20251028_074017.json
```

**Components**:
- `robot_` - Prefix for all robot configurations
- `<hostname>` - System hostname (sanitized, no spaces)
- `<timestamp>` - Creation timestamp (YYYYMMDD_HHMMSS)
- `.json` - JSON file extension

---

## 🔧 Configuration Options

### Recording Settings
- `recording_path`: Absolute path for bag file storage
- `max_bag_size`: Maximum bag file size in MB (default: 1024)
- `split_duration`: Split bag files by duration in seconds (default: 0, disabled)
- `compression`: Compression algorithm (zstd, lz4, none)

### Topic Selection
- `default_topics`: Array of topic names to auto-select
- `excluded_topics`: Array of topic patterns to exclude
- `topic_blacklist`: Array of exact topic names to never record

### Recording Behavior
- `auto_record`: Boolean, auto-start recording on launch
- `auto_stop_duration`: Auto-stop after N seconds (0 = disabled)
- `pre_record_buffer`: Pre-recording buffer in seconds

### Upload Settings
- `upload_enabled`: Enable automatic uploads
- `upload_server`: Upload server URL
- `upload_on_stop`: Upload immediately when recording stops
- `delete_after_upload`: Delete local file after successful upload

---

## 🛠️ Advanced Configuration

### Environment-Specific Configs
Create different configs for different environments:
```
robot_production_20251101_120000.json
robot_development_20251101_120000.json
robot_testing_20251101_120000.json
```

### Template Configuration
Use a template for new robots:
```bash
cp config/robot_template.json config/robot_new_robot_$(date +%Y%m%d_%H%M%S).json
```

---

## 🔒 Security Notes

### Sensitive Data
- Do **NOT** store passwords or API keys in config files
- Use environment variables for sensitive data
- Add `*.secret.json` to `.gitignore`

### File Permissions
```bash
# Ensure configs are readable only by owner
chmod 600 config/robot_*.json

# Or readable by group
chmod 640 config/robot_*.json
```

---

## 📚 Additional Resources

- **Main README**: `../README.md`
- **Documentation**: `../docs/`
- **Quick Start**: `../docs/QUICK_START_V2.2.md`

---

## ✅ Best Practices

- [ ] Use descriptive robot names
- [ ] Include timestamps in filenames
- [ ] Backup configs regularly
- [ ] Validate JSON before saving
- [ ] Document custom settings
- [ ] Version control configs (except secrets)

---

**Version**: 2.2  
**Last Updated**: November 1, 2025  
**Status**: ✅ Active  

---

*For questions about configuration, refer to the main documentation.*
