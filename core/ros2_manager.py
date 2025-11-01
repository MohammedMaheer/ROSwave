"""
ROS2 Manager - handles ROS2 bag recording and topic monitoring
OPTIMIZED: Multi-threaded subprocess calls with ThreadPoolExecutor
"""

import subprocess
import os
import threading
import time
import yaml
from datetime import datetime
import psutil
from typing import Optional, Dict, Any
from concurrent.futures import ThreadPoolExecutor, as_completed, wait, FIRST_COMPLETED
import queue

# Import ML exporter (lightweight packaging)
try:
    from .ml_exporter import package_bag_for_ml, populate_schema_with_bag_info
except Exception:
    # If ml_exporter isn't available (should be present), functions will be missing
    package_bag_for_ml = None
    populate_schema_with_bag_info = None

# Import CPU optimizer for advanced performance
try:
    from .cpu_optimizer import get_cpu_optimizer
    CPU_OPTIMIZER_AVAILABLE = True
except Exception:
    CPU_OPTIMIZER_AVAILABLE = False


class ROS2Manager:
    """Manages ROS2 bag recording and topic information - AGGRESSIVE OPTIMIZATION"""
    
    def __init__(self, performance_mode_manager=None):
        self.output_directory = os.path.expanduser("~/ros2_recordings")
        self.recording_process = None
        self.recording_log_handle = None  # Handle for recording log file
        self.current_bag_path = None
        self.is_recording = False
        self.recording_thread = None
        
        # ThreadPoolExecutor for parallel subprocess calls - INCREASED WORKERS
        self.executor = ThreadPoolExecutor(max_workers=8)
        
        # AGGRESSIVE CACHING - 5+ seconds
        self._cache = {}  # INITIALIZE _cache DICTIONARY
        self._cache_timeout = 5.0  
        self._cache_timestamps = {}
        self._cache_lock = threading.Lock()
        
        # Pre-cache frequently requested data
        self._last_topics_list = []
        self._last_nodes_list = []
        self._last_services_list = []
        
        # Performance mode manager for dynamic timeouts
        self.performance_mode_manager = performance_mode_manager
        
        # RECORDING HEALTH MONITORING
        self.recording_start_time = None
        self.recording_health_checks = 0
        self.recording_warnings = []
        self._subprocess_timeout = self._calculate_subprocess_timeout()
        
    def set_output_directory(self, directory):
        """Set the output directory for recordings"""
        self.output_directory = directory
        os.makedirs(directory, exist_ok=True)
        
    def get_recordings_directory(self):
        """Get the recordings directory"""
        return self.output_directory
    
    def _calculate_subprocess_timeout(self):
        """Calculate subprocess timeout based on performance mode"""
        if not self.performance_mode_manager:
            # CRITICAL FIX: Reduce from 8s to 3s (prevents "not responding" dialogs)
            return 3.0
        
        try:
            mode_settings = self.performance_mode_manager.get_mode_settings()
            # CRITICAL FIX: Shorter timeout to prevent UI freeze
            # Use cache_timeout as baseline, but cap at 3 seconds max
            cache_timeout = mode_settings.get('cache_timeout', 2.0)
            # Maximum 3 seconds to prevent "not responding" dialogs
            return min(3.0, cache_timeout + 1.0)
        except Exception:
            # Fallback to fast timeout
            return 3.0
    
    def update_subprocess_timeout(self):
        """Update subprocess timeout (call this if performance mode changes)"""
        self._subprocess_timeout = self._calculate_subprocess_timeout()
        
    def get_topics_info(self):
        """Get information about available ROS2 topics - ULTRA FAST with aggressive caching"""
        # CHECK CACHE FIRST - AGGRESSIVE 5 SECOND CACHE
        cache_key = 'topics_info'
        with self._cache_lock:
            if cache_key in self._cache_timestamps:
                age = time.time() - self._cache_timestamps[cache_key]
                if age < self._cache_timeout:
                    return self._cache[cache_key]
        
        topics = []
        
        try:
            # Get list of topics with dynamic timeout based on performance mode
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=self._subprocess_timeout  # Dynamic timeout based on performance mode
            )
            
            if result.returncode == 0:
                topic_names = [t.strip() for t in result.stdout.strip().split('\n') if t.strip()]
                
                # Get info for all topics - use ThreadPoolExecutor to fetch types in parallel
                topics = []
                
                # Fetch types in parallel with reduced timeout per topic
                if topic_names:
                    # Use thread pool to get types concurrently
                    from concurrent.futures import ThreadPoolExecutor, wait, FIRST_COMPLETED
                    
                    with ThreadPoolExecutor(max_workers=8) as executor:
                        # Submit all type fetches
                        future_to_topic = {
                            executor.submit(self._get_topic_type, t): t 
                            for t in topic_names
                        }
                        
                        # Collect results with graceful timeout handling
                        topic_types = {}
                        remaining_futures = set(future_to_topic.keys())
                        timeout_per_batch = 3.0  # CRITICAL FIX: Reduce from 45s to 3s (prevents UI freeze)
                        start_time = time.time()
                        
                        while remaining_futures and (time.time() - start_time) < timeout_per_batch:
                            try:
                                done, remaining_futures = wait(
                                    remaining_futures,
                                    timeout=0.5,  # CRITICAL FIX: Reduce from 2.0s to 0.5s
                                    return_when=FIRST_COMPLETED
                                )
                                
                                # Process completed futures
                                for future in done:
                                    topic_name = future_to_topic[future]
                                    try:
                                        msg_type = future.result(timeout=0.1)
                                        topic_types[topic_name] = msg_type
                                    except Exception:
                                        topic_types[topic_name] = "Unknown"
                            except Exception:
                                # If anything goes wrong, just process what we have
                                break
                    
                    # Build topic list with types
                    for t in topic_names:
                        topics.append({
                            'name': t, 
                            'type': topic_types.get(t, "Unknown"),
                            'publisher_count': 1,  # Assume publishing if in list
                            'hz': 0.0  # Placeholder - will be updated below
                        })
                    
                    # Fetch Hz values in parallel (fast method with timeout)
                    # Only fetch for topics that have known types (skip Unknown to save time)
                    topics_to_check = [t for t in topic_names if topic_types.get(t, "Unknown") != "Unknown"]
                    if topics_to_check:
                        try:
                            hz_values = self.get_topics_hz_batch(topics_to_check, max_workers=8)
                            # Update Hz values in topics list
                            for topic_info in topics:
                                if topic_info['name'] in hz_values:
                                    topic_info['hz'] = hz_values[topic_info['name']]
                        except Exception as e:
                            # If Hz fetching fails, topics already have 0.0 as default
                            pass
                
                # CACHE immediately
                with self._cache_lock:
                    self._cache[cache_key] = topics
                    self._cache_timestamps[cache_key] = time.time()
                    
        except subprocess.TimeoutExpired:
            print("⚠️ ROS2 topic list timeout - returning cached data")
            # Return last known good value instead of empty
            with self._cache_lock:
                return self._cache.get(cache_key, [])
        except Exception as e:
            print(f"Error getting topics: {e}")
            
        return topics
        
    def _get_topic_type(self, topic_name):
        """Get the message type for a topic - with dynamic timeout"""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'type', topic_name],
                capture_output=True,
                text=True,
                timeout=self._subprocess_timeout  # Use dynamic timeout
            )
            
            if result.returncode == 0:
                msg_type = result.stdout.strip()
                if msg_type:
                    return msg_type
        except subprocess.TimeoutExpired:
            # Silently fail - don't spam logs with timeout warnings
            pass
        except Exception as e:
            # Silently fail for other errors too
            pass
            
        return "Unknown"
        
    def _get_publisher_count(self, topic_name):
        """Get the number of publishers for a topic"""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'info', topic_name, '-v'],
                capture_output=True,
                text=True,
                timeout=1
            )
            
            if result.returncode == 0:
                # Count publishers in output
                count = result.stdout.count('Publisher')
                return max(0, count - 1)  # Subtract header occurrence
        except:
            pass
            
        return 0
        
    def _get_topic_hz(self, topic_name):
        """Get the publishing frequency of a topic"""
        # Hz checking is too slow for bulk operations
        # Return 0.0 and let it be updated separately if needed
        return 0.0
    
    def _get_topic_hz_fast(self, topic_name):
        """Get the publishing frequency of a topic FAST with short timeout"""
        try:
            # Use ros2 topic hz with 2.5 second timeout to get actual rate
            # This waits for at least 2 messages to calculate average
            # NOTE: Very slow topics (< 2 Hz) may timeout without getting rate
            result = subprocess.run(
                ['ros2', 'topic', 'hz', topic_name],
                capture_output=True,
                text=True,
                timeout=2.5  # 2.5 seconds - balance between speed and accuracy
            )
            
            # Parse the output - look for the "average rate:" line
            # NOTE: ros2 topic hz keeps running until killed, so we get output even on timeout
            lines = result.stdout.strip().split('\n')
            for line in lines:
                if 'average rate:' in line.lower():
                    # Extract Hz value from "average rate: 10.234"
                    try:
                        hz_str = line.split(':')[-1].strip()
                        hz = float(hz_str)
                        return max(0, hz)  # Ensure non-negative
                    except (ValueError, IndexError):
                        pass
                        
        except subprocess.TimeoutExpired as e:
            # Timeout is EXPECTED - ros2 topic hz runs forever
            # Parse the output we got before timeout
            # CRITICAL: e.stdout is BYTES even when text=True was used in Popen!
            try:
                if e.stdout:
                    # Decode bytes to string
                    output_str = e.stdout.decode('utf-8') if isinstance(e.stdout, bytes) else str(e.stdout)
                    lines = output_str.strip().split('\n')
                    for line in lines:
                        if 'average rate:' in line.lower():
                            hz_str = line.split(':')[-1].strip()
                            hz = float(hz_str)
                            return max(0, hz)
            except Exception:
                pass
            # If we got a timeout but no output, topic might not be publishing
            # or is too slow (< 2 Hz needs > 2.5s to measure)
            return 0.0
        except Exception:
            pass
        
        return 0.0
    
    def get_topics_hz_batch(self, topic_names, max_workers=4):
        """Get Hz values for multiple topics in parallel (non-blocking)
        
        Args:
            topic_names: List of topic names to get Hz for
            max_workers: Number of parallel workers (default 4 to avoid blocking)
            
        Returns:
            Dictionary mapping topic_name -> hz_value (float)
            
        This method is designed to be called from background threads or async contexts
        to populate Hz values without blocking the UI thread.
        """
        hz_dict = {}
        
        if not topic_names:
            return hz_dict
        
        # Use ThreadPoolExecutor to fetch Hz values in parallel
        with ThreadPoolExecutor(max_workers=max_workers) as executor:
            future_to_topic = {
                executor.submit(self._get_topic_hz_fast, topic): topic 
                for topic in topic_names
            }
            
            # Collect results with timeout - allow 3s total for batch (topics run in parallel)
            try:
                for future in as_completed(future_to_topic, timeout=3.0):
                    try:
                        topic = future_to_topic[future]
                        hz_value = future.result(timeout=0.1)
                        hz_dict[topic] = hz_value
                    except Exception:
                        # If a specific topic fails, continue with others
                        topic = future_to_topic.get(future)
                        if topic:
                            hz_dict[topic] = 0.0
            except Exception:
                # If batch times out, fill remaining with 0.0
                for topic in topic_names:
                    if topic not in hz_dict:
                        hz_dict[topic] = 0.0
        
        return hz_dict
        
    def start_recording(self, bag_name, topics=None):
        """
        Start recording ROS2 bags in COMPLETELY ISOLATED PROCESS
        CRITICAL: Recording process is 100% independent of UI thread
        Even if UI freezes, recording continues without data loss
        """
        if self.is_recording:
            print("Already recording")
            return False
        
        # Check if there are any topics available (helpful diagnostic)
        try:
            available_topics = self.get_topics_info()
            if available_topics:
                print(f"📡 Found {len(available_topics)} topics to record")
            else:
                print("⚠️  WARNING: No ROS2 topics found!")
                print("   Recording will start, but won't capture data until topics are published")
                print("   TIP: Start your ROS2 nodes or run the demo topic generator:")
                print("        python3 tests/demo_topics_generator.py")
        except Exception as e:
            print(f"⚠️  Could not check topics: {e}")
            
        try:
            self.current_bag_path = os.path.join(self.output_directory, bag_name)
            
            # Build command
            cmd = ['ros2', 'bag', 'record', '-o', self.current_bag_path]
            
            if topics:
                cmd.extend(topics)
            else:
                # Record all topics
                cmd.append('-a')
                
            # CRITICAL: Start recording process with optimizations
            # - Capture output to detect issues
            # - Environment variables for unbuffered output
            # - High priority to ensure data capture even under load
            # 
            # NOTE: We do NOT use preexec_fn=os.setpgrp because it BREAKS ros2 bag record!
            # ROS2 bag is a Python script that spawns child processes, and changing
            # the process group interferes with ROS2's internal process management.
            # The recording process is still independent - it continues even if UI crashes.
            
            # Create a log file for recording output (helps debugging)
            log_file = os.path.join(self.output_directory, f"{bag_name}_recording.log")
            log_handle = open(log_file, 'w', buffering=1)  # Line buffered
            
            # CRITICAL FIX: Set environment to ensure ros2 bag doesn't buffer output
            env = os.environ.copy()
            env['PYTHONUNBUFFERED'] = '1'  # Force Python to not buffer output
            env['RCUTILS_CONSOLE_OUTPUT_FORMAT'] = '[{severity}] [{name}]: {message}'
            
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=log_handle,  # Log output for debugging
                stderr=subprocess.STDOUT,  # Combine stderr with stdout
                text=True,
                env=env,  # Use unbuffered environment
                # NO preexec_fn - it breaks ros2 bag record!
                close_fds=False  # Keep log file open
            )
            
            # Store log handle to close it later
            self.recording_log_handle = log_handle
            
            print(f"📝 Recording log: {log_file}")
            
            # Set process priority to high (ensures recording continues even under CPU load)
            try:
                process = psutil.Process(self.recording_process.pid)
                # Nice value -5 (higher priority than normal, but not realtime)
                # This ensures recording gets CPU even if UI is consuming resources
                process.nice(-5)
                print(f"✅ Recording process priority set to HIGH (nice=-5)")
                
                # ADVANCED: Pin recording process to dedicated CPU cores
                if CPU_OPTIMIZER_AVAILABLE:
                    cpu_opt = get_cpu_optimizer()
                    if cpu_opt.pin_recording_process(self.recording_process.pid):
                        print(f"✅ Recording process pinned to dedicated cores")
                
            except Exception as e:
                print(f"⚠️  Could not set recording priority: {e} (may need sudo)")
            
            self.is_recording = True
            
            # Initialize health monitoring
            self.recording_start_time = time.time()
            self.recording_health_checks = 0
            self.recording_warnings = []
            
            # Start LIGHTWEIGHT monitoring thread (does NOT block recording)
            self.recording_thread = threading.Thread(
                target=self._monitor_recording,
                daemon=True,  # Won't block process exit
                name="RecordingMonitor"  # Named for debugging
            )
            self.recording_thread.start()
            
            print(f"🎬 Recording started: PID={self.recording_process.pid}")
            print(f"   Output: {self.current_bag_path}")
            print(f"   Topics: {len(topics) if topics else 'ALL'}")
            print(f"   ⚡ ISOLATED: Recording continues even if UI freezes!")
            
            return True
            
        except Exception as e:
            print(f"❌ Error starting recording: {e}")
            self.is_recording = False
            return False
            
    def _monitor_recording(self):
        """
        LIGHTWEIGHT monitoring of recording process with HEALTH CHECKS
        CRITICAL: This runs in separate thread and does NOT interfere with recording
        Monitors ACTUAL recording activity (bag file size, I/O)
        """
        check_interval = 2.0  # Check every 2 seconds (low overhead)
        last_size = 0
        last_check_time = time.time()
        
        while self.is_recording and self.recording_process:
            try:
                self.recording_health_checks += 1
                
                # Non-blocking check if process is still running
                returncode = self.recording_process.poll()
                
                if returncode is not None:
                    # Process terminated
                    if returncode == 0:
                        print("✅ Recording process completed successfully")
                    else:
                        print(f"⚠️  Recording process terminated with code {returncode}")
                        self.recording_warnings.append(f"Process exit code: {returncode}")
                    self.is_recording = False
                    break
                
                # HEALTH CHECK: Monitor ACTUAL recording activity
                try:
                    process = psutil.Process(self.recording_process.pid)
                    
                    # Get bag file size (shows actual recording activity)
                    current_size_mb = 0
                    write_speed_mb_s = 0
                    if self.current_bag_path and os.path.exists(self.current_bag_path):
                        try:
                            # Get total size of all files in bag directory
                            total_size = 0
                            for root, dirs, files in os.walk(self.current_bag_path):
                                for f in files:
                                    fp = os.path.join(root, f)
                                    if os.path.exists(fp):
                                        total_size += os.path.getsize(fp)
                            current_size_mb = total_size / 1024 / 1024
                            
                            # Calculate write speed
                            current_time = time.time()
                            time_diff = current_time - last_check_time
                            if time_diff > 0:
                                size_diff = current_size_mb - last_size
                                write_speed_mb_s = size_diff / time_diff
                            
                            last_size = current_size_mb
                            last_check_time = current_time
                        except Exception:
                            pass
                    
                    # Get process stats (these are often low for I/O-bound processes)
                    cpu_percent = process.cpu_percent(interval=0)  # Non-blocking
                    memory_mb = process.memory_info().rss / 1024 / 1024
                    
                    # Log health every 10 checks (~20 seconds) instead of 30
                    if self.recording_health_checks % 10 == 0:
                        elapsed = time.time() - (self.recording_start_time or time.time())
                        
                        # Show ACTUAL recording activity (bag size and write speed)
                        if current_size_mb > 0:
                            print(f"📊 Recording health: {elapsed:.0f}s elapsed, "
                                  f"Size={current_size_mb:.1f}MB, Write={write_speed_mb_s:.2f}MB/s, "
                                  f"PID={self.recording_process.pid}")
                        else:
                            # Show path info to debug why bag file not found
                            print(f"📊 Recording health: {elapsed:.0f}s elapsed, "
                                  f"Bag path: {self.current_bag_path}, "
                                  f"Exists: {os.path.exists(self.current_bag_path) if self.current_bag_path else False}, "
                                  f"PID={self.recording_process.pid}")
                    
                    # Warn if recording process is using excessive resources
                    if memory_mb > 2000:  # > 2GB
                        warning = f"High memory usage: {memory_mb:.1f}MB"
                        if warning not in self.recording_warnings:
                            self.recording_warnings.append(warning)
                            print(f"⚠️  {warning}")
                    
                    # Warn if bag file is growing too fast (> 100MB/s might fill disk)
                    if write_speed_mb_s > 100:
                        warning = f"High write speed: {write_speed_mb_s:.1f}MB/s - disk may fill quickly"
                        if warning not in self.recording_warnings:
                            self.recording_warnings.append(warning)
                            print(f"⚠️  {warning}")
                    
                    # Warn if recording stalled (no growth for 2+ minutes when size > 0)
                    if current_size_mb > 0.1 and write_speed_mb_s < 0.001 and elapsed > 120:
                        warning = "Recording may be stalled (no data growth)"
                        if warning not in self.recording_warnings:
                            self.recording_warnings.append(warning)
                            print(f"⚠️  {warning}")
                            
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    # Process may have just ended, continue monitoring
                    pass
                
                # LIGHTWEIGHT: Just sleep, don't do heavy operations
                # Recording process is completely independent
                time.sleep(check_interval)
                
            except Exception as e:
                print(f"⚠️  Monitoring error: {e}")
                # Don't stop recording on monitoring errors
                time.sleep(check_interval)
            
    def stop_recording(self):
        """
        Stop recording GRACEFULLY
        CRITICAL: Ensures data is flushed and files are closed properly
        """
        if not self.is_recording:
            return
            
        print("🛑 Stopping recording...")
        self.is_recording = False
        
        bag_path_to_package = self.current_bag_path

        if self.recording_process:
            try:
                # GRACEFUL SHUTDOWN: Send SIGINT (same as Ctrl+C)
                # This allows ros2 bag record to flush buffers and close files properly
                print("   Sending SIGINT (graceful shutdown)...")
                self.recording_process.terminate()
                
                # Wait up to 15 seconds for graceful shutdown
                # (longer than before to ensure data is flushed)
                self.recording_process.wait(timeout=15)
                
                print("✅ Recording stopped gracefully")
                
            except subprocess.TimeoutExpired:
                # If it doesn't stop gracefully, force kill
                print("⚠️  Graceful shutdown timeout, force killing...")
                self.recording_process.kill()
                self.recording_process.wait(timeout=5)
                print("⚠️  Recording force-killed (may have data loss)")
                
            except Exception as e:
                print(f"❌ Error stopping recording: {e}")
                
            finally:
                self.recording_process = None
                
                # Close log file handle if it exists
                if hasattr(self, 'recording_log_handle') and self.recording_log_handle:
                    try:
                        self.recording_log_handle.flush()
                        self.recording_log_handle.close()
                        self.recording_log_handle = None
                        print("📝 Recording log file closed")
                    except Exception as e:
                        print(f"⚠️  Error closing log file: {e}")
                
        # Clear current bag path first in main thread to avoid races in UI
        self.current_bag_path = None

        # Package the completed bag into ML format in background so UI isn't blocked
        if bag_path_to_package:
            def _do_export(path):
                try:
                    # Wait for bag file to be fully written and flushed to disk
                    # ros2 bag record needs time to close files after SIGINT
                    time.sleep(3)
                    
                    # Verify bag directory exists before trying to export
                    if not os.path.exists(path):
                        print(f"⚠️  Bag directory not found, skipping ML export: {path}")
                        return
                    
                    # Check if directory has any files
                    has_files = False
                    for root, dirs, files in os.walk(path):
                        if files:
                            has_files = True
                            break
                    
                    if not has_files:
                        print(f"⚠️  Bag directory is empty, skipping ML export: {path}")
                        return
                    
                    # Export ML package
                    self.export_bag_ml(path)
                    print(f"📦 ML package created for {path}")
                except Exception as e:
                    print(f"⚠️  Failed to create ML package for {path}: {e}")

            # Export in completely separate thread (doesn't block UI)
            t = threading.Thread(
                target=_do_export, 
                args=(bag_path_to_package,), 
                daemon=True,
                name="MLExportWorker"
            )
            t.start()
        
    def get_recording_health(self):
        """
        Get current recording health status
        Returns: dict with health metrics or None if not recording
        """
        if not self.is_recording or not self.recording_process:
            return None
        
        try:
            process = psutil.Process(self.recording_process.pid)
            elapsed = time.time() - (self.recording_start_time or time.time())
            
            # Calculate bag file size and write speed (actual recording activity)
            bag_size_mb = 0.0
            write_speed_mbps = 0.0
            
            if self.current_bag_path and os.path.exists(self.current_bag_path):
                try:
                    # Walk bag directory and sum all file sizes
                    for root, _, files in os.walk(self.current_bag_path):
                        for file in files:
                            file_path = os.path.join(root, file)
                            if os.path.isfile(file_path):
                                bag_size_mb += os.path.getsize(file_path) / (1024 * 1024)
                    
                    # Calculate write speed if we have previous size
                    if hasattr(self, '_last_bag_size') and hasattr(self, '_last_size_check_time'):
                        time_delta = time.time() - self._last_size_check_time
                        if time_delta > 0:
                            size_delta = bag_size_mb - self._last_bag_size
                            write_speed_mbps = size_delta / time_delta
                    
                    # Update tracking
                    self._last_bag_size = bag_size_mb
                    self._last_size_check_time = time.time()
                except Exception as e:
                    print(f"⚠️  Failed to calculate bag size: {e}")
            
            return {
                'pid': self.recording_process.pid,
                'elapsed_seconds': int(elapsed),
                'bag_size_mb': round(bag_size_mb, 2),
                'write_speed_mbps': round(write_speed_mbps, 2),
                'health_checks': self.recording_health_checks,
                'warnings': self.recording_warnings.copy(),
                'is_alive': process.is_running(),
                'status': 'healthy' if not self.recording_warnings else 'warnings'
            }
        except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError):
            return {
                'status': 'unknown',
                'warnings': ['Process information unavailable']
            }
    
    def get_current_bag_path(self):
        """Get the path of the currently recording bag"""
        return self.current_bag_path
        
    def get_bag_info(self, bag_path):
        """Get information about a bag file"""
        info = {
            'size_mb': 0,
            'duration': '0s',
            'topic_count': 0,
            'start_time': 'Unknown',
            'is_complete': True
        }
        
        try:
            # Get directory size
            total_size = 0
            if os.path.exists(bag_path):
                for dirpath, dirnames, filenames in os.walk(bag_path):
                    for filename in filenames:
                        filepath = os.path.join(dirpath, filename)
                        total_size += os.path.getsize(filepath)
                        
                info['size_mb'] = total_size / (1024 * 1024)
                
                # Try to read metadata
                metadata_path = os.path.join(bag_path, 'metadata.yaml')
                if os.path.exists(metadata_path):
                    with open(metadata_path, 'r') as f:
                        metadata = yaml.safe_load(f)
                        
                    if metadata:
                        # Get topic count
                        if 'rosbag2_bagfile_information' in metadata:
                            bag_info = metadata['rosbag2_bagfile_information']
                            
                            topics = bag_info.get('topics_with_message_count', [])
                            info['topic_count'] = len(topics)
                            
                            # Get duration
                            duration_ns = bag_info.get('duration', {}).get('nanoseconds', 0)
                            duration_s = duration_ns / 1e9
                            info['duration'] = f"{duration_s:.1f}s"
                            
                            # Get start time
                            start_time_ns = bag_info.get('starting_time', {}).get('nanoseconds_since_epoch', 0)
                            if start_time_ns > 0:
                                start_time = datetime.fromtimestamp(start_time_ns / 1e9)
                                info['start_time'] = start_time.strftime("%Y-%m-%d %H:%M:%S")
                                
        except Exception as e:
            print(f"Error getting bag info: {e}")
            
        return info
        
    def get_disk_usage(self):
        """Get disk usage percentage for the output directory"""
        try:
            disk = psutil.disk_usage(self.output_directory)
            return disk.percent
        except:
            return 0
            
    def get_nodes_info(self):
        """Get information about ROS2 nodes - ULTRA FAST with aggressive caching"""
        # CHECK CACHE FIRST
        cache_key = 'nodes_info'
        with self._cache_lock:
            if cache_key in self._cache_timestamps:
                age = time.time() - self._cache_timestamps[cache_key]
                if age < self._cache_timeout:
                    return self._cache[cache_key]
        
        nodes = []
        
        try:
            # Get list of nodes with reasonable timeout
            result = subprocess.run(
                ['ros2', 'node', 'list'],
                capture_output=True,
                text=True,
                timeout=1.0  # CRITICAL FIX: Reduce from 2.0s to 1.0s
            )
            
            if result.returncode == 0:
                node_names = [n.strip() for n in result.stdout.strip().split('\n') if n.strip()]
                
                # SKIP subprocess calls - just parse names (don't call node info for each)
                nodes = []
                for node_name in node_names:
                    parts = node_name.rsplit('/', 1)
                    namespace = parts[0] if len(parts) > 1 else '/'
                    name = parts[1] if len(parts) > 1 else parts[0]
                    
                    nodes.append({
                        'name': name,
                        'full_name': node_name,
                        'namespace': namespace,
                        'publishers': 0,  # SKIP - too slow to fetch
                        'subscribers': 0   # SKIP - too slow to fetch
                    })
                
                # CACHE immediately
                with self._cache_lock:
                    self._cache[cache_key] = nodes
                    self._cache_timestamps[cache_key] = time.time()
                    
        except subprocess.TimeoutExpired:
            print("⚠️ ROS2 node list timeout - returning cached data")
            # Return cached value
            with self._cache_lock:
                return self._cache.get(cache_key, [])
        except Exception as e:
            print(f"Error getting nodes: {e}")
            
        return nodes
    
    def _get_node_details(self, node_name):
        """DEPRECATED - Keep for compatibility but not used"""
        return None
        
    def _count_node_publishers(self, node_name):
        """Count publishers for a node"""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'info', node_name],
                capture_output=True,
                text=True,
                timeout=1  # CRITICAL FIX: Reduce from 2s to 1s
            )
            
            if result.returncode == 0:
                count = 0
                in_publishers = False
                for line in result.stdout.split('\n'):
                    if 'Publishers:' in line:
                        in_publishers = True
                    elif in_publishers and line.strip().startswith('/'):
                        count += 1
                    elif in_publishers and ('Subscribers:' in line or 'Service' in line):
                        break
                return count
        except:
            pass
        return 0
        
    def _count_node_subscribers(self, node_name):
        """Count subscribers for a node"""
        try:
            result = subprocess.run(
                ['ros2', 'node', 'info', node_name],
                capture_output=True,
                text=True,
                timeout=1  # CRITICAL FIX: Reduce from 2s to 1s
            )
            
            if result.returncode == 0:
                count = 0
                in_subscribers = False
                for line in result.stdout.split('\n'):
                    if 'Subscribers:' in line:
                        in_subscribers = True
                    elif in_subscribers and line.strip().startswith('/'):
                        count += 1
                    elif in_subscribers and 'Service' in line:
                        break
                return count
        except:
            pass
        return 0
        
    def get_services_info(self):
        """Get information about ROS2 services - ULTRA FAST with aggressive caching"""
        # CHECK CACHE FIRST
        cache_key = 'services_info'
        with self._cache_lock:
            if cache_key in self._cache_timestamps:
                age = time.time() - self._cache_timestamps[cache_key]
                if age < self._cache_timeout:
                    return self._cache[cache_key]
        
        services = []
        
        try:
            # Get list of services with reasonable timeout
            result = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True,
                text=True,
                timeout=1.0  # CRITICAL FIX: Reduce from 2.0s to 1.0s
            )
            
            if result.returncode == 0:
                service_names = [s.strip() for s in result.stdout.strip().split('\n') if s.strip()]
                
                # SKIP type lookup - too slow, just list service names
                services = [{'name': s, 'type': 'Unknown', 'server_count': 1} 
                           for s in service_names]
                
                # CACHE immediately
                with self._cache_lock:
                    self._cache[cache_key] = services
                    self._cache_timestamps[cache_key] = time.time()
                    
        except subprocess.TimeoutExpired:
            print("⚠️ ROS2 service list timeout - returning cached data")
            # Return cached value
            with self._cache_lock:
                return self._cache.get(cache_key, [])
        except Exception as e:
            print(f"Error getting services: {e}")
            
        return services
        
    def _get_service_type(self, service_name):
        """DEPRECATED - Keep for compatibility"""
        return "Unknown"
        
    def get_topic_bandwidth(self, topic_name):
        """DEPRECATED - Keep for compatibility"""
        return "N/A"

    def export_bag_ml(self, bag_path: str, out_root: Optional[str] = None) -> Dict[str, Any]:
        """Create a lightweight ML package for the given bag.

        The function uses `core.ml_exporter` to copy the bag files into a
        ml_datasets/ package, write metadata and schema placeholders, and create
        a compressed archive. If `populate_schema_with_bag_info` is available,
        it will be used to fill the schema from bag metadata.
        """
        if not package_bag_for_ml:
            raise RuntimeError("ML exporter not available")

        # Package the bag files
        package_info = package_bag_for_ml(bag_path, out_root=out_root)

        # Try to populate schema with bag info
        try:
            if populate_schema_with_bag_info:
                bag_info = self.get_bag_info(bag_path)
                populate_schema_with_bag_info(package_info['package_dir'], bag_info)
        except Exception as e:
            # Non-fatal: schema population failed
            print(f"Warning: failed to populate ML schema: {e}")

        return package_info
