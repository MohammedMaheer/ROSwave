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
from concurrent.futures import ThreadPoolExecutor, as_completed
import queue

# Import ML exporter (lightweight packaging)
try:
    from .ml_exporter import package_bag_for_ml, populate_schema_with_bag_info
except Exception:
    # If ml_exporter isn't available (should be present), functions will be missing
    package_bag_for_ml = None
    populate_schema_with_bag_info = None


class ROS2Manager:
    """Manages ROS2 bag recording and topic information - AGGRESSIVE OPTIMIZATION"""
    
    def __init__(self):
        self.output_directory = os.path.expanduser("~/ros2_recordings")
        self.recording_process = None
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
        
    def set_output_directory(self, directory):
        """Set the output directory for recordings"""
        self.output_directory = directory
        os.makedirs(directory, exist_ok=True)
        
    def get_recordings_directory(self):
        """Get the recordings directory"""
        return self.output_directory
        
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
            # Get list of topics with reasonable timeout (2 seconds for the list command)
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True,
                text=True,
                timeout=2.0  # Increased timeout - list can take time on first call
            )
            
            if result.returncode == 0:
                topic_names = [t.strip() for t in result.stdout.strip().split('\n') if t.strip()]
                
                # Get info for all topics - use ThreadPoolExecutor to fetch types in parallel
                topics = []
                
                # Fetch types in parallel with reduced timeout per topic
                if topic_names:
                    # Use thread pool to get types concurrently
                    from concurrent.futures import ThreadPoolExecutor, as_completed
                    
                    with ThreadPoolExecutor(max_workers=4) as executor:
                        # Submit all type fetches
                        future_to_topic = {
                            executor.submit(self._get_topic_type, t): t 
                            for t in topic_names
                        }
                        
                        # Collect results as they complete
                        topic_types = {}
                        for future in as_completed(future_to_topic, timeout=3.0):
                            topic_name = future_to_topic[future]
                            try:
                                msg_type = future.result()
                                topic_types[topic_name] = msg_type
                            except Exception:
                                topic_types[topic_name] = "Unknown"
                    
                    # Build topic list with types
                    for t in topic_names:
                        topics.append({
                            'name': t, 
                            'type': topic_types.get(t, "Unknown"),
                            'publisher_count': 1,  # Assume publishing if in list
                            'hz': 0.0  # Hz will be 0 (too slow to fetch for all)
                        })
                
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
        """Get the message type for a topic - with timeout"""
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'type', topic_name],
                capture_output=True,
                text=True,
                timeout=0.8  # 800ms timeout per topic
            )
            
            if result.returncode == 0:
                msg_type = result.stdout.strip()
                if msg_type:
                    return msg_type
        except subprocess.TimeoutExpired:
            print(f"⚠️ Timeout getting type for {topic_name}")
        except Exception as e:
            print(f"Error getting type for {topic_name}: {e}")
            
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
            # Use ros2 topic hz with 100ms timeout (ultra fast, just 1 message)
            result = subprocess.run(
                ['ros2', 'topic', 'hz', topic_name],
                capture_output=True,
                text=True,
                timeout=0.1  # ULTRA short timeout - just get 1 message
            )
            
            # Parse the output - look for the last "average:" line
            lines = result.stdout.strip().split('\n')
            for line in reversed(lines):
                if 'average:' in line.lower():
                    # Extract Hz value from "average: 10.23 Hz"
                    try:
                        hz_str = line.split(':')[-1].replace('Hz', '').strip()
                        hz = float(hz_str)
                        return max(0, hz)  # Ensure non-negative
                    except (ValueError, IndexError):
                        pass
        except subprocess.TimeoutExpired:
            # Timeout is expected - topic is likely publishing, return positive number
            return 1.0  # Assume at least 1 Hz if it started publishing
        except Exception as e:
            pass
        
        return 0.0
        
    def start_recording(self, bag_name, topics=None):
        """Start recording ROS2 bags"""
        if self.is_recording:
            print("Already recording")
            return False
            
        try:
            self.current_bag_path = os.path.join(self.output_directory, bag_name)
            
            # Build command
            cmd = ['ros2', 'bag', 'record', '-o', self.current_bag_path]
            
            if topics:
                cmd.extend(topics)
            else:
                # Record all topics
                cmd.append('-a')
                
            # Start recording process
            self.recording_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            self.is_recording = True
            
            # Start monitoring thread
            self.recording_thread = threading.Thread(
                target=self._monitor_recording,
                daemon=True
            )
            self.recording_thread.start()
            
            return True
            
        except Exception as e:
            print(f"Error starting recording: {e}")
            return False
            
    def _monitor_recording(self):
        """Monitor the recording process"""
        while self.is_recording and self.recording_process:
            # Check if process is still running
            if self.recording_process.poll() is not None:
                print("Recording process terminated unexpectedly")
                self.is_recording = False
                break
                
            time.sleep(1)
            
    def stop_recording(self):
        """Stop recording"""
        if not self.is_recording:
            return
            
        self.is_recording = False
        
        bag_path_to_package = self.current_bag_path

        if self.recording_process:
            try:
                # Send SIGINT to gracefully stop recording
                self.recording_process.terminate()
                self.recording_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                # Force kill if it doesn't stop
                self.recording_process.kill()
            finally:
                self.recording_process = None
                
        # Clear current bag path first in main thread to avoid races in UI
        self.current_bag_path = None

        # Package the completed bag into ML format in background so UI isn't blocked
        if bag_path_to_package:
            def _do_export(path):
                try:
                    self.export_bag_ml(path)
                    print(f"ML package created for {path}")
                except Exception as e:
                    print(f"Failed to create ML package for {path}: {e}")

            t = threading.Thread(target=_do_export, args=(bag_path_to_package,), daemon=True)
            t.start()
        
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
                timeout=2.0  # Increased timeout for first call
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
                timeout=2
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
                timeout=2
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
                timeout=2.0  # Increased timeout for first call
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
