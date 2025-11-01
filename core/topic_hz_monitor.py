"""
Advanced Topic Hz Monitor - Intelligent rate detection for all topic speeds
Uses adaptive timeouts, progressive measurement, and background monitoring
"""

import subprocess
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import Dict, Optional, Tuple


class TopicHzMonitor:
    """
    Advanced topic Hz monitoring with adaptive timeouts and continuous updates.
    
    Features:
    - Adaptive timeout based on topic speed (fast topics = short timeout, slow topics = long timeout)
    - Progressive measurement (quick estimate first, then refine in background)
    - Continuous background monitoring (keeps rates up-to-date)
    - Intelligent caching with age-based refresh
    - Per-topic optimization
    """
    
    def __init__(self, max_workers: int = 8, settings: dict = None):
        """
        Initialize monitor.
        
        Args:
            max_workers: Number of parallel workers for Hz measurements
            settings: Optional dict with performance settings:
                - hz_quick_timeout: Timeout for quick checks (default: 2.5)
                - hz_max_timeout: Maximum timeout for slow topics (default: 10.0)
                - hz_cache_fresh_age: Age for fresh cache (default: 5.0)
                - hz_cache_stale_age: Age for stale cache (default: 30.0)
                - hz_background_interval: Background monitoring interval (default: 15.0)
        """
        self.max_workers = max_workers
        self._hz_cache = {}
        self._cache_lock = threading.Lock()
        self._background_thread = None
        self._stop_background = False
        self._topics_to_monitor = set()  # Topics for background monitoring
        self._monitoring_active = False  # Flag for background monitoring state
        
        # Apply settings from performance mode or use defaults
        if settings:
            self.QUICK_TIMEOUT = settings.get('hz_quick_timeout', 2.5)
            self.MAX_TIMEOUT = settings.get('hz_max_timeout', 10.0)
            self.CACHE_FRESH_AGE = settings.get('hz_cache_fresh_age', 5.0)
            self.CACHE_STALE_AGE = settings.get('hz_cache_stale_age', 30.0)
            self._background_interval = settings.get('hz_background_interval', 15.0)
        else:
            # Default timeout constants - balanced for mid-range systems
            self.QUICK_TIMEOUT = 2.5    # For initial quick checks
            self.MAX_TIMEOUT = 10.0     # Maximum timeout for very slow topics
            self.CACHE_FRESH_AGE = 5.0  # Cache considered "fresh"
            self.CACHE_STALE_AGE = 30.0 # Cache considered "stale"
            self._background_interval = 15.0  # Background monitoring interval
        
        self.MIN_TIMEOUT = 2.0      # Minimum timeout (always same)
        
    def get_hz_smart(self, topic_name: str, use_cache: bool = True) -> Tuple[float, str]:
        """
        Get Hz for a topic with smart caching and adaptive measurement.
        
        Returns:
            (hz_value, confidence) where confidence is 'fresh', 'cached', or 'estimated'
        """
        # Check cache first
        if use_cache:
            with self._cache_lock:
                if topic_name in self._hz_cache:
                    cached = self._hz_cache[topic_name]
                    age = time.time() - cached['timestamp']
                    
                    # Cache is fresh (< 5 seconds old)
                    if age < 5.0:
                        return cached['hz'], 'fresh'
                    # Cache is somewhat old but still usable (< 30 seconds)
                    elif age < 30.0:
                        # Return cached but trigger background refresh
                        self._add_to_background_monitor(topic_name)
                        return cached['hz'], 'cached'
        
        # No cache or too old - measure now
        hz = self._measure_hz_adaptive(topic_name)
        
        # Cache the result
        with self._cache_lock:
            self._hz_cache[topic_name] = {
                'hz': hz,
                'timestamp': time.time(),
                'confidence': 'measured'
            }
        
        return hz, 'measured'
    
    def get_hz_batch_smart(self, topic_names: list, quick_mode: bool = False) -> Dict[str, float]:
        """
        Get Hz values for multiple topics with smart optimization.
        
        Args:
            topic_names: List of topic names
            quick_mode: If True, use quick estimates; refine in background
            
        Returns:
            Dictionary mapping topic_name -> hz_value
        """
        results = {}
        topics_to_measure = []
        
        # Check cache first
        with self._cache_lock:
            for topic in topic_names:
                if topic in self._hz_cache:
                    cached = self._hz_cache[topic]
                    age = time.time() - cached['timestamp']
                    
                    if age < 5.0:
                        # Fresh cache - use it
                        results[topic] = cached['hz']
                    elif age < 30.0:
                        # Old cache - use it but schedule refresh
                        results[topic] = cached['hz']
                        topics_to_measure.append(topic)
                    else:
                        # Very old - measure now
                        topics_to_measure.append(topic)
                else:
                    # Not in cache - measure now
                    topics_to_measure.append(topic)
        
        if not topics_to_measure:
            return results
        
        # Measure remaining topics
        if quick_mode:
            # Quick mode: Use short timeout, refine in background
            measured = self._measure_batch_quick(topics_to_measure)
            results.update(measured)
            
            # Schedule background refinement
            for topic in topics_to_measure:
                self._add_to_background_monitor(topic)
        else:
            # Normal mode: Adaptive measurement
            measured = self._measure_batch_adaptive(topics_to_measure)
            results.update(measured)
        
        return results
    
    def _measure_hz_adaptive(self, topic_name: str) -> float:
        """
        Measure Hz with adaptive timeout based on previous measurements.
        
        Strategy:
        1. Try quick check (1.5s) first
        2. If no result, try adaptive timeout based on expected rate
        3. Cache result for future adaptive decisions
        """
        # Try quick measurement first
        hz = self._measure_hz_with_timeout(topic_name, self.QUICK_TIMEOUT)
        
        if hz > 0:
            # Got a result - determine if we need longer measurement
            if hz < 1.0:
                # Very slow topic - try longer timeout for accuracy
                hz_refined = self._measure_hz_with_timeout(topic_name, self.MAX_TIMEOUT)
                return hz_refined if hz_refined > 0 else hz
            else:
                return hz
        
        # No result from quick check - try longer timeout
        # Adaptive: slower expected rate = longer timeout
        timeout = self._calculate_adaptive_timeout(topic_name)
        hz = self._measure_hz_with_timeout(topic_name, timeout)
        
        return hz
    
    def _measure_hz_with_timeout(self, topic_name: str, timeout: float) -> float:
        """Measure Hz with specific timeout."""
        try:
            # Use Popen to read output as it streams
            process = subprocess.Popen(
                ['ros2', 'topic', 'hz', topic_name],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            start_time = time.time()
            last_hz = 0.0
            
            # Read output line by line with timeout
            while time.time() - start_time < timeout:
                try:
                    # Try to read a line with a short timeout
                    import select
                    if select.select([process.stdout], [], [], 0.1)[0]:
                        line = process.stdout.readline()
                        if line:
                            if 'average rate:' in line.lower():
                                try:
                                    hz_str = line.split(':')[-1].strip().split()[0]
                                    last_hz = max(0.0, float(hz_str))
                                    # Got a valid measurement - terminate and return
                                    process.terminate()
                                    process.wait(timeout=0.5)
                                    return last_hz
                                except (ValueError, IndexError):
                                    pass
                except Exception:
                    pass
            
            # Timeout reached - kill process and return last measurement
            process.terminate()
            try:
                process.wait(timeout=0.5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            
            return last_hz
            
        except subprocess.TimeoutExpired as e:
            # Parse output from timeout
            try:
                if e.stdout:
                    output_str = e.stdout.decode('utf-8') if isinstance(e.stdout, bytes) else str(e.stdout)
                    lines = output_str.strip().split('\n')
                    for line in lines:
                        if 'average rate:' in line.lower():
                            hz_str = line.split(':')[-1].strip()
                            return max(0.0, float(hz_str))
            except Exception:
                pass
        except Exception:
            pass
        
        return 0.0
    
    def _calculate_adaptive_timeout(self, topic_name: str) -> float:
        """Calculate adaptive timeout based on topic history."""
        with self._cache_lock:
            if topic_name in self._hz_cache:
                cached_hz = self._hz_cache[topic_name]['hz']
                
                if cached_hz > 10:
                    # Fast topic - short timeout
                    return 2.5
                elif cached_hz > 5:
                    # Medium topic
                    return 4.0
                elif cached_hz > 1:
                    # Slow topic
                    return 6.0
                else:
                    # Very slow topic
                    return self.MAX_TIMEOUT
        
        # Unknown topic - use max timeout to ensure detection
        return self.MAX_TIMEOUT
    
    def _measure_batch_quick(self, topic_names: list) -> Dict[str, float]:
        """Quick batch measurement with short timeout."""
        results = {}
        
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            future_to_topic = {
                executor.submit(self._measure_hz_with_timeout, topic, self.QUICK_TIMEOUT): topic
                for topic in topic_names
            }
            
            # Give extra time for all futures to complete (max 6s total for slow topics)
            for future in as_completed(future_to_topic, timeout=6.0):
                try:
                    topic = future_to_topic[future]
                    hz = future.result(timeout=0.1)
                    results[topic] = hz
                    
                    # Cache it
                    with self._cache_lock:
                        self._hz_cache[topic] = {
                            'hz': hz,
                            'timestamp': time.time(),
                            'confidence': 'quick'
                        }
                except Exception:
                    topic = future_to_topic.get(future)
                    if topic:
                        results[topic] = 0.0
        
        return results
    
    def _measure_batch_adaptive(self, topic_names: list) -> Dict[str, float]:
        """Adaptive batch measurement with per-topic timeout."""
        results = {}
        
        with ThreadPoolExecutor(max_workers=self.max_workers) as executor:
            # Submit with adaptive timeouts
            future_to_topic = {}
            for topic in topic_names:
                timeout = self._calculate_adaptive_timeout(topic)
                future = executor.submit(self._measure_hz_with_timeout, topic, timeout)
                future_to_topic[future] = topic
            
            # Collect results
            max_wait = max(self.MAX_TIMEOUT + 2.0, 12.0)  # Increased from 8.0
            for future in as_completed(future_to_topic, timeout=max_wait):
                try:
                    topic = future_to_topic[future]
                    hz = future.result(timeout=0.1)
                    results[topic] = hz
                    
                    # Cache it
                    with self._cache_lock:
                        self._hz_cache[topic] = {
                            'hz': hz,
                            'timestamp': time.time(),
                            'confidence': 'adaptive'
                        }
                except Exception:
                    topic = future_to_topic.get(future)
                    if topic:
                        results[topic] = 0.0
        
        return results
    
    def _add_to_background_monitor(self, topic_name: str):
        """Add topic to background monitoring queue."""
        self._topics_to_monitor.add(topic_name)
        
        # Start background monitor if not running
        if not self._monitoring_active:
            self.start_background_monitoring()
    
    def start_background_monitoring(self, interval: float = 10.0):
        """Start background thread to continuously monitor topics."""
        if self._monitoring_active:
            return
        
        self._monitoring_active = True
        self._monitor_thread = threading.Thread(
            target=self._background_monitor_loop,
            args=(interval,),
            daemon=True,
            name="TopicHzBackgroundMonitor"
        )
        self._monitor_thread.start()
    
    def _background_monitor_loop(self, interval: float):
        """Background monitoring loop."""
        while self._monitoring_active:
            try:
                # Get topics to monitor
                topics = list(self._topics_to_monitor)
                
                if topics:
                    # Measure them with adaptive timeouts
                    results = self._measure_batch_adaptive(topics)
                    
                    # Update cache
                    with self._cache_lock:
                        for topic, hz in results.items():
                            self._hz_cache[topic] = {
                                'hz': hz,
                                'timestamp': time.time(),
                                'confidence': 'background'
                            }
                    
                    # Remove from monitoring queue (will be re-added if needed)
                    self._topics_to_monitor.clear()
                
                # Sleep before next cycle
                time.sleep(interval)
                
            except Exception as e:
                print(f"Background monitor error: {e}")
                time.sleep(interval)
    
    def stop_background_monitoring(self):
        """Stop background monitoring."""
        self._monitoring_active = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
    
    def clear_cache(self):
        """Clear the Hz cache."""
        with self._cache_lock:
            self._hz_cache.clear()
    
    def get_cache_info(self) -> Dict:
        """Get information about cached topics."""
        with self._cache_lock:
            return {
                topic: {
                    'hz': data['hz'],
                    'age': time.time() - data['timestamp'],
                    'confidence': data['confidence']
                }
                for topic, data in self._hz_cache.items()
            }
