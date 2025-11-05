"""
Dynamic Hz Scaling - Intelligently scale Hz monitoring for 1-1000+ topics
"""

import psutil
import os
import threading
from typing import Dict, Optional


class DynamicHzScaler:
    """Calculate optimal Hz monitoring parameters based on system resources"""
    
    def __init__(self):
        self._lock = threading.Lock()
        self._config_cache = {}
        self._last_update_time = 0
        self._cache_timeout = 5.0
    
    @staticmethod
    def get_system_resources() -> Dict:
        """Get current system resource availability"""
        try:
            cpu_count = os.cpu_count() or 4
            cpu_percent = psutil.cpu_percent(interval=0.1)
            mem = psutil.virtual_memory()
            available_gb = mem.available / (1024**3)
            
            return {
                'cpu_count': cpu_count,
                'cpu_percent': cpu_percent,
                'available_memory_gb': available_gb,
                'memory_percent': mem.percent
            }
        except Exception:
            return {
                'cpu_count': 4,
                'cpu_percent': 50,
                'available_memory_gb': 2.0,
                'memory_percent': 50
            }
    
    @staticmethod
    def calculate_optimal_workers(topic_count: int, system_resources: Optional[Dict] = None) -> Dict:
        """Calculate optimal Hz worker configuration for a given topic count"""
        
        if system_resources is None:
            system_resources = DynamicHzScaler.get_system_resources()
        
        if topic_count == 0:
            return {
                'max_workers': 1,
                'batch_size': 1,
                'timeout_per_topic': 1.5,
                'total_expected_time': 0,
                'reasoning': 'No topics to measure'
            }
        
        cpu_count = system_resources.get('cpu_count', 4)
        cpu_percent = system_resources.get('cpu_percent', 50)
        available_memory_gb = system_resources.get('available_memory_gb', 2.0)
        
        workers_from_memory = max(1, int(available_memory_gb * 100))
        workers_from_cpu = cpu_count * 2
        workers_from_topics = max(1, topic_count // 3)
        base_workers = min(workers_from_cpu, workers_from_topics, workers_from_memory)
        
        # CPU load adjustment
        if cpu_percent > 85:
            load_factor = 0.5
            reason = "CPU >85%"
        elif cpu_percent > 75:
            load_factor = 0.75
            reason = "CPU >75%"
        elif cpu_percent > 60:
            load_factor = 1.0
            reason = "CPU 60-75%"
        else:
            load_factor = 1.2
            reason = "CPU <60%"
        
        adjusted_workers = max(1, int(base_workers * load_factor))
        final_workers = min(adjusted_workers, topic_count, 256)
        
        # Timeout strategy
        if topic_count <= 10:
            timeout_per_topic = 2.0
        elif topic_count <= 50:
            timeout_per_topic = 1.5
        elif topic_count <= 200:
            timeout_per_topic = 1.0
        elif topic_count <= 500:
            timeout_per_topic = 0.5
        else:
            timeout_per_topic = 0.3
        
        batch_size = max(1, topic_count // final_workers)
        batches_needed = (topic_count + batch_size - 1) // batch_size
        total_expected_time = batches_needed * timeout_per_topic
        
        reasoning = f"Topics={topic_count}, Workers={final_workers}, Load={reason}"
        
        return {
            'max_workers': final_workers,
            'batch_size': batch_size,
            'timeout_per_topic': timeout_per_topic,
            'total_expected_time': total_expected_time,
            'reasoning': reasoning,
            'base_workers': base_workers,
            'load_factor': load_factor,
            'cpu_count': cpu_count,
            'available_memory_gb': available_memory_gb
        }
    
    def get_scaled_config(self, topic_count: int) -> Dict:
        """Get cached or recalculated configuration"""
        import time
        
        with self._lock:
            current_time = time.time()
            
            if (topic_count in self._config_cache and 
                current_time - self._last_update_time < self._cache_timeout):
                return self._config_cache[topic_count]
            
            config = self.calculate_optimal_workers(topic_count)
            self._config_cache[topic_count] = config
            self._last_update_time = current_time
            
            return config
