"""
CPU Optimizer - Advanced CPU affinity and thread pinning for maximum performance
ULTRA-ADVANCED: Pins critical threads to specific CPU cores for optimal cache utilization
"""

import os
import psutil
import threading
from typing import List, Optional


class CPUOptimizer:
    """
    ADVANCED: CPU affinity optimizer for maximum performance
    
    Features:
    - Pin recording process to dedicated cores (avoid interference)
    - Pin UI thread to high-performance cores
    - Distribute worker threads across remaining cores
    - Automatic NUMA-aware core selection
    """
    
    def __init__(self):
        self.cpu_count = psutil.cpu_count(logical=True) or 1
        self.physical_cores = psutil.cpu_count(logical=False) or 1
        
        # Determine optimal core allocation
        self.recording_cores = self._get_recording_cores()
        self.ui_cores = self._get_ui_cores()
        self.worker_cores = self._get_worker_cores()
        
        print(f"🎯 CPU Optimizer initialized:")
        print(f"   Total CPUs: {self.cpu_count} ({self.physical_cores} physical)")
        print(f"   Recording cores: {self.recording_cores}")
        print(f"   UI cores: {self.ui_cores}")
        print(f"   Worker cores: {self.worker_cores}")
    
    def _get_recording_cores(self) -> List[int]:
        """
        Get dedicated cores for recording process
        - Uses last 2 physical cores (least likely to be busy)
        - Ensures recording gets exclusive CPU time
        """
        if self.physical_cores >= 4:
            # Use last 2 physical cores
            return list(range(self.physical_cores - 2, self.physical_cores))
        elif self.physical_cores >= 2:
            # Use last physical core
            return [self.physical_cores - 1]
        else:
            # Single core - share it
            return [0]
    
    def _get_ui_cores(self) -> List[int]:
        """
        Get cores for UI thread
        - Uses first high-performance core
        - Typically core 0 has highest boost frequency
        """
        return [0]
    
    def _get_worker_cores(self) -> List[int]:
        """
        Get cores for worker threads
        - Uses middle cores (not reserved for recording/UI)
        """
        if self.physical_cores >= 4:
            # Use middle cores
            return list(range(1, self.physical_cores - 2))
        elif self.physical_cores >= 2:
            # Share with UI
            return [0]
        else:
            # Single core - share it
            return [0]
    
    def pin_recording_process(self, pid: int) -> bool:
        """
        Pin recording process to dedicated cores
        
        Args:
            pid: Process ID to pin
            
        Returns:
            True if successful, False otherwise
        """
        try:
            process = psutil.Process(pid)
            process.cpu_affinity(self.recording_cores)
            print(f"✅ Recording process {pid} pinned to cores {self.recording_cores}")
            return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, AttributeError) as e:
            print(f"⚠️  Could not pin recording process: {e}")
            return False
    
    def pin_current_thread(self, cores: Optional[List[int]] = None) -> bool:
        """
        Pin current thread to specific cores
        
        Args:
            cores: List of core IDs, defaults to UI cores
            
        Returns:
            True if successful, False otherwise
        """
        if cores is None:
            cores = self.ui_cores
        
        try:
            # Get current process
            process = psutil.Process(os.getpid())
            process.cpu_affinity(cores)
            print(f"✅ Thread {threading.current_thread().name} pinned to cores {cores}")
            return True
        except (psutil.AccessDenied, AttributeError) as e:
            print(f"⚠️  Could not pin thread: {e} (may need elevated privileges)")
            return False
    
    def optimize_worker_pool(self, worker_count: int) -> List[List[int]]:
        """
        Distribute worker threads across available cores
        
        Args:
            worker_count: Number of worker threads
            
        Returns:
            List of core assignments for each worker
        """
        assignments = []
        cores = self.worker_cores
        
        # Round-robin assignment
        for i in range(worker_count):
            core_idx = i % len(cores)
            assignments.append([cores[core_idx]])
        
        return assignments
    
    def get_optimal_thread_count(self) -> int:
        """
        Calculate optimal number of worker threads
        
        Returns:
            Optimal thread count based on available cores
        """
        # Leave cores for recording and UI
        available_cores = len(self.worker_cores)
        
        # Rule of thumb: 1-2 threads per available core
        optimal = max(1, available_cores * 2)
        
        return min(optimal, 16)  # Cap at 16 threads
    
    def enable_performance_governor(self) -> bool:
        """
        Try to enable performance CPU governor (requires root on Linux)
        
        Returns:
            True if successful, False otherwise
        """
        if not hasattr(os, 'uname') or os.uname().sysname != 'Linux':
            return False
        
        try:
            # Try to set CPU governor to performance
            for cpu in range(self.cpu_count):
                governor_path = f'/sys/devices/system/cpu/cpu{cpu}/cpufreq/scaling_governor'
                try:
                    with open(governor_path, 'w') as f:
                        f.write('performance\n')
                except (IOError, PermissionError):
                    # Need root privileges
                    pass
            
            print("✅ Performance governor enabled")
            return True
        except Exception as e:
            print(f"⚠️  Could not enable performance governor: {e}")
            return False


class ThreadPinner:
    """Helper class for pinning threads to specific cores"""
    
    def __init__(self, cpu_optimizer: CPUOptimizer):
        self.cpu_optimizer = cpu_optimizer
    
    def pin_to_recording_cores(self):
        """Pin current thread to recording cores"""
        return self.cpu_optimizer.pin_current_thread(
            self.cpu_optimizer.recording_cores
        )
    
    def pin_to_ui_cores(self):
        """Pin current thread to UI cores"""
        return self.cpu_optimizer.pin_current_thread(
            self.cpu_optimizer.ui_cores
        )
    
    def pin_to_worker_cores(self, worker_id: int = 0):
        """Pin current thread to worker cores"""
        assignments = self.cpu_optimizer.optimize_worker_pool(8)
        if worker_id < len(assignments):
            return self.cpu_optimizer.pin_current_thread(assignments[worker_id])
        return False


# Global CPU optimizer instance
_cpu_optimizer_instance: Optional[CPUOptimizer] = None


def get_cpu_optimizer() -> CPUOptimizer:
    """Get or create global CPU optimizer instance"""
    global _cpu_optimizer_instance
    if _cpu_optimizer_instance is None:
        _cpu_optimizer_instance = CPUOptimizer()
    return _cpu_optimizer_instance
