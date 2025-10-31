"""
Network Manager - Handles offline-first data upload with resume capability
"""

import os
import json
import time
import threading
import hashlib
import requests
from datetime import datetime
from queue import Queue, PriorityQueue
import sqlite3
from pathlib import Path


class UploadTask:
    """Represents an upload task with priority and metadata"""
    
    def __init__(self, file_path, priority=5, metadata=None, chunk_size=5*1024*1024):
        self.file_path = file_path
        self.priority = priority  # Lower number = higher priority
        self.metadata = metadata or {}
        self.chunk_size = chunk_size  # 5MB chunks by default
        self.upload_id = None
        self.uploaded_chunks = set()
        self.total_chunks = 0
        self.created_at = time.time()
        self.retry_count = 0
        self.last_error = None
        self.last_chunk_time = None  # Track last successful chunk for timeout detection
        self.chunks_since_progress = 0  # Count chunks without progress for stall detection
        
    def __lt__(self, other):
        # For priority queue comparison
        return self.priority < other.priority
        

class NetworkManager:
    """
    Manages offline-first data uploads with resume capability.
    Features:
    - Automatic retry on network failure
    - Chunked uploads with resume capability
    - Priority-based upload queue
    - Persistent state across restarts
    - Bandwidth throttling
    - Upload progress tracking
    """
    
    def __init__(self, upload_url=None, cache_dir=None):
        self.upload_url = upload_url or "http://localhost:8080/upload"
        self.cache_dir = cache_dir or os.path.expanduser("~/.ros2_dashboard/upload_cache")
        os.makedirs(self.cache_dir, exist_ok=True)
        
        # Database for persistent state
        self.db_path = os.path.join(self.cache_dir, "upload_state.db")
        self._init_database()
        
        # Upload queue (priority queue)
        self.upload_queue = PriorityQueue()
        self.active_uploads = {}
        
        # Network state
        self.is_online = False
        self.last_connection_check = 0
        self.connection_check_interval = 5  # seconds
        
        # Upload statistics
        self.stats = {
            'total_uploaded': 0,
            'total_failed': 0,
            'bytes_uploaded': 0,
            'bytes_pending': 0,
            'active_uploads': 0,
            'queued_uploads': 0
        }
        
        # Threading
        self.upload_thread = None
        self.monitor_thread = None
        self.running = False
        self.upload_lock = threading.Lock()
        
        # Configuration
        self.max_retries = 5
        self.retry_delay = 10  # seconds
        self.max_concurrent_uploads = 2
        self.bandwidth_limit = None  # bytes/second (None = unlimited)
        
        # Timeout configuration
        self.upload_timeout = 300  # 5 minutes timeout for chunk uploads
        self.init_timeout = 10  # seconds for init/finalize
        self.stalled_timeout = 60  # Mark transfer as stalled after 60s without progress
        
        # Load pending uploads from database
        self._load_pending_uploads()
        
    def _init_database(self):
        """Initialize SQLite database for persistent state"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS upload_tasks (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                file_path TEXT NOT NULL,
                upload_id TEXT,
                priority INTEGER DEFAULT 5,
                status TEXT DEFAULT 'pending',
                metadata TEXT,
                uploaded_chunks TEXT,
                total_chunks INTEGER,
                created_at REAL,
                updated_at REAL,
                retry_count INTEGER DEFAULT 0,
                last_error TEXT,
                file_size INTEGER,
                bytes_uploaded INTEGER DEFAULT 0
            )
        ''')
        
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS upload_history (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                file_path TEXT NOT NULL,
                upload_id TEXT,
                status TEXT,
                completed_at REAL,
                file_size INTEGER,
                upload_duration REAL,
                error TEXT
            )
        ''')
        
        conn.commit()
        conn.close()
        
    def _load_pending_uploads(self):
        """Load pending uploads from database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT file_path, upload_id, priority, metadata, uploaded_chunks, 
                   total_chunks, retry_count, last_error
            FROM upload_tasks 
            WHERE status IN ('pending', 'uploading', 'paused')
            ORDER BY priority ASC, created_at ASC
        ''')
        
        for row in cursor.fetchall():
            file_path, upload_id, priority, metadata_json, chunks_json, total_chunks, retry_count, last_error = row
            
            if not os.path.exists(file_path):
                # File no longer exists, mark as failed
                cursor.execute('UPDATE upload_tasks SET status = ? WHERE file_path = ?', 
                             ('failed', file_path))
                continue
                
            task = UploadTask(file_path, priority)
            task.upload_id = upload_id
            task.metadata = json.loads(metadata_json) if metadata_json else {}
            task.uploaded_chunks = set(json.loads(chunks_json)) if chunks_json else set()
            task.total_chunks = total_chunks or 0
            task.retry_count = retry_count or 0
            task.last_error = last_error
            
            self.upload_queue.put(task)
            
        conn.commit()
        conn.close()
        
    def start(self):
        """Start the network manager"""
        if self.running:
            return
            
        self.running = True
        
        # Start network monitor thread first (non-blocking)
        self.monitor_thread = threading.Thread(target=self._network_monitor, daemon=True)
        self.monitor_thread.start()
        
        # Start upload worker thread
        self.upload_thread = threading.Thread(target=self._upload_worker, daemon=True)
        self.upload_thread.start()
        
        print("Network Manager started (background mode)")
        
    def stop(self):
        """Stop the network manager"""
        self.running = False
        
        if self.upload_thread:
            self.upload_thread.join(timeout=5)
            
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5)
            
        print("Network Manager stopped")
        
    def add_upload(self, file_path, priority=5, metadata=None):
        """
        Add a file to the upload queue
        
        Args:
            file_path: Path to the file to upload
            priority: Upload priority (lower = higher priority)
            metadata: Additional metadata to send with the file
        """
        if not os.path.exists(file_path):
            print(f"File not found: {file_path}")
            return False
            
        # Create upload task
        task = UploadTask(file_path, priority, metadata)
        
        # Calculate file size and chunks
        file_size = os.path.getsize(file_path)
        task.total_chunks = (file_size + task.chunk_size - 1) // task.chunk_size
        
        # Save to database
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            INSERT INTO upload_tasks 
            (file_path, priority, status, metadata, total_chunks, created_at, updated_at, file_size)
            VALUES (?, ?, ?, ?, ?, ?, ?, ?)
        ''', (file_path, priority, 'pending', json.dumps(metadata or {}), 
              task.total_chunks, task.created_at, time.time(), file_size))
        
        conn.commit()
        conn.close()
        
        # Add to queue
        self.upload_queue.put(task)
        self.stats['queued_uploads'] += 1
        
        print(f"Added upload task: {file_path} (priority: {priority}, chunks: {task.total_chunks})")
        return True
        
    def _network_monitor(self):
        """Monitor network connectivity"""
        while self.running:
            current_time = time.time()
            
            if current_time - self.last_connection_check >= self.connection_check_interval:
                self.is_online = self._check_connectivity()
                self.last_connection_check = current_time
                
            time.sleep(1)
            
    def _check_connectivity(self):
        """Check if server is reachable"""
        try:
            # Try to connect to upload server with short timeout
            response = requests.get(
                self.upload_url.replace('/upload', '/health'),
                timeout=1  # Very short timeout
            )
            return response.status_code == 200
        except:
            # If health endpoint doesn't exist, try a simple HEAD request
            try:
                response = requests.head(self.upload_url, timeout=1)
                return True
            except:
                # Don't print errors - normal when server not running
                return False
                
    def _upload_worker(self):
        """Worker thread for processing uploads"""
        while self.running:
            # Check if we can upload
            if not self.is_online:
                time.sleep(2)
                continue
                
            # Check concurrent upload limit
            if len(self.active_uploads) >= self.max_concurrent_uploads:
                time.sleep(1)
                continue
                
            # Get next task from queue
            try:
                task = self.upload_queue.get(timeout=1)
            except:
                continue
                
            # Check if file still exists
            if not os.path.exists(task.file_path):
                self._mark_task_failed(task, "File not found")
                continue
                
            # Check retry limit
            if task.retry_count >= self.max_retries:
                self._mark_task_failed(task, f"Max retries ({self.max_retries}) exceeded")
                continue
                
            # Start upload
            self.active_uploads[task.file_path] = task
            self.stats['active_uploads'] = len(self.active_uploads)
            
            upload_thread = threading.Thread(
                target=self._upload_file,
                args=(task,),
                daemon=True
            )
            upload_thread.start()
            
    def _upload_file(self, task):
        """Upload a file with chunked upload and resume capability"""
        try:
            file_size = os.path.getsize(task.file_path)
            
            # Initialize upload if not already started
            if not task.upload_id:
                task.upload_id = self._initialize_upload(task)
                if not task.upload_id:
                    raise Exception("Failed to initialize upload")
                    
            # Upload chunks
            with open(task.file_path, 'rb') as f:
                for chunk_index in range(task.total_chunks):
                    # Skip already uploaded chunks
                    if chunk_index in task.uploaded_chunks:
                        continue
                        
                    # Check if still online
                    if not self.is_online:
                        raise Exception("Network connection lost")
                        
                    # Read chunk
                    f.seek(chunk_index * task.chunk_size)
                    chunk_data = f.read(task.chunk_size)
                    
                    # Upload chunk
                    if self._upload_chunk(task, chunk_index, chunk_data):
                        task.uploaded_chunks.add(chunk_index)
                        self._update_task_progress(task)
                        
                        # Update stats
                        self.stats['bytes_uploaded'] += len(chunk_data)
                    else:
                        raise Exception(f"Failed to upload chunk {chunk_index}")
                        
                    # Bandwidth throttling
                    if self.bandwidth_limit:
                        time.sleep(len(chunk_data) / self.bandwidth_limit)
                        
            # Finalize upload
            if self._finalize_upload(task):
                self._mark_task_completed(task)
                print(f"✓ Upload completed: {task.file_path}")
            else:
                raise Exception("Failed to finalize upload")
                
        except Exception as e:
            task.retry_count += 1
            task.last_error = str(e)
            
            print(f"✗ Upload failed: {task.file_path} - {e}")
            
            # Re-queue if retries available
            if task.retry_count < self.max_retries:
                self._update_task_status(task, 'paused')
                time.sleep(self.retry_delay)
                self.upload_queue.put(task)
            else:
                self._mark_task_failed(task, str(e))
                
        finally:
            # Remove from active uploads
            if task.file_path in self.active_uploads:
                del self.active_uploads[task.file_path]
                self.stats['active_uploads'] = len(self.active_uploads)
                
    def _initialize_upload(self, task):
        """Initialize a new upload session"""
        try:
            file_name = os.path.basename(task.file_path)
            file_size = os.path.getsize(task.file_path)
            
            response = requests.post(
                f"{self.upload_url}/init",
                json={
                    'filename': file_name,
                    'filesize': file_size,
                    'chunks': task.total_chunks,
                    'metadata': task.metadata,
                    'checksum': self._calculate_checksum(task.file_path),
                    'compression': file_size >= 10 * 1024 * 1024  # Compression for large files
                },
                timeout=self.init_timeout
            )
            
            if response.status_code == 200:
                data = response.json()
                task.last_chunk_time = time.time()  # Initialize progress timer
                return data.get('upload_id')
                
        except requests.Timeout:
            print(f"⏱️ Timeout initializing upload for {task.file_path}")
            raise Exception("Upload initialization timeout")
        except Exception as e:
            print(f"Failed to initialize upload: {e}")
            
        return None
        
    def _upload_chunk(self, task, chunk_index, chunk_data):
        """Upload a single chunk with timeout and stall detection"""
        try:
            # Check for stalled transfer (no progress for extended time)
            current_time = time.time()
            if task.last_chunk_time and (current_time - task.last_chunk_time) > self.stalled_timeout:
                print(f"⏱️ Stalled upload detected for {task.file_path} (no progress for {self.stalled_timeout}s)")
                raise Exception("Upload stalled - no progress detected")
            
            files = {
                'chunk': (f'chunk_{chunk_index}', chunk_data)
            }
            
            data = {
                'upload_id': task.upload_id,
                'chunk_index': chunk_index,
                'chunk_total': task.total_chunks
            }
            
            response = requests.post(
                f"{self.upload_url}/chunk",
                files=files,
                data=data,
                timeout=self.upload_timeout
            )
            
            if response.status_code == 200:
                # Reset stall detector on successful chunk
                task.last_chunk_time = time.time()
                task.chunks_since_progress = 0
                return True
            else:
                task.chunks_since_progress += 1
                return False
            
        except requests.Timeout:
            print(f"⏱️ Timeout uploading chunk {chunk_index} for {task.file_path}")
            raise Exception(f"Chunk upload timeout after {self.upload_timeout}s")
        except Exception as e:
            print(f"Failed to upload chunk {chunk_index}: {e}")
            return False
            
    def _finalize_upload(self, task):
        """Finalize the upload"""
        try:
            response = requests.post(
                f"{self.upload_url}/finalize",
                json={
                    'upload_id': task.upload_id,
                    'checksum': self._calculate_checksum(task.file_path)
                },
                timeout=self.init_timeout
            )
            
            if response.status_code == 200:
                task.last_chunk_time = time.time()
                return True
            return False
            
        except requests.Timeout:
            print(f"⏱️ Timeout finalizing upload for {task.file_path}")
            raise Exception("Upload finalization timeout")
        except Exception as e:
            print(f"Failed to finalize upload: {e}")
            return False
            
    def _calculate_checksum(self, file_path):
        """Calculate MD5 checksum of file"""
        md5 = hashlib.md5()
        
        with open(file_path, 'rb') as f:
            for chunk in iter(lambda: f.read(8192), b''):
                md5.update(chunk)
                
        return md5.hexdigest()
        
    def _update_task_progress(self, task):
        """Update task progress in database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            UPDATE upload_tasks 
            SET uploaded_chunks = ?, updated_at = ?, bytes_uploaded = ?
            WHERE file_path = ?
        ''', (json.dumps(list(task.uploaded_chunks)), time.time(),
              len(task.uploaded_chunks) * task.chunk_size, task.file_path))
        
        conn.commit()
        conn.close()
        
    def _update_task_status(self, task, status):
        """Update task status in database"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            UPDATE upload_tasks 
            SET status = ?, updated_at = ?, retry_count = ?, last_error = ?
            WHERE file_path = ?
        ''', (status, time.time(), task.retry_count, task.last_error, task.file_path))
        
        conn.commit()
        conn.close()
        
    def _mark_task_completed(self, task):
        """Mark task as completed"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Update task status
        cursor.execute('''
            UPDATE upload_tasks 
            SET status = ?, updated_at = ?
            WHERE file_path = ?
        ''', ('completed', time.time(), task.file_path))
        
        # Add to history
        file_size = os.path.getsize(task.file_path)
        duration = time.time() - task.created_at
        
        cursor.execute('''
            INSERT INTO upload_history 
            (file_path, upload_id, status, completed_at, file_size, upload_duration)
            VALUES (?, ?, ?, ?, ?, ?)
        ''', (task.file_path, task.upload_id, 'completed', time.time(), file_size, duration))
        
        conn.commit()
        conn.close()
        
        self.stats['total_uploaded'] += 1
        
    def _mark_task_failed(self, task, error):
        """Mark task as failed"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            UPDATE upload_tasks 
            SET status = ?, updated_at = ?, last_error = ?
            WHERE file_path = ?
        ''', ('failed', time.time(), error, task.file_path))
        
        # Add to history
        cursor.execute('''
            INSERT INTO upload_history 
            (file_path, upload_id, status, completed_at, error)
            VALUES (?, ?, ?, ?, ?)
        ''', (task.file_path, task.upload_id, 'failed', time.time(), error))
        
        conn.commit()
        conn.close()
        
        self.stats['total_failed'] += 1
        
    def get_stats(self):
        """Get current upload statistics"""
        return self.stats.copy()
        
    def get_pending_uploads(self):
        """Get list of pending uploads"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT file_path, priority, status, retry_count, 
                   total_chunks, uploaded_chunks, file_size, bytes_uploaded
            FROM upload_tasks 
            WHERE status IN ('pending', 'uploading', 'paused')
            ORDER BY priority ASC, created_at ASC
        ''')
        
        pending = []
        for row in cursor.fetchall():
            file_path, priority, status, retry_count, total_chunks, chunks_json, file_size, bytes_uploaded = row
            uploaded_chunks = len(json.loads(chunks_json)) if chunks_json else 0
            
            progress = (uploaded_chunks / total_chunks * 100) if total_chunks > 0 else 0
            
            pending.append({
                'file_path': file_path,
                'priority': priority,
                'status': status,
                'retry_count': retry_count,
                'progress': progress,
                'file_size': file_size,
                'bytes_uploaded': bytes_uploaded
            })
            
        conn.close()
        return pending
        
    def get_upload_history(self, limit=50):
        """Get upload history"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        cursor.execute('''
            SELECT file_path, status, completed_at, file_size, upload_duration, error
            FROM upload_history 
            ORDER BY completed_at DESC
            LIMIT ?
        ''', (limit,))
        
        history = []
        for row in cursor.fetchall():
            file_path, status, completed_at, file_size, upload_duration, error = row
            
            history.append({
                'file_path': file_path,
                'status': status,
                'completed_at': datetime.fromtimestamp(completed_at).strftime('%Y-%m-%d %H:%M:%S'),
                'file_size': file_size,
                'upload_duration': upload_duration,
                'error': error
            })
            
        conn.close()
        return history
    
    def retry_upload(self, file_path):
        """Retry a failed upload"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Check if file exists and is failed
        cursor.execute('''
            SELECT retry_count FROM upload_tasks 
            WHERE file_path = ? AND status = 'failed'
        ''', (file_path,))
        
        result = cursor.fetchone()
        if result:
            # Reset to pending for retry
            cursor.execute('''
                UPDATE upload_tasks 
                SET status = 'pending', uploaded_chunks = '[]', bytes_uploaded = 0, updated_at = ?
                WHERE file_path = ?
            ''', (time.time(), file_path))
            
            conn.commit()
        
        conn.close()
    
    def clear_upload(self, file_path):
        """Clear a completed upload from the list"""
        conn = sqlite3.connect(self.db_path)
        cursor = conn.cursor()
        
        # Delete completed upload task
        cursor.execute('''
            DELETE FROM upload_tasks 
            WHERE file_path = ? AND status = 'completed'
        ''', (file_path,))
        
        conn.commit()
        conn.close()
