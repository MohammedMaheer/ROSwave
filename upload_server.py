#!/usr/bin/env python3
"""
Production-Ready Upload Server for ROS2 Dashboard
Features:
- Chunked uploads with resume capability
- Rate limiting to prevent abuse
- SSL/TLS support for HTTPS
- Compression support (gzip) for large files
- Comprehensive error handling and logging
"""

from flask import Flask, request, jsonify
from flask_cors import CORS
try:
    from flask_limiter import Limiter
    from flask_limiter.util import get_remote_address
    LIMITER_AVAILABLE = True
except ImportError:
    LIMITER_AVAILABLE = False
    print("⚠️ flask-limiter not installed. Rate limiting disabled.")

import os
import uuid
import hashlib
import json
import gzip
import shutil
import ssl
from datetime import datetime
from typing import Dict, Any, Tuple, Optional
import logging

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

app = Flask(__name__)
CORS(app)

# Rate limiter: Prevent abuse (optional)
if LIMITER_AVAILABLE:
    limiter = Limiter(
        app=app,
        key_func=get_remote_address,
        default_limits=["200 per day", "50 per hour"],
        storage_uri="memory://"
    )
else:
    # Dummy decorator when limiter not available
    class DummyLimiter:
        def limit(self, *args, **kwargs):
            def decorator(f):
                return f
            return decorator
    limiter = DummyLimiter()

# Configuration
UPLOAD_DIR = os.path.expanduser("~/ros2_uploads")
TEMP_DIR = os.path.join(UPLOAD_DIR, "temp")
COMPLETED_DIR = os.path.join(UPLOAD_DIR, "completed")
COMPRESSED_DIR = os.path.join(UPLOAD_DIR, "compressed")

os.makedirs(UPLOAD_DIR, exist_ok=True)
os.makedirs(TEMP_DIR, exist_ok=True)
os.makedirs(COMPLETED_DIR, exist_ok=True)
os.makedirs(COMPRESSED_DIR, exist_ok=True)

# In-memory storage for upload sessions
upload_sessions: Dict[str, Dict[str, Any]] = {}

# Compression settings
COMPRESSION_THRESHOLD = 10 * 1024 * 1024  # Compress files > 10MB
COMPRESSION_LEVEL = 6  # 1-9 (6 is default balance)


@app.route('/health', methods=['GET'])
def health_check() -> Tuple[Any, int]:
    """Health check endpoint"""
    return jsonify({
        'status': 'ok',
        'timestamp': datetime.now().isoformat(),
        'version': '2.0-production',
        'features': ['chunked-upload', 'compression', 'rate-limiting', 'ssl-support']
    }), 200


@app.route('/upload/init', methods=['POST'])
@limiter.limit("20 per hour")
def initialize_upload() -> Tuple[Any, int]:
    """Initialize a new upload session with compression option"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({'success': False, 'error': 'Invalid JSON'}), 400

        filename: Optional[str] = data.get('filename')
        filesize: Optional[int] = data.get('filesize')
        chunks: Optional[int] = data.get('chunks')
        metadata: Dict[str, Any] = data.get('metadata', {})
        checksum: Optional[str] = data.get('checksum')
        compression: bool = data.get('compression', False)

        # Validate required fields
        if not filename or filesize is None or chunks is None:
            return jsonify({
                'success': False,
                'error': 'Missing required fields: filename, filesize, chunks'
            }), 400

        # Validate file size (limit to 5GB)
        max_file_size = 5 * 1024 * 1024 * 1024
        if filesize > max_file_size:
            return jsonify({
                'success': False,
                'error': f'File size exceeds maximum (5GB)'
            }), 413

        # Generate upload ID
        upload_id = str(uuid.uuid4())

        # Create session
        upload_sessions[upload_id] = {
            'filename': filename,
            'filesize': filesize,
            'chunks': chunks,
            'metadata': metadata,
            'checksum': checksum,
            'compression': compression and filesize >= COMPRESSION_THRESHOLD,
            'received_chunks': set(),
            'created_at': datetime.now().isoformat(),
            'temp_dir': os.path.join(TEMP_DIR, upload_id)
        }

        # Create temp directory for chunks
        os.makedirs(upload_sessions[upload_id]['temp_dir'], exist_ok=True)

        logger.info(
            f"Initialized upload: {filename} ({filesize} bytes, {chunks} chunks) "
            f"- ID: {upload_id} - Compression: {upload_sessions[upload_id]['compression']}"
        )

        return jsonify({
            'success': True,
            'upload_id': upload_id,
            'message': 'Upload session created',
            'compression_enabled': upload_sessions[upload_id]['compression']
        }), 200

    except Exception as e:
        logger.error(f"Error initializing upload: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/upload/chunk', methods=['POST'])
@limiter.limit("100 per minute")
def upload_chunk() -> Tuple[Any, int]:
    """Upload a single chunk"""
    try:
        upload_id: Optional[str] = request.form.get('upload_id')
        chunk_index_str: Optional[str] = request.form.get('chunk_index')
        chunk_total_str: Optional[str] = request.form.get('chunk_total')

        # Validate required fields
        if not upload_id or not chunk_index_str or not chunk_total_str:
            return jsonify({
                'success': False,
                'error': 'Missing required fields: upload_id, chunk_index, chunk_total'
            }), 400

        try:
            chunk_index = int(chunk_index_str)
            chunk_total = int(chunk_total_str)
        except (ValueError, TypeError):
            return jsonify({
                'success': False,
                'error': 'Invalid chunk_index or chunk_total (must be integers)'
            }), 400

        if upload_id not in upload_sessions:
            return jsonify({'success': False, 'error': 'Invalid upload ID'}), 400

        session = upload_sessions[upload_id]

        # Get chunk data
        chunk_file = request.files.get('chunk')
        if not chunk_file:
            return jsonify({'success': False, 'error': 'No chunk data provided'}), 400

        # Save chunk
        chunk_path = os.path.join(session['temp_dir'], f'chunk_{chunk_index}')
        chunk_file.save(chunk_path)

        # Mark chunk as received
        session['received_chunks'].add(chunk_index)

        progress = len(session['received_chunks']) / chunk_total * 100
        logger.info(
            f"Received chunk {chunk_index + 1}/{chunk_total} for {session['filename']} ({progress:.1f}%)"
        )

        return jsonify({
            'success': True,
            'received_chunks': len(session['received_chunks']),
            'total_chunks': chunk_total,
            'progress': progress
        }), 200

    except Exception as e:
        logger.error(f"Error uploading chunk: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/upload/finalize', methods=['POST'])
@limiter.limit("10 per hour")
def finalize_upload() -> Tuple[Any, int]:
    """Finalize upload by combining chunks with optional compression"""
    try:
        data = request.get_json()
        if not data:
            return jsonify({'success': False, 'error': 'Invalid JSON'}), 400

        upload_id: Optional[str] = data.get('upload_id')
        checksum: Optional[str] = data.get('checksum')

        if not upload_id:
            return jsonify({'success': False, 'error': 'Missing upload_id'}), 400

        if upload_id not in upload_sessions:
            return jsonify({'success': False, 'error': 'Invalid upload ID'}), 400

        session = upload_sessions[upload_id]

        # Verify all chunks received
        if len(session['received_chunks']) != session['chunks']:
            missing = session['chunks'] - len(session['received_chunks'])
            return jsonify({
                'success': False,
                'error': f"Missing {missing} chunk(s): {len(session['received_chunks'])}/{session['chunks']}"
            }), 400

        # Combine chunks
        output_path = os.path.join(COMPLETED_DIR, session['filename'])

        with open(output_path, 'wb') as output_file:
            for i in range(session['chunks']):
                chunk_path = os.path.join(session['temp_dir'], f'chunk_{i}')
                with open(chunk_path, 'rb') as chunk_file:
                    output_file.write(chunk_file.read())

        # Verify checksum
        calculated_checksum = calculate_checksum(output_path)

        if checksum and calculated_checksum != checksum:
            os.remove(output_path)
            logger.warning(f"Checksum mismatch for {session['filename']}")
            return jsonify({
                'success': False,
                'error': f'Checksum mismatch (expected: {checksum}, got: {calculated_checksum})'
            }), 400

        # Handle compression if needed
        compression_applied = False
        compressed_path = None

        if session['compression']:
            try:
                compressed_path = os.path.join(COMPRESSED_DIR, f"{session['filename']}.gz")
                with open(output_path, 'rb') as f_in:
                    with gzip.open(compressed_path, 'wb', compresslevel=COMPRESSION_LEVEL) as f_out:
                        shutil.copyfileobj(f_in, f_out)

                # Get compression stats
                original_size = os.path.getsize(output_path)
                compressed_size = os.path.getsize(compressed_path)
                compression_ratio = (1 - compressed_size / original_size) * 100

                logger.info(
                    f"Compressed {session['filename']}: {original_size} → {compressed_size} bytes "
                    f"({compression_ratio:.1f}% reduction)"
                )
                compression_applied = True

            except Exception as e:
                logger.warning(f"Compression failed for {session['filename']}: {e}")

        # Clean up temp files
        for i in range(session['chunks']):
            chunk_path = os.path.join(session['temp_dir'], f'chunk_{i}')
            if os.path.exists(chunk_path):
                os.remove(chunk_path)
        try:
            os.rmdir(session['temp_dir'])
        except OSError:
            pass

        # Save metadata
        metadata_path = output_path + '.metadata.json'
        with open(metadata_path, 'w') as f:
            json.dump({
                'filename': session['filename'],
                'filesize': session['filesize'],
                'checksum': calculated_checksum,
                'metadata': session['metadata'],
                'compression_enabled': session['compression'],
                'compression_applied': compression_applied,
                'compressed_path': compressed_path,
                'uploaded_at': datetime.now().isoformat()
            }, f, indent=2)

        logger.info(f"Upload completed: {session['filename']} → {output_path}")

        # Remove session
        del upload_sessions[upload_id]

        return jsonify({
            'success': True,
            'message': 'Upload completed',
            'file_path': output_path,
            'compressed_path': compressed_path if compression_applied else None,
            'checksum': calculated_checksum,
            'compression_applied': compression_applied
        }), 200

    except Exception as e:
        logger.error(f"Error finalizing upload: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500


@app.route('/upload/status/<upload_id>', methods=['GET'])
def upload_status(upload_id: str) -> Tuple[Any, int]:
    """Get upload status"""
    if upload_id not in upload_sessions:
        return jsonify({'success': False, 'error': 'Invalid upload ID'}), 404

    session = upload_sessions[upload_id]

    return jsonify({
        'success': True,
        'filename': session['filename'],
        'received_chunks': len(session['received_chunks']),
        'total_chunks': session['chunks'],
        'progress': len(session['received_chunks']) / session['chunks'] * 100,
        'compression_enabled': session['compression']
    }), 200


@app.route('/uploads', methods=['GET'])
@limiter.limit("30 per minute")
def list_uploads() -> Tuple[Any, int]:
    """List completed uploads"""
    uploads = []

    for filename in os.listdir(COMPLETED_DIR):
        if filename.endswith('.metadata.json'):
            continue

        filepath = os.path.join(COMPLETED_DIR, filename)
        metadata_path = filepath + '.metadata.json'

        upload_info = {
            'filename': filename,
            'size': os.path.getsize(filepath),
            'uploaded_at': datetime.fromtimestamp(os.path.getmtime(filepath)).isoformat()
        }

        if os.path.exists(metadata_path):
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)
                upload_info.update(metadata)

        uploads.append(upload_info)

    return jsonify({'success': True, 'uploads': uploads}), 200


def calculate_checksum(file_path: str) -> str:
    """Calculate MD5 checksum"""
    md5 = hashlib.md5()
    with open(file_path, 'rb') as f:
        for chunk in iter(lambda: f.read(8192), b''):
            md5.update(chunk)
    return md5.hexdigest()


def setup_ssl(cert_file: Optional[str] = None, key_file: Optional[str] = None) -> Optional[ssl.SSLContext]:
    """
    Setup SSL/TLS context for HTTPS
    
    Args:
        cert_file: Path to SSL certificate file
        key_file: Path to SSL private key file
        
    Returns:
        SSLContext if files exist, None otherwise
    """
    if not cert_file or not key_file:
        return None

    if not os.path.exists(cert_file) or not os.path.exists(key_file):
        logger.warning(f"SSL cert or key file not found: {cert_file}, {key_file}")
        return None

    try:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.load_cert_chain(cert_file, key_file)
        logger.info(f"SSL/TLS context loaded successfully from {cert_file}")
        return context
    except Exception as e:
        logger.error(f"Failed to setup SSL/TLS: {e}")
        return None


if __name__ == '__main__':
    print(f"""
╔═══════════════════════════════════════════════════════════╗
║     ROS2 Dashboard Upload Server v2.0 (Production)        ║
╠═══════════════════════════════════════════════════════════╣
║  Upload Directory: {UPLOAD_DIR:<37} ║
║  Features:                                                ║
║    ✓ Chunked uploads with resume                         ║
║    ✓ Rate limiting (200/day, 50/hour)                    ║
║    ✓ Compression support (gzip, >10MB)                   ║
║    ✓ SSL/TLS ready                                       ║
║    ✓ Comprehensive logging                               ║
║                                                           ║
║  To enable SSL/TLS, set:                                 ║
║    SSL_CERT=/path/to/cert.pem                            ║
║    SSL_KEY=/path/to/key.pem                              ║
║                                                           ║
║  HTTP:  python3 upload_server.py                         ║
║  HTTPS: SSL_CERT=cert.pem SSL_KEY=key.pem \\              ║
║         python3 upload_server.py                         ║
╚═══════════════════════════════════════════════════════════╝
    """)

    # Check for SSL configuration from environment
    ssl_cert = os.environ.get('SSL_CERT')
    ssl_key = os.environ.get('SSL_KEY')
    ssl_context = setup_ssl(ssl_cert, ssl_key)

    # Run server with optional SSL
    if ssl_context:
        logger.info("Starting server with SSL/TLS enabled (HTTPS)")
        app.run(host='0.0.0.0', port=8080, debug=False, ssl_context=ssl_context)
    else:
        logger.info("Starting server without SSL (HTTP)")
        app.run(host='0.0.0.0', port=8080, debug=False)
