"""
Guardian Mobility v0.0 - 3DGS Reconstruction API Server

Flask后端服务器，提供3DGS重建API

功能:
- ROS bag文件上传
- 重建任务管理
- 实时进度推送 (WebSocket)
- 结果查询和下载

@author Li Yixiang
@institution City University of Hong Kong
"""

from flask import Flask, request, jsonify, send_file
from flask_cors import CORS
from flask_socketio import SocketIO, emit, join_room
import os
import uuid
import json
import threading
from pathlib import Path
from datetime import datetime
from typing import Dict, Optional
import logging

# 导入重建模块
import sys
sys.path.append(os.path.dirname(__file__))
from rosbag_to_3dgs import (
    ReconstructionConfig,
    ReconstructionPipeline,
    ReconstructionResult
)

# 配置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

# 创建Flask应用
app = Flask(__name__)
app.config['SECRET_KEY'] = 'guardian-mobility-secret-key'
app.config['MAX_CONTENT_LENGTH'] = 10 * 1024 * 1024 * 1024  # 10GB
app.config['UPLOAD_FOLDER'] = '/tmp/guardian_mobility/uploads'
app.config['OUTPUT_FOLDER'] = '/tmp/guardian_mobility/outputs'

# 启用CORS
CORS(app, resources={r"/api/*": {"origins": "*"}})

# 启用WebSocket
socketio = SocketIO(app, cors_allowed_origins="*")

# 创建必要的目录
Path(app.config['UPLOAD_FOLDER']).mkdir(parents=True, exist_ok=True)
Path(app.config['OUTPUT_FOLDER']).mkdir(parents=True, exist_ok=True)

# 任务存储
jobs: Dict[str, Dict] = {}
job_threads: Dict[str, threading.Thread] = {}


class ReconstructionJob:
    """重建任务"""

    def __init__(self, job_id: str, config: ReconstructionConfig):
        self.job_id = job_id
        self.config = config
        self.status = 'pending'
        self.progress = 0
        self.phase = '准备中...'
        self.result: Optional[ReconstructionResult] = None
        self.error: Optional[str] = None
        self.start_time = datetime.now()
        self.end_time: Optional[datetime] = None

    def to_dict(self) -> Dict:
        return {
            'job_id': self.job_id,
            'status': self.status,
            'progress': self.progress,
            'phase': self.phase,
            'start_time': self.start_time.isoformat(),
            'end_time': self.end_time.isoformat() if self.end_time else None,
            'result': self.result.__dict__ if self.result else None,
            'error': self.error
        }


def run_reconstruction(job_id: str, config: ReconstructionConfig):
    """
    运行重建任务（在后台线程中）
    """
    job = jobs[job_id]

    try:
        logger.info(f"[Job {job_id}] 开始重建")
        job['status'] = 'running'

        # 创建重建流水线
        pipeline = ReconstructionPipeline(config)

        # 自定义进度回调
        def progress_callback(phase: str, percentage: int, message: str):
            job['phase'] = phase
            job['progress'] = percentage

            # 通过WebSocket推送进度
            socketio.emit('progress', {
                'job_id': job_id,
                'phase': phase,
                'percentage': percentage,
                'message': message
            }, room=job_id)

            # 推送日志
            socketio.emit('log', {
                'job_id': job_id,
                'level': 'info',
                'message': message
            }, room=job_id)

        # 运行重建
        result = pipeline.run()

        # 更新任务状态
        job['status'] = 'completed' if result.success else 'failed'
        job['result'] = result
        job['end_time'] = datetime.now()

        if result.success:
            logger.info(f"[Job {job_id}] 重建成功")
            socketio.emit('completed', {
                'job_id': job_id,
                'result': result.__dict__
            }, room=job_id)
        else:
            logger.error(f"[Job {job_id}] 重建失败: {result.error}")
            job['error'] = result.error
            socketio.emit('failed', {
                'job_id': job_id,
                'error': result.error
            }, room=job_id)

    except Exception as e:
        logger.error(f"[Job {job_id}] 异常: {e}")
        job['status'] = 'failed'
        job['error'] = str(e)
        job['end_time'] = datetime.now()

        socketio.emit('failed', {
            'job_id': job_id,
            'error': str(e)
        }, room=job_id)


# ==================== REST API ====================

@app.route('/api/reconstruction/start', methods=['POST'])
def start_reconstruction():
    """开始重建"""
    try:
        data = request.json

        # 生成任务ID
        job_id = str(uuid.uuid4())

        # 创建输出目录
        output_dir = os.path.join(app.config['OUTPUT_FOLDER'], job_id)
        os.makedirs(output_dir, exist_ok=True)

        # 创建配置
        config = ReconstructionConfig(
            rosbag_path=data['rosbagPath'],
            output_dir=output_dir,
            image_topic=data.get('imageTopic', '/camera/image_raw'),
            camera_info_topic=data.get('cameraInfoTopic', '/camera/camera_info'),
            max_images=data.get('maxImages'),
            colmap_quality=data.get('colmapQuality', 'high'),
            colmap_matcher=data.get('colmapMatcher', 'exhaustive'),
            iterations=data.get('iterations', 30000),
            resolution=data.get('resolution', 1),
            sh_degree=data.get('shDegree', 3),
            use_gpu=data.get('useGpu', True),
            gpu_id=data.get('gpuId', 0),
            downsample_factor=data.get('downsampleFactor', 1.0)
        )

        # 创建任务
        job = {
            'job_id': job_id,
            'config': config,
            'status': 'pending',
            'progress': 0,
            'phase': '准备中...',
            'start_time': datetime.now(),
            'end_time': None,
            'result': None,
            'error': None
        }
        jobs[job_id] = job

        # 启动后台线程
        thread = threading.Thread(
            target=run_reconstruction,
            args=(job_id, config)
        )
        thread.daemon = True
        thread.start()
        job_threads[job_id] = thread

        logger.info(f"[Job {job_id}] 任务已创建")

        return jsonify({
            'success': True,
            'jobId': job_id,
            'message': '重建任务已启动'
        })

    except Exception as e:
        logger.error(f"启动重建失败: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/reconstruction/status/<job_id>', methods=['GET'])
def get_status(job_id: str):
    """获取任务状态"""
    if job_id not in jobs:
        return jsonify({'error': 'Job not found'}), 404

    job = jobs[job_id]
    return jsonify({
        'jobId': job_id,
        'status': job['status'],
        'progress': job['progress'],
        'phase': job['phase']
    })


@app.route('/api/reconstruction/result/<job_id>', methods=['GET'])
def get_result(job_id: str):
    """获取重建结果"""
    if job_id not in jobs:
        return jsonify({'error': 'Job not found'}), 404

    job = jobs[job_id]

    if job['status'] != 'completed':
        return jsonify({'error': 'Job not completed'}), 400

    result = job['result']
    return jsonify({
        'success': result.success,
        'message': result.message,
        'outputPath': result.output_path,
        'colmapPath': result.colmap_path,
        'gaussianPath': result.gaussian_path,
        'viewerUrl': result.viewer_url,
        'stats': result.stats
    })


@app.route('/api/reconstruction/cancel/<job_id>', methods=['POST'])
def cancel_reconstruction(job_id: str):
    """取消重建"""
    if job_id not in jobs:
        return jsonify({'error': 'Job not found'}), 404

    job = jobs[job_id]
    job['status'] = 'cancelled'

    logger.info(f"[Job {job_id}] 任务已取消")

    return jsonify({
        'success': True,
        'message': '任务已取消'
    })


@app.route('/api/reconstruction/history', methods=['GET'])
def get_history():
    """获取历史记录"""
    history = []
    for job_id, job in jobs.items():
        history.append({
            'jobId': job_id,
            'status': job['status'],
            'startTime': job['start_time'].isoformat(),
            'endTime': job['end_time'].isoformat() if job['end_time'] else None
        })

    return jsonify(history)


@app.route('/api/reconstruction/delete/<job_id>', methods=['DELETE'])
def delete_result(job_id: str):
    """删除重建结果"""
    if job_id not in jobs:
        return jsonify({'error': 'Job not found'}), 404

    # 删除输出目录
    output_dir = os.path.join(app.config['OUTPUT_FOLDER'], job_id)
    if os.path.exists(output_dir):
        import shutil
        shutil.rmtree(output_dir)

    # 删除任务记录
    del jobs[job_id]
    if job_id in job_threads:
        del job_threads[job_id]

    logger.info(f"[Job {job_id}] 任务已删除")

    return jsonify({
        'success': True,
        'message': '任务已删除'
    })


@app.route('/api/reconstruction/upload', methods=['POST'])
def upload_rosbag():
    """上传ROS bag文件"""
    try:
        if 'file' not in request.files:
            return jsonify({'error': 'No file provided'}), 400

        file = request.files['file']
        if file.filename == '':
            return jsonify({'error': 'No file selected'}), 400

        # 保存文件
        filename = f"{uuid.uuid4()}_{file.filename}"
        filepath = os.path.join(app.config['UPLOAD_FOLDER'], filename)
        file.save(filepath)

        logger.info(f"文件已上传: {filepath}")

        return jsonify({
            'success': True,
            'path': filepath,
            'filename': filename
        })

    except Exception as e:
        logger.error(f"上传失败: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500


@app.route('/api/reconstruction/check-dependencies', methods=['GET'])
def check_dependencies():
    """检查系统依赖"""
    import subprocess

    def check_command(cmd: str) -> bool:
        try:
            subprocess.run([cmd, '--version'], capture_output=True, timeout=5)
            return True
        except:
            return False

    dependencies = {
        'ros': check_command('rosbag'),
        'colmap': check_command('colmap'),
        'gaussian_splatting': os.path.exists('/usr/local/bin/gaussian-splatting'),
        'gpu': check_command('nvidia-smi')
    }

    return jsonify(dependencies)


# ==================== WebSocket ====================

@socketio.on('connect')
def handle_connect():
    """客户端连接"""
    logger.info(f"客户端已连接: {request.sid}")


@socketio.on('disconnect')
def handle_disconnect():
    """客户端断开"""
    logger.info(f"客户端已断开: {request.sid}")


@socketio.on('join')
def handle_join(data):
    """加入任务房间"""
    job_id = data.get('jobId')
    if job_id:
        join_room(job_id)
        logger.info(f"客户端 {request.sid} 加入房间 {job_id}")


# ==================== 主程序 ====================

if __name__ == '__main__':
    logger.info("=" * 60)
    logger.info("Guardian Mobility - 3DGS Reconstruction API Server")
    logger.info("=" * 60)
    logger.info(f"Upload folder: {app.config['UPLOAD_FOLDER']}")
    logger.info(f"Output folder: {app.config['OUTPUT_FOLDER']}")
    logger.info("Server starting on http://localhost:5000")
    logger.info("=" * 60)

    socketio.run(app, host='0.0.0.0', port=5000, debug=True)
