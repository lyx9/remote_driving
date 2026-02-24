#!/usr/bin/env python3
"""
RosBag Streaming WebSocket Server
Guardian Mobility v0.0
Provides real-time RosBag playback via WebSocket
"""

import asyncio
import json
import os
import base64
from pathlib import Path
from typing import Dict, List, Optional, Set
import logging

try:
    import rosbag
    import rospy
    from sensor_msgs.msg import Image, CompressedImage, PointCloud2
    from std_msgs.msg import String
    from cv_bridge import CvBridge
    import cv2
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("Warning: ROS packages not available. Running in demo mode.")

try:
    from rosbags.rosbag2 import Reader as ROS2Reader
    from rosbags.serde import deserialize_cdr
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("Warning: ROS2 rosbags package not available. Install with: pip install rosbags")

try:
    import websockets
    from websockets.asyncio.server import serve
except ImportError:
    print("Error: websockets package not installed. Run: pip install websockets")
    exit(1)

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class RosBagStreamServer:
    """WebSocket server for streaming RosBag data"""

    def __init__(self, host: str = 'localhost', port: int = 8765, rosbag_dir: str = '/home/lyx/fsm/rosbag'):
        self.host = host
        self.port = port
        self.rosbag_dir = Path(rosbag_dir)
        self.clients: Set[websockets.WebSocketServerProtocol] = set()
        self.current_bag: Optional[str] = None
        self.current_bag_info: Optional[Dict] = None
        self.is_streaming = False
        self.stream_task: Optional[asyncio.Task] = None
        self.selected_topics: List[str] = []
        self.playback_rate = 1.0

        if HAS_ROS:
            self.bridge = CvBridge()

        logger.info(f"RosBag directory: {self.rosbag_dir}")

    async def register_client(self, websocket):
        """Register a new client connection"""
        self.clients.add(websocket)
        logger.info(f"Client connected. Total clients: {len(self.clients)}")

        # Send initial state
        await self.send_to_client(websocket, {
            'type': 'connection_state',
            'state': 'connected'
        })

    async def unregister_client(self, websocket):
        """Unregister a client connection"""
        self.clients.discard(websocket)
        logger.info(f"Client disconnected. Total clients: {len(self.clients)}")

    async def send_to_client(self, websocket, message: Dict):
        """Send message to a specific client"""
        try:
            await websocket.send(json.dumps(message))
        except Exception as e:
            logger.error(f"Error sending to client: {e}")

    async def broadcast(self, message: Dict):
        """Broadcast message to all connected clients"""
        if self.clients:
            await asyncio.gather(
                *[self.send_to_client(client, message) for client in self.clients],
                return_exceptions=True
            )

    def list_rosbag_files(self) -> List[Dict]:
        """List all RosBag files in the directory"""
        bags = []

        if not self.rosbag_dir.exists():
            logger.warning(f"RosBag directory does not exist: {self.rosbag_dir}")
            return bags

        # Find ROS1 .bag files
        for bag_file in self.rosbag_dir.rglob('*.bag'):
            try:
                stat = bag_file.stat()
                bag_info = {
                    'path': str(bag_file),
                    'name': bag_file.name,
                    'size': stat.st_size,
                    'modified': stat.st_mtime,
                    'format': 'ROS1'
                }

                # Try to get bag info if ROS is available
                if HAS_ROS:
                    try:
                        with rosbag.Bag(str(bag_file), 'r') as bag:
                            bag_info['duration'] = bag.get_end_time() - bag.get_start_time()
                            bag_info['message_count'] = bag.get_message_count()
                            bag_info['topics'] = []

                            for topic, msg_type, _ in bag.get_type_and_topic_info()[1].items():
                                bag_info['topics'].append({
                                    'name': topic,
                                    'type': msg_type.msg_type,
                                    'count': msg_type.message_count
                                })
                    except Exception as e:
                        logger.warning(f"Could not read bag info for {bag_file.name}: {e}")

                bags.append(bag_info)
            except Exception as e:
                logger.error(f"Error processing {bag_file}: {e}")

        # Find ROS2 bags (directories with metadata.yaml)
        for metadata_file in self.rosbag_dir.rglob('metadata.yaml'):
            try:
                bag_dir = metadata_file.parent

                # Check if there are .db3 files
                db3_files = list(bag_dir.glob('*.db3'))
                if not db3_files:
                    continue

                # Calculate total size
                total_size = sum(f.stat().st_size for f in db3_files)

                bag_info = {
                    'path': str(bag_dir),
                    'name': bag_dir.name,
                    'size': total_size,
                    'modified': metadata_file.stat().st_mtime,
                    'format': 'ROS2',
                    'db3_files': len(db3_files)
                }

                bags.append(bag_info)
            except Exception as e:
                logger.error(f"Error processing ROS2 bag {metadata_file.parent}: {e}")

        logger.info(f"Found {len(bags)} RosBag files")
        return bags

    def get_bag_info(self, bag_path: str) -> Optional[Dict]:
        """Get detailed information about a RosBag file"""
        bag_path_obj = Path(bag_path)

        # Check if it's a ROS2 bag (directory with metadata.yaml)
        if bag_path_obj.is_dir() and (bag_path_obj / 'metadata.yaml').exists():
            return self.get_ros2_bag_info(bag_path)
        # Check if it's a ROS1 bag (single .bag file)
        elif bag_path_obj.is_file() and bag_path_obj.suffix == '.bag':
            return self.get_ros1_bag_info(bag_path)
        else:
            logger.error(f"Unknown bag format: {bag_path}")
            return None

    def get_ros1_bag_info(self, bag_path: str) -> Optional[Dict]:
        """Get detailed information about a ROS1 bag file"""
        if not HAS_ROS:
            return None

        try:
            with rosbag.Bag(bag_path, 'r') as bag:
                info = bag.get_type_and_topic_info()

                topics = []
                for topic, topic_info in info[1].items():
                    topics.append({
                        'name': topic,
                        'type': topic_info.msg_type,
                        'message_count': topic_info.message_count,
                        'frequency': topic_info.frequency if hasattr(topic_info, 'frequency') else 0
                    })

                return {
                    'path': bag_path,
                    'name': Path(bag_path).stem,
                    'format': 'ROS1',
                    'duration': bag.get_end_time() - bag.get_start_time(),
                    'start_time': bag.get_start_time(),
                    'end_time': bag.get_end_time(),
                    'message_count': bag.get_message_count(),
                    'topics': topics
                }
        except Exception as e:
            logger.error(f"Error getting ROS1 bag info: {e}")
            return None

    def get_ros2_bag_info(self, bag_path: str) -> Optional[Dict]:
        """Get detailed information about a ROS2 bag directory"""
        if not HAS_ROS2:
            logger.warning("ROS2 rosbags package not available")
            # Return basic info from metadata.yaml
            return self.get_ros2_bag_info_from_metadata(bag_path)

        try:
            with ROS2Reader(bag_path) as reader:
                # Get topics
                topics = []
                topic_msg_counts = {}

                # Count messages per topic
                for connection in reader.connections:
                    topic_msg_counts[connection.topic] = 0

                for connection, timestamp, rawdata in reader.messages():
                    topic_msg_counts[connection.topic] += 1

                # Build topic list
                for connection in reader.connections:
                    topics.append({
                        'name': connection.topic,
                        'type': connection.msgtype,
                        'message_count': topic_msg_counts.get(connection.topic, 0),
                        'frequency': 0  # Would need to calculate from timestamps
                    })

                total_messages = sum(topic_msg_counts.values())

                return {
                    'path': bag_path,
                    'name': Path(bag_path).name,
                    'format': 'ROS2',
                    'duration': (reader.end_time - reader.start_time) / 1e9,  # Convert ns to seconds
                    'start_time': reader.start_time / 1e9,
                    'end_time': reader.end_time / 1e9,
                    'message_count': total_messages,
                    'topics': topics
                }
        except Exception as e:
            logger.error(f"Error getting ROS2 bag info: {e}")
            # Fallback to metadata
            return self.get_ros2_bag_info_from_metadata(bag_path)

    def get_ros2_bag_info_from_metadata(self, bag_path: str) -> Optional[Dict]:
        """Get basic ROS2 bag info from metadata.yaml"""
        try:
            import yaml
            metadata_path = Path(bag_path) / 'metadata.yaml'

            if not metadata_path.exists():
                return None

            with open(metadata_path, 'r') as f:
                metadata = yaml.safe_load(f)

            # Get basic info from metadata
            topics = []
            if 'rosbag2_bagfile_information' in metadata:
                bag_info = metadata['rosbag2_bagfile_information']

                # Extract topics
                for topic_info in bag_info.get('topics_with_message_count', []):
                    topics.append({
                        'name': topic_info.get('topic_metadata', {}).get('name', 'unknown'),
                        'type': topic_info.get('topic_metadata', {}).get('type', 'unknown'),
                        'message_count': topic_info.get('message_count', 0),
                        'frequency': 0
                    })

                return {
                    'path': bag_path,
                    'name': Path(bag_path).name,
                    'format': 'ROS2',
                    'duration': bag_info.get('duration', {}).get('nanoseconds', 0) / 1e9,
                    'start_time': 0,
                    'end_time': 0,
                    'message_count': bag_info.get('message_count', 0),
                    'topics': topics
                }

            return None
        except Exception as e:
            logger.error(f"Error reading ROS2 metadata: {e}")
            return None

    async def open_bag(self, bag_path: str):
        """Open a RosBag file"""
        try:
            # Stop current streaming if any
            if self.is_streaming:
                await self.stop_streaming()

            self.current_bag = bag_path
            self.current_bag_info = self.get_bag_info(bag_path)

            if self.current_bag_info:
                await self.broadcast({
                    'type': 'bag_opened',
                    'bag': self.current_bag_info
                })
                logger.info(f"Opened bag: {bag_path}")
            else:
                await self.broadcast({
                    'type': 'error',
                    'message': 'Failed to open bag file'
                })
        except Exception as e:
            logger.error(f"Error opening bag: {e}")
            await self.broadcast({
                'type': 'error',
                'message': str(e)
            })

    async def start_streaming(self, topics: List[str]):
        """Start streaming selected topics"""
        if not self.current_bag or not HAS_ROS:
            await self.broadcast({
                'type': 'error',
                'message': 'No bag file opened or ROS not available'
            })
            return

        if self.is_streaming:
            logger.warning("Already streaming")
            return

        self.selected_topics = topics
        self.is_streaming = True

        await self.broadcast({
            'type': 'stream_state',
            'state': 'streaming'
        })

        # Start streaming task
        self.stream_task = asyncio.create_task(self._stream_bag())
        logger.info(f"Started streaming topics: {topics}")

    async def stop_streaming(self):
        """Stop streaming"""
        self.is_streaming = False

        if self.stream_task:
            self.stream_task.cancel()
            try:
                await self.stream_task
            except asyncio.CancelledError:
                pass
            self.stream_task = None

        await self.broadcast({
            'type': 'stream_state',
            'state': 'stopped'
        })
        logger.info("Stopped streaming")

    async def _stream_bag(self):
        """Stream bag messages"""
        try:
            with rosbag.Bag(self.current_bag, 'r') as bag:
                start_time = None
                message_count = 0

                for topic, msg, t in bag.read_messages(topics=self.selected_topics):
                    if not self.is_streaming:
                        break

                    # Initialize start time
                    if start_time is None:
                        start_time = t.to_sec()

                    # Calculate delay for playback rate
                    current_time = t.to_sec()
                    delay = (current_time - start_time) / self.playback_rate

                    if delay > 0:
                        await asyncio.sleep(delay)
                        start_time = current_time

                    # Process and send message
                    await self._process_message(topic, msg, t)
                    message_count += 1

                    # Send progress update every 10 messages
                    if message_count % 10 == 0:
                        await self.broadcast({
                            'type': 'stream_progress',
                            'message_count': message_count
                        })

                # Streaming completed
                self.is_streaming = False
                await self.broadcast({
                    'type': 'stream_state',
                    'state': 'completed',
                    'total_messages': message_count
                })
                logger.info(f"Streaming completed. Total messages: {message_count}")

        except Exception as e:
            logger.error(f"Error during streaming: {e}")
            self.is_streaming = False
            await self.broadcast({
                'type': 'error',
                'message': f'Streaming error: {str(e)}'
            })

    async def _process_message(self, topic: str, msg, timestamp):
        """Process and broadcast a ROS message"""
        try:
            message_data = {
                'type': 'message',
                'topic': topic,
                'timestamp': timestamp.to_nsec(),
                'message_type': type(msg).__name__
            }

            # Handle different message types
            if isinstance(msg, Image):
                # Convert ROS Image to base64 JPEG
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                    _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    image_base64 = base64.b64encode(buffer).decode('utf-8')
                    message_data['data'] = {
                        'image': f'data:image/jpeg;base64,{image_base64}',
                        'width': msg.width,
                        'height': msg.height
                    }
                except Exception as e:
                    logger.error(f"Error converting image: {e}")
                    return

            elif isinstance(msg, CompressedImage):
                # Compressed image - decode and re-encode
                try:
                    np_arr = np.frombuffer(msg.data, np.uint8)
                    cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                    _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                    image_base64 = base64.b64encode(buffer).decode('utf-8')
                    message_data['data'] = {
                        'image': f'data:image/jpeg;base64,{image_base64}'
                    }
                except Exception as e:
                    logger.error(f"Error processing compressed image: {e}")
                    return

            elif isinstance(msg, PointCloud2):
                # Point cloud - send metadata only (full point cloud is too large)
                message_data['data'] = {
                    'width': msg.width,
                    'height': msg.height,
                    'point_step': msg.point_step,
                    'row_step': msg.row_step,
                    'is_dense': msg.is_dense
                }
            else:
                # Generic message - try to serialize
                try:
                    message_data['data'] = self._serialize_message(msg)
                except Exception as e:
                    logger.debug(f"Could not serialize message type {type(msg).__name__}: {e}")
                    message_data['data'] = {'raw': str(msg)}

            await self.broadcast(message_data)

        except Exception as e:
            logger.error(f"Error processing message: {e}")

    def _serialize_message(self, msg) -> Dict:
        """Serialize a ROS message to dict"""
        result = {}
        for slot in msg.__slots__:
            try:
                value = getattr(msg, slot)
                if hasattr(value, '__slots__'):
                    result[slot] = self._serialize_message(value)
                elif isinstance(value, (list, tuple)):
                    result[slot] = [self._serialize_message(v) if hasattr(v, '__slots__') else v for v in value]
                else:
                    result[slot] = value
            except Exception as e:
                logger.debug(f"Could not serialize slot {slot}: {e}")
        return result

    async def handle_client(self, websocket):
        """Handle client connection and messages"""
        await self.register_client(websocket)

        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    await self.handle_message(websocket, data)
                except json.JSONDecodeError:
                    logger.error(f"Invalid JSON: {message}")
                except Exception as e:
                    logger.error(f"Error handling message: {e}")
        except websockets.exceptions.ConnectionClosed:
            logger.info("Client connection closed")
        finally:
            await self.unregister_client(websocket)

    async def handle_message(self, websocket, data: Dict):
        """Handle incoming client messages"""
        # Support both 'type' and 'command' fields for compatibility
        msg_type = data.get('type') or data.get('command')

        if msg_type == 'list_bags':
            bags = self.list_rosbag_files()
            await self.send_to_client(websocket, {
                'type': 'bag_list',
                'bags': bags
            })

        elif msg_type == 'open_bag':
            bag_path = data.get('path') or data.get('bagPath')
            if bag_path:
                await self.open_bag(bag_path)

        elif msg_type == 'start_stream' or msg_type == 'stream_topics':
            topics = data.get('topics', [])
            self.playback_rate = data.get('playback_rate', 1.0)
            await self.start_streaming(topics)

        elif msg_type == 'stop_stream':
            await self.stop_streaming()

        elif msg_type == 'set_playback_rate':
            self.playback_rate = data.get('rate', 1.0)
            await self.broadcast({
                'type': 'playback_rate_changed',
                'rate': self.playback_rate
            })

        else:
            logger.warning(f"Unknown message type: {msg_type}")

    async def start(self):
        """Start the WebSocket server"""
        logger.info(f"Starting RosBag Stream Server on {self.host}:{self.port}")

        if not HAS_ROS:
            logger.warning("ROS packages not available - running in limited mode")

        async with serve(self.handle_client, self.host, self.port):
            logger.info(f"Server running on ws://{self.host}:{self.port}")
            await asyncio.Future()  # Run forever


async def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description='RosBag Streaming WebSocket Server')
    parser.add_argument('--host', default='localhost', help='Server host')
    parser.add_argument('--port', type=int, default=8765, help='Server port')
    parser.add_argument('--rosbag-dir', default='/home/lyx/fsm/rosbag', help='RosBag directory')

    args = parser.parse_args()

    server = RosBagStreamServer(
        host=args.host,
        port=args.port,
        rosbag_dir=args.rosbag_dir
    )

    try:
        await server.start()
    except KeyboardInterrupt:
        logger.info("Server stopped by user")


if __name__ == '__main__':
    asyncio.run(main())
