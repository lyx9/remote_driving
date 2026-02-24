# RosBag Stream Server

WebSocket server for streaming RosBag data to the Guardian Mobility frontend.

## Features

- ✅ WebSocket-based real-time streaming
- ✅ Support for ROS1 (.bag) and ROS2 (.db3) formats
- ✅ Automatic bag file discovery
- ✅ Real-time progress updates
- ✅ Multiple client support
- ✅ Graceful error handling

## Quick Start

### 1. Install Dependencies

```bash
pip install websockets
```

### 2. Start the Server

```bash
# Using the startup script (recommended)
./rosbag_server.sh start

# Or directly
python3 rosbag_stream_server.py --rosbag-dir /home/lyx/fsm/rosbag
```

### 3. Check Status

```bash
./rosbag_server.sh status
```

### 4. View Logs

```bash
./rosbag_server.sh logs
```

### 5. Stop the Server

```bash
./rosbag_server.sh stop
```

## Server Management

The `rosbag_server.sh` script provides easy server management:

```bash
./rosbag_server.sh {start|stop|restart|status|logs}
```

**Commands:**
- `start` - Start the server in background
- `stop` - Stop the server
- `restart` - Restart the server
- `status` - Check if server is running
- `logs` - View server logs (tail -f)

## Configuration

### Server Settings

- **Host**: localhost (127.0.0.1)
- **Port**: 8765
- **Protocol**: WebSocket (ws://)
- **RosBag Directory**: /home/lyx/fsm/rosbag

### Command Line Options

```bash
python3 rosbag_stream_server.py --help

Options:
  --host HOST           Server host (default: localhost)
  --port PORT           Server port (default: 8765)
  --rosbag-dir DIR      RosBag directory (default: /home/lyx/fsm/rosbag)
```

## WebSocket API

### Client → Server Messages

#### List Available Bags
```json
{
  "type": "list_bags"
}
```

#### Open a Bag
```json
{
  "type": "open_bag",
  "path": "/path/to/bag"
}
```

#### Start Streaming
```json
{
  "type": "start_stream",
  "topics": ["/camera/image", "/lidar/points"],
  "playback_rate": 1.0
}
```

#### Stop Streaming
```json
{
  "type": "stop_stream"
}
```

### Server → Client Messages

#### Connection State
```json
{
  "type": "connection_state",
  "state": "connected"
}
```

#### Bag List
```json
{
  "type": "bag_list",
  "bags": [
    {
      "name": "rosbag2_2025_02_23-16_49_58",
      "path": "/home/lyx/fsm/rosbag/rosbag2_2025_02_23-16_49_58",
      "size": 821329920,
      "format": "ROS2",
      "db3_files": 1
    }
  ]
}
```

#### Bag Opened
```json
{
  "type": "bag_opened",
  "bag": {
    "path": "/path/to/bag",
    "duration": 120.5,
    "topics": [...]
  }
}
```

#### Stream State
```json
{
  "type": "stream_state",
  "state": "streaming"
}
```

#### Message Data
```json
{
  "type": "message",
  "topic": "/camera/image",
  "timestamp": 1234567890,
  "message_type": "Image",
  "data": {...}
}
```

#### Error
```json
{
  "type": "error",
  "message": "Error description"
}
```

## Supported Formats

### ROS1 Bags
- File extension: `.bag`
- Requires: `rosbag`, `cv_bridge` (ROS Noetic)

### ROS2 Bags
- File structure: Directory with `metadata.yaml` and `.db3` files
- Currently: Metadata only (full playback requires ROS2 libraries)

## Testing

### Test Connection

```bash
# Using the test page
http://localhost:3000/rosbag-test.html

# Using Python
python3 -c "
import asyncio
import websockets
import json

async def test():
    async with websockets.connect('ws://localhost:8765') as ws:
        await ws.recv()  # connection_state
        await ws.send(json.dumps({'type': 'list_bags'}))
        response = await ws.recv()
        print(json.loads(response))

asyncio.run(test())
"
```

## Troubleshooting

### Server won't start

**Check if port is in use:**
```bash
lsof -i :8765
```

**Kill existing process:**
```bash
pkill -f rosbag_stream_server
```

### No bags found

**Check rosbag directory:**
```bash
ls -la /home/lyx/fsm/rosbag
```

**Verify bag format:**
- ROS1: Look for `.bag` files
- ROS2: Look for directories with `metadata.yaml` and `.db3` files

### Connection refused

**Ensure server is running:**
```bash
./rosbag_server.sh status
```

**Check logs:**
```bash
./rosbag_server.sh logs
```

## Architecture

```
Frontend (Vue/TypeScript)
    ↓ WebSocket (ws://localhost:8765)
RosBag Stream Server (Python)
    ↓
RosBag Files (ROS1/ROS2)
```

## Files

- `rosbag_stream_server.py` - Main server implementation
- `rosbag_server.sh` - Startup/management script
- `requirements.txt` - Python dependencies
- `/tmp/rosbag_server.log` - Server logs
- `/tmp/rosbag_server.pid` - Process ID file

## Integration

The server integrates with the Guardian Mobility frontend:

1. Frontend component: `src/components/RosBagReplayPro.vue`
2. Service layer: `src/services/rosbagStreamService.ts`
3. Test page: `public/rosbag-test.html`

## Development

### Running in Development Mode

```bash
# With debug logging
python3 rosbag_stream_server.py --rosbag-dir /home/lyx/fsm/rosbag
```

### Adding New Message Types

Edit `_process_message()` in `rosbag_stream_server.py` to handle additional ROS message types.

## Production Deployment

For production use:

1. Use a process manager (systemd, supervisor)
2. Configure firewall rules
3. Add authentication/authorization
4. Enable SSL/TLS (wss://)
5. Set up monitoring and alerting

## License

Part of Guardian Mobility v0.0
Copyright © 2026 City University of Hong Kong

## Author

Li Yixiang
City University of Hong Kong

## Version

1.0.0 (2026-02-04)
