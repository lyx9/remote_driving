/**
 * FSM-Pilot V2.0 - RosBag Streaming Server
 *
 * @project     FSM-Pilot Remote Driving Platform
 * @author      Li Yixiang
 * @institution City University of Hong Kong
 * @copyright   2025 City University of Hong Kong. All rights reserved.
 * @license     Proprietary
 *
 * @module      RosBag Streaming Server
 * @description WebSocket server for streaming large RosBag files (>15GB)
 *              Supports Autoware.universe ROS2 topics with image decoding
 */

const express = require('express');
const http = require('http');
const WebSocket = require('ws');
const sqlite3 = require('sqlite3').verbose();
const fs = require('fs');
const path = require('path');
const yaml = require('js-yaml');
const { decodeMessage } = require('./cdr-decoder');

// ======================== Configuration ========================

const PORT = 8765;
const ROSBAG_BASE_PATH = '/home/lyx/fsm/rosbag/1210';

// ======================== Express App Setup ========================

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

// Enable CORS
app.use((req, res, next) => {
  res.header('Access-Control-Allow-Origin', '*');
  res.header('Access-Control-Allow-Headers', 'Origin, X-Requested-With, Content-Type, Accept');
  next();
});

// ======================== RosBag Manager ========================

class RosBagManager {
  constructor(bagPath) {
    this.bagPath = bagPath;
    this.db = null;
    this.metadata = null;
    this.topics = [];
  }

  async initialize() {
    console.log(`[RosBagManager] Initializing bag: ${this.bagPath}`);

    // Load metadata
    const metadataPath = path.join(this.bagPath, 'metadata.yaml');
    if (!fs.existsSync(metadataPath)) {
      throw new Error(`Metadata not found: ${metadataPath}`);
    }

    const metadataContent = fs.readFileSync(metadataPath, 'utf8');
    this.metadata = yaml.load(metadataContent);

    console.log(`[RosBagManager] Loaded metadata:`, {
      duration: this.metadata.rosbag2_bagfile_information.duration.nanoseconds / 1e9,
      messageCount: this.metadata.rosbag2_bagfile_information.message_count,
      topicCount: this.metadata.rosbag2_bagfile_information.topics_with_message_count.length
    });

    // Extract topics
    this.topics = this.metadata.rosbag2_bagfile_information.topics_with_message_count.map(t => ({
      name: t.topic_metadata.name,
      type: t.topic_metadata.type,
      messageCount: t.message_count
    }));

    // Open database connection
    const dbFile = this.metadata.rosbag2_bagfile_information.relative_file_paths[0];
    const dbPath = path.join(this.bagPath, dbFile);

    if (!fs.existsSync(dbPath)) {
      throw new Error(`Database file not found: ${dbPath}`);
    }

    console.log(`[RosBagManager] Opening database: ${dbPath}`);
    console.log(`[RosBagManager] File size: ${(fs.statSync(dbPath).size / 1024 / 1024 / 1024).toFixed(2)} GB`);

    this.db = new sqlite3.Database(dbPath, sqlite3.OPEN_READONLY, (err) => {
      if (err) {
        console.error('[RosBagManager] Database open error:', err);
        throw err;
      }
      console.log('[RosBagManager] Database opened successfully');
    });

    return this;
  }

  getTopics() {
    return this.topics;
  }

  getMetadata() {
    return this.metadata;
  }

  async getMessageCount(topicName = null) {
    return new Promise((resolve, reject) => {
      let query = 'SELECT COUNT(*) as count FROM messages';
      const params = [];

      if (topicName) {
        query += ' WHERE topic_id = (SELECT id FROM topics WHERE name = ?)';
        params.push(topicName);
      }

      this.db.get(query, params, (err, row) => {
        if (err) reject(err);
        else resolve(row.count);
      });
    });
  }

  async streamMessages(topicNames, startTime, endTime, callback) {
    console.log(`[RosBagManager] Streaming messages for topics:`, topicNames);

    // Build query
    let query = `
      SELECT m.id, m.topic_id, m.timestamp, m.data, t.name as topic_name, t.type as topic_type
      FROM messages m
      JOIN topics t ON m.topic_id = t.id
      WHERE 1=1
    `;

    const params = [];

    if (topicNames && topicNames.length > 0) {
      const placeholders = topicNames.map(() => '?').join(',');
      query += ` AND t.name IN (${placeholders})`;
      params.push(...topicNames);
    }

    if (startTime !== null && startTime !== undefined) {
      query += ' AND m.timestamp >= ?';
      params.push(startTime);
    }

    if (endTime !== null && endTime !== undefined) {
      query += ' AND m.timestamp <= ?';
      params.push(endTime);
    }

    query += ' ORDER BY m.timestamp ASC';

    console.log(`[RosBagManager] Query:`, query);
    console.log(`[RosBagManager] Params:`, params);

    return new Promise((resolve, reject) => {
      let count = 0;

      this.db.each(
        query,
        params,
        (err, row) => {
          if (err) {
            console.error('[RosBagManager] Query error:', err);
            reject(err);
            return;
          }

          count++;
          callback(row);

          if (count % 1000 === 0) {
            console.log(`[RosBagManager] Streamed ${count} messages...`);
          }
        },
        (err, rowCount) => {
          if (err) {
            console.error('[RosBagManager] Query completion error:', err);
            reject(err);
          } else {
            console.log(`[RosBagManager] Streaming complete: ${rowCount} messages`);
            resolve(rowCount);
          }
        }
      );
    });
  }

  close() {
    if (this.db) {
      this.db.close((err) => {
        if (err) {
          console.error('[RosBagManager] Error closing database:', err);
        } else {
          console.log('[RosBagManager] Database closed');
        }
      });
    }
  }
}

// ======================== WebSocket Handler ========================

const activeBags = new Map();

wss.on('connection', (ws) => {
  console.log('[WebSocket] Client connected');

  let currentBag = null;
  let isStreaming = false;

  ws.on('message', async (message) => {
    try {
      const data = JSON.parse(message);
      console.log('[WebSocket] Received command:', data.command);

      switch (data.command) {
        case 'list_bags':
          await handleListBags(ws);
          break;

        case 'open_bag':
          currentBag = await handleOpenBag(ws, data.bagPath);
          break;

        case 'get_topics':
          await handleGetTopics(ws, currentBag);
          break;

        case 'stream_topics':
          isStreaming = true;
          await handleStreamTopics(ws, currentBag, data);
          break;

        case 'stop_stream':
          isStreaming = false;
          ws.send(JSON.stringify({ type: 'stream_stopped' }));
          break;

        case 'close_bag':
          if (currentBag) {
            currentBag.close();
            currentBag = null;
          }
          ws.send(JSON.stringify({ type: 'bag_closed' }));
          break;

        default:
          ws.send(JSON.stringify({
            type: 'error',
            message: `Unknown command: ${data.command}`
          }));
      }
    } catch (error) {
      console.error('[WebSocket] Error handling message:', error);
      ws.send(JSON.stringify({
        type: 'error',
        message: error.message
      }));
    }
  });

  ws.on('close', () => {
    console.log('[WebSocket] Client disconnected');
    if (currentBag) {
      currentBag.close();
    }
  });

  ws.on('error', (error) => {
    console.error('[WebSocket] Error:', error);
  });
});

// ======================== Command Handlers ========================

async function handleListBags(ws) {
  try {
    const bags = [];
    const entries = fs.readdirSync(ROSBAG_BASE_PATH);

    for (const entry of entries) {
      const fullPath = path.join(ROSBAG_BASE_PATH, entry);
      const stat = fs.statSync(fullPath);

      if (stat.isDirectory()) {
        const metadataPath = path.join(fullPath, 'metadata.yaml');
        if (fs.existsSync(metadataPath)) {
          const metadataContent = fs.readFileSync(metadataPath, 'utf8');
          const metadata = yaml.load(metadataContent);

          const dbFile = metadata.rosbag2_bagfile_information.relative_file_paths[0];
          const dbPath = path.join(fullPath, dbFile);
          const dbSize = fs.existsSync(dbPath) ? fs.statSync(dbPath).size : 0;

          bags.push({
            name: entry,
            path: fullPath,
            duration: metadata.rosbag2_bagfile_information.duration.nanoseconds / 1e9,
            messageCount: metadata.rosbag2_bagfile_information.message_count,
            topicCount: metadata.rosbag2_bagfile_information.topics_with_message_count.length,
            size: dbSize,
            sizeGB: (dbSize / 1024 / 1024 / 1024).toFixed(2)
          });
        }
      }
    }

    ws.send(JSON.stringify({
      type: 'bag_list',
      bags
    }));
  } catch (error) {
    console.error('[handleListBags] Error:', error);
    ws.send(JSON.stringify({
      type: 'error',
      message: error.message
    }));
  }
}

async function handleOpenBag(ws, bagPath) {
  try {
    console.log(`[handleOpenBag] Opening bag: ${bagPath}`);

    const bag = new RosBagManager(bagPath);
    await bag.initialize();

    ws.send(JSON.stringify({
      type: 'bag_opened',
      metadata: bag.getMetadata(),
      topics: bag.getTopics()
    }));

    return bag;
  } catch (error) {
    console.error('[handleOpenBag] Error:', error);
    ws.send(JSON.stringify({
      type: 'error',
      message: error.message
    }));
    throw error;
  }
}

async function handleGetTopics(ws, bag) {
  if (!bag) {
    ws.send(JSON.stringify({
      type: 'error',
      message: 'No bag opened'
    }));
    return;
  }

  ws.send(JSON.stringify({
    type: 'topics',
    topics: bag.getTopics()
  }));
}

async function handleStreamTopics(ws, bag, data) {
  if (!bag) {
    ws.send(JSON.stringify({
      type: 'error',
      message: 'No bag opened'
    }));
    return;
  }

  const { topics, startTime, endTime, playbackRate = 1.0 } = data;

  console.log(`[handleStreamTopics] Starting stream:`, {
    topics,
    startTime,
    endTime,
    playbackRate
  });

  ws.send(JSON.stringify({ type: 'stream_started' }));

  let lastTimestamp = null;
  let messageCount = 0;
  let imageCount = 0;

  try {
    await bag.streamMessages(topics, startTime, endTime, (row) => {
      // Decode message based on topic type
      const decoded = decodeMessage(row.topic_type, row.data);

      let messageData;

      if (decoded && row.topic_type === 'sensor_msgs/msg/CompressedImage') {
        // For compressed images, send the JPEG/PNG data as base64
        messageData = {
          type: 'message',
          topic: row.topic_name,
          topicType: row.topic_type,
          timestamp: row.timestamp,
          messageType: 'image',
          format: decoded.format,
          frameId: decoded.header.frameId,
          data: decoded.data.toString('base64'), // JPEG/PNG data
          stamp: decoded.header.stamp
        };
        imageCount++;

        if (imageCount % 10 === 0) {
          console.log(`[handleStreamTopics] Sent ${imageCount} images`);
        }
      } else if (decoded && (row.topic_type === 'std_msgs/msg/Float32' ||
                             row.topic_type === 'std_msgs/msg/Int16' ||
                             row.topic_type === 'std_msgs/msg/String')) {
        // For simple types, send the decoded value
        messageData = {
          type: 'message',
          topic: row.topic_name,
          topicType: row.topic_type,
          timestamp: row.timestamp,
          messageType: 'data',
          data: decoded.data
        };
      } else {
        // For unknown types or decode failures, send raw data
        messageData = {
          type: 'message',
          topic: row.topic_name,
          topicType: row.topic_type,
          timestamp: row.timestamp,
          messageType: 'raw',
          data: row.data.toString('base64')
        };
      }

      if (ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(messageData));
        messageCount++;

        if (messageCount % 100 === 0) {
          ws.send(JSON.stringify({
            type: 'stream_progress',
            messageCount,
            imageCount
          }));
        }
      }

      lastTimestamp = row.timestamp;
    });

    ws.send(JSON.stringify({
      type: 'stream_complete',
      messageCount,
      imageCount
    }));

    console.log(`[handleStreamTopics] Stream complete: ${messageCount} messages, ${imageCount} images`);
  } catch (error) {
    console.error('[handleStreamTopics] Error:', error);
    ws.send(JSON.stringify({
      type: 'error',
      message: error.message
    }));
  }
}

// ======================== HTTP Endpoints ========================

app.get('/health', (req, res) => {
  res.json({ status: 'ok', timestamp: Date.now() });
});

app.get('/api/bags', async (req, res) => {
  try {
    const bags = [];
    const entries = fs.readdirSync(ROSBAG_BASE_PATH);

    for (const entry of entries) {
      const fullPath = path.join(ROSBAG_BASE_PATH, entry);
      const stat = fs.statSync(fullPath);

      if (stat.isDirectory()) {
        const metadataPath = path.join(fullPath, 'metadata.yaml');
        if (fs.existsSync(metadataPath)) {
          const metadataContent = fs.readFileSync(metadataPath, 'utf8');
          const metadata = yaml.load(metadataContent);

          const dbFile = metadata.rosbag2_bagfile_information.relative_file_paths[0];
          const dbPath = path.join(fullPath, dbFile);
          const dbSize = fs.existsSync(dbPath) ? fs.statSync(dbPath).size : 0;

          bags.push({
            name: entry,
            path: fullPath,
            duration: metadata.rosbag2_bagfile_information.duration.nanoseconds / 1e9,
            messageCount: metadata.rosbag2_bagfile_information.message_count,
            topicCount: metadata.rosbag2_bagfile_information.topics_with_message_count.length,
            size: dbSize,
            sizeGB: (dbSize / 1024 / 1024 / 1024).toFixed(2)
          });
        }
      }
    }

    res.json({ bags });
  } catch (error) {
    console.error('[/api/bags] Error:', error);
    res.status(500).json({ error: error.message });
  }
});

// ======================== Server Startup ========================

server.listen(PORT, () => {
  console.log('');
  console.log('╔════════════════════════════════════════════════════════════════╗');
  console.log('║                                                                ║');
  console.log('║          FSM-Pilot V2.0 - RosBag Streaming Server             ║');
  console.log('║                                                                ║');
  console.log('╚════════════════════════════════════════════════════════════════╝');
  console.log('');
  console.log(`✓ HTTP Server:      http://localhost:${PORT}`);
  console.log(`✓ WebSocket Server: ws://localhost:${PORT}`);
  console.log(`✓ RosBag Path:      ${ROSBAG_BASE_PATH}`);
  console.log('');
  console.log('Server is ready to stream large RosBag files (>15GB)');
  console.log('');
});

// ======================== Graceful Shutdown ========================

process.on('SIGINT', () => {
  console.log('\n[Server] Shutting down gracefully...');

  // Close all active bags
  activeBags.forEach(bag => bag.close());
  activeBags.clear();

  // Close WebSocket server
  wss.close(() => {
    console.log('[Server] WebSocket server closed');
  });

  // Close HTTP server
  server.close(() => {
    console.log('[Server] HTTP server closed');
    process.exit(0);
  });
});
