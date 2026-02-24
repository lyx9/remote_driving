#!/usr/bin/env node

/**
 * Test All 6 Cameras in RosBag Replay
 *
 * This script verifies that all 6 cameras can be properly decoded and displayed
 */

const WebSocket = require('ws');

const WS_URL = 'ws://localhost:8765';
const BAG_PATH = '/home/lyx/fsm/rosbag/1210/rosbag2_2025_12_10-17_25_58';

const cameraStats = {
  camera0: 0,
  camera1: 0,
  camera2: 0,
  camera3: 0,
  camera4: 0,
  camera5: 0
};

let totalImages = 0;
const TARGET_IMAGES_PER_CAMERA = 10;

console.log('🧪 Testing All 6 Cameras in RosBag Replay\n');
console.log('Connecting to:', WS_URL);

const ws = new WebSocket(WS_URL);

ws.on('open', () => {
  console.log('✓ WebSocket connected\n');

  // Step 1: Open bag
  console.log('Step 1: Opening bag...');
  ws.send(JSON.stringify({
    command: 'open_bag',
    bagPath: BAG_PATH
  }));
});

ws.on('message', (data) => {
  try {
    const message = JSON.parse(data.toString());

    switch (message.type) {
      case 'bag_opened':
        console.log('✓ Bag opened successfully');
        console.log(`  Topics: ${message.topics.length}`);
        console.log(`  Duration: ${(message.metadata.rosbag2_bagfile_information.duration.nanoseconds / 1e9).toFixed(2)}s\n`);

        // Find all camera topics (only /compressed, not /compressedDepth)
        const cameraTopics = message.topics
          .filter(t =>
            t.name.includes('/camera') &&
            t.name.endsWith('/compressed') &&
            t.type === 'sensor_msgs/msg/CompressedImage'
          )
          .map(t => t.name)
          .sort();

        console.log('Step 2: Streaming all camera topics...');
        console.log('Camera topics found:', cameraTopics.length);
        cameraTopics.forEach(topic => console.log(`  - ${topic}`));
        console.log('');

        if (cameraTopics.length !== 6) {
          console.error(`❌ Expected 6 camera topics, found ${cameraTopics.length}`);
          ws.close();
          process.exit(1);
        }

        // Stream all 6 cameras
        ws.send(JSON.stringify({
          command: 'stream_topics',
          topics: cameraTopics,
          startTime: null,
          endTime: null,
          playbackRate: 1.0
        }));
        break;

      case 'stream_started':
        console.log('✓ Stream started\n');
        console.log('Receiving images from all cameras...\n');
        break;

      case 'message':
        if (message.messageType === 'image' && message.data) {
          totalImages++;

          // Extract camera number from topic
          const match = message.topic.match(/camera(\d+)/);
          if (match) {
            const cameraNum = match[1];
            const cameraKey = `camera${cameraNum}`;
            cameraStats[cameraKey]++;

            // Log first image from each camera
            if (cameraStats[cameraKey] === 1) {
              console.log(`📷 First image from Camera ${cameraNum}:`);
              console.log(`   Topic: ${message.topic}`);
              console.log(`   Format: ${message.format}`);
              console.log(`   Data size: ${message.data.length} bytes (base64)`);

              // Validate JPEG
              const buffer = Buffer.from(message.data, 'base64');
              const isJPEG = buffer[0] === 0xFF && buffer[1] === 0xD8 && buffer[2] === 0xFF;
              console.log(`   Valid JPEG: ${isJPEG ? '✓' : '✗'}`);
              console.log('');
            }
          }

          // Check if we have enough images from all cameras
          const allCamerasHaveImages = Object.values(cameraStats).every(count => count >= TARGET_IMAGES_PER_CAMERA);
          if (allCamerasHaveImages) {
            console.log('\n✓ Test complete! All cameras working\n');
            printSummary();
            ws.close();
            process.exit(0);
          }
        }
        break;

      case 'stream_complete':
        console.log('\n✓ Stream complete');
        printSummary();
        ws.close();
        break;

      case 'error':
        console.error('❌ Error:', message.message);
        ws.close();
        process.exit(1);
        break;
    }
  } catch (error) {
    console.error('Error parsing message:', error);
  }
});

ws.on('error', (error) => {
  console.error('❌ WebSocket error:', error);
  process.exit(1);
});

ws.on('close', () => {
  console.log('\n👋 WebSocket closed');
});

function printSummary() {
  console.log('Test Summary:');
  console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');
  console.log(`Total images received: ${totalImages}`);
  console.log('');
  console.log('Images per camera:');

  let allPassed = true;
  for (let i = 0; i < 6; i++) {
    const cameraKey = `camera${i}`;
    const count = cameraStats[cameraKey];
    const status = count >= TARGET_IMAGES_PER_CAMERA ? '✓' : '✗';
    const statusColor = count >= TARGET_IMAGES_PER_CAMERA ? '\x1b[32m' : '\x1b[31m';
    console.log(`  ${statusColor}${status}\x1b[0m Camera ${i}: ${count} images`);

    if (count < TARGET_IMAGES_PER_CAMERA) {
      allPassed = false;
    }
  }

  console.log('━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━');

  if (allPassed) {
    console.log('\n\x1b[32m✅ SUCCESS: All 6 cameras working correctly!\x1b[0m\n');
  } else {
    console.log('\n\x1b[31m❌ FAILURE: Some cameras not working\x1b[0m\n');
    process.exit(1);
  }
}

// Timeout after 30 seconds
setTimeout(() => {
  console.error('\n❌ Test timeout - not all cameras received images within 30 seconds');
  printSummary();
  ws.close();
  process.exit(1);
}, 30000);
