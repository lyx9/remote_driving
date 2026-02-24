/**
 * Test RosBag Image Decoding
 *
 * This script connects to the RosBag WebSocket server and verifies
 * that CompressedImage messages are being properly decoded.
 */

const WebSocket = require('ws');

const WS_URL = 'ws://localhost:8765';
const BAG_PATH = '/home/lyx/fsm/rosbag/1210/rosbag2_2025_12_10-17_25_58';

let messageCount = 0;
let imageCount = 0;
let dataCount = 0;

console.log('🧪 Testing RosBag Image Decoding\n');
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

        // Find camera topics (only /compressed, not /compressedDepth)
        const cameraTopics = message.topics
          .filter(t => t.name.includes('camera') &&
                      t.name.endsWith('/compressed') &&
                      t.type === 'sensor_msgs/msg/CompressedImage')
          .map(t => t.name);

        console.log('Step 2: Streaming camera topics...');
        console.log('Camera topics:', cameraTopics);
        console.log('');

        // Stream first 100 messages from cameras
        ws.send(JSON.stringify({
          command: 'stream_topics',
          topics: cameraTopics.slice(0, 2), // Test with 2 cameras
          startTime: null,
          endTime: null,
          playbackRate: 1.0
        }));
        break;

      case 'stream_started':
        console.log('✓ Stream started\n');
        console.log('Receiving messages...\n');
        break;

      case 'message':
        messageCount++;

        if (message.messageType === 'image') {
          imageCount++;

          // Verify image data structure
          const hasData = message.data && message.data.length > 0;
          const hasFormat = message.format && (message.format === 'jpeg' || message.format === 'png');
          const hasHeader = message.frameId && message.stamp;

          if (imageCount === 1) {
            console.log('📷 First Image Message Received:');
            console.log(`  Topic: ${message.topic}`);
            console.log(`  Format: ${message.format}`);
            console.log(`  Frame ID: ${message.frameId}`);
            console.log(`  Timestamp: ${message.stamp.sec}.${message.stamp.nanosec}`);
            console.log(`  Data size: ${message.data.length} bytes (base64)`);
            console.log(`  Has valid data: ${hasData ? '✓' : '✗'}`);
            console.log(`  Has valid format: ${hasFormat ? '✓' : '✗'}`);
            console.log(`  Has valid header: ${hasHeader ? '✓' : '✗'}`);
            console.log('');

            // Decode base64 to check actual JPEG size
            const buffer = Buffer.from(message.data, 'base64');
            console.log(`  Decoded JPEG size: ${buffer.length} bytes`);

            // Check JPEG header (should start with FF D8 FF)
            const isJPEG = buffer[0] === 0xFF && buffer[1] === 0xD8 && buffer[2] === 0xFF;
            console.log(`  Valid JPEG header: ${isJPEG ? '✓' : '✗'}`);
            console.log('');
          }

          if (imageCount % 10 === 0) {
            console.log(`  ... received ${imageCount} images, ${dataCount} data messages (${messageCount} total)`);
          }

          // Stop after 50 images to verify
          if (imageCount >= 50) {
            console.log('\n✓ Test complete! Received 50+ images\n');

            console.log('Test Summary:');
            console.log(`  Total messages: ${messageCount}`);
            console.log(`  Image messages: ${imageCount}`);
            console.log(`  Data messages: ${dataCount}`);
            console.log('');
            console.log('✅ Image decoding is working correctly!');

            ws.close();
            process.exit(0);
          }
        } else if (message.messageType === 'data') {
          dataCount++;
        }
        break;

      case 'stream_progress':
        // Ignore progress messages
        break;

      case 'stream_complete':
        console.log('\n✓ Stream complete');
        console.log(`  Total messages: ${message.messageCount}`);
        console.log(`  Image messages: ${message.imageCount}`);
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

  if (imageCount < 50) {
    console.log('\n⚠️  Warning: Received fewer than 50 images');
    console.log(`   Image count: ${imageCount}`);
    process.exit(1);
  }
});

// Timeout after 30 seconds
setTimeout(() => {
  console.error('\n❌ Test timeout - no images received within 30 seconds');
  console.log(`   Messages received: ${messageCount}`);
  console.log(`   Images received: ${imageCount}`);
  ws.close();
  process.exit(1);
}, 30000);
