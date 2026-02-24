/**
 * Guardian Mobility - RosBag Message Decoder
 *
 * Decodes ROS2 CDR messages for CompressedImage topics
 */

class CDRDecoder {
  constructor(buffer) {
    this.buffer = buffer;
    this.offset = 0;
    this.littleEndian = true; // ROS2 uses little-endian
  }

  // Skip CDR header (4 bytes)
  skipHeader() {
    this.offset += 4;
  }

  // Read uint32
  readUint32() {
    const value = this.buffer.readUInt32LE(this.offset);
    this.offset += 4;
    return value;
  }

  // Read uint64 (as BigInt)
  readUint64() {
    const value = this.buffer.readBigUInt64LE(this.offset);
    this.offset += 8;
    return value;
  }

  // Read string
  readString() {
    const length = this.readUint32();
    if (length === 0) return '';

    const str = this.buffer.toString('utf8', this.offset, this.offset + length - 1); // -1 for null terminator
    this.offset += length;

    // Align to 4-byte boundary
    const remainder = this.offset % 4;
    if (remainder !== 0) {
      this.offset += (4 - remainder);
    }

    return str;
  }

  // Read byte array
  readByteArray() {
    const length = this.readUint32();
    if (length === 0) return Buffer.alloc(0);

    const data = this.buffer.slice(this.offset, this.offset + length);
    this.offset += length;

    // Align to 4-byte boundary
    const remainder = this.offset % 4;
    if (remainder !== 0) {
      this.offset += (4 - remainder);
    }

    return data;
  }
}

/**
 * Decode CompressedImage message
 *
 * Message structure:
 * - std_msgs/Header header
 *   - builtin_interfaces/Time stamp
 *     - int32 sec
 *     - uint32 nanosec
 *   - string frame_id
 * - string format (e.g., "jpeg", "png")
 * - uint8[] data
 */
function decodeCompressedImage(buffer) {
  try {
    const decoder = new CDRDecoder(buffer);

    // Skip CDR header
    decoder.skipHeader();

    // Read Header
    // Read stamp (Time)
    const sec = decoder.readUint32();
    const nanosec = decoder.readUint32();

    // Read frame_id
    const frameId = decoder.readString();

    // Read format
    const format = decoder.readString();

    // Read image data
    const imageData = decoder.readByteArray();

    return {
      header: {
        stamp: {
          sec,
          nanosec
        },
        frameId
      },
      format,
      data: imageData
    };
  } catch (error) {
    console.error('[decodeCompressedImage] Error:', error);
    return null;
  }
}

/**
 * Decode Float32 message (for chassis CAN data)
 */
function decodeFloat32(buffer) {
  try {
    const decoder = new CDRDecoder(buffer);
    decoder.skipHeader();

    const value = buffer.readFloatLE(decoder.offset);
    return { data: value };
  } catch (error) {
    console.error('[decodeFloat32] Error:', error);
    return null;
  }
}

/**
 * Decode Int16 message (for SOC)
 */
function decodeInt16(buffer) {
  try {
    const decoder = new CDRDecoder(buffer);
    decoder.skipHeader();

    const value = buffer.readInt16LE(decoder.offset);
    return { data: value };
  } catch (error) {
    console.error('[decodeInt16] Error:', error);
    return null;
  }
}

/**
 * Decode String message
 */
function decodeString(buffer) {
  try {
    const decoder = new CDRDecoder(buffer);
    decoder.skipHeader();

    const value = decoder.readString();
    return { data: value };
  } catch (error) {
    console.error('[decodeString] Error:', error);
    return null;
  }
}

/**
 * Main decoder function - routes to appropriate decoder based on topic type
 */
function decodeMessage(topicType, buffer) {
  if (topicType === 'sensor_msgs/msg/CompressedImage') {
    return decodeCompressedImage(buffer);
  } else if (topicType === 'std_msgs/msg/Float32') {
    return decodeFloat32(buffer);
  } else if (topicType === 'std_msgs/msg/Int16') {
    return decodeInt16(buffer);
  } else if (topicType === 'std_msgs/msg/String') {
    return decodeString(buffer);
  }

  // For unknown types, return null
  return null;
}

module.exports = {
  decodeMessage,
  decodeCompressedImage,
  decodeFloat32,
  decodeInt16,
  decodeString
};
