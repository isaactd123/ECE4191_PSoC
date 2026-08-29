#pragma once

#include <Arduino.h>
#include <stddef.h>

namespace WriteCodeBot {

constexpr uint8_t MAGIC_0 = 'W';
constexpr uint8_t MAGIC_1 = 'C';
constexpr uint8_t VERSION = 1;
constexpr size_t MAX_PAYLOAD = 240;

enum class MessageType : uint8_t { Program = 1, Vowel = 2, Ack = 3, Status = 4 };

struct __attribute__((packed)) Packet {
  uint8_t magic0, magic1, version, type;
  uint16_t sequence, length;
  uint8_t payload[MAX_PAYLOAD];
  uint16_t checksum;
};

constexpr size_t HEADER_SIZE = offsetof(Packet, payload);
constexpr size_t CHECKSUM_SIZE = sizeof(uint16_t);

inline uint16_t crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < length; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                           : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

inline size_t buildPacket(Packet &packet, MessageType type, uint16_t sequence,
                          const uint8_t *payload, size_t length) {
  if (length > MAX_PAYLOAD || (length > 0 && payload == nullptr)) return 0;
  packet.magic0 = MAGIC_0; packet.magic1 = MAGIC_1; packet.version = VERSION;
  packet.type = static_cast<uint8_t>(type); packet.sequence = sequence;
  packet.length = static_cast<uint16_t>(length);
  if (length > 0) memcpy(packet.payload, payload, length);
  const uint16_t checksum = crc16(reinterpret_cast<const uint8_t *>(&packet), HEADER_SIZE + length);
  memcpy(reinterpret_cast<uint8_t *>(&packet) + HEADER_SIZE + length,
         &checksum, CHECKSUM_SIZE);
  return HEADER_SIZE + length + CHECKSUM_SIZE;
}

inline bool decodePacket(const uint8_t *data, size_t receivedLength, Packet &packet) {
  if (!data || receivedLength < HEADER_SIZE + CHECKSUM_SIZE) return false;
  memcpy(&packet, data, receivedLength > sizeof(Packet) ? sizeof(Packet) : receivedLength);
  if (packet.magic0 != MAGIC_0 || packet.magic1 != MAGIC_1 ||
      packet.version != VERSION || packet.length > MAX_PAYLOAD) return false;
  const size_t expected = HEADER_SIZE + packet.length + CHECKSUM_SIZE;
  if (receivedLength != expected) return false;
  uint16_t receivedCrc;
  memcpy(&receivedCrc, data + HEADER_SIZE + packet.length, CHECKSUM_SIZE);
  return receivedCrc == crc16(data, HEADER_SIZE + packet.length);
}

}  // namespace WriteCodeBot
