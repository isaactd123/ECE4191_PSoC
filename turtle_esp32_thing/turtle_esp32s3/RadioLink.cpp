/*
    RadioLink.cpp

    Based on the original "Wireless and PSoC bridge" section, with two
    corrections marked FIX below.
*/

#include "RadioLink.h"
#include "TurtleConfig.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <string.h>

using namespace WriteCodeBot;

namespace {

const uint8_t BROADCAST_ADDRESS[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool radioReady = false;

uint16_t nextRadioSequence = 1;
uint16_t lastProgramSequence = 0;
bool haveLastProgramSequence = false;

uint8_t interfaceAddress[6] = {};
bool interfaceAddressKnown = false;

volatile bool programReady = false;
volatile size_t queuedProgramLength = 0;
uint8_t queuedProgram[MAX_PAYLOAD] = {};
portMUX_TYPE programMux = portMUX_INITIALIZER_UNLOCKED;

Packet pendingVowelPacket = {};
size_t pendingVowelPacketSize = 0;
uint16_t pendingVowelSequence = 0;
uint8_t pendingVowelAttempts = 0;
uint32_t lastVowelSendMs = 0;
bool waitingForVowelAck = false;

volatile bool vowelAckReady = false;
volatile uint16_t vowelAckSequence = 0;
portMUX_TYPE vowelAckMux = portMUX_INITIALIZER_UNLOCKED;

bool ensurePeer(const uint8_t* address) {
  if (esp_now_is_peer_exist(address)) return true;
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, address, 6);
  peer.channel = ESPNOW_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

bool sendRadioPacket(const uint8_t* address, MessageType type,
                     uint16_t sequence, const uint8_t* payload, size_t length) {
  Packet packet = {};
  const size_t packetSize = buildPacket(packet, type, sequence, payload, length);
  if (!packetSize || !ensurePeer(address)) return false;
  return esp_now_send(address, reinterpret_cast<const uint8_t*>(&packet), packetSize) == ESP_OK;
}

void sendProgramAck(const uint8_t* address, uint16_t sequence) {
  sendRadioPacket(address, MessageType::Ack, sequence, nullptr, 0);
}

void onEspNowReceive(const esp_now_recv_info_t* info, const uint8_t* data, int length) {
  if (!info || !data || length <= 0) return;
  Packet packet = {};
  if (!decodePacket(data, static_cast<size_t>(length), packet)) return;

  memcpy(interfaceAddress, info->src_addr, 6);
  interfaceAddressKnown = true;
  ensurePeer(info->src_addr);

  const MessageType messageType = static_cast<MessageType>(packet.type);
  if (messageType == MessageType::Ack) {
    portENTER_CRITICAL(&vowelAckMux);
    vowelAckSequence = packet.sequence;
    vowelAckReady = true;
    portEXIT_CRITICAL(&vowelAckMux);
    return;
  }
  if (messageType != MessageType::Program) return;

  if (packet.length == 0 || packet.length > MAX_PAYLOAD) return;

  // The master retries until it receives the acknowledgement. Do not
  // forward a retry to the PSoC a second time.
  if (haveLastProgramSequence && packet.sequence == lastProgramSequence) {
    sendProgramAck(info->src_addr, packet.sequence);
    return;
  }

  /*
      FIX 1: the original returned here when programReady was still
      true, WITHOUT sending an acknowledgement. The master then retried
      four times and gave up with PROGRAM_ACK_TIMEOUT. Because the main
      loop can be busy classifying a frame for over a second, that
      window was easy to hit.

      Acknowledge the frame so the master stops retransmitting, and
      drop only the duplicate copy.
  */
  if (programReady) {
    sendProgramAck(info->src_addr, packet.sequence);
    return;
  }

  portENTER_CRITICAL(&programMux);
  memcpy(queuedProgram, packet.payload, packet.length);
  queuedProgramLength = packet.length;
  lastProgramSequence = packet.sequence;
  haveLastProgramSequence = true;
  programReady = true;
  portEXIT_CRITICAL(&programMux);
  sendProgramAck(info->src_addr, packet.sequence);
}

}  // namespace

namespace RadioLink {

bool begin() {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR:ESPNOW_INIT");
    return false;
  }

  ensurePeer(BROADCAST_ADDRESS);
  esp_now_register_recv_cb(onEspNowReceive);

  Serial.printf("ESP-NOW MAC: %s, channel: %u\n",
                WiFi.macAddress().c_str(), ESPNOW_CHANNEL);

  radioReady = true;
  return true;
}

bool isReady() {
  return radioReady;
}

size_t takeQueuedProgram(uint8_t* destination, size_t capacity) {
  if (!programReady) return 0;

  portENTER_CRITICAL(&programMux);
  size_t length = queuedProgramLength;
  if (length > capacity) length = capacity;
  memcpy(destination, queuedProgram, length);
  programReady = false;
  portEXIT_CRITICAL(&programMux);

  return length;
}

void sendVowel(char normalizedVowel) {
  if (!radioReady) {
    Serial.println("ERROR:VOWEL_RADIO_NOT_READY");
    return;
  }

  const uint8_t payload = static_cast<uint8_t>(normalizedVowel);
  const uint8_t* destination =
      interfaceAddressKnown ? interfaceAddress : BROADCAST_ADDRESS;

  pendingVowelSequence = nextRadioSequence++;
  pendingVowelPacketSize = buildPacket(pendingVowelPacket, MessageType::Vowel,
                                       pendingVowelSequence, &payload, 1);

  const bool sent = pendingVowelPacketSize && ensurePeer(destination) &&
      esp_now_send(destination,
                   reinterpret_cast<const uint8_t*>(&pendingVowelPacket),
                   pendingVowelPacketSize) == ESP_OK;

  waitingForVowelAck = sent;
  pendingVowelAttempts = sent ? 1 : 0;
  lastVowelSendMs = millis();

  if (!sent) Serial.println("ERROR:VOWEL_RADIO_SEND");
}

void service() {
  if (vowelAckReady) {
    portENTER_CRITICAL(&vowelAckMux);
    const uint16_t sequence = vowelAckSequence;
    vowelAckReady = false;
    portEXIT_CRITICAL(&vowelAckMux);
    if (waitingForVowelAck && sequence == pendingVowelSequence)
      waitingForVowelAck = false;
  }

  if (!waitingForVowelAck || millis() - lastVowelSendMs < RADIO_ACK_TIMEOUT_MS) return;

  if (pendingVowelAttempts >= MAX_VOWEL_SEND_ATTEMPTS) {
    waitingForVowelAck = false;
    Serial.println("ERROR:VOWEL_ACK_TIMEOUT");
    return;
  }

  const uint8_t* destination =
      interfaceAddressKnown ? interfaceAddress : BROADCAST_ADDRESS;
  ++pendingVowelAttempts;
  lastVowelSendMs = millis();
  esp_now_send(destination, reinterpret_cast<const uint8_t*>(&pendingVowelPacket),
               pendingVowelPacketSize);
}

}  // namespace RadioLink
