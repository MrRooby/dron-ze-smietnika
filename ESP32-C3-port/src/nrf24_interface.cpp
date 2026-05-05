#include "nrf24_interface.h"
#include <RF24.h>
#include <Arduino.h>

// ============================================================================
// NRF24 INSTANCE & STATE
// ============================================================================

// RF24 radio object (CE, CSN pins)
static RF24 radio(10, 7);  // CE=10, CSN=7

// Radio state
static struct {
  uint8_t initialized;
  uint32_t last_packet_ms;
} nrf24_state = {0, 0};

// Receiving pipe address
static const uint8_t address[5] = {0xE8, 0xE8, 0xF0, 0xF0, 0xE1};

// ============================================================================
// INITIALIZATION
// ============================================================================

uint8_t nrf24_init() {
  Serial.println("[NRF24] Initializing radio module...");
  
  // Initialize SPI bus with custom pins
  SPI.begin(4, 5, 6, 7);  // SCK=4, MISO=5, MOSI=6, CSN=7
  
  // Begin radio with SPI
  if (!radio.begin()) {
    Serial.println("[NRF24] ERROR: radio.begin() failed!");
    nrf24_state.initialized = 0;
    return 0;
  }
  
  // Configure to match transmitter
  radio.setDataRate(RF24_1MBPS);
  radio.setChannel(32);
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(6);  // 6-byte packets
  
  // Disable dynamic payloads
  radio.disableDynamicPayloads();
  radio.setCRCLength(RF24_CRC_16);
  
  // Open Pipe 0 for listening
  radio.openReadingPipe(0, address);
  
  // Enable auto-acknowledge
  radio.setAutoAck(true);
  
  // Start listening
  radio.startListening();
  nrf24_state.last_packet_ms = millis();
  nrf24_state.initialized = 1;
  
  Serial.println("[NRF24] ✓ Radio initialized successfully");
  Serial.println("[NRF24] Channel: 32, Data Rate: 1Mbps, Payload: 6 bytes");
  
  return 1;
}

// ============================================================================
// COMMAND RECEPTION
// ============================================================================

uint8_t nrf24_read_commands(RF24_Packet *packet) {
  if (!nrf24_state.initialized) return 0;
  if (!packet) return 0;
  
  // Check if data is available
  if (!radio.available()) {
    return 0;  // No new packet
  }
  
  // Receive packet
  radio.read(packet, sizeof(RF24_Packet));
  
  // Update timestamp
  nrf24_state.last_packet_ms = millis();
  
  return 1;  // New packet received
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

uint8_t nrf24_is_connected() {
  if (!nrf24_state.initialized) return 0;
  return radio.isChipConnected();
}

uint32_t nrf24_get_last_packet_ms() {
  return nrf24_state.last_packet_ms;
}
