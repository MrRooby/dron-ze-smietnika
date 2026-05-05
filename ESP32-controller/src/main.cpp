#include <Arduino.h>
#include "SPI.h"
#include "RF24.h"
#include "nRF24L01.h"

// ============================================================================
// RADIO CONFIGURATION
// ============================================================================
#define CE_PIN 10  // CE on GPIO 10
#define CSN_PIN 7  // CSN on GPIO 7

// Custom SPI pins - ACTUAL PHYSICAL CONNECTIONS
#define SPI_SCK 4    // SCK on GPIO 4 (actual hardware)
#define SPI_MISO 5   // MISO on GPIO 5 (actual hardware)
#define SPI_MOSI 6   // MOSI on GPIO 6 (actual hardware)

#define LED_PIN 8  // Inbuilt LED on ESP32-C3

// Transmitter address - drone receiver listens on this address
const uint8_t TX_ADDRESS[5] = {0xE8, 0xE8, 0xF0, 0xF0, 0xE1};

// NRF24 Parameters - MUST MATCH RECEIVER
#define RF24_CHANNEL 32        // Receiver uses channel 32
#define RF24_DATARATE RF24_1MBPS   // 1Mbps (matches receiver)
#define RF24_PAYLOAD_SIZE 6    // 6-byte packets (no checksum)

// ============================================================================
// CONTROL INPUT CONFIGURATION
// ============================================================================
// Optional analog input pins for joystick (comment out to use serial only)
#define THROTTLE_PIN 36 // A0 on ESP32
#define YAW_PIN 39      // A1 on ESP32
#define PITCH_PIN 34    // A2 on ESP32
#define ROLL_PIN 35     // A3 on ESP32
#define AUX1_PIN 32     // A4 on ESP32
#define AUX2_PIN 33     // A5 on ESP32

// ============================================================================
// DATA STRUCTURES
// ============================================================================
RF24 radio(CE_PIN, CSN_PIN);

// 6-byte RF24 packet format (matches drone receiver)
struct Payload {
  uint8_t throttle;   // [0-255]
  uint8_t yaw;        // [0-255]
  uint8_t pitch;      // [0-255]
  uint8_t roll;       // [0-255]
  uint8_t AUX1;       // [0-255]
  uint8_t AUX2;       // [0-255]
};

Payload transmitData;

// ============================================================================
// TIMING & STATISTICS
// ============================================================================
unsigned long lastTransmit = 0;
unsigned long lastDebug = 0;
unsigned long lastLedBlink = 0;
const unsigned long TRANSMIT_INTERVAL = 50;  // Send every 50ms (20Hz to match drone)
const unsigned long DEBUG_INTERVAL = 2000;   // Debug output every 2 seconds
const unsigned long LED_BLINK_DURATION = 50; // LED blink duration in ms

uint32_t txPacketCount = 0;

// Helper function declaration
void printAddress(const uint8_t addr[5]);
void update_led_status();

void setup() {
  Serial.begin(115200);
  delay(3000);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Initialize analog pins as inputs
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(YAW_PIN, INPUT);
  pinMode(PITCH_PIN, INPUT);
  pinMode(ROLL_PIN, INPUT);
  pinMode(AUX1_PIN, INPUT);
  pinMode(AUX2_PIN, INPUT);
  
  Serial.println("\n╔════════════════════════════════════════════════╗");
  Serial.println("║  ESP32 NRF24 DRONE TRANSMITTER (COMPATIBLE)    ║");
  Serial.println("╚════════════════════════════════════════════════╝");
  Serial.printf("CE Pin: %d | CSN Pin: %d | LED Pin: %d\n", CE_PIN, CSN_PIN, LED_PIN);
  Serial.printf("SPI - SCK: %d | MOSI: %d | MISO: %d\n", SPI_SCK, SPI_MOSI, SPI_MISO);
  Serial.println("\nInitializing SPI bus with custom pins...");
  
  // Initialize SPI bus with custom pins (critical!)
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, CSN_PIN);
  
  Serial.println("Initializing radio module...");
  
  // Try to initialize radio
  if (!radio.begin()) {
    digitalWrite(LED_PIN, LOW);
    Serial.println("\n⚠ ERROR: NRF24 not responding!");
    Serial.println("Verify connections:");
    Serial.println("  • VCC: 3.3V with 10µF capacitor");
    Serial.println("  • GND: Ground connection");
    Serial.printf("  • CE: GPIO%d | CSN: GPIO%d\n", CE_PIN, CSN_PIN);
    Serial.printf("  • SCK: GPIO%d | MOSI: GPIO%d | MISO: GPIO%d\n", SPI_SCK, SPI_MOSI, SPI_MISO);
    
    // Blink LED rapidly on error
    while (1) {
      digitalWrite(LED_PIN, HIGH);
      delay(100);
      digitalWrite(LED_PIN, LOW);
      delay(100);
    }
  }
  
  Serial.println("✓ Radio initialized!");
  
  // Configure to match drone receiver
  radio.setAutoAck(true);                // Enable auto-acknowledge
  radio.setDataRate(RF24_1MBPS);         // 1 Mbps - CRITICAL for compatibility
  radio.setChannel(RF24_CHANNEL);        // Channel 32 - CRITICAL for compatibility
  radio.setPALevel(RF24_PA_LOW);         // Low power for testing
  radio.setPayloadSize(RF24_PAYLOAD_SIZE); // 6 bytes - CRITICAL for compatibility
  
  // Set TX address (drone receives on this address)
  radio.openWritingPipe(TX_ADDRESS);
  
  // Enter TX mode (stop listening)
  radio.stopListening();
  
  delay(100);
  
  // Print radio configuration
  Serial.println("\n┌─ RADIO CONFIGURATION ─────────────────────────┐");
  Serial.printf("│ Payload Size:  %d bytes\n", radio.getPayloadSize());
  
  uint8_t rate = radio.getDataRate();
  Serial.print("│ Data Rate:     ");
  if (rate == RF24_1MBPS) Serial.println("1 Mbps ✓");
  else if (rate == RF24_250KBPS) Serial.println("250 Kbps");
  else Serial.println("2 Mbps");
  
  Serial.printf("│ Channel:       %d (2%03d MHz)\n", radio.getChannel(), 
                2300 + radio.getChannel());
  
  uint8_t pa = radio.getPALevel();
  Serial.print("│ PA Level:      ");
  if (pa == RF24_PA_MIN) Serial.println("MIN");
  else if (pa == RF24_PA_LOW) Serial.println("LOW");
  else if (pa == RF24_PA_HIGH) Serial.println("HIGH");
  else Serial.println("MAX");
  
  Serial.println("├─ TRANSMITTER ADDRESS ────────────────────────┤");
  Serial.print("│ TX Address:    ");
  printAddress(TX_ADDRESS);
  Serial.println("└────────────────────────────────────────────────┘");
  
  Serial.println("\n┌─ CONTROL INPUT OPTIONS ───────────────────────┐");
  Serial.println("│ Serial Commands (format: COMMAND<value>):      │");
  Serial.println("│   T<0-255>   - Set throttle                   │");
  Serial.println("│   Y<0-255>   - Set yaw (→ roll on drone)      │");
  Serial.println("│   P<0-255>   - Set pitch                      │");
  Serial.println("│   R<0-255>   - Set roll (→ yaw on drone)      │");
  Serial.println("│   A<0-255>   - Set aux1 (mode)                │");
  Serial.println("│   B<0-255>   - Set aux2                       │");
  Serial.println("│                                                 │");
  Serial.println("│ Analog Joystick (uncomment in code):           │");
  Serial.println("│   Reads A0-A5 and auto-scales to 0-255        │");
  Serial.println("└────────────────────────────────────────────────┘");
  
  Serial.println("\n┌─ OPERATION ────────────────────────────────────┐");
  Serial.println("│ Ready to transmit to drone receiver...          │");
  Serial.println("│ Default values: THR=120, YAW=128, PIT=128,      │");
  Serial.println("│                 ROLL=128, AUX1=255, AUX2=255   │");
  Serial.println("└────────────────────────────────────────────────┘\n");
  
  // Initialize transmit data (safe defaults)
  transmitData.throttle = 120;  // Low throttle (safe)
  transmitData.yaw = 128;       // Center
  transmitData.pitch = 128;     // Center
  transmitData.roll = 128;      // Center
  transmitData.AUX1 = 255;      // Max (mode dependent on drone)
  transmitData.AUX2 = 255;      // Max
}


void loop() {
  unsigned long now = millis();
  
  // ===== UPDATE LED STATUS =====
  update_led_status();
  
  // ===== SERIAL COMMAND PROCESSING =====
  if (Serial.available()) {
    char cmd = Serial.read();
    int value = Serial.parseInt();
    
    // Constrain value to 0-255
    if (value < 0) value = 0;
    if (value > 255) value = 255;
    
    switch (cmd) {
      case 'T': 
        transmitData.throttle = value;
        Serial.printf("✓ Throttle set to %d\n", value);
        break;
      case 'Y':
        transmitData.yaw = value;
        Serial.printf("✓ Yaw (→ drone roll) set to %d\n", value);
        break;
      case 'P':
        transmitData.pitch = value;
        Serial.printf("✓ Pitch set to %d\n", value);
        break;
      case 'R':
        transmitData.roll = value;
        Serial.printf("✓ Roll (→ drone yaw) set to %d\n", value);
        break;
      case 'A':
        transmitData.AUX1 = value;
        Serial.printf("✓ AUX1 set to %d\n", value);
        break;
      case 'B':
        transmitData.AUX2 = value;
        Serial.printf("✓ AUX2 set to %d\n", value);
        break;
    }
  }
  
  // ===== READ ANALOG INPUTS =====
  // Uncomment this section to read from joystick/potentiometers
  // Maps 0-1023 analog to 0-255
  /*
  transmitData.throttle = analogRead(THROTTLE_PIN) >> 2;
  transmitData.yaw = analogRead(YAW_PIN) >> 2;
  transmitData.pitch = analogRead(PITCH_PIN) >> 2;
  transmitData.roll = analogRead(ROLL_PIN) >> 2;
  transmitData.aux1 = analogRead(AUX1_PIN) >> 2;
  transmitData.aux2 = analogRead(AUX2_PIN) >> 2;
  */
  
  // ===== TRANSMIT PERIODICALLY =====
  if (now - lastTransmit >= TRANSMIT_INTERVAL) {
    lastTransmit = now;
    
    if (radio.write(&transmitData, sizeof(transmitData))) {
      txPacketCount++;
      // Trigger LED blink on successful transmit
      lastLedBlink = now;
    }
  }
  
  // ===== PERIODIC DEBUG OUTPUT =====
  if (now - lastDebug > DEBUG_INTERVAL) {
    lastDebug = now;
    
    Serial.println("┌─ TRANSMITTER STATUS ──────────────────────────┐");
    Serial.printf("│ Packets TX:   %lu\n", txPacketCount);
    Serial.println("├─ CURRENT CONTROL VALUES ──────────────────────┤");
    Serial.printf("│ Throttle:     %3d → Drone: %4d µs (%.0f%%)\n", 
      transmitData.throttle,
      2000 - (transmitData.throttle * 10.0f),
      (255.0f - transmitData.throttle) / 2.55f);
    Serial.printf("│ Yaw→Roll:     %3d → Drone: %4d µs\n", 
      transmitData.yaw,
      2000 - (transmitData.yaw * 10.0f));
    Serial.printf("│ Pitch:        %3d → Drone: %4d µs\n", 
      transmitData.pitch,
      1000 + (transmitData.pitch * 10.0f));
    Serial.printf("│ Roll→Yaw:     %3d → Drone: %4d µs\n", 
      transmitData.roll,
      2000 - (transmitData.roll * 10.0f));
    Serial.printf("│ AUX1:         %3d → Drone: %4d µs\n", 
      transmitData.AUX1,
      1000 + (transmitData.AUX1 * 10.0f));
    Serial.printf("│ AUX2:         %3d → Drone: %4d µs\n", 
      transmitData.AUX2,
      1000 + (transmitData.AUX2 * 10.0f));
    Serial.println("├─ RADIO STATUS ────────────────────────────────┤");
    Serial.printf("│ Module:       %s\n", 
      radio.isChipConnected() ? "✓ Connected" : "✗ DISCONNECTED");
    Serial.printf("│ TX Address:   ");
    printAddress(TX_ADDRESS);
    Serial.println("└────────────────────────────────────────────────┘\n");
  }
}


// Helper function to print address in hex format
void printAddress(const uint8_t addr[5]) {
  for (int i = 0; i < 5; i++) {
    if (addr[i] < 16) Serial.print("0");
    Serial.print(addr[i], HEX);
    if (i < 4) Serial.print(" ");
  }
  Serial.println();
}

// ============================================================================
// LED STATUS INDICATOR
// ============================================================================
// Status indicators:
//   - Blink (50ms pulse):  Successful transmission on GPIO 8 (inbuilt LED)
//   - Steady ON:           Radio connected (between packets)
//   - OFF:                 Radio disconnected / failed
void update_led_status() {
  unsigned long now = millis();
  
  // If radio not connected, turn LED off
  if (!radio.isChipConnected()) {
    digitalWrite(LED_PIN, LOW);
    return;
  }
  
  // Handle LED blink for successful transmission (50ms pulse)
  if (now - lastLedBlink < LED_BLINK_DURATION) {
    // Blink is active
    digitalWrite(LED_PIN, HIGH);
  } else {
    // Between blinks, keep LED on to show radio is connected
    digitalWrite(LED_PIN, HIGH);
  }
}
