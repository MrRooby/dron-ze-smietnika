#ifndef NRF24_INTERFACE_H
#define NRF24_INTERFACE_H

#include "config.h"
#include "pins.h"
#include "types.h"

// ============================================================================
// NRF24 RADIO INTERFACE MODULE
// ============================================================================
// Handles SPI communication with NRF24L01 transceiver
// Receives RC commands from transmitter
// Implements failsafe on signal loss
// ============================================================================

/**
 * @brief Initialize NRF24L01 radio module
 *        - Setup SPI interface with custom pins
 *        - Configure radio parameters (1Mbps, channel 32, 6-byte payload)
 *        - Open reading pipe (Pipe 0) for RC commands
 *        - Enable auto-acknowledge
 * @return 1 if initialized successfully, 0 if failed
 */
uint8_t nrf24_init();

/**
 * @brief Read RC commands from NRF24L01
 *        - Polls radio for new packets (Pipe 0)
 *        - Decodes 6-byte payload into RC channels
 *        - Direct 0-255 values (no PWM mapping here)
 *        - Updates signal timestamp
 * @param packet Pointer to RF24_Packet struct to fill
 * @return 1 if new packet received, 0 if no data available
 */
uint8_t nrf24_read_commands(RF24_Packet *packet);

/**
 * @brief Check if radio module is connected
 * @return 1 if chip is connected, 0 otherwise
 */
uint8_t nrf24_is_connected();

/**
 * @brief Get time since last packet received (milliseconds)
 * @return Time elapsed in ms
 */
uint32_t nrf24_get_last_packet_ms();

#endif // NRF24_INTERFACE_H
