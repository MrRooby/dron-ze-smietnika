#!/bin/bash
# Quick Testing Script for STM8S005 Drone UART Communication
# Usage: ./test_uart.sh

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${YELLOW}STM8S005 Drone UART Test Script${NC}"
echo "=================================="

# Configuration
SERIAL_PORT="/dev/ttyUSB0"
BAUD_RATE="115200"

# Check if serial port exists
if [ ! -e "$SERIAL_PORT" ]; then
    echo -e "${RED}ERROR: Serial port $SERIAL_PORT not found${NC}"
    echo "Available serial ports:"
    ls /dev/ttyUSB* 2>/dev/null || ls /dev/ttyACM* 2>/dev/null || echo "None found"
    exit 1
fi

echo -e "${GREEN}Serial port: $SERIAL_PORT${NC}"
echo ""

# Function to send packet
send_packet() {
    local desc="$1"
    local packet="$2"
    echo -e "${YELLOW}Test: $desc${NC}"
    echo -e "  Packet: $packet"
    echo -ne "$packet\n" > "$SERIAL_PORT"
    echo "  Sent!"
    sleep 0.2
}

# Test 1: All motors off
send_packet "All Motors OFF" "O1:00-O2:00-B1:00-B2:00"

# Test 2: All motors at 50%
send_packet "All Motors at 50%" "O1:7F-O2:7F-B1:7F-B2:7F"

# Test 3: Front/Back at 75%, Left/Right at 50%
send_packet "Front/Back at 75%, L/R at 50%" "O1:C0-O2:C0-B1:80-B2:80"

# Test 4: Ramp test - increasing speed
for i in {0..8}; do
    val=$(printf "%02X" $((i * 32)))
    send_packet "Ramp $((i * 32)) -> $((i * 32))" "O1:$val-O2:$val-B1:$val-B2:$val"
done

# Test 5: Individual motor test
send_packet "Test Front Motor (O1) only" "O1:80-O2:00-B1:00-B2:00"
sleep 1

send_packet "Test Back Motor (O2) only" "O1:00-O2:80-B1:00-B2:00"
sleep 1

send_packet "Test Left Motor (B1) only" "O1:00-O2:00-B1:80-B2:00"
sleep 1

send_packet "Test Right Motor (B2) only" "O1:00-O2:00-B1:00-B2:80"
sleep 1

# Final: Safe shutdown
send_packet "Safe Shutdown - All Motors OFF" "O1:00-O2:00-B1:00-B2:00"

echo -e "${GREEN}Test sequence completed!${NC}"
