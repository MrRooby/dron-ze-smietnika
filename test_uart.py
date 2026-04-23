#!/usr/bin/env python3
"""
STM8S005 Drone UART Communication Test Tool
Sends rotor PWM control packets via UART
"""

import serial
import time
import sys
import argparse

# ANSI color codes
class Colors:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    END = '\033[0m'

class DroneUART:
    """Interface for communicating with STM8S005 drone via UART"""
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
        """Initialize UART connection"""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            print(f"{Colors.GREEN}✓ Connected to {port} at {baudrate} baud{Colors.END}")
        except serial.SerialException as e:
            print(f"{Colors.RED}✗ Failed to connect: {e}{Colors.END}")
            sys.exit(1)
    
    def send_rotor_command(self, o1, o2, b1, b2, verbose=True):
        """
        Send rotor PWM command
        
        Args:
            o1 (int): Front rotor PWM (0-255)
            o2 (int): Back rotor PWM (0-255)
            b1 (int): Left rotor PWM (0-255)
            b2 (int): Right rotor PWM (0-255)
            verbose (bool): Print command details
        """
        # Clamp values to 0-255
        o1 = max(0, min(255, o1))
        o2 = max(0, min(255, o2))
        b1 = max(0, min(255, b1))
        b2 = max(0, min(255, b2))
        
        # Format packet
        packet = f"O1:{o1:02X}-O2:{o2:02X}-B1:{b1:02X}-B2:{b2:02X}\n"
        
        if verbose:
            print(f"{Colors.BLUE}→ Sending:{Colors.END} O1={o1:3d} O2={o2:3d} B1={b1:3d} B2={b2:3d}")
            print(f"  {Colors.YELLOW}Raw: {packet.strip()}{Colors.END}")
        
        self.ser.write(packet.encode())
        time.sleep(0.01)  # Small delay for processing
    
    def close(self):
        """Close serial connection"""
        self.ser.close()
        print(f"{Colors.GREEN}Connection closed{Colors.END}")

def test_all_off(drone):
    """Test: All motors off"""
    print(f"\n{Colors.BLUE}=== Test 1: All Motors OFF ==={Colors.END}")
    drone.send_rotor_command(0, 0, 0, 0)
    time.sleep(0.5)

def test_all_half_speed(drone):
    """Test: All motors at 50% speed"""
    print(f"\n{Colors.BLUE}=== Test 2: All Motors at 50% ==={Colors.END}")
    drone.send_rotor_command(127, 127, 127, 127)
    time.sleep(0.5)

def test_all_full_speed(drone):
    """Test: All motors at full speed"""
    print(f"\n{Colors.BLUE}=== Test 3: All Motors at 100% ==={Colors.END}")
    drone.send_rotor_command(255, 255, 255, 255)
    time.sleep(0.5)

def test_individual_motors(drone):
    """Test: Each motor individually"""
    print(f"\n{Colors.BLUE}=== Test 4: Individual Motor Test ==={Colors.END}")
    
    motors = [
        ("Front (O1)", 200, 0, 0, 0),
        ("Back (O2)", 0, 200, 0, 0),
        ("Left (B1)", 0, 0, 200, 0),
        ("Right (B2)", 0, 0, 0, 200),
    ]
    
    for name, o1, o2, b1, b2 in motors:
        print(f"\n  Testing {name}:")
        drone.send_rotor_command(o1, o2, b1, b2)
        time.sleep(1)
    
    # All off
    drone.send_rotor_command(0, 0, 0, 0)

def test_ramp(drone):
    """Test: Gradual speed ramp"""
    print(f"\n{Colors.BLUE}=== Test 5: Speed Ramp (0% to 100%) ==={Colors.END}")
    
    for speed in range(0, 256, 32):
        print(f"\n  Speed: {speed:3d}/255 ({speed*100//255:3d}%)")
        drone.send_rotor_command(speed, speed, speed, speed, verbose=False)
        time.sleep(0.2)
    
    # Return to off
    drone.send_rotor_command(0, 0, 0, 0)

def test_asymmetric(drone):
    """Test: Asymmetric commands (different motor speeds)"""
    print(f"\n{Colors.BLUE}=== Test 6: Asymmetric Control ==={Colors.END}")
    
    patterns = [
        ("Front heavy", 200, 100, 50, 50),
        ("Back heavy", 100, 200, 50, 50),
        ("Left turn (high right)", 100, 100, 50, 200),
        ("Right turn (high left)", 100, 100, 200, 50),
    ]
    
    for name, o1, o2, b1, b2 in patterns:
        print(f"\n  {name}:")
        drone.send_rotor_command(o1, o2, b1, b2)
        time.sleep(1)
    
    drone.send_rotor_command(0, 0, 0, 0)

def test_continuous_stream(drone, duration=10):
    """Test: Continuous packet stream"""
    print(f"\n{Colors.BLUE}=== Test 7: Continuous Packet Stream ({duration}s) ==={Colors.END}")
    
    start = time.time()
    packet_count = 0
    
    # Pulsing pattern
    while time.time() - start < duration:
        # Create pulsing effect
        pulse = int(127.5 + 127.5 * (time.time() % 2 - 0.5) / 0.5)
        drone.send_rotor_command(pulse, pulse, pulse, pulse, verbose=False)
        packet_count += 1
    
    print(f"  Sent {packet_count} packets in {duration}s ({packet_count/duration:.1f} pkt/s)")
    drone.send_rotor_command(0, 0, 0, 0)

def main():
    parser = argparse.ArgumentParser(description='STM8S005 Drone UART Test Tool')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    parser.add_argument('--test', type=int, default=0, help='Run specific test (1-7), 0=all')
    parser.add_argument('--o1', type=int, help='Send custom command: O1 value (0-255)')
    parser.add_argument('--o2', type=int, help='O2 value (0-255)')
    parser.add_argument('--b1', type=int, help='B1 value (0-255)')
    parser.add_argument('--b2', type=int, help='B2 value (0-255)')
    
    args = parser.parse_args()
    
    drone = DroneUART(port=args.port, baudrate=args.baud)
    
    try:
        # Custom command
        if args.o1 is not None:
            o1 = args.o1 if args.o1 is not None else 0
            o2 = args.o2 if args.o2 is not None else 0
            b1 = args.b1 if args.b1 is not None else 0
            b2 = args.b2 if args.b2 is not None else 0
            print(f"{Colors.BLUE}=== Custom Command ==={Colors.END}")
            drone.send_rotor_command(o1, o2, b1, b2)
        
        # Run tests
        elif args.test == 0:
            # Run all tests
            test_all_off(drone)
            test_all_half_speed(drone)
            test_all_full_speed(drone)
            test_individual_motors(drone)
            test_ramp(drone)
            test_asymmetric(drone)
            test_continuous_stream(drone, duration=5)
        elif args.test == 1:
            test_all_off(drone)
        elif args.test == 2:
            test_all_half_speed(drone)
        elif args.test == 3:
            test_all_full_speed(drone)
        elif args.test == 4:
            test_individual_motors(drone)
        elif args.test == 5:
            test_ramp(drone)
        elif args.test == 6:
            test_asymmetric(drone)
        elif args.test == 7:
            test_continuous_stream(drone, duration=30)
        else:
            print(f"{Colors.RED}Invalid test number: {args.test}{Colors.END}")
        
        print(f"\n{Colors.GREEN}✓ All tests completed successfully{Colors.END}")
    
    except KeyboardInterrupt:
        print(f"\n{Colors.YELLOW}Test interrupted by user{Colors.END}")
        drone.send_rotor_command(0, 0, 0, 0)  # Safe shutdown
    
    except Exception as e:
        print(f"{Colors.RED}Error: {e}{Colors.END}")
    
    finally:
        drone.close()

if __name__ == '__main__':
    main()
