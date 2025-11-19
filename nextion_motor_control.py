#!/usr/bin/env python3
"""
Nextion Display + Motor Control Integration

Monitors Nextion display button presses and controls the motor:
- START button -> Start motor jogging
- STOP button -> Stop motor
"""

import serial
import sys
import time
import re
from motor_api import connect, disconnect, enable_motor, move_motor, stop_motor, get_motor_position

# Configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
TIMEOUT = 1
MOTOR_NODE_ID = 5
JOG_SPEED = 1000  # Adjust as needed
POSITION_UPDATE_INTERVAL = 0.5  # Update position twice per second (0.5 seconds)

def parse_nextion_data(data):
    """Parse incoming data from Nextion display"""
    try:
        text = data.decode('utf-8', errors='ignore')
    except:
        return None
    
    # Look for button patterns
    button_match = re.search(r'BTN:(\w+)', text)
    if button_match:
        return button_match.group(1)  # Return the button name
    
    return None

def update_nextion_position(port, position):
    """
    Update Nextion display text field t0 with position value.
    
    Args:
        port: Serial port object
        position: Position value to display
    """
    try:
        # Nextion command format: t0.txt="value" followed by 0xFF 0xFF 0xFF
        command = f't0.txt="{position}"'
        command_bytes = command.encode('utf-8')
        termination = bytes([0xFF, 0xFF, 0xFF])
        full_command = command_bytes + termination
        
        port.reset_output_buffer()
        port.write(full_command)
        port.flush()
        return True
    except Exception as e:
        print(f"  Warning: Failed to update Nextion display: {e}")
        return False

def main():
    """Main control loop"""
    print("Nextion + Motor Control Integration")
    print("=" * 60)
    print(f"Nextion: {SERIAL_PORT} at {BAUD_RATE} baud")
    print(f"Motor: Node ID {MOTOR_NODE_ID}, Jog Speed: {JOG_SPEED}")
    print("=" * 60)
    print()
    
    # Connect to motor gateway
    print("Connecting to motor gateway...")
    success, error = connect()
    if not success:
        print(f"✗ Failed to connect to gateway: {error}")
        print("\nTroubleshooting:")
        print("1. Check gateway is powered on")
        print("2. Run: ./setup_network.sh")
        print("3. Verify gateway IP: 192.168.1.254")
        sys.exit(1)
    print("✓ Connected to gateway")
    
    # Enable motor
    print(f"Enabling motor (node ID {MOTOR_NODE_ID})...")
    success, error = enable_motor(MOTOR_NODE_ID, True)
    if not success:
        print(f"✗ Failed to enable motor: {error}")
        disconnect()
        sys.exit(1)
    print("✓ Motor enabled")
    
    # Connect to Nextion display
    print(f"\nConnecting to Nextion display at {SERIAL_PORT}...")
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        time.sleep(0.5)
        ser.reset_input_buffer()
        print("✓ Connected to Nextion display")
    except Exception as e:
        print(f"✗ Failed to connect to Nextion: {e}")
        print("\nTroubleshooting:")
        print(f"1. Check if {SERIAL_PORT} exists")
        print("2. Verify USB-TTL adapter is connected")
        print("3. Check wiring (TX/RX may be swapped)")
        stop_motor(MOTOR_NODE_ID)
        enable_motor(MOTOR_NODE_ID, False)
        disconnect()
        sys.exit(1)
    
    print("\n" + "=" * 60)
    print("Ready! Waiting for button presses...")
    print("  START button -> Start motor jogging")
    print("  STOP button  -> Stop motor")
    print("  Press Ctrl+C to exit")
    print("=" * 60)
    print()
    
    buffer = b''
    motor_running = False
    last_position_update = 0
    
    try:
        while True:
            current_time = time.time()
            
            # Update position display if motor is running and enough time has passed
            if motor_running and (current_time - last_position_update) >= POSITION_UPDATE_INTERVAL:
                success, position, error = get_motor_position(MOTOR_NODE_ID)
                if success and position is not None:
                    update_nextion_position(ser, position)
                    last_position_update = current_time
                # Silently handle errors to avoid spam - position updates will just be skipped
            
            # Read available data from Nextion
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                buffer += data
                
                # Look for newline (0x0A) which indicates end of message
                while b'\n' in buffer or b'\x0A' in buffer:
                    # Find the position of newline
                    if b'\n' in buffer:
                        idx = buffer.index(b'\n')
                    else:
                        idx = buffer.index(b'\x0A')
                    
                    # Extract complete message
                    message = buffer[:idx]
                    buffer = buffer[idx+1:]
                    
                    # Parse and handle the message
                    if message:
                        button = parse_nextion_data(message)
                        if button:
                            timestamp = time.strftime("%H:%M:%S")
                            
                            if button.upper() == 'START':
                                if not motor_running:
                                    print(f"[{timestamp}] START button pressed - Starting motor...")
                                    success, error = move_motor(MOTOR_NODE_ID, jog_speed=JOG_SPEED)
                                    if success:
                                        motor_running = True
                                        last_position_update = 0  # Reset to update immediately
                                        print(f"  ✓ Motor jogging at speed {JOG_SPEED}")
                                        # Get initial position and update display
                                        success, position, error = get_motor_position(MOTOR_NODE_ID)
                                        if success and position is not None:
                                            update_nextion_position(ser, position)
                                    else:
                                        print(f"  ✗ Failed to start motor: {error}")
                                else:
                                    print(f"[{timestamp}] START button pressed - Motor already running")
                            
                            elif button.upper() == 'STOP':
                                if motor_running:
                                    print(f"[{timestamp}] STOP button pressed - Stopping motor...")
                                    success, error = stop_motor(MOTOR_NODE_ID)
                                    if success:
                                        motor_running = False
                                        # Get final position and update display
                                        success, position, error = get_motor_position(MOTOR_NODE_ID)
                                        if success and position is not None:
                                            update_nextion_position(ser, position)
                                        print("  ✓ Motor stopped")
                                    else:
                                        print(f"  ✗ Failed to stop motor: {error}")
                                else:
                                    print(f"[{timestamp}] STOP button pressed - Motor not running")
                            
                            else:
                                print(f"[{timestamp}] Unknown button: {button}")
            else:
                # Small delay to avoid CPU spinning
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup
        print("\nCleaning up...")
        if motor_running:
            stop_motor(MOTOR_NODE_ID)
        enable_motor(MOTOR_NODE_ID, False)
        disconnect()
        if ser.is_open:
            ser.close()
        print("✓ Disconnected and cleaned up")


if __name__ == '__main__':
    main()

