#!/usr/bin/env python3
"""
Main program file for Silex3
"""

import serial
import sys
import time
from motor_api import connect, disconnect, enable_motor, move_motor, stop_motor

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
PAGE_ID = 1  # Page 1
MESSAGE_TIMEOUT = 0.1  # Consider message complete after 100ms of silence

# Motor configuration
MOTOR_NODE_ID = 5  # CAN node ID of the motor
MANUAL_CLAMP_POSITION = 50000  # Target position for manual clamp
MANUAL_CLAMP_SPEED = 15000  # Speed in RPM for manual clamp (5x faster)
RESET_HOME_POSITION = 0  # Home position
RESET_HOME_SPEED = 15000  # Speed in RPM for reset home (5x faster)

# Map button messages to button names
BUTTON_MESSAGES = {
    'ESTOP': 'E-Stop',
    'MANCLAMP': 'Manual-Clamp',
    'RESETHOME': 'RESET-HOME',
}

def parse_message(data):
    """
    Parse Nextion message - can be text or binary.
    Returns: (message_text, has_termination) or (None, False) if invalid
    """
    if len(data) == 0:
        return None, False
    
    # Check for termination bytes (0xFF 0xFF 0xFF)
    has_termination = len(data) >= 3 and data[-3:] == bytes([0xFF, 0xFF, 0xFF])
    
    if has_termination:
        data_part = data[:-3]
    else:
        data_part = data
    
    # Try to decode as text
    try:
        message_text = data_part.decode('ascii')
        return message_text, has_termination
    except (UnicodeDecodeError, ValueError):
        # Not text, return hex representation
        return data_part.hex(), has_termination

def handle_button_click(button_name):
    """
    Handle button click actions.
    
    Args:
        button_name: Name of the button that was clicked
    """
    if button_name == 'E-Stop':
        # Emergency stop - stop motor immediately
        print("  → Emergency stop activated!")
        success, error = stop_motor(MOTOR_NODE_ID)
        if success:
            print("  ✓ Motor stopped")
        else:
            print(f"  ✗ Failed to stop motor: {error}")
    
    elif button_name == 'Manual-Clamp':
        # Move motor to position 50000 at 3000 RPM
        print(f"  → Moving motor to position {MANUAL_CLAMP_POSITION} at {MANUAL_CLAMP_SPEED} RPM...")
        success, error = move_motor(
            MOTOR_NODE_ID,
            position=MANUAL_CLAMP_POSITION,
            relative=False,
            speed=MANUAL_CLAMP_SPEED
        )
        if success:
            print(f"  ✓ Motor motion started to position {MANUAL_CLAMP_POSITION}")
        else:
            print(f"  ✗ Failed to start motion: {error}")
    
    elif button_name == 'RESET-HOME':
        # Move motor back to position 0
        print(f"  → Moving motor to home position {RESET_HOME_POSITION} at {RESET_HOME_SPEED} RPM...")
        success, error = move_motor(
            MOTOR_NODE_ID,
            position=RESET_HOME_POSITION,
            relative=False,
            speed=RESET_HOME_SPEED
        )
        if success:
            print(f"  ✓ Motor motion started to home position {RESET_HOME_POSITION}")
        else:
            print(f"  ✗ Failed to start motion: {error}")

def main():
    print("Silex3 Main Program")
    print("Listening for button clicks on Nextion display...")
    print("=" * 50)
    
    # Connect to motor gateway
    print("\nConnecting to motor gateway...")
    success, error = connect()
    if not success:
        print(f"✗ Failed to connect to motor gateway: {error}")
        print("Continuing without motor control...")
        motor_connected = False
    else:
        print("✓ Connected to motor gateway")
        motor_connected = True
        
        # Enable motor
        print("Enabling motor...")
        success, error = enable_motor(MOTOR_NODE_ID, True)
        if success:
            print("✓ Motor enabled")
        else:
            print(f"✗ Failed to enable motor: {error}")
            motor_connected = False
    
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        time.sleep(0.5)
        print(f"\nConnected to {SERIAL_PORT} at {BAUD_RATE} baud")
        print("\nWaiting for button clicks:")
        print("  - E-Stop (ESTOP) - Emergency stop motor")
        print("  - Manual-Clamp (MANCLAMP) - Move to position 50000")
        print("  - RESET-HOME (RESETHOME) - Move to position 0\n")
        
        buffer = bytearray()
        last_data_time = None
        
        while True:
            current_time = time.time()
            
            if ser.in_waiting > 0:
                # Read available data
                data = ser.read(ser.in_waiting)
                last_data_time = current_time
                
                buffer.extend(data)
                
                # First, check for termination sequence (0xFF 0xFF 0xFF)
                while len(buffer) >= 3:
                    term_pos = -1
                    for i in range(len(buffer) - 2):
                        if buffer[i:i+3] == bytes([0xFF, 0xFF, 0xFF]):
                            term_pos = i
                            break
                    
                    if term_pos != -1:
                        # Found termination, extract message
                        message = bytes(buffer[:term_pos + 3])
                        buffer = buffer[term_pos + 3:]
                        
                        # Parse and handle message
                        message_text, has_term = parse_message(message)
                        if message_text and message_text in BUTTON_MESSAGES:
                            button_name = BUTTON_MESSAGES[message_text]
                            print(f"Button clicked: {button_name}")
                            if motor_connected:
                                handle_button_click(button_name)
                            else:
                                print("  (Motor not connected - action skipped)")
                        else:
                            print(f"Received message: '{message_text}' (not mapped to a button)")
                    else:
                        break
            else:
                # No data available - check if we should timeout the buffer
                if last_data_time is not None and len(buffer) > 0:
                    time_since_last = current_time - last_data_time
                    if time_since_last >= MESSAGE_TIMEOUT:
                        # Timeout reached, consider buffer a complete message
                        message = bytes(buffer)
                        buffer = bytearray()
                        last_data_time = None
                        
                        # Parse and handle message
                        message_text, has_term = parse_message(message)
                        if message_text and message_text in BUTTON_MESSAGES:
                            button_name = BUTTON_MESSAGES[message_text]
                            print(f"Button clicked: {button_name}")
                            if motor_connected:
                                handle_button_click(button_name)
                            else:
                                print("  (Motor not connected - action skipped)")
                        else:
                            print(f"Received message: '{message_text}' (not mapped to a button)")
                
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\n\nShutting down...")
        
        # Cleanup motor
        if motor_connected:
            print("Stopping motor...")
            stop_motor(MOTOR_NODE_ID)
            print("Disabling motor...")
            enable_motor(MOTOR_NODE_ID, False)
            print("Disconnecting from motor gateway...")
            disconnect()
        
        if 'ser' in locals():
            ser.close()
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        
        # Cleanup motor on error
        if motor_connected:
            print("\nCleaning up motor...")
            stop_motor(MOTOR_NODE_ID)
            enable_motor(MOTOR_NODE_ID, False)
            disconnect()
        
        if 'ser' in locals():
            ser.close()
        sys.exit(1)

if __name__ == "__main__":
    main()

