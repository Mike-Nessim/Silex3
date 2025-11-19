#!/usr/bin/env python3
"""
Monitor Nextion display for button press events.
Listens for button press messages and prints when detected.
"""

import serial
import sys
import time
import re

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
TIMEOUT = 1

def parse_nextion_data(data):
    """
    Parse incoming data from Nextion display.
    Nextion sends button events like "BTN:START" followed by 0x0A (newline)
    """
    # Convert bytes to string, handling any encoding issues
    try:
        text = data.decode('utf-8', errors='ignore')
    except:
        return None
    
    # Look for button patterns
    # Pattern: "BTN:START" or "BTN:STOP" etc.
    button_match = re.search(r'BTN:(\w+)', text)
    if button_match:
        return button_match.group(1)  # Return the button name
    
    return None

def monitor_buttons():
    """Main monitoring loop"""
    print("Nextion Button Monitor")
    print("=" * 60)
    print(f"Monitoring {SERIAL_PORT} at {BAUD_RATE} baud")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    print()
    
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=TIMEOUT,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        # Wait for connection to stabilize
        time.sleep(0.5)
        ser.reset_input_buffer()
        
        print("Ready! Waiting for button presses...\n")
        
        buffer = b''  # Buffer for incomplete messages
        
        while True:
            # Read available data
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
                            print(f"[{timestamp}] Button pressed: {button}")
                        else:
                            # Print raw data for debugging
                            timestamp = time.strftime("%H:%M:%S")
                            print(f"[{timestamp}] Raw data: {message.hex()} ({message})")
            else:
                # Small delay to avoid CPU spinning
                time.sleep(0.01)
                
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print(f"\nTroubleshooting:")
        print(f"1. Check if {SERIAL_PORT} exists: ls -la {SERIAL_PORT}")
        print(f"2. Check permissions: sudo usermod -a -G dialout $USER")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nMonitoring stopped by user")
        if 'ser' in locals():
            ser.close()
        sys.exit(0)
    except Exception as e:
        print(f"Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        if 'ser' in locals():
            ser.close()
        sys.exit(1)

if __name__ == '__main__':
    monitor_buttons()

