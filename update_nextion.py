#!/usr/bin/env python3
"""
Script to update Nextion display text field via serial communication.
Updates text field t0 to "Axton Robotics"
"""

import serial
import sys
import time

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600  # Common Nextion baud rate, adjust if needed
TIMEOUT = 1

def update_nextion_text(port, field_name, text):
    """
    Update a Nextion text field.
    
    Args:
        port: Serial port object
        field_name: Name of the text field (e.g., 't0')
        text: Text to display
    """
    # Nextion command format: field_name.txt="text" followed by 0xFF 0xFF 0xFF
    command = f'{field_name}.txt="{text}"'
    
    # Convert command to bytes and append termination bytes
    command_bytes = command.encode('utf-8')
    termination = bytes([0xFF, 0xFF, 0xFF])
    full_command = command_bytes + termination
    
    try:
        # Send command
        port.write(full_command)
        print(f"Sent command: {command}")
        
        # Wait a bit for the display to process
        time.sleep(0.1)
        
        # Try to read response (Nextion may send acknowledgment)
        if port.in_waiting > 0:
            response = port.read(port.in_waiting)
            print(f"Response: {response}")
        
        print("Text field updated successfully!")
        return True
        
    except Exception as e:
        print(f"Error updating display: {e}")
        return False

def main():
    """Main function"""
    field_name = 't0'
    text = 'Axton Robotics'
    
    try:
        # Open serial port
        print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
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
        
        print(f"Connected! Updating {field_name} to '{text}'...")
        
        # Update the text field
        success = update_nextion_text(ser, field_name, text)
        
        # Close serial port
        ser.close()
        print("Connection closed.")
        
        sys.exit(0 if success else 1)
        
    except serial.SerialException as e:
        print(f"Serial port error: {e}")
        print(f"\nTroubleshooting:")
        print(f"1. Check if {SERIAL_PORT} exists: ls -la {SERIAL_PORT}")
        print(f"2. Check permissions: sudo usermod -a -G dialout $USER")
        print(f"3. Try running with sudo if needed")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"Unexpected error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()

