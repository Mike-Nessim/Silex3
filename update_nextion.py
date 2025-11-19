#!/usr/bin/env python3
"""
Script to update Nextion display text field via serial communication.
Updates text field t0 to "Axton Robotics"
For Nextion NX8048P070-011C-Y display
"""

import serial
import sys
import time

# Serial port configuration
SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600
TIMEOUT = 1

def send_nextion_command(port, command_str):
    """
    Send a command to Nextion display with proper termination.
    
    Args:
        port: Serial port object
        command_str: Command string (without termination bytes)
    """
    # Convert command to bytes and append termination bytes (0xFF 0xFF 0xFF)
    command_bytes = command_str.encode('utf-8')
    termination = bytes([0xFF, 0xFF, 0xFF])
    full_command = command_bytes + termination
    
    # Clear buffers
    port.reset_input_buffer()
    port.reset_output_buffer()
    time.sleep(0.05)
    
    # Send command
    bytes_written = port.write(full_command)
    port.flush()  # Ensure data is sent immediately
    
    # Wait for processing
    time.sleep(0.15)
    
    # Check for response
    if port.in_waiting > 0:
        response = port.read(port.in_waiting)
        return response
    return None

def update_nextion_text(port, field_name, text):
    """
    Update a Nextion text field.
    
    Args:
        port: Serial port object
        field_name: Name of the text field (e.g., 't0')
        text: Text to display
    """
    try:
        # Method 1: Standard command format
        print(f"Method 1: Standard format...")
        command = f'{field_name}.txt="{text}"'
        print(f"  Sending: {command}")
        response = send_nextion_command(port, command)
        if response:
            print(f"  Response: {response.hex()}")
        time.sleep(0.2)
        
        # Method 2: Try with page 0 first (some displays need this)
        print(f"\nMethod 2: Setting page 0, then updating text...")
        send_nextion_command(port, "page 0")
        time.sleep(0.1)
        command = f'{field_name}.txt="{text}"'
        print(f"  Sending: {command}")
        response = send_nextion_command(port, command)
        if response:
            print(f"  Response: {response.hex()}")
        time.sleep(0.2)
        
        # Method 3: Try refresh command after update
        print(f"\nMethod 3: Update then refresh...")
        command = f'{field_name}.txt="{text}"'
        send_nextion_command(port, command)
        time.sleep(0.1)
        send_nextion_command(port, f"ref {field_name}")
        print(f"  Sent update + refresh")
        time.sleep(0.2)
        
        # Method 4: Try without quotes (some Nextion firmware versions)
        print(f"\nMethod 4: Without quotes...")
        command = f'{field_name}.txt={text}'
        print(f"  Sending: {command}")
        response = send_nextion_command(port, command)
        if response:
            print(f"  Response: {response.hex()}")
        time.sleep(0.2)
        
        print("\n✓ All methods attempted. Check your display.")
        return True
        
    except Exception as e:
        print(f"Error updating display: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Main function"""
    field_name = 't0'
    text = 'Axton Robotics'
    
    try:
        # Open serial port
        print(f"Connecting to {SERIAL_PORT} at {BAUD_RATE} baud...")
        print(f"Display: NX8048P070-011C-Y")
        print(f"Field: {field_name}")
        print(f"Text: '{text}'")
        print("=" * 60)
        
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
        
        print("Connected! Attempting to update text field...\n")
        
        # Update the text field
        success = update_nextion_text(ser, field_name, text)
        
        # Close serial port
        ser.close()
        print("\n" + "=" * 60)
        print("Connection closed.")
        
        if success:
            print("\n✓ Commands sent successfully!")
            print("If the display still shows 'newtxt', check:")
            print("1. Wiring: TX->RX, RX->TX, GND->GND")
            print("2. Try power cycling the display")
            print("3. Verify the field name in Nextion Editor")
        
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
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
