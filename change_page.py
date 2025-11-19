#!/usr/bin/env python3
"""
Simple script to change the Nextion display page.
"""

import serial
import sys
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

def send_command(port, command_str):
    """Send a command to Nextion display"""
    command_bytes = command_str.encode('utf-8')
    termination = bytes([0xFF, 0xFF, 0xFF])
    full_command = command_bytes + termination
    
    port.reset_input_buffer()
    port.reset_output_buffer()
    time.sleep(0.1)
    
    port.write(full_command)
    port.flush()
    print(f"Sent command: {command_str}")
    
    time.sleep(0.3)
    
    if port.in_waiting > 0:
        response = port.read(port.in_waiting)
        print(f"Response: {response.hex()}")
    else:
        print("No response (this is normal for page commands)")

def main():
    if len(sys.argv) > 1:
        page_num = sys.argv[1]
    else:
        page_num = "2"  # Default to page 2
    
    print(f"Changing Nextion display to page {page_num}...")
    print("=" * 50)
    
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
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud\n")
        
        send_command(ser, f"page {page_num}")
        
        ser.close()
        print("\n" + "=" * 50)
        print(f"Page change command sent. Display should now show page {page_num}.")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        if 'ser' in locals():
            ser.close()
        sys.exit(1)

if __name__ == '__main__':
    main()

