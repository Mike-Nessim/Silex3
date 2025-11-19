#!/usr/bin/env python3
"""
Diagnostic script to test Nextion display communication.
Tests basic communication and tries different field names.
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
    print(f"Sent: {command_str}")
    
    time.sleep(0.3)
    
    if port.in_waiting > 0:
        response = port.read(port.in_waiting)
        print(f"Response: {response.hex()}")
        return True
    else:
        print("No response")
        return False

def main():
    print("Nextion Display Diagnostic Tool")
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
        
        # Test 1: Try to refresh the display
        print("Test 1: Refreshing display (ref)...")
        send_command(ser, "ref t0")
        time.sleep(0.5)
        
        # Test 2: Try different field names
        print("\nTest 2: Trying different field names...")
        field_names = ['t0', 't1', 't2', 'txt0', 'txt1', 'text0', 'text1']
        for field in field_names:
            print(f"\nTrying field: {field}")
            send_command(ser, f'{field}.txt="Test {field}"')
            time.sleep(0.5)
        
        # Test 3: Try setting page (some displays need this)
        print("\nTest 3: Setting page 0...")
        send_command(ser, "page 0")
        time.sleep(0.5)
        
        # Test 4: Try the original command again
        print("\nTest 4: Original command...")
        send_command(ser, 't0.txt="Axton Robotics"')
        time.sleep(0.5)
        
        # Test 5: Try without quotes (some Nextion versions)
        print("\nTest 5: Command without quotes...")
        send_command(ser, 't0.txt=Axton Robotics')
        time.sleep(0.5)
        
        ser.close()
        print("\n" + "=" * 50)
        print("Diagnostics complete. Check your display.")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()

