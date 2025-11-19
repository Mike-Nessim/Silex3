#!/usr/bin/env python3
"""
Test script to verify Nextion display connection and communication.
"""

import serial
import sys
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

def main():
    print("Nextion Connection Test")
    print("=" * 60)
    
    try:
        ser = serial.Serial(
            port=SERIAL_PORT,
            baudrate=BAUD_RATE,
            timeout=2,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        
        time.sleep(1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud\n")
        
        # Clear any existing data
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        time.sleep(0.5)
        
        # Check if display is sending any data
        print("1. Checking for incoming data from display...")
        time.sleep(1)
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print(f"   Received: {data.hex()}")
            print(f"   Display is sending data - connection is working!")
        else:
            print("   No data received (this is normal if display isn't sending)")
        
        # Test 2: Send a simple command and check response
        print("\n2. Sending test command: 't0.txt=\"TEST\"'...")
        command = 't0.txt="TEST"'
        command_bytes = command.encode('utf-8') + bytes([0xFF, 0xFF, 0xFF])
        
        ser.write(command_bytes)
        ser.flush()
        print(f"   Sent: {command}")
        print(f"   Bytes: {command_bytes.hex()}")
        
        time.sleep(0.5)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"   Response: {response.hex()}")
        else:
            print("   No response (display may not send acknowledgments)")
        
        # Test 3: Try reading display info
        print("\n3. Attempting to read display information...")
        # Send "connect" command (some Nextion displays respond to this)
        connect_cmd = b'connect' + bytes([0xFF, 0xFF, 0xFF])
        ser.write(connect_cmd)
        ser.flush()
        time.sleep(0.5)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"   Response: {response.hex()}")
            print(f"   Display responded!")
        else:
            print("   No response to connect command")
        
        # Test 4: Try the actual command again with more delay
        print("\n4. Sending final command: 't0.txt=\"Axton Robotics\"'...")
        command = 't0.txt="Axton Robotics"'
        command_bytes = command.encode('utf-8') + bytes([0xFF, 0xFF, 0xFF])
        
        ser.write(command_bytes)
        ser.flush()
        print(f"   Sent: {command}")
        print(f"   Waiting 1 second for display to process...")
        time.sleep(1)
        
        if ser.in_waiting > 0:
            response = ser.read(ser.in_waiting)
            print(f"   Response: {response.hex()}")
        
        ser.close()
        
        print("\n" + "=" * 60)
        print("Test complete!")
        print("\nIf the display still doesn't update:")
        print("1. Verify wiring:")
        print("   - USB-TTL TX pin -> Nextion RX pin")
        print("   - USB-TTL RX pin -> Nextion TX pin")
        print("   - USB-TTL GND -> Nextion GND")
        print("2. Check that Nextion is powered (5V)")
        print("3. Try swapping TX/RX wires (common issue)")
        print("4. Power cycle the Nextion display")
        
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()

