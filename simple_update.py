#!/usr/bin/env python3
"""
Simple script to update Nextion text field t0.
Sends the command multiple times to ensure it's received.
"""

import serial
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

def send_command(ser, cmd):
    """Send command with proper termination"""
    cmd_bytes = cmd.encode('utf-8') + bytes([0xFF, 0xFF, 0xFF])
    ser.write(cmd_bytes)
    ser.flush()

try:
    print("Opening serial port...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(1)  # Wait for connection
    
    print("Sending command 5 times with delays...")
    
    # Send the command multiple times
    for i in range(5):
        print(f"  Attempt {i+1}/5: t0.txt=\"Axton Robotics\"")
        send_command(ser, 't0.txt="Axton Robotics"')
        time.sleep(0.5)  # Wait between sends
    
    ser.close()
    print("\nDone! Check your display.")
    print("\nIf it still doesn't work, the most common issue is swapped TX/RX wires.")
    print("Try swapping the TX and RX connections between the USB-TTL and Nextion.")
    
except Exception as e:
    print(f"Error: {e}")

