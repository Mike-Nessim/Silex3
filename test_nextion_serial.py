#!/usr/bin/env python3
"""
Test script to print raw serial messages from Nextion display.
This will help us see exactly what data is being sent.
"""

import serial
import sys
import time

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 9600

def main():
    print("Nextion Serial Raw Data Monitor")
    print("=" * 50)
    print(f"Listening on {SERIAL_PORT} at {BAUD_RATE} baud")
    print("Press buttons on the Nextion display to see raw data")
    print("Press Ctrl+C to exit\n")
    print("=" * 50)
    print()
    
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
        print("Connected! Waiting for data...")
        print("Press the SAME button multiple times to check consistency\n")
        
        buffer = bytearray()
        event_count = 0
        last_complete_message = None
        message_history = []
        last_data_time = None
        MESSAGE_TIMEOUT = 0.1  # Consider message complete after 100ms of silence
        
        def process_complete_message(msg_bytes, source="unknown"):
            nonlocal event_count, last_complete_message, message_history
            event_count += 1
            timestamp = time.time()
            
            # Check if message has termination bytes
            has_termination = len(msg_bytes) >= 3 and msg_bytes[-3:] == bytes([0xFF, 0xFF, 0xFF])
            if has_termination:
                data_part = msg_bytes[:-3]
            else:
                data_part = msg_bytes
            
            # Show complete message
            print(f"\n{'='*60}")
            print(f"COMPLETE EVENT #{event_count} ({source})")
            print(f"{'='*60}")
            
            ascii_text = ''.join([chr(b) if 32 <= b < 127 else f'\\x{b:02x}' for b in data_part])
            print(f"TEXT:       '{ascii_text}'")
            print(f"Hex:        {data_part.hex()}")
            if has_termination:
                print(f"Full Hex:   {msg_bytes.hex()} (with 0xFF 0xFF 0xFF termination)")
            print(f"Length:     {len(data_part)} bytes")
            
            # Compare with last message
            if last_complete_message is not None:
                if msg_bytes == last_complete_message:
                    print(f"✓ MATCHES previous event #{event_count - 1}")
                else:
                    print(f"✗ DIFFERENT from previous event #{event_count - 1}")
                    prev_data = last_complete_message[:-3] if len(last_complete_message) >= 3 and last_complete_message[-3:] == bytes([0xFF, 0xFF, 0xFF]) else last_complete_message
                    print(f"  Previous: {prev_data.hex()} ('{''.join([chr(b) if 32 <= b < 127 else '.' for b in prev_data])}')")
                    print(f"  Current:  {data_part.hex()} ('{ascii_text}')")
            
            # Show individual bytes
            if len(data_part) > 0:
                print(f"\nBytes breakdown:")
                for i, byte in enumerate(data_part):
                    char = chr(byte) if 32 <= byte < 127 else '.'
                    print(f"  [{i:2d}] 0x{byte:02X} ({byte:3d}) '{char}'")
            
            # Store for comparison
            last_complete_message = msg_bytes
            message_history.append((event_count, msg_bytes, timestamp))
            print()
        
        while True:
            current_time = time.time()
            
            if ser.in_waiting > 0:
                # Read available data
                data = ser.read(ser.in_waiting)
                timestamp = current_time
                last_data_time = current_time
                
                # Show raw bytes received
                print(f"\n[RAW] Received {len(data)} bytes at {timestamp:.3f}")
                print(f"      Hex: {data.hex()}")
                ascii_raw = ''.join([chr(b) if 32 <= b < 127 else '.' for b in data])
                print(f"      Text: '{ascii_raw}'")
                
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
                        process_complete_message(message, "Termination bytes found")
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
                        process_complete_message(message, "Timeout (no data for 100ms)")
                
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        # Process any remaining buffer as final message
        if len(buffer) > 0:
            message = bytes(buffer)
            process_complete_message(message, "Final buffer on exit")
        
        print("\n\n" + "=" * 60)
        print("SUMMARY")
        print("=" * 60)
        print(f"Total complete events captured: {event_count}")
        
        if message_history:
            print(f"\nAll messages received:")
            unique_messages = {}
            for event_num, msg, ts in message_history:
                msg_hex = msg.hex()
                if msg_hex not in unique_messages:
                    unique_messages[msg_hex] = []
                unique_messages[msg_hex].append(event_num)
            
            print(f"\nUnique message patterns: {len(unique_messages)}")
            for i, (msg_hex, event_nums) in enumerate(unique_messages.items(), 1):
                msg_bytes = bytes.fromhex(msg_hex)
                # Check if it has termination bytes
                if len(msg_bytes) >= 3 and msg_bytes[-3:] == bytes([0xFF, 0xFF, 0xFF]):
                    data_part = msg_bytes[:-3]
                else:
                    data_part = msg_bytes
                ascii_text = ''.join([chr(b) if 32 <= b < 127 else f'\\x{b:02x}' for b in data_part])
                print(f"  Pattern #{i}: '{ascii_text}' | Hex: {data_part.hex()}")
                print(f"    Seen in events: {event_nums} ({len(event_nums)} times)")
        
        print("\nShutting down...")
        ser.close()
        sys.exit(0)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        if 'ser' in locals():
            ser.close()
        sys.exit(1)

if __name__ == "__main__":
    main()

