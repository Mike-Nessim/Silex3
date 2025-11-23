#!/usr/bin/env python3
"""
Actuator Test Script

This script provides homing and clamping functionality for the actuator:
- Homing: Move until GPIO 10 goes high, then move away from sensor
- Clamping: Move until GPIO 9 goes high

Commands:
  h or H - Start homing sequence
  c or C - Start clamping sequence (after homing)
  s or S - Emergency stop motor
  q or Q - Quit program
"""

import sys
import time
import threading
import socket
import glob
import os
import re
from datetime import datetime
from motor_api import connect, disconnect, enable_motor, move_motor, stop_motor, get_motor_position, reset_position_to_zero
import motor_api

try:
    import serial
except ImportError:
    print("Error: pyserial library not found.")
    print("Install it with: pip install pyserial")
    sys.exit(1)

try:
    import RPi.GPIO as GPIO
except ImportError:
    print("Error: RPi.GPIO library not found.")
    print("Install it with: pip install RPi.GPIO")
    sys.exit(1)

# ============================================================================
# CONFIGURATION VARIABLES - Adjust these as needed
# ============================================================================

# Motor configuration
MOTOR_NODE_ID = 5
GATEWAY_IP = "192.168.1.254"
GATEWAY_PORT = 8888

# Serial configuration (for Nextion display)
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD_RATE = 921600
SERIAL_TIMEOUT = 1
MESSAGE_TIMEOUT = 0.05  # Consider message complete after 50ms of silence (reduced for higher baud rate)

# Motor speed configuration (unified for both homing and clamping)
MOTOR_BASE_SPEED = 12000  # Base speed for motor motion (jog mode - continuous motion)
# Speed multiplier controlled by MOTSPEED command (0-100): 0 -> 0.5x, 50 -> 1.0x, 100 -> 2.0x

# Homing configuration
HOMING_GPIO = 10  # GPIO pin for homing sensor
HOMING_DIRECTION = 1  # Direction for homing: 1 = positive, -1 = negative (change sign if wrong direction)
HOMING_MAX_DISTANCE = 70000000  # Maximum distance to travel before error (in position units)
HOMING_BACKOFF_DISTANCE = 350000  # Distance to move away from sensor after finding it (relative position)
HOMING_BACKOFF_DIRECTION = -1  # Direction for backoff: -1 to move away from sensor, +1 to move toward sensor
HOMING_BACKOFF_PTP_SPEED = 10000  # Speed for backoff motion (PTP mode)
# Note: If motor moves in wrong direction during homing, change HOMING_DIRECTION sign
# If backoff moves in wrong direction, change HOMING_BACKOFF_DIRECTION sign

# Clamping configuration
CLAMPING_GPIO = 9  # GPIO pin for clamping sensor
CLAMPING_DIRECTION = -1  # Direction for clamping: 1 = positive, -1 = negative (change sign if wrong direction)
CLAMPING_MAX_DISTANCE = 75000000  # Maximum distance to travel before error (in position units)
# Note: If motor moves in wrong direction during clamping, change CLAMPING_DIRECTION sign

# External control GPIO
CONTROL_GPIO = 26  # GPIO pin for external control: HIGH = clamp, LOW = home

# Position polling interval (seconds)
POSITION_POLL_INTERVAL = 0.1  # Check position every 100ms

# Motion timeout (seconds)
MOTION_TIMEOUT = 60.0  # Maximum time to wait for motion to complete (safety timeout)

# ============================================================================
# Global state variables
# ============================================================================

motor_connected = False
stop_requested = False
command_queue = []
input_thread_running = False
homing_complete = False  # Track if homing has been completed (zero point established)
serial_thread_running = False
serial_port_obj = None
_sdk_instance = None
_gateway_info = None
motor_speed_multiplier = 1.0  # Multiplier for motor speed (1.0 = base speed, range: 0.5 to 2.0)
motspeed_buffer = ""  # Buffer for accumulating MOTSPEED messages
motspeed_buffer_time = None  # Timestamp for MOTSPEED buffer

def get_motor_velocity_and_position(node_id):
    """
    Get motor velocity and position.
    Returns: (success, velocity, position, error_message)
    """
    try:
        # Access SDK instance from motor_api module
        if hasattr(motor_api, '_sdk_instance') and motor_api._sdk_instance:
            if hasattr(motor_api, '_gateway_info') and motor_api._gateway_info:
                error_code, velocity, position = motor_api._sdk_instance.SdkGetPtpMxnA(
                    motor_api._gateway_info.GtwyHandle,
                    node_id
                )
                if error_code == 0:
                    return (True, velocity, position, None)
                else:
                    return (False, 0, 0, f"Error code {error_code}")
    except Exception as e:
        pass
    # Fallback to regular position getter (no velocity)
    success, position, error = get_motor_position(node_id)
    return (success, 0, position, error)

def read_user_input():
    """Read user input in a separate thread"""
    global command_queue, input_thread_running
    input_thread_running = True
    try:
        while input_thread_running:
            try:
                # Try to read input (this will block, but that's okay in a thread)
                line = sys.stdin.readline()
                if line:
                    cmd = line.strip().lower()
                    if cmd:
                        command_queue.append(cmd)
            except:
                time.sleep(0.1)
    except:
        pass

def read_serial_messages():
    """Read serial messages from Nextion display in a separate thread"""
    global serial_thread_running, serial_port_obj, command_queue, stop_requested
    
    serial_thread_running = True
    buffer = bytearray()
    last_data_time = None
    
    try:
        while serial_thread_running and serial_port_obj and serial_port_obj.is_open:
            current_time = time.time()
            
            if serial_port_obj.in_waiting > 0:
                # Read available data
                data = serial_port_obj.read(serial_port_obj.in_waiting)
                
                # Filter out zero-only chunks, BUT allow them if we're receiving a MOTORPOWER message
                # (MOTORPOWER messages contain null bytes as part of the value: MOTORPOWER=\x00\x00\x00\x00\r\n)
                buffer_upper = bytes(buffer).upper() if len(buffer) > 0 else b''
                is_motorpower_message = buffer_upper.startswith(b'MOTORPOWER') or (len(data) > 0 and b'MOTORPOWER' in (buffer_upper + data.upper()))
                
                if len(data) > 0:
                    if not all(b == 0 for b in data) or is_motorpower_message:
                        # Not all zeros, OR part of MOTORPOWER message (which may contain zeros)
                        buffer.extend(data)
                        last_data_time = current_time
                    else:
                        # All zeros and not MOTORPOWER - discard immediately
                        pass
                
                # Check for newline delimiter first (0x0A or \n) - Nextion uses this
                while b'\n' in buffer or b'\x0A' in buffer:
                    if b'\n' in buffer:
                        idx = buffer.index(b'\n')
                    else:
                        idx = buffer.index(b'\x0A')
                    
                    message = bytes(buffer[:idx])
                    buffer = buffer[idx+1:]
                    
                    # Filter out zero-only messages
                    if len(message) > 0 and not all(b == 0 for b in message):
                        process_serial_message(message)
                
                # Check for termination sequence (0xFF 0xFF 0xFF) as backup
                while len(buffer) >= 3:
                    term_pos = -1
                    for i in range(len(buffer) - 2):
                        if buffer[i:i+3] == bytes([0xFF, 0xFF, 0xFF]):
                            term_pos = i
                            break
                    
                    if term_pos != -1:
                        # Found termination, extract message
                        message = bytes(buffer[:term_pos])
                        buffer = buffer[term_pos + 3:]
                        # Filter out zero-only messages
                        if len(message) > 0 and not all(b == 0 for b in message):
                            process_serial_message(message)
                    else:
                        break
            else:
                # No data available - check if we should timeout the buffer
                if last_data_time is not None and len(buffer) > 0:
                    time_since_last = current_time - last_data_time
                    if time_since_last >= MESSAGE_TIMEOUT:
                        # Timeout reached, consider buffer a complete message
                        # Filter out zero-only messages (likely noise)
                        if len(buffer) > 0 and not all(b == 0 for b in buffer):
                            message = bytes(buffer)
                            process_serial_message(message)
                        buffer = bytearray()
                        last_data_time = None
                
                # Reduced sleep time for higher baud rate
                time.sleep(0.001)  # Poll every 1ms instead of 10ms
    except Exception as e:
        if serial_thread_running:  # Only print if we're still supposed to be running
            print(f"  Warning: Serial reading thread error: {e}")
    finally:
        serial_thread_running = False

def process_serial_message(message_bytes):
    """Process a complete serial message and trigger appropriate actions"""
    global command_queue, stop_requested, motor_speed_multiplier, motspeed_buffer, motspeed_buffer_time
    
    try:
        # Filter out zero-only messages (noise) - be more aggressive
        if len(message_bytes) == 0:
            return
        
        # Check for MOTORPOWER= with byte value BEFORE noise filtering (to catch zero values)
        # Nextion sends: "MOTORPOWER=" + byte(0 or 1) + multiple nulls + "\r\n"
        motorpower_bytes_upper = message_bytes.upper()
        if motorpower_bytes_upper.startswith(b'MOTORPOWER'):
            equals_pos = motorpower_bytes_upper.find(b'=')
            if equals_pos >= 0 and equals_pos + 1 < len(message_bytes):
                # Get the byte immediately after = (ignore trailing nulls, \r\n or other control chars)
                byte_value = message_bytes[equals_pos + 1]
                
                # Debug: show hex for troubleshooting
                hex_str = message_bytes.hex()
                timestamp = time.strftime("%H:%M:%S")
                
                # Interpret: 0 = disable, 1 = enable
                # Only accept 0 or 1, ignore other values
                if byte_value == 0:
                    enable_state = False
                    print(f"[{timestamp}] Serial command: MOTORPOWER=0 (hex: {hex_str}) - Disabling motor...")
                elif byte_value == 1:
                    enable_state = True
                    print(f"[{timestamp}] Serial command: MOTORPOWER=1 (hex: {hex_str}) - Enabling motor...")
                else:
                    # Invalid value, show debug info
                    print(f"[{timestamp}] DEBUG: MOTORPOWER with invalid byte value {byte_value} (hex: {hex_str})")
                    return
                
                # Ensure motor connection before enabling/disabling
                if ensure_motor_connection():
                    success, error = enable_motor(MOTOR_NODE_ID, enable_state)
                    if success:
                        print(f"  ✓ Motor {'enabled' if enable_state else 'disabled'}")
                        if enable_state:
                            update_motor_status(0)  # Not moving
                        else:
                            update_motor_status(2)  # Error/disabled status
                    else:
                        print(f"  ✗ Failed to {'enable' if enable_state else 'disable'} motor: {error}")
                        update_motor_status(2)  # Error status
                else:
                    print(f"  ✗ Cannot {'enable' if enable_state else 'disable'} motor: connection failed")
                    update_motor_status(2)  # Error status
                return  # Processed
        
        # Filter out messages that are all zeros or mostly zeros
        non_zero_count = sum(1 for b in message_bytes if b != 0)
        if non_zero_count == 0:
            return  # All zeros, ignore
        
        # If message is mostly zeros, ignore it (likely noise)
        if len(message_bytes) > 3 and non_zero_count < len(message_bytes) * 0.3:
            return  # Less than 30% non-zero bytes, likely noise
        
        # Check for MOTSPEED= with byte value BEFORE decoding (to catch raw byte values)
        # Nextion sends: "MOTSPEED=" + byte(0-100) + null bytes + "\r\n"
        motspeed_bytes_upper = message_bytes.upper()
        if motspeed_bytes_upper.startswith(b'MOTSPEED'):
            equals_pos = motspeed_bytes_upper.find(b'=')
            if equals_pos >= 0 and equals_pos + 1 < len(message_bytes):
                # Get the byte immediately after = (ignore trailing nulls, \r\n or other control chars)
                byte_value = message_bytes[equals_pos + 1]
                
                # Clamp to 0-100 range
                speed_value = max(0, min(100, byte_value))
                
                # Calculate multiplier: 0 -> 0.5x, 50 -> 1.0x, 100 -> 2.0x
                motor_speed_multiplier = 0.5 + (speed_value / 100.0) * 1.5
                
                timestamp = time.strftime("%H:%M:%S")
                new_speed = int(MOTOR_BASE_SPEED * motor_speed_multiplier)
                print(f"[{timestamp}] Serial command: MOTSPEED={speed_value} (byte value) - Motor speed set to {new_speed} ({motor_speed_multiplier:.2f}x base speed)")
                return  # Processed
        
        # Decode message to text
        message_text = message_bytes.decode('utf-8', errors='ignore').strip()
        
        # If decoded text is empty but we have non-zero bytes, try to see what we got
        if not message_text and non_zero_count > 0:
            # Debug: show hex for non-text messages (but limit output to avoid spam)
            # Only show first few non-zero messages for debugging
            hex_str = message_bytes.hex()
            timestamp = time.strftime("%H:%M:%S")
            # Show hex if it looks like it might be data (not just random bytes)
            # Limit to first 50 bytes to avoid spam
            print(f"[{timestamp}] DEBUG: Received non-text data (hex: {hex_str[:100]}, len: {len(message_bytes)})")
            return
        
        if not message_text:
            return
        
        # Handle MOTSPEED messages - they might come in parts, so buffer them
        current_time = time.time()
        
        # Check if this looks like part of a MOTSPEED message
        if message_text.upper().startswith('MOTSPEED'):
            # Add to buffer
            if motspeed_buffer_time is None or (current_time - motspeed_buffer_time) > 0.2:
                # New MOTSPEED message, start fresh buffer
                motspeed_buffer = message_text
            else:
                # Continue accumulating
                motspeed_buffer += message_text
            
            motspeed_buffer_time = current_time
            
            # Try to extract complete MOTSPEED=value from buffer
            # First try numeric format: MOTSPEED=50
            motspeed_match = re.search(r'MOTSPEED\s*=\s*(\d+)', motspeed_buffer, re.IGNORECASE)
            if motspeed_match:
                try:
                    speed_value = int(motspeed_match.group(1))
                    # Clamp value to 0-100 range
                    speed_value = max(0, min(100, speed_value))
                    
                    # Calculate multiplier: 0 -> 0.5x, 50 -> 1.0x, 100 -> 2.0x
                    # Formula: multiplier = 0.5 + (value / 100) * 1.5
                    motor_speed_multiplier = 0.5 + (speed_value / 100.0) * 1.5
                    
                    timestamp = time.strftime("%H:%M:%S")
                    new_speed = int(MOTOR_BASE_SPEED * motor_speed_multiplier)
                    print(f"[{timestamp}] Serial command: MOTSPEED={speed_value} - Motor speed set to {new_speed} ({motor_speed_multiplier:.2f}x base speed)")
                    
                    # Clear buffer after successful processing
                    motspeed_buffer = ""
                    motspeed_buffer_time = None
                    return  # Processed, don't check for button commands
                except ValueError:
                    # Invalid value, but keep buffer in case more data comes
                    pass
            
            # Try to match MOTSPEED= followed by content (might be single byte or partial)
            # Check if we have MOTSPEED= in the buffer
            if '=' in motspeed_buffer.upper():
                # Find the position after the = sign
                equals_pos = motspeed_buffer.upper().find('=')
                if equals_pos >= 0:
                    # Get everything after the = sign
                    value_part = motspeed_buffer[equals_pos + 1:]
                    
                    # Try to extract value from raw bytes (in case it's sent as byte value)
                    # Look for MOTSPEED= in the original bytes
                    equals_pos_bytes = message_bytes.upper().find(b'=')
                    if equals_pos_bytes >= 0 and equals_pos_bytes + 1 < len(message_bytes):
                        # Get the byte(s) after =
                        value_bytes = message_bytes[equals_pos_bytes + 1:]
                        
                        # If there's exactly one byte after =, interpret it as the value (0-100)
                        if len(value_bytes) == 1:
                            byte_value = value_bytes[0]
                            # Clamp to 0-100 range
                            speed_value = max(0, min(100, byte_value))
                            
                            # Calculate multiplier: 0 -> 0.5x, 50 -> 1.0x, 100 -> 2.0x
                            motor_speed_multiplier = 0.5 + (speed_value / 100.0) * 1.5
                            
                            timestamp = time.strftime("%H:%M:%S")
                            new_speed = int(MOTOR_BASE_SPEED * motor_speed_multiplier)
                            print(f"[{timestamp}] Serial command: MOTSPEED={speed_value} (byte value) - Motor speed set to {new_speed} ({motor_speed_multiplier:.2f}x base speed)")
                            
                            # Clear buffer after successful processing
                            motspeed_buffer = ""
                            motspeed_buffer_time = None
                            return  # Processed, don't check for button commands
                    
                    # Try to parse as string number
                    value_part_stripped = value_part.strip()
                    if value_part_stripped and value_part_stripped.isdigit():
                        try:
                            speed_value = int(value_part_stripped)
                            # Clamp value to 0-100 range
                            speed_value = max(0, min(100, speed_value))
                            
                            # Calculate multiplier
                            motor_speed_multiplier = 0.5 + (speed_value / 100.0) * 1.5
                            
                            timestamp = time.strftime("%H:%M:%S")
                            new_speed = int(MOTOR_BASE_SPEED * motor_speed_multiplier)
                            print(f"[{timestamp}] Serial command: MOTSPEED={speed_value} - Motor speed set to {new_speed} ({motor_speed_multiplier:.2f}x base speed)")
                            
                            # Clear buffer after successful processing
                            motspeed_buffer = ""
                            motspeed_buffer_time = None
                            return  # Processed, don't check for button commands
                        except ValueError:
                            pass
                
                # Incomplete MOTSPEED message, wait for more
                # Don't process as other command yet
                return
        
        # Clear MOTSPEED buffer if we got a different message (timeout or new command)
        if motspeed_buffer_time is not None and (current_time - motspeed_buffer_time) > 0.2:
            # Buffer timed out, clear it
            if motspeed_buffer:
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] Warning: Incomplete MOTSPEED message discarded: '{motspeed_buffer}'")
            motspeed_buffer = ""
            motspeed_buffer_time = None
        
        # Handle MOTORPOWER command (enable/disable motor)
        # Check for MOTORPOWER= in raw bytes first (before decoding)
        # Nextion sends: "MOTORPOWER=" + byte(0 or 1) + "\r\n"
        motorpower_bytes_upper = message_bytes.upper()
        if motorpower_bytes_upper.startswith(b'MOTORPOWER'):
            equals_pos = motorpower_bytes_upper.find(b'=')
            if equals_pos >= 0 and equals_pos + 1 < len(message_bytes):
                # Get the byte immediately after = (ignore trailing \r\n or other control chars)
                byte_value = message_bytes[equals_pos + 1]
                
                # Interpret: 0 = disable, 1 = enable
                # Only accept 0 or 1, ignore other values
                if byte_value == 0:
                    enable_state = False
                elif byte_value == 1:
                    enable_state = True
                else:
                    # Invalid value, ignore
                    return
                
                timestamp = time.strftime("%H:%M:%S")
                print(f"[{timestamp}] Serial command: MOTORPOWER={byte_value} - {'Enabling' if enable_state else 'Disabling'} motor...")
                
                # Ensure motor connection before enabling/disabling
                if ensure_motor_connection():
                    success, error = enable_motor(MOTOR_NODE_ID, enable_state)
                    if success:
                        print(f"  ✓ Motor {'enabled' if enable_state else 'disabled'}")
                        if enable_state:
                            update_motor_status(0)  # Not moving
                        else:
                            update_motor_status(2)  # Error/disabled status
                    else:
                        print(f"  ✗ Failed to {'enable' if enable_state else 'disable'} motor: {error}")
                        update_motor_status(2)  # Error status
                else:
                    print(f"  ✗ Cannot {'enable' if enable_state else 'disable'} motor: connection failed")
                    update_motor_status(2)  # Error status
                return  # Processed (already handled above, before decoding)
        
        # Decode message to text (for other commands)
        message_text = message_bytes.decode('utf-8', errors='ignore').strip()
        
        # Also check for MOTORPOWER= in decoded text (fallback for ASCII format)
        if message_text.upper().startswith('MOTORPOWER'):
            motorpower_match = re.search(r'MOTORPOWER\s*=\s*(\d+)', message_text, re.IGNORECASE)
            if motorpower_match:
                try:
                    power_value = int(motorpower_match.group(1))
                    # Interpret: 0 = disable, 1 = enable, any other value = enable if > 0
                    enable_state = (power_value != 0)
                    
                    timestamp = time.strftime("%H:%M:%S")
                    print(f"[{timestamp}] Serial command: MOTORPOWER={power_value} - {'Enabling' if enable_state else 'Disabling'} motor...")
                    
                    # Ensure motor connection before enabling/disabling
                    if ensure_motor_connection():
                        success, error = enable_motor(MOTOR_NODE_ID, enable_state)
                        if success:
                            print(f"  ✓ Motor {'enabled' if enable_state else 'disabled'}")
                            if enable_state:
                                update_motor_status(0)  # Not moving
                            else:
                                update_motor_status(2)  # Error/disabled status
                        else:
                            print(f"  ✗ Failed to {'enable' if enable_state else 'disable'} motor: {error}")
                            update_motor_status(2)  # Error status
                    else:
                        print(f"  ✗ Cannot {'enable' if enable_state else 'disable'} motor: connection failed")
                        update_motor_status(2)  # Error status
                    return  # Processed
                except ValueError:
                    pass  # Invalid value, continue to other handlers
        
        # Try to extract button name from format like "BTN:RESETHOME" or just "RESETHOME"
        # Check for BTN: prefix pattern
        button_match = re.search(r'BTN:(\w+)', message_text, re.IGNORECASE)
        if button_match:
            message_text = button_match.group(1)
        
        # Convert to uppercase for case-insensitive matching
        message_upper = message_text.upper().strip()
        
        timestamp = time.strftime("%H:%M:%S")
        
        if message_upper == 'RESETHOME':
            print(f"\n[{timestamp}] Serial command: RESETHOME - Starting homing sequence...")
            command_queue.append('h')
        
        elif message_upper == 'MANCLAMP':
            print(f"\n[{timestamp}] Serial command: MANCLAMP - Starting clamping sequence...")
            command_queue.append('c')
        
        elif message_upper == 'ESTOP':
            print(f"\n[{timestamp}] Serial command: ESTOP - Emergency stop!")
            stop_requested = True
            command_queue.append('s')
        
        else:
            # Unknown message - print for debugging (only if it looks like actual text)
            if any(32 <= b < 127 for b in message_bytes):
                print(f"[{timestamp}] Serial message received (not recognized): '{message_text}' (hex: {message_bytes.hex()[:50]})")
    
    except Exception as e:
        print(f"  Warning: Error processing serial message: {e}")

def test_nextion_communication(port_obj):
    """
    Test if we can communicate with the Nextion display by sending a command and checking response.
    
    Args:
        port_obj: Serial port object
        
    Returns:
        bool: True if communication seems to work, False otherwise
    """
    try:
        # Send a simple command to Nextion (refresh command)
        # This should not cause any issues even if display is busy
        test_cmd = b"ref t0\xFF\xFF\xFF"
        port_obj.write(test_cmd)
        port_obj.flush()
        time.sleep(0.1)
        
        # Check if we get any non-zero response (even if it's just an ACK)
        if port_obj.in_waiting > 0:
            response = port_obj.read(port_obj.in_waiting)
            # If we get any non-zero data, communication might be working
            if len(response) > 0 and not all(b == 0 for b in response):
                return True
        
        # Even if no response, if we can write without error, port is likely OK
        return True
    except Exception as e:
        return False

def find_nextion_port():
    """
    Auto-detect the Nextion display serial port by searching for common USB serial devices.
    Tests multiple baud rates to find the correct one.
    
    Returns:
        tuple: (port_path, baud_rate) or (None, None) if not found
    """
    # Common serial device patterns
    device_patterns = [
        '/dev/ttyUSB*',  # USB-to-serial adapters
        '/dev/ttyACM*',  # USB CDC devices
    ]
    
    found_ports = []
    for pattern in device_patterns:
        found_ports.extend(glob.glob(pattern))
    
    # Sort ports to check ttyUSB0 before ttyUSB1, etc.
    found_ports.sort()
    
    if not found_ports:
        print("  No serial devices found matching common patterns")
        return (None, None)
    
    print(f"  Found {len(found_ports)} potential serial device(s): {', '.join(found_ports)}")
    
    # Common baud rates to try (in order of likelihood)
    baud_rates_to_try = [
        SERIAL_BAUD_RATE,  # User's configured rate first
        115200,  # Common high-speed rate
        9600,    # Standard rate
        38400,   # Alternative rate
        57600,   # Alternative rate
        230400,  # Higher rate
        460800,  # Higher rate
    ]
    
    # Try each port with different baud rates
    for port in found_ports:
        print(f"  Testing {port}...")
        for baud_rate in baud_rates_to_try:
            try:
                print(f"    Trying baud rate {baud_rate}...")
                test_port = serial.Serial(
                    port=port,
                    baudrate=baud_rate,
                    timeout=SERIAL_TIMEOUT,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
                time.sleep(0.5)
                test_port.reset_input_buffer()
                test_port.reset_output_buffer()
                
                # Test communication
                if test_nextion_communication(test_port):
                    test_port.close()
                    print(f"  ✓ Successfully connected to {port} at {baud_rate} baud")
                    return (port, baud_rate)
                
                test_port.close()
            except serial.SerialException as e:
                print(f"    ✗ Baud {baud_rate}: {e}")
                continue
            except Exception as e:
                print(f"    ✗ Baud {baud_rate}: Unexpected error: {e}")
                continue
        
        print(f"  ✗ Could not connect to {port} with any tested baud rate")
    
    print("  ✗ Could not connect to any serial device")
    return (None, None)

def update_motor_status(status):
    """
    Update the Nextion display motor status variable.
    
    Args:
        status: 0 = motor not moving, 1 = motor moving, 2 = error, 3 = clamped
    """
    global serial_port_obj
    
    if serial_port_obj is None or not serial_port_obj.is_open:
        return  # Can't update if serial port not available
    
    try:
        # Format: motstatus=0\xFF\xFF\xFF
        command_str = f"motstatus={status}"
        command_bytes = command_str.encode('utf-8')
        termination = bytes([0xFF, 0xFF, 0xFF])
        full_command = command_bytes + termination
        
        # Send command
        serial_port_obj.write(full_command)
        serial_port_obj.flush()  # Ensure data is sent immediately
        
    except Exception as e:
        # Silently fail - don't spam errors if Nextion is disconnected
        pass

def ensure_motor_connection():
    """
    Ensure motor gateway connection is active. Reconnect if needed.
    
    Returns:
        bool: True if connected (or successfully reconnected), False if reconnection failed
    """
    global motor_connected
    
    # Check if we're already connected by trying a simple operation
    if motor_connected:
        try:
            # Try to get position as a connectivity test
            success, _, error = get_motor_position(MOTOR_NODE_ID)
            if success:
                return True
            # If we get here, connection might be lost
            print(f"⚠ Connection check failed: {error}")
        except Exception as e:
            print(f"⚠ Connection check exception: {e}")
    
    # Connection is lost or not established - try to reconnect
    print("\n⚠ Motor connection lost. Attempting to reconnect...")
    update_motor_status(2)  # Error status
    motor_connected = False
    
    retry_delay = 2
    max_attempts = 10  # Try up to 10 times before giving up on this operation
    
    for attempt in range(max_attempts):
        if attempt > 0:
            print(f"  Reconnection attempt {attempt + 1}/{max_attempts}...")
            time.sleep(retry_delay)
            retry_delay = min(retry_delay + 1, 5)  # Max 5 second delay
        
        # Try to disconnect first (clean up any stale connection)
        try:
            disconnect()
        except:
            pass
        
        # Try to reconnect
        success, error = connect(GATEWAY_IP, GATEWAY_PORT)
        if success:
            print("✓ Reconnected to motor gateway")
            motor_connected = True
            
            # Try to enable motor
            success, error = enable_motor(MOTOR_NODE_ID, True)
            if success:
                print("✓ Motor re-enabled")
                update_motor_status(0)  # Reset to not moving
                return True
            else:
                print(f"⚠ Failed to re-enable motor: {error}")
                # Disconnect and try again
                disconnect()
                motor_connected = False
        else:
            print(f"  Connection attempt failed: {error}")
    
    print("✗ Failed to reconnect after multiple attempts")
    update_motor_status(2)  # Keep error status
    return False

def setup_gpio():
    """Setup GPIO pins for sensor monitoring"""
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    # Configure homing sensor pin
    GPIO.setup(HOMING_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print(f"✓ Configured GPIO {HOMING_GPIO} as homing sensor input")
    
    # Configure clamping sensor pin
    GPIO.setup(CLAMPING_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print(f"✓ Configured GPIO {CLAMPING_GPIO} as clamping sensor input")
    
    # Configure external control pin
    GPIO.setup(CONTROL_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    print(f"✓ Configured GPIO {CONTROL_GPIO} as external control input (HIGH=clamp, LOW=home)")

def check_stop_command():
    """Check if stop command was issued"""
    global stop_requested, command_queue
    while command_queue:
        cmd = command_queue.pop(0)
        if cmd in ['s', 'stop']:
            stop_requested = True
            return True
    return False

def wait_for_gpio_high(gpio_pin, timeout=None):
    """
    Wait for GPIO pin to go high.
    
    Args:
        gpio_pin: GPIO pin number to monitor
        timeout: Maximum time to wait in seconds (None for no timeout)
    
    Returns:
        True if pin went high, False if timeout or stop requested
    """
    global stop_requested
    start_time = time.time()
    
    while True:
        # Check for stop command
        if check_stop_command() or stop_requested:
            return False
        
        # Check GPIO pin
        if GPIO.input(gpio_pin):
            return True
        
        # Check timeout
        if timeout is not None:
            if (time.time() - start_time) >= timeout:
                return False
        
        time.sleep(0.01)  # Poll every 10ms

def monitor_gpio_control():
    """
    Monitor GPIO 26 for state changes and trigger homing/clamping accordingly.
    HIGH = clamp, LOW = home
    """
    global gpio_control_thread_running, command_queue, homing_complete
    
    gpio_control_thread_running = True
    last_state = None
    
    try:
        while gpio_control_thread_running:
            current_state = GPIO.input(CONTROL_GPIO)
            
            # Detect state change (edge detection)
            if last_state is not None and current_state != last_state:
                timestamp = time.strftime("%H:%M:%S")
                
                if current_state == GPIO.HIGH:
                    # GPIO went HIGH - trigger clamping
                    print(f"\n[{timestamp}] GPIO {CONTROL_GPIO} went HIGH - Triggering clamping sequence...")
                    if homing_complete:
                        command_queue.append('c')
                    else:
                        print(f"  Warning: Homing not complete. Please home first.")
                else:
                    # GPIO went LOW - trigger homing
                    print(f"\n[{timestamp}] GPIO {CONTROL_GPIO} went LOW - Triggering homing sequence...")
                    command_queue.append('h')
            
            last_state = current_state
            time.sleep(0.01)  # Poll every 10ms
            
    except Exception as e:
        if gpio_control_thread_running:
            print(f"  Warning: GPIO control monitoring thread error: {e}")
    finally:
        gpio_control_thread_running = False

def homing_sequence():
    """
    Perform homing sequence:
    - If already homed (position near zero), just move to position 0
    - If not homed, search for sensor using continuous jog motion, then set zero point
    """
    global stop_requested, homing_complete
    
    print("\n" + "=" * 60)
    print("Starting Homing Sequence")
    print("=" * 60)
    
    # Ensure motor connection is active
    if not ensure_motor_connection():
        print("✗ Cannot proceed with homing - motor connection unavailable")
        return False
    
    # Get initial position
    success, start_position, error = get_motor_position(MOTOR_NODE_ID)
    if not success:
        print(f"✗ Failed to get initial position: {error}")
        # Try to reconnect and retry once
        if ensure_motor_connection():
            success, start_position, error = get_motor_position(MOTOR_NODE_ID)
            if not success:
                print(f"✗ Failed to get initial position after reconnection: {error}")
                update_motor_status(2)  # Error status
                return False
        else:
            update_motor_status(2)  # Error status
            return False
    
    print(f"Initial position: {start_position}")
    
    # Stop motor first to ensure clean state
    try:
        stop_motor(MOTOR_NODE_ID)
        time.sleep(0.1)
    except Exception as e:
        print(f"⚠ Warning during stop: {e}")
        if not ensure_motor_connection():
            return False
        try:
            stop_motor(MOTOR_NODE_ID)
            time.sleep(0.1)
        except:
            pass
    
    # Ensure motor is enabled
    try:
        enable_motor(MOTOR_NODE_ID, True)
        time.sleep(0.05)
    except Exception as e:
        print(f"⚠ Warning during enable: {e}")
        if not ensure_motor_connection():
            return False
        success, error = enable_motor(MOTOR_NODE_ID, True)
        if not success:
            print(f"✗ Failed to enable motor after reconnection: {error}")
            update_motor_status(2)
            return False
        time.sleep(0.05)
    
    # Calculate actual homing speed based on multiplier and direction
    actual_homing_speed = int(MOTOR_BASE_SPEED * HOMING_DIRECTION * motor_speed_multiplier)
    
    direction_str = "positive" if actual_homing_speed > 0 else "negative"
    print(f"Searching for homing sensor on GPIO {HOMING_GPIO}")
    print(f"Moving continuously in {direction_str} direction at speed {abs(actual_homing_speed)}")
    print(f"  (Speed multiplier: {motor_speed_multiplier:.2f}x, base speed: {MOTOR_BASE_SPEED})")
    print("(Press 's' to stop at any time)")
    
    # Start continuous jog motion
    success, error = move_motor(MOTOR_NODE_ID, jog_speed=actual_homing_speed)
    if not success:
        print(f"✗ Failed to start motor: {error}")
        # Try to reconnect and retry
        if ensure_motor_connection():
            success, error = move_motor(MOTOR_NODE_ID, jog_speed=actual_homing_speed)
            if not success:
                print(f"✗ Failed to start motor after reconnection: {error}")
                update_motor_status(2)  # Error status
                return False
        else:
            update_motor_status(2)  # Error status
            return False
    
    print("✓ Motion started - monitoring sensor continuously...")
    update_motor_status(1)  # Motor is moving
    
    # Monitor sensor continuously while motor is moving
    last_position = start_position
    last_status_print = time.time()
    sensor_detected = False
    
    while True:
        # Check for stop command
        if check_stop_command() or stop_requested:
            print("\nStop requested - stopping motor...")
            try:
                stop_motor(MOTOR_NODE_ID)
            except Exception as e:
                print(f"⚠ Warning during stop: {e}")
                ensure_motor_connection()
            return False
        
        # Check sensor continuously (every 1ms for fast response)
        if GPIO.input(HOMING_GPIO):
            print(f"\n✓ Homing sensor detected! (GPIO {HOMING_GPIO} went HIGH)")
            try:
                stop_motor(MOTOR_NODE_ID)  # Stop immediately!
            except Exception as e:
                print(f"⚠ Warning during stop: {e}")
                ensure_motor_connection()
                try:
                    stop_motor(MOTOR_NODE_ID)
                except:
                    pass
            update_motor_status(0)  # Motor stopped
            time.sleep(0.05)  # Brief pause to ensure stop command is processed
            sensor_detected = True
            break
        
        # Check position periodically to detect max distance
        current_time = time.time()
        if (current_time - last_status_print) >= 1.0:  # Print status every second
            success, current_position, error = get_motor_position(MOTOR_NODE_ID)
            if success:
                distance_traveled = abs(current_position - start_position)
                print(f"  Distance traveled: {distance_traveled} ticks | Current position: {current_position}")
                
                if distance_traveled >= HOMING_MAX_DISTANCE:
                    print(f"\n✗ ERROR: No Homing Sensor Found!")
                    print(f"Motor traveled {distance_traveled} units without finding sensor")
                    print(f"Maximum allowed distance: {HOMING_MAX_DISTANCE}")
                    try:
                        stop_motor(MOTOR_NODE_ID)
                    except Exception as e:
                        print(f"⚠ Warning during stop: {e}")
                        ensure_motor_connection()
                    update_motor_status(2)  # Error status
                    return False
                
                last_position = current_position
            else:
                # Connection might be lost - try to reconnect
                print(f"⚠ Position check failed: {error}")
                if ensure_motor_connection():
                    # Retry getting position
                    success, current_position, error = get_motor_position(MOTOR_NODE_ID)
                    if success:
                        distance_traveled = abs(current_position - start_position)
                        print(f"  Distance traveled: {distance_traveled} ticks | Current position: {current_position}")
                        last_position = current_position
                else:
                    print("✗ Cannot continue - connection lost and reconnection failed")
                    try:
                        stop_motor(MOTOR_NODE_ID)
                    except Exception as e:
                        print(f"⚠ Warning during stop: {e}")
                    update_motor_status(2)  # Error status
                    return False
            
            last_status_print = current_time
        
        time.sleep(0.001)  # Check sensor every 1ms for fast response
    
    if not sensor_detected:
        return False
    
    # Get position when sensor was found
    success, sensor_position, error = get_motor_position(MOTOR_NODE_ID)
    if success:
        distance_traveled = abs(sensor_position - start_position)
        print(f"Position when sensor found: {sensor_position}")
        print(f"Total distance traveled: {distance_traveled} ticks")
    
    time.sleep(0.1)  # Brief pause
    
    # Move away from sensor (relative position using PTP)
    backoff_distance = HOMING_BACKOFF_DISTANCE * HOMING_BACKOFF_DIRECTION
    direction_str = "away from" if HOMING_BACKOFF_DIRECTION < 0 else "toward"
    print(f"\nMoving {direction_str} sensor by {abs(backoff_distance)} units (relative PTP)...")
    
    # Move relative distance (direction determined by HOMING_BACKOFF_DIRECTION)
    success, error = move_motor(
        MOTOR_NODE_ID,
        position=backoff_distance,
        relative=True,
        speed=HOMING_BACKOFF_PTP_SPEED
    )
    
    if not success:
        print(f"✗ Failed to start backoff motion: {error}")
        # Try to reconnect and retry
        if ensure_motor_connection():
            success, error = move_motor(
                MOTOR_NODE_ID,
                position=backoff_distance,
                relative=True,
                speed=HOMING_BACKOFF_PTP_SPEED
            )
            if not success:
                print(f"✗ Failed to start backoff motion after reconnection: {error}")
                update_motor_status(2)  # Error status
                return False
        else:
            update_motor_status(2)  # Error status
            return False
    
    update_motor_status(1)  # Motor is moving
    
    # Wait for motion to complete
    print("Waiting for backoff motion to complete...")
    time.sleep(2)  # Adjust based on your motor speed and distance
    
    # Stop motor (in case it's still moving)
    try:
        stop_motor(MOTOR_NODE_ID)
    except Exception as e:
        print(f"⚠ Warning during stop: {e}")
        ensure_motor_connection()
        try:
            stop_motor(MOTOR_NODE_ID)
        except:
            pass
    update_motor_status(0)  # Motor stopped
    time.sleep(0.1)
    
    # Get final position before resetting
    success, final_position, error = get_motor_position(MOTOR_NODE_ID)
    if success:
        print(f"Position before reset: {final_position}")
    else:
        # Try to reconnect and retry
        if ensure_motor_connection():
            success, final_position, error = get_motor_position(MOTOR_NODE_ID)
            if success:
                print(f"Position before reset: {final_position}")
    
    # Reset position to zero (set current position as zero reference)
    print("Resetting position to zero...")
    success, error = reset_position_to_zero(MOTOR_NODE_ID)
    if success:
        print("✓ Position reset to zero")
        # Verify the reset worked
        success, new_position, error = get_motor_position(MOTOR_NODE_ID)
        if success:
            print(f"New position after reset: {new_position}")
            homing_complete = True  # Mark homing as complete
    else:
        print(f"⚠ Warning: Failed to reset position: {error}")
        # Try to reconnect and retry
        if ensure_motor_connection():
            success, error = reset_position_to_zero(MOTOR_NODE_ID)
            if success:
                print("✓ Position reset to zero (after reconnection)")
                success, new_position, error = get_motor_position(MOTOR_NODE_ID)
                if success:
                    print(f"New position after reset: {new_position}")
                    homing_complete = True
            else:
                print(f"✗ Failed to reset position after reconnection: {error}")
                return False
        else:
            return False
    
    print("\n✓ Homing sequence complete!")
    update_motor_status(0)  # Motor stopped
    return True

def clamping_sequence():
    """
    Perform clamping sequence using continuous jog motion:
    1. Start continuous motion in CLAMPING_DIRECTION (adjusted by speed multiplier)
    2. Continuously monitor sensor while moving
    3. Stop immediately when GPIO 9 goes high or stop command received
    """
    global stop_requested, motor_speed_multiplier
    
    print("\n" + "=" * 60)
    print("Starting Clamping Sequence")
    print("=" * 60)
    
    # Ensure motor connection is active
    if not ensure_motor_connection():
        print("✗ Cannot proceed with clamping - motor connection unavailable")
        return False
    
    # Get initial position
    success, start_position, error = get_motor_position(MOTOR_NODE_ID)
    if not success:
        print(f"✗ Failed to get initial position: {error}")
        # Try to reconnect and retry once
        if ensure_motor_connection():
            success, start_position, error = get_motor_position(MOTOR_NODE_ID)
            if not success:
                print(f"✗ Failed to get initial position after reconnection: {error}")
                update_motor_status(2)  # Error status
                return False
        else:
            update_motor_status(2)  # Error status
            return False
    
    print(f"Initial position: {start_position}")
    
    # Stop motor first to ensure clean state
    try:
        stop_motor(MOTOR_NODE_ID)
        time.sleep(0.1)
    except Exception as e:
        print(f"⚠ Warning during stop: {e}")
        if not ensure_motor_connection():
            return False
        try:
            stop_motor(MOTOR_NODE_ID)
            time.sleep(0.1)
        except:
            pass
    
    # Ensure motor is enabled
    try:
        enable_motor(MOTOR_NODE_ID, True)
        time.sleep(0.05)
    except Exception as e:
        print(f"⚠ Warning during enable: {e}")
        if not ensure_motor_connection():
            return False
        success, error = enable_motor(MOTOR_NODE_ID, True)
        if not success:
            print(f"✗ Failed to enable motor after reconnection: {error}")
            update_motor_status(2)
            return False
        time.sleep(0.05)
    
    # Calculate actual clamping speed based on multiplier and direction
    actual_clamping_speed = int(MOTOR_BASE_SPEED * CLAMPING_DIRECTION * motor_speed_multiplier)
    
    direction_str = "positive" if actual_clamping_speed > 0 else "negative"
    print(f"Searching for clamping sensor on GPIO {CLAMPING_GPIO}")
    print(f"Moving continuously in {direction_str} direction at speed {abs(actual_clamping_speed)}")
    print(f"  (Speed multiplier: {motor_speed_multiplier:.2f}x, base speed: {MOTOR_BASE_SPEED})")
    print("(Press 's' to stop at any time)")
    
    # Start continuous jog motion with calculated speed
    success, error = move_motor(MOTOR_NODE_ID, jog_speed=actual_clamping_speed)
    if not success:
        print(f"✗ Failed to start motor: {error}")
        # Try to reconnect and retry
        if ensure_motor_connection():
            success, error = move_motor(MOTOR_NODE_ID, jog_speed=actual_clamping_speed)
            if not success:
                print(f"✗ Failed to start motor after reconnection: {error}")
                update_motor_status(2)  # Error status
                return False
        else:
            update_motor_status(2)  # Error status
            return False
    
    print("✓ Motion started - monitoring sensor continuously...")
    update_motor_status(1)  # Motor is moving
    
    # Monitor sensor continuously while motor is moving
    last_status_print = time.time()
    sensor_detected = False
    motion_start_time = time.time()
    
    while True:
        # Check for stop command
        if check_stop_command() or stop_requested:
            print("\nStop requested - stopping motor...")
            try:
                stop_motor(MOTOR_NODE_ID)
            except Exception as e:
                print(f"⚠ Warning during stop: {e}")
                ensure_motor_connection()
            update_motor_status(0)  # Motor stopped
            return False
        
        # Check sensor continuously (every 1ms for fast response)
        if GPIO.input(CLAMPING_GPIO):
            print(f"\n✓ Clamping sensor detected! (GPIO {CLAMPING_GPIO} went HIGH)")
            try:
                stop_motor(MOTOR_NODE_ID)  # Stop immediately!
            except Exception as e:
                print(f"⚠ Warning during stop: {e}")
                ensure_motor_connection()
                try:
                    stop_motor(MOTOR_NODE_ID)
                except:
                    pass
            update_motor_status(0)  # Motor stopped
            time.sleep(0.05)  # Brief pause to ensure stop command is processed
            sensor_detected = True
            break
        
        # Check distance traveled to prevent exceeding max distance
        success, current_position, error = get_motor_position(MOTOR_NODE_ID)
        if success:
            distance_traveled = abs(current_position - start_position)
            
            if distance_traveled >= CLAMPING_MAX_DISTANCE:
                print(f"\n✗ ERROR: Max clamping distance ({CLAMPING_MAX_DISTANCE}) exceeded without sensor trigger.")
                print(f"Motor traveled {distance_traveled} units without finding sensor")
                stop_motor(MOTOR_NODE_ID)
                update_motor_status(2)  # Error status
                return False
        else:
            # Connection might be lost - try to reconnect
            print(f"⚠ Position check failed: {error}")
            if ensure_motor_connection():
                # Retry getting position
                success, current_position, error = get_motor_position(MOTOR_NODE_ID)
                if success:
                    distance_traveled = abs(current_position - start_position)
                    if distance_traveled >= CLAMPING_MAX_DISTANCE:
                        print(f"\n✗ ERROR: Max clamping distance ({CLAMPING_MAX_DISTANCE}) exceeded without sensor trigger.")
                        print(f"Motor traveled {distance_traveled} units without finding sensor")
                        try:
                            stop_motor(MOTOR_NODE_ID)
                        except Exception as e:
                            print(f"⚠ Warning during stop: {e}")
                            ensure_motor_connection()
                        update_motor_status(2)  # Error status
                        return False
            else:
                print("✗ Cannot continue - connection lost and reconnection failed")
                try:
                    stop_motor(MOTOR_NODE_ID)
                except Exception as e:
                    print(f"⚠ Warning during stop: {e}")
                update_motor_status(2)  # Error status
                return False
        
        # Print status periodically
        current_time = time.time()
        if (current_time - last_status_print) >= 1.0:  # Print status every second
            if success:
                print(f"  Distance traveled: {distance_traveled} ticks | Current position: {current_position}")
            
            last_status_print = current_time
        
        # Check timeout
        if (time.time() - motion_start_time) > MOTION_TIMEOUT:
            print(f"\n⚠ WARNING: Clamping motion timed out after {MOTION_TIMEOUT} seconds.")
            stop_motor(MOTOR_NODE_ID)
            update_motor_status(2)  # Error status
            return False
        
        time.sleep(0.001)  # Check sensor every 1ms for fast response
    
    if not sensor_detected:
        return False
    
    # Get final position
    success, final_position, error = get_motor_position(MOTOR_NODE_ID)
    if success:
        distance_traveled = abs(final_position - start_position)
        print(f"Final position: {final_position}")
        print(f"Total distance traveled: {distance_traveled} ticks")
    
    print("\n✓ Clamping sequence complete!")
    update_motor_status(3)  # Clamped status
    return True

def main():
    global motor_connected, stop_requested, command_queue, input_thread_running
    
    print("Actuator Test Script")
    print("=" * 60)
    print("\nCommands:")
    print("  h or H - Start homing sequence")
    print("  c or C - Start clamping sequence (after homing)")
    print("  s or S - Emergency stop motor")
    print("  q or Q - Quit program")
    print("=" * 60)
    
    # Setup GPIO
    try:
        setup_gpio()
    except Exception as e:
        print(f"✗ Failed to setup GPIO: {e}")
        sys.exit(1)
    
    # Connect to Nextion display FIRST (so we can update status even if gateway fails)
    global serial_port_obj
    print(f"\nSearching for Nextion display...")
    
    # Try to find the serial port and correct baud rate automatically
    detected_port, detected_baud = find_nextion_port()
    
    if detected_port and detected_baud:
        print(f"Connecting to Nextion display at {detected_port} (baud: {detected_baud})...")
        try:
            # Update the baud rate if different from configured
            actual_baud = detected_baud
            if detected_baud != SERIAL_BAUD_RATE:
                print(f"  Note: Using detected baud rate {detected_baud} instead of configured {SERIAL_BAUD_RATE}")
            
            serial_port_obj = serial.Serial(
                port=detected_port,
                baudrate=actual_baud,
                timeout=SERIAL_TIMEOUT,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Wait for connection to stabilize
            time.sleep(0.5)
            # Flush all buffers aggressively
            serial_port_obj.reset_input_buffer()
            serial_port_obj.reset_output_buffer()
            # Read and discard any initial noise
            if serial_port_obj.in_waiting > 0:
                noise = serial_port_obj.read(serial_port_obj.in_waiting)
                if len(noise) > 0 and not all(b == 0 for b in noise):
                    print(f"  Note: Discarded {len(noise)} bytes of initial data")
            print("✓ Connected to Nextion display")
            
            # Initialize motor status to 0 (not moving)
            update_motor_status(0)
            
            # Start serial reading thread
            serial_thread = threading.Thread(target=read_serial_messages, daemon=True)
            serial_thread.start()
            print("✓ Serial message monitoring started")
            print(f"  Waiting for button presses (expecting format: BTN:RESETHOME, BTN:MANCLAMP, or BTN:ESTOP)")
        except Exception as e:
            print(f"⚠ Warning: Failed to connect to Nextion display at {detected_port}: {e}")
            print("  Continuing without serial input (keyboard commands still work)")
            serial_port_obj = None
    else:
        # Fallback: try the configured port if auto-detection failed
        print(f"Auto-detection failed, trying configured port {SERIAL_PORT} at {SERIAL_BAUD_RATE} baud...")
        try:
            serial_port_obj = serial.Serial(
                port=SERIAL_PORT,
                baudrate=SERIAL_BAUD_RATE,
                timeout=SERIAL_TIMEOUT,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # Wait for connection to stabilize
            time.sleep(0.5)
            # Flush all buffers aggressively
            serial_port_obj.reset_input_buffer()
            serial_port_obj.reset_output_buffer()
            # Read and discard any initial noise
            if serial_port_obj.in_waiting > 0:
                noise = serial_port_obj.read(serial_port_obj.in_waiting)
                if len(noise) > 0 and not all(b == 0 for b in noise):
                    print(f"  Note: Discarded {len(noise)} bytes of initial data")
            print("✓ Connected to Nextion display (using configured port)")
            
            # Initialize motor status to 0 (not moving)
            update_motor_status(0)
            
            # Start serial reading thread
            serial_thread = threading.Thread(target=read_serial_messages, daemon=True)
            serial_thread.start()
            print("✓ Serial message monitoring started")
        except Exception as e:
            print(f"⚠ Warning: Failed to connect to Nextion display: {e}")
            print("  Continuing without serial input (keyboard commands still work)")
            serial_port_obj = None
    
    # Connect to motor gateway with infinite retries
    print("\nConnecting to motor gateway...")
    print(f"Gateway: {GATEWAY_IP}:{GATEWAY_PORT}")
    print("(Will retry indefinitely until connection succeeds)")
    
    retry_delay = 2  # Start with 2 second delay
    attempt = 0
    success = False
    
    while not success:
        attempt += 1
        if attempt > 1:
            print(f"\nRetry attempt {attempt} (will keep trying until successful)...")
            time.sleep(retry_delay)
            # Gradually increase delay up to 10 seconds
            retry_delay = min(retry_delay + 1, 10)
        
        # Test connectivity first
        import socket
        try:
            test_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            test_socket.settimeout(2)
            result = test_socket.connect_ex((GATEWAY_IP, GATEWAY_PORT))
            test_socket.close()
            
            if result != 0:
                print(f"✗ Cannot connect to {GATEWAY_IP}:{GATEWAY_PORT} (Error code: {result})")
                update_motor_status(2)  # Error status - update Nextion
                print("  Retrying in a few seconds...")
                continue
        except Exception as e:
            print(f"⚠ Could not test connectivity: {e}")
            update_motor_status(2)  # Error status - update Nextion
            continue
        
        # Try to connect
        success, error = connect(GATEWAY_IP, GATEWAY_PORT)
        if success:
            break
        else:
            print(f"✗ Connection attempt failed: {error}")
            update_motor_status(2)  # Error status - update Nextion
            print("  Retrying in a few seconds...")
    
    print("✓ Connected to motor gateway")
    motor_connected = True
    
    # Enable motor
    print("Enabling motor...")
    success, error = enable_motor(MOTOR_NODE_ID, True)
    if not success:
        print(f"✗ Failed to enable motor: {error}")
        update_motor_status(2)  # Error status
        # Don't exit - keep retrying connection
        print("  Will retry motor enable on next connection attempt")
        # Disconnect and retry
        disconnect()
        motor_connected = False
        # Go back to connection retry loop
        while not success:
            attempt += 1
            print(f"\nRetry attempt {attempt}...")
            time.sleep(retry_delay)
            retry_delay = min(retry_delay + 1, 10)
            
            success, error = connect(GATEWAY_IP, GATEWAY_PORT)
            if success:
                motor_connected = True
                success, error = enable_motor(MOTOR_NODE_ID, True)
                if success:
                    break
                else:
                    print(f"✗ Failed to enable motor: {error}")
                    update_motor_status(2)
                    disconnect()
                    motor_connected = False
                    success = False
    
    print("✓ Motor enabled")
    
    # Start input thread for non-blocking command input
    input_thread = threading.Thread(target=read_user_input, daemon=True)
    input_thread.start()
    
    # Start GPIO control monitoring thread
    gpio_control_thread = threading.Thread(target=monitor_gpio_control, daemon=True)
    gpio_control_thread.start()
    print("✓ GPIO control monitoring started (GPIO 26: HIGH=clamp, LOW=home)")
    
    # Initialize homing_complete flag
    global homing_complete
    homing_complete = False
    
    try:
        print("\n" + "=" * 60)
        print("Ready for commands:")
        print("  Keyboard: 'h' for homing, 'c' for clamping, 's' to stop, 'q' to quit")
        if serial_port_obj:
            print("  Serial: RESETHOME (homing), MANCLAMP (clamping), ESTOP (emergency stop)")
        print("=" * 60)
        print("(Note: Commands are read in a separate thread, type and press Enter)")
        print("=" * 60)
        
        # Automatically start homing sequence on startup
        print("\n" + "=" * 60)
        print("Starting automatic homing sequence...")
        print("=" * 60)
        stop_requested = False
        homing_complete = homing_sequence()
        if homing_complete:
            print("\n✓ Automatic homing completed successfully!")
        else:
            print("\n⚠ Warning: Automatic homing failed or was interrupted")
        print("=" * 60)
        print("\nReady for commands...")
        
        while True:
            # Check for commands
            while command_queue:
                cmd = command_queue.pop(0)
                
                if cmd in ['q', 'quit', 'exit']:
                    print("\nQuitting...")
                    return
                
                elif cmd in ['s', 'stop']:
                    print("\nEmergency stop requested!")
                    stop_motor(MOTOR_NODE_ID)
                    update_motor_status(0)  # Motor stopped
                    stop_requested = True
                    time.sleep(0.1)
                    stop_requested = False
                    print("Motor stopped")
                
                elif cmd in ['h', 'home']:
                    stop_requested = False
                    homing_complete = homing_sequence()
                
                elif cmd in ['c', 'clamp']:
                    if homing_complete:
                        stop_requested = False
                        clamping_sequence()
                    else:
                        print("\nPlease complete homing first (type 'h')")
                else:
                    print(f"\nUnknown command: {cmd}")
                    print("Valid commands: h (home), c (clamp), s (stop), q (quit)")
            
            time.sleep(0.1)  # Small delay to avoid excessive CPU usage
            
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup - ensure proper disconnection
        print("\nCleaning up...")
        input_thread_running = False
        serial_thread_running = False
        gpio_control_thread_running = False
        
        # Update status to stopped before closing serial port
        update_motor_status(0)
        
        # Stop motor first (critical - must stop before disabling)
        try:
            stop_motor(MOTOR_NODE_ID)
            time.sleep(0.2)  # Give more time for stop to complete
        except Exception as e:
            print(f"  Warning during stop: {e}")
        
        # Close serial port
        if serial_port_obj and serial_port_obj.is_open:
            try:
                serial_port_obj.close()
                print("  Serial port closed")
            except Exception as e:
                print(f"  Warning during serial close: {e}")
        
        # Disable motor (critical - must disable before disconnect)
        try:
            enable_motor(MOTOR_NODE_ID, False)
            time.sleep(0.2)  # Give more time for disable to complete
            print("  Motor disabled")
        except Exception as e:
            print(f"  Warning during disable: {e}")
        
        # Disconnect from gateway - with retry and force close
        try:
            # First try normal disconnect
            success, error = disconnect()
            if not success:
                print(f"  Warning: Normal disconnect failed: {error}")
            
            time.sleep(0.3)  # Give time for disconnect to complete
            
            # Force close socket if still connected (in case normal disconnect didn't work)
            import motor_api
            if hasattr(motor_api, '_sdk_instance') and motor_api._sdk_instance:
                try:
                    sdk = motor_api._sdk_instance
                    # Close socket directly
                    if hasattr(sdk, 'socket') and sdk.socket:
                        try:
                            sdk.socket.shutdown(socket.SHUT_RDWR)
                        except:
                            pass
                        try:
                            sdk.socket.close()
                        except:
                            pass
                        sdk.socket = None
                    # Reset connection state
                    sdk.connected = False
                    sdk.gateway_handle = 0
                    # Also reset in motor_api module
                    motor_api._connected = False
                except Exception as e:
                    print(f"  Warning during force close: {e}")
        except Exception as e:
            print(f"Warning during disconnect: {e}")
            # Try to force close anyway
            try:
                import motor_api
                import socket
                if hasattr(motor_api, '_sdk_instance') and motor_api._sdk_instance:
                    sdk = motor_api._sdk_instance
                    if hasattr(sdk, 'socket') and sdk.socket:
                        try:
                            sdk.socket.shutdown(socket.SHUT_RDWR)
                        except:
                            pass
                        try:
                            sdk.socket.close()
                        except:
                            pass
                        sdk.socket = None
                    sdk.connected = False
                    sdk.gateway_handle = 0
                    motor_api._connected = False
            except:
                pass
        
        # Cleanup GPIO
        try:
            GPIO.cleanup()
        except:
            pass
        
        print("Done!")

if __name__ == '__main__':
    main()

