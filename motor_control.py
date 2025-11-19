#!/usr/bin/env python3
"""
Motor control for UIRobot UIM5756CM via UIM2523 Ethernet-CAN gateway.

This script provides control interface for the motor using CAN bus communication
through the Ethernet gateway.

Hardware:
- Motor: UIRobot UIM5756CM
- Gateway: UIRobot UIM2523 (Ethernet to CAN)
"""

import sys
import time
from uirobot_sdk import UIRobotSDK, GATEWAY_INFO_OBJ, MEMBER_INFO_OBJ

# Configuration
GATEWAY_IP = "192.168.1.254"
GATEWAY_PORT = 8888
CAN_NODE_ID = 5  # Motor CAN node ID

class MotorController:
    """Controller for UIM5756CM motor via UIM2523 gateway"""
    
    def __init__(self, gateway_ip, gateway_port, can_node_id):
        """
        Initialize motor controller.
        
        Args:
            gateway_ip: IP address of UIM2523 gateway
            gateway_port: Port number for gateway communication
            can_node_id: CAN node ID of the motor
        """
        self.gateway_ip = gateway_ip
        self.gateway_port = gateway_port
        self.can_node_id = can_node_id
        self.sdk = UIRobotSDK()
        self.gateway_info = GATEWAY_INFO_OBJ()
        self.member_info = MEMBER_INFO_OBJ()
        
    def connect(self):
        """Establish connection to the gateway and open CAN network"""
        # Configure gateway info
        # IP format: [254, 1, 168, 192] = 192.168.1.254
        ip_parts = [int(x) for x in self.gateway_ip.split('.')]
        self.gateway_info.IP = [ip_parts[3], ip_parts[2], ip_parts[1], ip_parts[0]]
        self.gateway_info.IPport = self.gateway_port
        self.gateway_info.type = 2  # UIGW2_ETHCAN
        
        # Configure member info
        self.member_info.CANnid = self.can_node_id
        
        # Open CAN network
        result = self.sdk.SdkStartCanNet(self.gateway_info, self.member_info)
        if result == 0:
            print(f"✓ Connected to gateway and opened CAN network")
            print(f"  Gateway: {self.gateway_ip}:{self.gateway_port}")
            print(f"  Motor CAN ID: {self.can_node_id}")
            return True
        else:
            print(f"✗ Failed to connect: Error code {result}")
            return False
    
    def disconnect(self):
        """Close connection to gateway"""
        self.sdk.SdkCloseCanNet()
        print("Disconnected from gateway")
    
    def enable_motor(self, enable=True):
        """
        Enable or disable the motor.
        
        Args:
            enable: True to enable, False to disable
        """
        result = self.sdk.SdkSetMotorOn(
            self.gateway_info.GtwyHandle,
            self.can_node_id,
            1 if enable else 0
        )
        if result == 0:
            status = "enabled" if enable else "disabled"
            print(f"✓ Motor {status}")
        else:
            print(f"✗ Failed to set motor enable: Error {result}")
        return result == 0
    
    def set_jog_speed(self, speed):
        """
        Set jog motion speed (for continuous motion)
        
        Args:
            speed: Speed value (signed 32-bit, units depend on motor configuration)
                  Positive = forward, Negative = reverse
        """
        result = self.sdk.SdkSetJogVelocity(
            self.gateway_info.GtwyHandle,
            self.can_node_id,
            speed
        )
        if result == 0:
            print(f"✓ Jog speed set to {speed}")
        else:
            print(f"✗ Failed to set jog speed: Error {result}")
        return result == 0
    
    def set_ptp_speed(self, speed):
        """
        Set PTP (Point-to-Point) motion speed
        
        Args:
            speed: Speed value (signed 32-bit, units depend on motor configuration)
        """
        result = self.sdk.SdkSetPtpSpeed(
            self.gateway_info.GtwyHandle,
            self.can_node_id,
            speed
        )
        if result == 0:
            print(f"✓ PTP speed set to {speed}")
        else:
            print(f"✗ Failed to set PTP speed: Error {result}")
        return result == 0
    
    def set_position(self, position, absolute=True):
        """
        Set target position for PTP (Point-to-Point) motion
        
        Args:
            position: Target position (signed 32-bit, units depend on motor configuration)
            absolute: True for absolute position, False for relative
        """
        if absolute:
            result = self.sdk.SdkSetPositionAbsolute(
                self.gateway_info.GtwyHandle,
                self.can_node_id,
                position
            )
            pos_type = "absolute"
        else:
            result = self.sdk.SdkSetPositionRelative(
                self.gateway_info.GtwyHandle,
                self.can_node_id,
                position
            )
            pos_type = "relative"
        
        if result == 0:
            print(f"✓ Position set to {position} ({pos_type})")
        else:
            print(f"✗ Failed to set position: Error {result}")
        return result == 0
    
    def start_motion(self):
        """
        Start motor motion
        
        Note: Motion parameters (speed/position) must be set first using:
        - set_jog_speed() for continuous motion
        - set_position() for point-to-point motion
        """
        result = self.sdk.SdkSetMotionMxn(
            self.gateway_info.GtwyHandle,
            self.can_node_id
        )
        if result == 0:
            print("✓ Motion started (BG command sent)")
            print("  Note: If motor doesn't move, set motion parameters first:")
            print("    - Use set_jog_speed() for continuous motion")
            print("    - Use set_position() for point-to-point motion")
        else:
            print(f"✗ Failed to start motion: Error {result}")
        return result == 0
    
    def emergency_stop(self):
        """Emergency stop the motor"""
        result = self.sdk.SdkSetStopMxn(
            self.gateway_info.GtwyHandle,
            self.can_node_id
        )
        if result == 0:
            print("✓ Emergency stop executed")
        else:
            print(f"✗ Failed to execute emergency stop: Error {result}")
        return result == 0


def main():
    """Example usage"""
    print("UIRobot Motor Control")
    print("=" * 60)
    print(f"Gateway: {GATEWAY_IP}:{GATEWAY_PORT}")
    print(f"Motor CAN ID: {CAN_NODE_ID}")
    print("=" * 60)
    print()
    
    # Check network setup
    import subprocess
    result = subprocess.run(['ip', 'addr', 'show'], capture_output=True, text=True)
    if '192.168.1.222' not in result.stdout:
        print("⚠ Warning: Virtual IP 192.168.1.222 not configured")
        print("  Run: ./setup_network.sh")
        print()
    
    # Initialize controller
    controller = MotorController(GATEWAY_IP, GATEWAY_PORT, CAN_NODE_ID)
    
    # Connect to gateway
    if not controller.connect():
        print("\nFailed to connect. Please check:")
        print(f"1. Gateway IP: {GATEWAY_IP}")
        print(f"2. Gateway Port: {GATEWAY_PORT}")
        print("3. Network connectivity (run ./setup_network.sh)")
        print("4. Gateway is powered on")
        sys.exit(1)
    
    try:
        print("\n" + "=" * 60)
        print("Motor Control Ready")
        print("=" * 60)
        print("\nAvailable commands:")
        print("  1 - Enable motor")
        print("  2 - Disable motor")
        print("  3 - Start motion (BG - requires motion params set first)")
        print("  4 - Emergency stop")
        print("  5 - Set jog speed (for continuous motion)")
        print("  6 - Set PTP speed (for point-to-point motion)")
        print("  7 - Set absolute position")
        print("  8 - Set relative position")
        print("  q - Quit")
        print("\nNote: For motion to work:")
        print("  - Jog mode: Set jog speed (5), then start motion (3)")
        print("  - PTP mode: Set PTP speed (6) and position (7 or 8), then start motion (3)")
        print("=" * 60)
        print()
        
        # Interactive command loop
        while True:
            try:
                cmd = input("Enter command (1-4, q): ").strip().lower()
                
                if cmd == 'q' or cmd == 'quit':
                    print("\nExiting...")
                    break
                elif cmd == '1':
                    controller.enable_motor(True)
                elif cmd == '2':
                    controller.enable_motor(False)
                elif cmd == '3':
                    controller.start_motion()
                elif cmd == '4':
                    controller.emergency_stop()
                elif cmd == '5':
                    try:
                        speed = int(input("Enter jog speed (signed int, e.g. 1000): "))
                        controller.set_jog_speed(speed)
                    except ValueError:
                        print("Invalid speed value")
                elif cmd == '6':
                    try:
                        speed = int(input("Enter PTP speed (signed int, e.g. 1000): "))
                        controller.set_ptp_speed(speed)
                    except ValueError:
                        print("Invalid speed value")
                elif cmd == '7':
                    try:
                        position = int(input("Enter absolute position (signed int): "))
                        controller.set_position(position, absolute=True)
                    except ValueError:
                        print("Invalid position value")
                elif cmd == '8':
                    try:
                        position = int(input("Enter relative position (signed int): "))
                        controller.set_position(position, absolute=False)
                    except ValueError:
                        print("Invalid position value")
                elif cmd == '':
                    continue
                else:
                    print(f"Unknown command: {cmd}")
                    print("Valid commands: 1, 2, 3, 4, 5, 6, 7, 8, q")
                
                print()  # Blank line for readability
                
            except EOFError:
                # Handle Ctrl+D
                print("\nExiting...")
                break
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        controller.emergency_stop()
    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
