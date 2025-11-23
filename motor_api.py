#!/usr/bin/env python3
"""
Simple Motor Control API for UIRobot UIM5756CM via UIM2523 Gateway

This module provides a clean, simple interface for controlling motors.
"""

from uirobot_sdk import UIRobotSDK, GATEWAY_INFO_OBJ, MEMBER_INFO_OBJ
from typing import Optional, Tuple

# Default gateway configuration
DEFAULT_GATEWAY_IP = "192.168.1.254"
DEFAULT_GATEWAY_PORT = 8888

# Global SDK instance (singleton pattern)
_sdk_instance = None
_gateway_info = None
_connected = False


def connect(gateway_ip: str = DEFAULT_GATEWAY_IP, gateway_port: int = DEFAULT_GATEWAY_PORT) -> Tuple[bool, Optional[str]]:
    """
    Connect to the gateway and initialize CAN network.
    
    Args:
        gateway_ip: Gateway IP address (default: 192.168.1.254)
        gateway_port: Gateway port (default: 8888)
        
    Returns:
        Tuple of (success: bool, error_message: str or None)
    """
    global _sdk_instance, _gateway_info, _connected
    
    try:
        _sdk_instance = UIRobotSDK()
        _gateway_info = GATEWAY_INFO_OBJ()
        
        # Configure gateway info
        ip_parts = [int(x) for x in gateway_ip.split('.')]
        _gateway_info.IP = [ip_parts[3], ip_parts[2], ip_parts[1], ip_parts[0]]
        _gateway_info.IPport = gateway_port
        _gateway_info.type = 2  # UIGW2_ETHCAN
        
        # Open CAN network
        member_info = MEMBER_INFO_OBJ()
        result = _sdk_instance.SdkStartCanNet(_gateway_info, member_info)
        
        if result == 0:
            _connected = True
            return (True, None)
        else:
            return (False, f"Failed to connect: Error code {result}")
            
    except Exception as e:
        return (False, f"Connection error: {str(e)}")


def disconnect() -> Tuple[bool, Optional[str]]:
    """
    Disconnect from the gateway.
    
    Returns:
        Tuple of (success: bool, error_message: str or None)
    """
    global _sdk_instance, _connected
    
    try:
        if _sdk_instance:
            _sdk_instance.SdkCloseCanNet()
        _connected = False
        return (True, None)
    except Exception as e:
        return (False, f"Disconnect error: {str(e)}")


def enable_motor(node_id: int, enable: bool = True) -> Tuple[bool, Optional[str]]:
    """
    Enable or disable a motor.
    
    Args:
        node_id: Motor CAN node ID
        enable: True to enable, False to disable
        
    Returns:
        Tuple of (success: bool, error_message: str or None)
    """
    global _sdk_instance, _gateway_info, _connected
    
    if not _connected or not _sdk_instance:
        return (False, "Not connected to gateway. Call connect() first.")
    
    try:
        result = _sdk_instance.SdkSetMotorOn(
            _gateway_info.GtwyHandle,
            node_id,
            1 if enable else 0
        )
        
        if result == 0:
            return (True, None)
        else:
            return (False, f"Failed to set motor enable: Error code {result}")
            
    except Exception as e:
        return (False, f"Error: {str(e)}")


def move_motor(node_id: int, 
               position: Optional[int] = None,
               relative: bool = False,
               speed: Optional[int] = None,
               jog_speed: Optional[int] = None) -> Tuple[bool, Optional[str]]:
    """
    Move motor to a position or start continuous motion.
    
    Args:
        node_id: Motor CAN node ID
        position: Target position (for PTP motion). If None, uses jog mode.
        relative: If True, position is relative. If False, position is absolute.
                  Only used when position is provided.
        speed: PTP speed (for point-to-point motion). Required if position is provided.
        jog_speed: Jog velocity (for continuous motion). Required if position is None.
        
    Returns:
        Tuple of (success: bool, error_message: str or None)
        
    Examples:
        # Continuous forward motion at speed 1000
        move_motor(5, jog_speed=1000)
        
        # Move to absolute position 5000 at speed 2000
        move_motor(5, position=5000, relative=False, speed=2000)
        
        # Move relative +1000 steps at speed 1500
        move_motor(5, position=1000, relative=True, speed=1500)
    """
    global _sdk_instance, _gateway_info, _connected
    
    if not _connected or not _sdk_instance:
        return (False, "Not connected to gateway. Call connect() first.")
    
    try:
        # Determine motion mode
        if position is not None:
            # PTP (Point-to-Point) motion
            if speed is None:
                return (False, "PTP motion requires speed parameter")
            
            # Set PTP speed
            result = _sdk_instance.SdkSetPtpSpeed(
                _gateway_info.GtwyHandle,
                node_id,
                speed
            )
            if result != 0:
                return (False, f"Failed to set PTP speed: Error code {result}")
            
            # Set position
            if relative:
                result = _sdk_instance.SdkSetPositionRelative(
                    _gateway_info.GtwyHandle,
                    node_id,
                    position
                )
            else:
                result = _sdk_instance.SdkSetPositionAbsolute(
                    _gateway_info.GtwyHandle,
                    node_id,
                    position
                )
            
            if result != 0:
                return (False, f"Failed to set position: Error code {result}")
                
        else:
            # Jog (continuous) motion
            if jog_speed is None:
                return (False, "Jog motion requires jog_speed parameter")
            
            # Set jog velocity
            result = _sdk_instance.SdkSetJogVelocity(
                _gateway_info.GtwyHandle,
                node_id,
                jog_speed
            )
            if result != 0:
                return (False, f"Failed to set jog speed: Error code {result}")
        
        # Start motion (BG command)
        result = _sdk_instance.SdkSetMotionMxn(
            _gateway_info.GtwyHandle,
            node_id
        )
        
        if result == 0:
            return (True, None)
        else:
            return (False, f"Failed to start motion: Error code {result}")
            
    except Exception as e:
        return (False, f"Error: {str(e)}")


def stop_motor(node_id: int) -> Tuple[bool, Optional[str]]:
    """
    Emergency stop the motor.
    
    Args:
        node_id: Motor CAN node ID
        
    Returns:
        Tuple of (success: bool, error_message: str or None)
    """
    global _sdk_instance, _gateway_info, _connected
    
    if not _connected or not _sdk_instance:
        return (False, "Not connected to gateway. Call connect() first.")
    
    try:
        result = _sdk_instance.SdkSetStopMxn(
            _gateway_info.GtwyHandle,
            node_id
        )
        
        if result == 0:
            return (True, None)
        else:
            return (False, f"Failed to stop motor: Error code {result}")
            
    except Exception as e:
        return (False, f"Error: {str(e)}")


def get_motor_position(node_id: int) -> Tuple[bool, Optional[int], Optional[str]]:
    """
    Get motor's current absolute position.
    
    Args:
        node_id: Motor CAN node ID
        
    Returns:
        Tuple of (success: bool, position: int or None, error_message: str or None)
    """
    global _sdk_instance, _gateway_info, _connected
    
    if not _connected or not _sdk_instance:
        return (False, None, "Not connected to gateway. Call connect() first.")
    
    try:
        error_code, velocity, position = _sdk_instance.SdkGetPtpMxnA(
            _gateway_info.GtwyHandle,
            node_id
        )
        
        if error_code == 0:
            return (True, position, None)
        else:
            return (False, None, f"Failed to get position: Error code {error_code}")
            
    except Exception as e:
        return (False, None, f"Error: {str(e)}")


def reset_position_to_zero(node_id: int) -> Tuple[bool, Optional[str]]:
    """
    Reset motor's position register to zero without moving the motor.
    This sets the current position as the zero/reference point using the OG (Set Origin) command.
    
    Args:
        node_id: Motor CAN node ID
        
    Returns:
        Tuple of (success: bool, error_message: str or None)
    """
    global _sdk_instance, _gateway_info, _connected
    
    if not _connected or not _sdk_instance:
        return (False, "Not connected to gateway. Call connect() first.")
    
    try:
        # Use OG (Set Origin) command to set current position as zero
        result = _sdk_instance.SdkSetOrigin(
            _gateway_info.GtwyHandle,
            node_id
        )
        
        if result == 0:
            return (True, None)
        else:
            return (False, f"Failed to reset position: Error code {result}")
            
    except Exception as e:
        return (False, f"Error: {str(e)}")


# Convenience functions with default node ID
def move_motor_5(position: Optional[int] = None,
                 relative: bool = False,
                 speed: Optional[int] = None,
                 jog_speed: Optional[int] = None) -> Tuple[bool, Optional[str]]:
    """
    Move motor with node ID 5 (convenience function).
    
    See move_motor() for parameter details.
    """
    return move_motor(5, position, relative, speed, jog_speed)


def enable_motor_5(enable: bool = True) -> Tuple[bool, Optional[str]]:
    """Enable/disable motor with node ID 5 (convenience function)."""
    return enable_motor(5, enable)


def stop_motor_5() -> Tuple[bool, Optional[str]]:
    """Stop motor with node ID 5 (convenience function)."""
    return stop_motor(5)


if __name__ == '__main__':
    # Example usage
    print("Motor Control API Example")
    print("=" * 60)
    
    # Connect
    success, error = connect()
    if not success:
        print(f"Failed to connect: {error}")
        exit(1)
    print("✓ Connected to gateway")
    
    try:
        # Enable motor
        success, error = enable_motor(5, True)
        if not success:
            print(f"Failed to enable motor: {error}")
        else:
            print("✓ Motor enabled")
        
        # Example 1: Continuous forward motion
        print("\nExample 1: Starting continuous forward motion...")
        success, error = move_motor(5, jog_speed=1000)
        if success:
            print("✓ Motion started")
        else:
            print(f"✗ Failed: {error}")
        
        # Wait a bit
        import time
        time.sleep(2)
        
        # Stop
        print("\nStopping motor...")
        success, error = stop_motor(5)
        if success:
            print("✓ Motor stopped")
        else:
            print(f"✗ Failed: {error}")
        
        # Example 2: Move to absolute position
        print("\nExample 2: Moving to absolute position 5000...")
        success, error = move_motor(5, position=5000, relative=False, speed=2000)
        if success:
            print("✓ Motion started")
        else:
            print(f"✗ Failed: {error}")
        
        time.sleep(3)
        
        # Stop
        stop_motor(5)
        
        # Disable motor
        enable_motor(5, False)
        print("\n✓ Motor disabled")
        
    finally:
        disconnect()
        print("Disconnected")

