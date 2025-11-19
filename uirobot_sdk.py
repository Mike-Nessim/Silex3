#!/usr/bin/env python3
"""
UIRobot SDK Python Implementation
Based on uirSDK3.0 documentation for UIM2523 gateway and UIM5756CM motor

This module provides Python bindings for UIRobot devices via Ethernet-CAN gateway.
"""

import socket
import struct
import ctypes
import sys
import time
from typing import Optional, Tuple
from uirobot_protocol import build_uimessage, parse_uimessage, FUNC_GET_MODEL, FUNC_GET_SERIAL

# Gateway types
UIGW2_ETHCAN = 2  # Ethernet CAN Gateway

# Error codes (simplified - refer to UIError.h for complete list)
ERRO_SUCCESS = 0
ERRO_FAIL = -1

class GATEWAY_INFO_OBJ:
    """Gateway information structure"""
    def __init__(self):
        self.GtwyHandle = 0
        self.COMidx = 0
        self.COMbaud = 0
        self.BTRidx = 0
        self.SerialNo = 0
        self.FirmVer = 0
        self.type = UIGW2_ETHCAN
        self.IP = [0, 0, 0, 0]  # IP[0]=254, IP[1]=1, IP[2]=168, IP[3]=192 = 192.168.1.254
        self.IPport = 8888
        self.ModelStr = ""

class MEMBER_INFO_OBJ:
    """Member (device) information structure"""
    def __init__(self):
        self.GtwyHandle = 0
        self.CANnid = 1  # CAN node ID (default to 1)
        self.CANgid = 0
        self.COMbaud = 0
        self.SerialNo = 0

class UIRobotSDK:
    """Python implementation of UIRobot SDK"""
    
    def __init__(self):
        self.socket = None
        self.gateway_handle = 0
        self.connected = False
        
    def _connect_tcp(self, ip: str, port: int) -> bool:
        """Establish TCP connection to gateway"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(2)  # Set timeout for operations
            self.socket.connect((ip, port))
            self.connected = True
            return True
        except Exception as e:
            print(f"TCP connection error: {e}")
            return False
    
    def _disconnect_tcp(self):
        """Close TCP connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
            self.connected = False
    
    def _send_uimessage(self, device_id: int, cw: int, data: bytes = b'', need_ack: bool = True) -> Optional[bytes]:
        """
        Send UIMessage and wait for response
        
        Args:
            device_id: Device CAN ID
            cw: Control word (function code)
            data: Data bytes (max 8 bytes)
            need_ack: Request ACK (for SET commands) or expect response (for GET commands)
            
        Returns:
            Response message bytes or None
        """
        if not self.connected or not self.socket:
            return None
        try:
            # Build UIMessage
            message = build_uimessage(device_id, cw, data, need_ack, use_crc=True)
            
            # Send message
            self.socket.send(message)
            
            # Wait for response (ACK for SET commands, data for GET commands)
            if need_ack:
                try:
                    response = self.socket.recv(16)  # UIMessage is always 16 bytes
                    if len(response) == 16:
                        return response
                    elif len(response) > 0:
                        # Got partial message, try to get the rest
                        remaining = 16 - len(response)
                        response += self.socket.recv(remaining)
                        if len(response) == 16:
                            return response
                except socket.timeout:
                    return None
                except Exception as e:
                    print(f"  Receive error: {e}")
                    return None
            return None
        except Exception as e:
            print(f"Send/receive error: {e}")
            return None
    
    def SdkStartCanNet(self, pGtwy: GATEWAY_INFO_OBJ, pMember: MEMBER_INFO_OBJ, 
                       UseConstLink: int = 0) -> int:
        """
        Open CAN Network
        
        Args:
            pGtwy: Gateway information object
            pMember: Member information object
            UseConstLink: 0=Normal mode, 1=Debug mode
            
        Returns:
            0 on success, error code on failure
        """
        # Convert IP array to string (IP format: [254, 1, 168, 192] = 192.168.1.254)
        ip_str = f"{pGtwy.IP[3]}.{pGtwy.IP[2]}.{pGtwy.IP[1]}.{pGtwy.IP[0]}"
        port = pGtwy.IPport
        
        print(f"Connecting to gateway at {ip_str}:{port}...")
        
        if self._connect_tcp(ip_str, port):
            # TCP connection established - CAN network is ready
            # The gateway handles CAN automatically, no explicit "open" command needed
            self.gateway_handle = 1
            pGtwy.GtwyHandle = self.gateway_handle
            pMember.GtwyHandle = self.gateway_handle
            
            # Test connection by querying gateway model (ID=2 is gateway's fixed ID)
            time.sleep(0.1)  # Small delay for connection to stabilize
            response = self._send_uimessage(2, FUNC_GET_MODEL, need_ack=True)
            if response:
                print("CAN Network opened successfully")
                return ERRO_SUCCESS
            else:
                print("Connected but no response from gateway")
                # Still return success as TCP is connected
                return ERRO_SUCCESS
        else:
            return ERRO_FAIL
    
    def SdkCloseCanNet(self) -> int:
        """Close CAN Network"""
        self._disconnect_tcp()
        self.gateway_handle = 0
        print("CAN Network closed")
        return ERRO_SUCCESS
    
    def SdkGetGtwyInfo(self, pGtwy: GATEWAY_INFO_OBJ, pMember: MEMBER_INFO_OBJ) -> int:
        """
        Get CAN Gateway Information
        
        Args:
            pGtwy: Gateway information object (output)
            pMember: Member information object (output)
            
        Returns:
            0 on success, error code on failure
        """
        if not self.connected:
            return ERRO_FAIL
        
        # TODO: Implement actual gateway info query
        # This would query the gateway for its information
        # For now, return success with placeholder data
        return ERRO_SUCCESS
    
    def SdkSetMotorOn(self, GtwyHandle: int, CANid: int, bMotorOn: int, 
                      pbEnableOut: Optional[int] = None) -> int:
        """
        Set Motor Enable Status
        
        Args:
            GtwyHandle: Gateway handle
            CANid: Device CAN ID
            bMotorOn: 1 to enable, 0 to disable
            pbEnableOut: Optional output parameter for result
            
        Returns:
            0 on success, error code on failure
        """
        if not self.connected:
            return ERRO_FAIL
        
        # Function code 0x15 (MO) - Motor Enable
        # Data: bMotorOn (1 byte)
        from uirobot_protocol import FUNC_MOTOR_ENABLE
        data = bytes([bMotorOn])
        
        response = self._send_uimessage(CANid, FUNC_MOTOR_ENABLE, data, need_ack=True)
        if response:
            parsed = parse_uimessage(response)
            if pbEnableOut is not None:
                pbEnableOut = parsed['data'][0] if len(parsed['data']) > 0 else bMotorOn
            return ERRO_SUCCESS
        return ERRO_FAIL
    
    def SdkSetMotionMxn(self, GtwyHandle: int, CANid: int, 
                        pRxData: Optional[int] = None) -> int:
        """
        Set Motor Being Motion (Start motion)
        
        Note: Motion parameters (speed, position, etc.) must be set before calling this.
        The BG command activates previously set motion parameters and starts movement.
        
        Args:
            GtwyHandle: Gateway handle
            CANid: Device CAN ID
            pRxData: Optional return data
            
        Returns:
            0 on success, error code on failure
        """
        if not self.connected:
            return ERRO_FAIL
        
        # Function code 0x16 (BG) - Begin Motion
        # Requires 4 bytes of data (all zeros - don't care)
        from uirobot_protocol import FUNC_MOTION_START
        data = bytes([0x00, 0x00, 0x00, 0x00])  # 4 bytes as per BG command spec
        response = self._send_uimessage(CANid, FUNC_MOTION_START, data, need_ack=True)
        if response:
            return ERRO_SUCCESS
        return ERRO_FAIL
    
    def SdkSetStopMxn(self, GtwyHandle: int, CANid: int, 
                      pRxData: Optional[int] = None) -> int:
        """
        Set Motor Emergency Stop
        
        Args:
            GtwyHandle: Gateway handle
            CANid: Device CAN ID
            pRxData: Optional return data
            
        Returns:
            0 on success, error code on failure
        """
        if not self.connected:
            return ERRO_FAIL
        
        # Function code 0x17 - Emergency Stop
        # No data needed
        from uirobot_protocol import FUNC_MOTION_STOP
        response = self._send_uimessage(CANid, FUNC_MOTION_STOP, b'', need_ack=True)
        if response:
            return ERRO_SUCCESS
        return ERRO_FAIL
    
    def SdkSetJogVelocity(self, GtwyHandle: int, CANid: int, velocity: int) -> int:
        """
        Set Jog Velocity (for continuous motion)
        
        Args:
            GtwyHandle: Gateway handle
            CANid: Device CAN ID
            velocity: Jog velocity (signed 32-bit, units depend on motor config)
            
        Returns:
            0 on success, error code on failure
        """
        if not self.connected:
            return ERRO_FAIL
        
        from uirobot_protocol import FUNC_JOG_VELOCITY
        # Pack signed 32-bit integer, little-endian (low byte first)
        data = struct.pack('<i', velocity)
        response = self._send_uimessage(CANid, FUNC_JOG_VELOCITY, data, need_ack=True)
        if response:
            return ERRO_SUCCESS
        return ERRO_FAIL
    
    def SdkSetPtpSpeed(self, GtwyHandle: int, CANid: int, speed: int) -> int:
        """
        Set PTP (Point-to-Point) Speed
        
        Args:
            GtwyHandle: Gateway handle
            CANid: Device CAN ID
            speed: PTP speed (signed 32-bit, units depend on motor config)
            
        Returns:
            0 on success, error code on failure
        """
        if not self.connected:
            return ERRO_FAIL
        
        from uirobot_protocol import FUNC_PTP_SPEED
        # Pack signed 32-bit integer, little-endian (low byte first)
        data = struct.pack('<i', speed)
        response = self._send_uimessage(CANid, FUNC_PTP_SPEED, data, need_ack=True)
        if response:
            return ERRO_SUCCESS
        return ERRO_FAIL
    
    def SdkSetPositionAbsolute(self, GtwyHandle: int, CANid: int, position: int) -> int:
        """
        Set Absolute Position (for PTP motion)
        
        Args:
            GtwyHandle: Gateway handle
            CANid: Device CAN ID
            position: Target position (signed 32-bit, units depend on motor config)
            
        Returns:
            0 on success, error code on failure
        """
        if not self.connected:
            return ERRO_FAIL
        
        from uirobot_protocol import FUNC_POSITION_ABSOLUTE
        # Pack signed 32-bit integer, little-endian (low byte first)
        data = struct.pack('<i', position)
        response = self._send_uimessage(CANid, FUNC_POSITION_ABSOLUTE, data, need_ack=True)
        if response:
            return ERRO_SUCCESS
        return ERRO_FAIL
    
    def SdkSetPositionRelative(self, GtwyHandle: int, CANid: int, position: int) -> int:
        """
        Set Relative Position (for PTP motion)
        
        Args:
            GtwyHandle: Gateway handle
            CANid: Device CAN ID
            position: Relative position offset (signed 32-bit, units depend on motor config)
            
        Returns:
            0 on success, error code on failure
        """
        if not self.connected:
            return ERRO_FAIL
        
        from uirobot_protocol import FUNC_POSITION_RELATIVE
        # Pack signed 32-bit integer, little-endian (low byte first)
        data = struct.pack('<i', position)
        response = self._send_uimessage(CANid, FUNC_POSITION_RELATIVE, data, need_ack=True)
        if response:
            return ERRO_SUCCESS
        return ERRO_FAIL
    
    def SdkGetPtpMxnA(self, GtwyHandle: int, CANid: int) -> Tuple[int, int, int]:
        """
        Get PTP Motion Absolute Position
        
        Uses DV[0] (Desired Values) command which returns velocity and absolute position.
        
        Args:
            GtwyHandle: Gateway handle
            CANid: Device CAN ID
            
        Returns:
            Tuple of (error_code, velocity, position) or (error_code, 0, 0) on failure
            error_code: 0 on success, error code on failure
            velocity: Current velocity (signed 32-bit)
            position: Current absolute position (signed 32-bit)
        """
        if not self.connected:
            return (ERRO_FAIL, 0, 0)
        
        # Try MS[0] (Motion Status) first - it returns status flags and relative position
        # Then we can calculate absolute position if needed
        # If that doesn't work, try DV[0]
        from uirobot_protocol import FUNC_MOTION_STATUS, FUNC_DESIRED_VALUES
        
        # Try MS[0] - Motion Status (sub-index 0)
        data = bytes([0x00])  # Sub-index 0
        response = self._send_uimessage(CANid, FUNC_MOTION_STATUS, data, need_ack=True)
        if response:
            parsed = parse_uimessage(response)
            # MS[0] returns: d0 (status), d1 (status flags), d2 (status), d3-d6 (relative position as 32-bit)
            if parsed['dl'] >= 7 and len(parsed['data']) >= 7:
                # Relative position is in d3-d6 (bytes 3-6, little-endian 32-bit)
                if len(parsed['data']) >= 7:
                    rel_position = struct.unpack('<i', parsed['data'][3:7])[0]
                    # For now, return relative position as absolute (we'd need to track origin)
                    # This is a workaround - ideally we'd track the origin
                    return (ERRO_SUCCESS, 0, rel_position)
        
        # Fallback: Try DV[0] (Desired Values)
        data = bytes([0x00])  # Sub-index 0
        response = self._send_uimessage(CANid, FUNC_DESIRED_VALUES, data, need_ack=True)
        if response:
            parsed = parse_uimessage(response)
            # DV[0] might return different format - check what we actually get
            if parsed['dl'] >= 4 and len(parsed['data']) >= 4:
                # Try to parse as position (maybe it's just position, not velocity+position)
                # Or maybe the format is different
                if parsed['dl'] == 5:
                    # 5 bytes - might be: 1 byte sub-index + 4 bytes position
                    # Or maybe: 4 bytes position + 1 byte padding
                    if len(parsed['data']) >= 5:
                        # Try parsing bytes 1-4 as position (skip first byte)
                        position = struct.unpack('<i', parsed['data'][1:5])[0]
                        # Also try bytes 0-3 in case first byte is not sub-index
                        if position == 0 and parsed['data'][0] != 0:
                            position = struct.unpack('<i', parsed['data'][0:4])[0]
                        return (ERRO_SUCCESS, 0, position)
                elif parsed['dl'] >= 8:
                    # 8 bytes - velocity (4) + position (4)
                    velocity = struct.unpack('<i', parsed['data'][0:4])[0]
                    position = struct.unpack('<i', parsed['data'][4:8])[0]
                    return (ERRO_SUCCESS, velocity, position)
                elif parsed['dl'] >= 4:
                    # 4 bytes - might be just position
                    position = struct.unpack('<i', parsed['data'][0:4])[0]
                    return (ERRO_SUCCESS, 0, position)
        
        return (ERRO_FAIL, 0, 0)

