# Motor Control Documentation

## Hardware

- **Motor**: UIRobot UIM5756CM
- **Gateway**: UIRobot UIM2523 (Ethernet to CAN)

## Network Setup

The system requires a virtual network interface to communicate with devices on the 192.168.1.0 subnet while maintaining connection to the main network (192.168.4.0).

### Setup Virtual Network Interface

Run the setup script:
```bash
./setup_network.sh
```

This will:
- Add virtual IP address 192.168.1.222/24 to eth0
- Test connectivity to gateway (192.168.1.254)

The virtual IP is added temporarily. To make it persistent, add to `/etc/network/interfaces` or use NetworkManager.

## Configuration

- **Gateway IP**: 192.168.1.254
- **Gateway Port**: 8888
- **Motor CAN Node ID**: 1 (default, may need adjustment)

## Implementation Status

### ✅ Completed:
- [x] Network interface setup script
- [x] SDK Python wrapper structure (`uirobot_sdk.py`)
- [x] Motor controller class (`motor_control.py`)
- [x] Basic connection handling
- [x] Function stubs for motor control

### ⚠️ Needs Protocol Implementation:
- [ ] Actual TCP message format/encoding
- [ ] CAN network open command implementation
- [ ] Motor enable/disable command implementation
- [ ] Motion start/stop command implementation
- [ ] Status query implementation
- [ ] Error handling and response parsing

## Protocol Notes

The UIRobot SDK uses a TCP-based protocol on port 8888. The SDK documentation shows function calls (SdkStartCanNet, SdkSetMotorOn, etc.) but the underlying TCP message format needs to be determined.

**Options to complete implementation:**
1. Use Wireshark to capture actual SDK traffic
2. Obtain detailed protocol specification
3. Test with actual hardware and reverse engineer

## Usage

### Basic Usage:
```bash
python3 motor_control.py
```

### Programmatic Usage:
```python
from motor_control import MotorController

controller = MotorController("192.168.1.254", 8888, 1)
controller.connect()
controller.enable_motor(True)
controller.start_motion()
# ... do work ...
controller.emergency_stop()
controller.disconnect()
```

## Files

- `motor_control.py` - Main motor control script
- `uirobot_sdk.py` - SDK Python wrapper
- `setup_network.sh` - Network interface setup script
- `docs/` - Documentation folder

## Troubleshooting

1. **Cannot connect to gateway:**
   - Verify gateway IP: `ping 192.168.1.254`
   - Check virtual IP: `ip addr show | grep 192.168.1.222`
   - Run `./setup_network.sh`

2. **Motor not responding:**
   - Verify CAN node ID is correct
   - Check motor is powered on
   - Verify CAN bus connections

3. **Protocol errors:**
   - The TCP message format needs to be implemented
   - May need to capture actual SDK traffic with Wireshark
   - Check SDK documentation for binary protocol details
