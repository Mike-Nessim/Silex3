# Silex3

Repository for Silex3 project.

## Nextion Display Control

This project includes scripts to control a Nextion intelligent display via USB-TTL adapter.

### Setup

1. Connect your USB-TTL adapter to the Raspberry Pi
2. Connect the adapter to your Nextion display
3. Install dependencies: `pip install -r requirements.txt`

### Usage

**Update text field `t0` on the Nextion display:**
```bash
python3 update_nextion.py
```

The script will:
- Connect to `/dev/ttyUSB0` at 9600 baud
- Update text field `t0` to "Axton Robotics"
- Display status messages

**Monitor button presses:**
```bash
python3 monitor_buttons.py
```

This script will:
- Continuously monitor the serial port for button events
- Print a message when START or STOP buttons are pressed
- Display timestamps for each button press
- Press Ctrl+C to stop monitoring

**Nextion + Motor Integration:**
```bash
python3 nextion_motor_control.py
```

This script integrates Nextion display buttons with motor control:
- START button → Starts motor jogging
- STOP button → Stops motor
- **Position Display**: Updates text field `t0` with motor's absolute position twice per second while motor is running
- Automatically connects to both Nextion and motor gateway
- Handles cleanup on exit

### Configuration

You can modify the script to:
- Change the serial port (default: `/dev/ttyUSB0`)
- Change the baud rate (default: `9600`)
- Update different text fields or other Nextion components

## Motor Control

This project includes control for UIRobot UIM5756CM motor via UIM2523 Ethernet-CAN gateway.

### Hardware
- **Motor**: UIRobot UIM5756CM
- **Gateway**: UIRobot UIM2523 (Ethernet to CAN)
- **Gateway IP**: 192.168.1.254
- **Gateway Port**: 8888

### Setup

1. **Setup network interface:**
   ```bash
   ./setup_network.sh
   ```
   This adds virtual IP 192.168.1.222 to communicate with devices on 192.168.1.0 subnet.

2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

### Status
✅ **Fully Implemented**

**Current status:**
- ✅ Network setup script
- ✅ SDK Python wrapper with full protocol implementation
- ✅ Motor controller class (interactive CLI)
- ✅ Simple Motor Control API (`motor_api.py`)

See `docs/README_MOTOR.md` for detailed information.

### Usage

**Interactive CLI:**
```bash
python3 motor_control.py
```

**Simple API (recommended for integration):**
```python
from motor_api import connect, enable_motor, move_motor, stop_motor

# Connect to gateway
success, error = connect()
if not success:
    print(f"Error: {error}")
    exit(1)

# Enable motor
enable_motor(5, True)

# Move to absolute position 5000 at speed 2000
success, error = move_motor(5, position=5000, relative=False, speed=2000)
if not success:
    print(f"Error: {error}")

# Or continuous motion
move_motor(5, jog_speed=1000)  # Forward at speed 1000
time.sleep(2)
stop_motor(5)

# Disable motor
enable_motor(5, False)
```

See `motor_api_example.py` for more examples.

