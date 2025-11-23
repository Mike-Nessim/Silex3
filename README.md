# Silex3

Repository for Silex3 project - Actuator control system with Nextion HMI integration.

## Overview

This project provides a complete actuator control system that integrates:
- **Motor Control**: UIRobot UIM5756CM motor via UIM2523 Ethernet-CAN gateway
- **Nextion HMI**: Touchscreen display for user interface and control
- **GPIO Sensors**: Homing and clamping sensors via Raspberry Pi GPIO
- **External Control**: GPIO 26 for external hardware control

The main program (`main.py`) provides automated homing and clamping sequences with robust error handling, automatic reconnection, and status updates to the Nextion display.

## Features

- ✅ **Automated Homing Sequence**: Moves until homing sensor (GPIO 10) is detected, then backs off and sets zero position
- ✅ **Automated Clamping Sequence**: Moves until clamping sensor (GPIO 9) is detected
- ✅ **Nextion HMI Integration**: Serial communication for commands and status display
- ✅ **GPIO Control**: External hardware control via GPIO 26 (HIGH=clamp, LOW=home)
- ✅ **Speed Control**: Unified speed control via Nextion MOTSPEED command (0-100, where 50 = base speed)
- ✅ **Motor Power Control**: Enable/disable motor via Nextion MOTORPOWER command
- ✅ **Automatic Reconnection**: Robust reconnection handling for both motor gateway and serial connections
- ✅ **Auto-start at Boot**: Systemd service for automatic startup
- ✅ **Status Updates**: Real-time motor status updates to Nextion display (0=not moving, 1=moving, 2=error, 3=clamped)

## Hardware Setup

### Motor Configuration
- **Motor**: UIRobot UIM5756CM
- **Gateway**: UIRobot UIM2523 (Ethernet to CAN)
- **Gateway IP**: 192.168.1.254
- **Gateway Port**: 8888
- **Motor Node ID**: 5

### GPIO Configuration
- **GPIO 10**: Homing sensor input (pull-down)
- **GPIO 9**: Clamping sensor input (pull-down)
- **GPIO 26**: External control input (HIGH=clamp, LOW=home, pull-down)

### Nextion Display
- **Serial Port**: Auto-detected (typically `/dev/ttyUSB0` or `/dev/ttyUSB1`)
- **Baud Rate**: 921600 (auto-detected)
- **Commands**:
  - `RESETHOME` - Start homing sequence
  - `MANCLAMP` - Start clamping sequence
  - `ESTOP` - Emergency stop
  - `MOTSPEED=value` - Set motor speed (0-100, where 50 = base speed)
  - `MOTORPOWER=0/1` - Disable/Enable motor

## Installation

### 1. Clone Repository
```bash
git clone <repository-url>
cd Silex3
```

### 2. Setup Network Interface
```bash
./setup_network.sh
```
This adds virtual IP 192.168.1.222 to communicate with devices on 192.168.1.0 subnet.

### 3. Create Virtual Environment
```bash
python3 -m venv venv
source venv/bin/activate
```

### 4. Install Dependencies
```bash
pip install -r requirements.txt
```

### 5. Configure Systemd Service (for auto-start at boot)

The service file is already created in the repository. To install it:

```bash
# Copy service file to systemd directory
sudo cp silex3.service /etc/systemd/system/

# Reload systemd daemon
sudo systemctl daemon-reload

# Enable service to start at boot
sudo systemctl enable silex3.service

# Start the service manually (optional, for testing)
sudo systemctl start silex3
```

## Usage

### Manual Execution

Run the program manually:

```bash
source venv/bin/activate
python3 main.py
```

### Service Management

Once the systemd service is installed:

```bash
# Start the service
sudo systemctl start silex3

# Stop the service
sudo systemctl stop silex3

# Restart the service (after making code changes)
sudo systemctl restart silex3

# Check service status
sudo systemctl status silex3

# View logs (live)
sudo journalctl -u silex3 -f

# View last 100 lines of logs
sudo journalctl -u silex3 -n 100

# Disable auto-start at boot (if needed)
sudo systemctl disable silex3
```

### Keyboard Commands (when running manually)

- `h` or `H` - Start homing sequence
- `c` or `C` - Start clamping sequence (after homing)
- `s` or `S` - Emergency stop motor
- `q` or `Q` - Quit program

### Nextion Serial Commands

The program listens for serial commands from the Nextion display:

- **RESETHOME** - Start homing sequence
- **MANCLAMP** - Start clamping sequence
- **ESTOP** - Emergency stop motor
- **MOTSPEED=value** - Set motor speed (0-100)
  - `0` = 0.5x base speed
  - `50` = 1.0x base speed (base speed = 12000)
  - `100` = 2.0x base speed
- **MOTORPOWER=0** - Disable motor
- **MOTORPOWER=1** - Enable motor

### GPIO 26 External Control

- **GPIO 26 HIGH** → Triggers clamping sequence (if homing is complete)
- **GPIO 26 LOW** → Triggers homing sequence

## Configuration

Edit `main.py` to adjust configuration variables:

```python
# Motor configuration
MOTOR_NODE_ID = 5
GATEWAY_IP = "192.168.1.254"
GATEWAY_PORT = 8888

# Serial configuration (for Nextion display)
SERIAL_PORT = '/dev/ttyUSB0'
SERIAL_BAUD_RATE = 921600

# Motor speed configuration
MOTOR_BASE_SPEED = 12000  # Base speed for motor motion

# Homing configuration
HOMING_GPIO = 10
HOMING_DIRECTION = 1  # 1 = positive, -1 = negative
HOMING_MAX_DISTANCE = 70000000
HOMING_BACKOFF_DISTANCE = 350000
HOMING_BACKOFF_DIRECTION = -1

# Clamping configuration
CLAMPING_GPIO = 9
CLAMPING_DIRECTION = -1  # 1 = positive, -1 = negative
CLAMPING_MAX_DISTANCE = 75000000

# External control GPIO
CONTROL_GPIO = 26  # HIGH = clamp, LOW = home
```

## Program Behavior

### Startup Sequence

1. **GPIO Setup**: Configures GPIO pins for sensors and external control
2. **Nextion Connection**: Auto-detects serial port and baud rate, connects to Nextion display
3. **Motor Gateway Connection**: Connects to motor gateway (retries indefinitely until successful)
4. **Motor Enable**: Enables the motor
5. **Automatic Homing**: Performs automatic homing sequence on startup
6. **Ready State**: Waits for commands from keyboard, serial, or GPIO

### Homing Sequence

1. Moves continuously in the homing direction at configured speed
2. Monitors GPIO 10 (homing sensor) continuously (every 1ms)
3. Stops immediately when sensor goes HIGH
4. Moves away from sensor by backoff distance
5. Resets motor position to zero
6. Marks homing as complete

### Clamping Sequence

1. Moves continuously in the clamping direction at configured speed
2. Monitors GPIO 9 (clamping sensor) continuously (every 1ms)
3. Stops immediately when sensor goes HIGH
4. Updates status to "clamped" (status = 3)

### Error Handling

- **Connection Loss**: Automatically attempts to reconnect to motor gateway
- **Serial Disconnection**: Continues operation, attempts to reconnect when serial messages are sent
- **Motor Errors**: Updates Nextion status to error state (status = 2)
- **Timeout Protection**: Maximum motion timeout prevents infinite motion
- **Distance Limits**: Maximum distance checks prevent over-travel

## Status Codes

The program updates the Nextion `motstatus` variable with:

- **0** - Motor not moving (idle)
- **1** - Motor moving
- **2** - Error or motor disabled
- **3** - Clamped (clamping sequence complete)

## Troubleshooting

### Service Won't Start

1. Check service status: `sudo systemctl status silex3`
2. View logs: `sudo journalctl -u silex3 -n 50`
3. Verify virtual environment exists: `ls -la venv/bin/python3`
4. Check file permissions: `ls -la main.py`

### Motor Connection Issues

- Verify network setup: `ip addr show` (should show 192.168.1.222)
- Test gateway connectivity: `ping 192.168.1.254`
- Check gateway IP configuration in `main.py`

### Serial Communication Issues

- Check serial port: `ls -la /dev/ttyUSB*`
- Verify baud rate matches Nextion configuration
- Check serial port permissions: `sudo usermod -a -G dialout $USER` (then logout/login)

### GPIO Issues

- Verify GPIO permissions: User must be in `gpio` group
- Check sensor wiring and pull-up/pull-down configuration
- Test GPIO inputs: `python3 test_gpio_inputs.py`

## File Structure

```
Silex3/
├── main.py                 # Main actuator control program
├── actuator_test.py        # Original test script (backup)
├── motor_api.py            # Motor control API wrapper
├── uirobot_sdk.py          # UIRobot SDK Python wrapper
├── uirobot_protocol.py     # Protocol definitions
├── silex3.service          # Systemd service file
├── requirements.txt        # Python dependencies
├── setup_network.sh       # Network setup script
└── README.md              # This file
```

## Development

### Testing Changes

1. Make changes to `main.py`
2. Test manually: `python3 main.py`
3. If using service: `sudo systemctl restart silex3`
4. Monitor logs: `sudo journalctl -u silex3 -f`

### Dependencies

See `requirements.txt` for full list. Main dependencies:
- `pyserial` - Serial communication with Nextion
- `RPi.GPIO` - GPIO control for Raspberry Pi

## License

[Add your license information here]

## Support

For issues or questions, please check:
- `TROUBLESHOOTING.md` for common issues
- Motor documentation in `docs/README_MOTOR.md`
- Nextion display documentation
