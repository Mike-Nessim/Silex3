# Silex3

Repository for Silex3 project.

## Nextion Display Control

This project includes scripts to control a Nextion intelligent display via USB-TTL adapter.

### Setup

1. Connect your USB-TTL adapter to the Raspberry Pi
2. Connect the adapter to your Nextion display
3. Install dependencies: `pip install -r requirements.txt`

### Usage

Update text field `t0` on the Nextion display:
```bash
python3 update_nextion.py
```

The script will:
- Connect to `/dev/ttyUSB0` at 9600 baud
- Update text field `t0` to "Axton Robotics"
- Display status messages

### Configuration

You can modify the script to:
- Change the serial port (default: `/dev/ttyUSB0`)
- Change the baud rate (default: `9600`)
- Update different text fields or other Nextion components

