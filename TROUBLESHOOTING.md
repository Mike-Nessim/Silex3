# Nextion Display Troubleshooting Guide

## Current Status
- Display Model: NX8048P070-011C-Y
- Field Name: t0 (confirmed)
- Baud Rate: 9600 (confirmed)
- Display is powered and showing default text "newtxt"
- Commands are being sent correctly from the Pi

## Most Common Issue: Wiring

The commands are being sent, but the display isn't updating. This is almost always a **wiring problem**.

### Correct Wiring:
```
USB-TTL Adapter    →    Nextion Display
─────────────────      ────────────────
TX (Transmit)     →    RX (Receive)
RX (Receive)      →    TX (Transmit)
GND (Ground)      →    GND (Ground)
VCC (5V)          →    (Optional - if Nextion needs external power)
```

### Common Mistakes:
1. **TX/RX Swapped** - This is the #1 issue! Try swapping them.
2. **Wrong GND** - Make sure GND is connected (required for communication)
3. **Wrong pins** - Some USB-TTL adapters have multiple TX/RX pins, use the correct ones

### How to Test Wiring:
1. **Swap TX and RX** - Disconnect, swap them, reconnect, and try again
2. **Check continuity** - Use a multimeter to verify connections
3. **Try different USB-TTL adapter** - If available, test with another adapter

## Other Possible Issues:

### 1. Power Issues
- Nextion needs stable 5V power
- If powered via USB-TTL, make sure the adapter can supply enough current
- Try powering Nextion separately

### 2. Baud Rate Mismatch
- Script is using 9600 baud (confirmed correct)
- If you changed baud rate in Nextion Editor, update the script

### 3. Display Not Ready
- Power cycle the Nextion display
- Wait a few seconds after power-on before sending commands

### 4. Field Name
- Field name is confirmed as `t0`
- If you have the .HMI file, double-check the exact name

## Testing Commands

Run these scripts in order:

1. **Simple update (sends command 5 times):**
   ```bash
   python3 simple_update.py
   ```

2. **Full diagnostic:**
   ```bash
   python3 update_nextion.py
   ```

3. **Connection test:**
   ```bash
   python3 test_connection.py
   ```

## Next Steps

1. **First**: Try swapping TX and RX wires
2. **Second**: Power cycle the Nextion display
3. **Third**: Verify all connections are secure
4. **Fourth**: Check if you can see the command bytes with an oscilloscope or logic analyzer

## USB-TTL Adapter Pin Identification

Common USB-TTL adapters (like PL2303, CH340, FT232):
- **TX** - Usually labeled "TX" or "TXD" - sends data TO the device
- **RX** - Usually labeled "RX" or "RXD" - receives data FROM the device  
- **GND** - Ground - MUST be connected
- **VCC** - Power (usually 5V or 3.3V) - may not be needed if Nextion has separate power

## Nextion Display Pins

Check your Nextion display documentation, but typically:
- **RX** - Receives data (connects to USB-TTL TX)
- **TX** - Sends data (connects to USB-TTL RX)
- **GND** - Ground (connects to USB-TTL GND)
- **VCC** - Power (5V)

