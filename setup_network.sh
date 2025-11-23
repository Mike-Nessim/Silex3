#!/bin/bash
# Setup virtual network interface for UIRobot devices
# This adds a static IP 192.168.1.173 to communicate with devices on 192.168.1.0 subnet
# Uses NetworkManager for persistent configuration

VIRTUAL_IP="192.168.1.173/24"
CONNECTION_NAME="Wired connection 1"

echo "Setting up virtual network interface (persistent)..."
echo ""

# Check if IP already exists in NetworkManager config
if nmcli connection show "$CONNECTION_NAME" | grep -q "192.168.1.173"; then
    echo "Virtual IP 192.168.1.173 already configured in NetworkManager"
    # Make sure it's active
    if ! ip addr show | grep -q "192.168.1.173"; then
        echo "Activating connection..."
        sudo nmcli connection up "$CONNECTION_NAME"
    fi
else
    echo "Adding virtual IP to NetworkManager connection..."
    # Add the IP address to the connection (persistent)
    sudo nmcli connection modify "$CONNECTION_NAME" +ipv4.addresses $VIRTUAL_IP
    
    if [ $? -eq 0 ]; then
        echo "✓ Virtual IP added to connection configuration"
        echo "Activating connection..."
        sudo nmcli connection up "$CONNECTION_NAME"
    else
        echo "✗ Failed to add virtual IP to NetworkManager"
        exit 1
    fi
fi

# Verify
if ip addr show | grep -q "192.168.1.173"; then
    echo ""
    echo "✓ Virtual network interface configured successfully (persistent)"
    ip addr show | grep "192.168.1.173"
    echo ""
    echo "This configuration will persist across reboots."
else
    echo "✗ Failed to configure virtual network interface"
    exit 1
fi

# Test connectivity to gateway
echo ""
echo "Testing connectivity to gateway (192.168.1.254)..."
if ping -c 1 -W 2 192.168.1.254 > /dev/null 2>&1; then
    echo "✓ Gateway is reachable"
else
    echo "⚠ Gateway not reachable (may be normal if device is off)"
fi

