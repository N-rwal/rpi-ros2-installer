#!/bin/bash

# Configuration
PI_LAN_IP="192.168.3.1"  # Pi's IP on your LAN
CAMERA_IP="192.168.10.60"
LISTEN_PORT="5554"
CAMERA_PORT="554"
LAN_IFACE="wlan0"           # Pi's interface connected to your router
CAMERA_IFACE="enx9c69d31df95f"        # Pi's USB interface connected to camera

echo "Disabling RTSP port forwarding"

# Remove PREROUTING rules
sudo iptables -t nat -D PREROUTING -i $LAN_IFACE -p tcp --dport $LISTEN_PORT -j DNAT --to-destination $CAMERA_IP:$CAMERA_PORT 2>/dev/null
sudo iptables -t nat -D PREROUTING -i $LAN_IFACE -p udp --dport $LISTEN_PORT -j DNAT --to-destination $CAMERA_IP:$CAMERA_PORT 2>/dev/null

# Remove FORWARD rules
sudo iptables -D FORWARD -p tcp --dport $CAMERA_PORT -j ACCEPT 2>/dev/null
sudo iptables -D FORWARD -p udp --dport $CAMERA_PORT -j ACCEPT 2>/dev/null

# Remove POSTROUTING rules
sudo iptables -t nat -D POSTROUTING -o $CAMERA_IFACE -p tcp --dport $CAMERA_PORT -j MASQUERADE 2>/dev/null
sudo iptables -t nat -D POSTROUTING -o $CAMERA_IFACE -p udp --dport $CAMERA_PORT -j MASQUERADE 2>/dev/null

# Optional: Remove logging rule if you added it
# sudo iptables -D FORWARD -j LOG --log-prefix "RTSP_FWD: " --log-level 4 2>/dev/null

echo "Rules removed"

# Show remaining rules to verify
echo -e "\nRemaining NAT rules on port $LISTEN_PORT:"
sudo iptables -t nat -L PREROUTING -n -v | grep $LISTEN_PORT || echo "None"
echo -e "\nRemaining FORWARD rules on port $CAMERA_PORT:"
sudo iptables -L FORWARD -n -v | grep $CAMERA_PORT || echo "None"
