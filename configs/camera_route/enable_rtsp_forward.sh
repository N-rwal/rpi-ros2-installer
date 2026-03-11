#!/bin/bash

# Configuration
PI_LAN_IP="192.168.3.1"  # Pi's IP on your LAN
CAMERA_IP="192.168.10.60"
LISTEN_PORT="5554"
CAMERA_PORT="554"
LAN_IFACE="wlan0"           # Pi's interface connected to your router
CAMERA_IFACE="enx9c69d31df95f"        # Pi's USB interface connected to camera

echo "Enabling RTSP port forwarding from $PI_LAN_IP:$LISTEN_PORT to $CAMERA_IP:$CAMERA_PORT"

# Enable IP forwarding if not already enabled
sudo sysctl -w net.ipv4.ip_forward=1

# Flush any existing rules for a clean start (optional - comment out if you have other rules)
# sudo iptables -t nat -F
# sudo iptables -F

# PREROUTING rules - redirect incoming traffic on LISTEN_PORT to camera
sudo iptables -t nat -A PREROUTING -i $LAN_IFACE -p tcp --dport $LISTEN_PORT -j DNAT --to-destination $CAMERA_IP:$CAMERA_PORT
sudo iptables -t nat -A PREROUTING -i $LAN_IFACE -p udp --dport $LISTEN_PORT -j DNAT --to-destination $CAMERA_IP:$CAMERA_PORT

# FORWARD rules - allow the forwarded traffic
sudo iptables -A FORWARD -p tcp --dport $CAMERA_PORT -j ACCEPT
sudo iptables -A FORWARD -p udp --dport $CAMERA_PORT -j ACCEPT

# POSTROUTING rule - masquerade so return traffic comes back through Pi
sudo iptables -t nat -A POSTROUTING -o $CAMERA_IFACE -p tcp --dport $CAMERA_PORT -j MASQUERADE
sudo iptables -t nat -A POSTROUTING -o $CAMERA_IFACE -p udp --dport $CAMERA_PORT -j MASQUERADE

# Optional: Add logging to debug (remove if not needed)
# sudo iptables -A FORWARD -j LOG --log-prefix "RTSP_FWD: " --log-level 4

echo "Rules added. Connect to: rtsp://$PI_LAN_IP:$LISTEN_PORT/1"
echo "                      rtsp://$PI_LAN_IP:$LISTEN_PORT/2"

# Show the rules we just added
echo -e "\nCurrent NAT rules:"
sudo iptables -t nat -L PREROUTING -n -v | grep $LISTEN_PORT
echo -e "\nCurrent FORWARD rules:"
sudo iptables -L FORWARD -n -v | grep $CAMERA_PORT
