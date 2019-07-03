#!/bin/bash

# Load modules and attach USB-to-CAN to network device
sudo modprobe can
sudo modprobe can-raw
sudo modprobe slcan

sudo slcan_attach -f -s$3 -o /dev/ttyACM$1
sudo slcand ttyACM$1 slcan$2
sudo ifconfig slcan$2 up
