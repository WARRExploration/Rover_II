#!/bin/bash

#first stop everything that could be run
service isc-dhcp-server stop
service hostapd stop
sleep 3

#start running things
#start dhcp server and hotspot services
service isc-dhcp-server start
#service hostapd start
service hostapd start

echo "done" >> /home/exploration/test.log
