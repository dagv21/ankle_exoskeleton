#!/bin/bash

echo Enabling ports permissions
sudo chmod 777 /dev/ttyUSB3
sudo chmod 777 /dev/ttyUSB4

echo Configuring ports to minimum latency timer
sudo chmod 777 -R /sys/bus/usb-serial/devices/ttyUSB3/latency_timer 
sudo chmod 777 -R /sys/bus/usb-serial/devices/ttyUSB4/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB3/latency_timer
echo 1 > /sys/bus/usb-serial/devices/ttyUSB4/latency_timer
