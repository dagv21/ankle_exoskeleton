#!/bin/bash

echo Enabling ports permissions
sudo chmod 777 /dev/ttyUSB0
sudo chmod 777 /dev/ttyUSB1

echo Configuring ports to minimum latency timer
sudo chmod 777 -R /sys/bus/usb-serial/devices/ttyUSB0/latency_timer 
sudo chmod 777 -R /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
echo 3 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
echo 3 > /sys/bus/usb-serial/devices/ttyUSB1/latency_timer
