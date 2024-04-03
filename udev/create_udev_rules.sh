#!/bin/bash

echo "remap the device serial port(ttyACM*) to  roboteq_sbl"
echo "roboteq_sbl usb connection as /dev/roboteq_sbl , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy roboteq_sbl.rules to  /etc/udev/rules.d/"
sudo cp ./roboteq_sbl.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish "