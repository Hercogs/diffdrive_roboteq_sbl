#!/bin/bash

echo "delete remap the device serial port(ttyACM*) to roboteq_sbl"
echo "sudo rm   /etc/udev/rules.d/roboteq_sbl.rules"
sudo rm   /etc/udev/rules.d/roboteq_sbl.rules
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "finish  delete"