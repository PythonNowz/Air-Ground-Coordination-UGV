#!/bin/bash

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout",  SYMLINK+="lidar"' >/etc/udev/rules.d/lidar_cp210x.rules
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="2e3c", ATTRS{idProduct}=="5740", MODE:="0777", GROUP:="dialout",  SYMLINK+="nvilidar"' >/etc/udev/rules.d/nvilidar_usb.rules

systemctl daemon-reload
service udev reload
sleep 1
service udev restart
