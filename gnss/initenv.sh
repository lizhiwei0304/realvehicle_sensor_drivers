#!/bin/bash
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="067b", ATTRS{idProduct}=="2303", MODE:="0666",  SYMLINK+="gps"' >/etc/udev/rules.d/gps.rules

service udev reload
sleep 2
service udev restart
