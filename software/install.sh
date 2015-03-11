#!/bin/bash
echo "This script installs the CCD spectrometer software with graphical user interface on Debian/Ubuntu (>= 8.04)" 
if [ "$(whoami)" != 'root' ]; then echo "Need to be ran as root!"; exit 1; fi

echo "Enable user access to the device"
echo 'SUBSYSTEM=="usb", ENV{DEVTYPE}=="usb_device",SYSFS{idVendor}=="16c0", SYSFS{idProduct}=="05df", MODE="0666"' > /lib/udev/rules.d/62-usb-spectrometer.rules
sudo /etc/init.d/udev restart

echo "Installing required packages not present by default"
PACKAGES=" python-matplotlib python-scipy"
apt-get install $PACKAGES || echo "... installation via apt-get did not work, please install $PACKAGES manually"
