#!/usr/bin/env bash

megaAVR_rules () {
    echo ""
    echo "# Arduino Mega AVR boards bootloader mode udev rules"
    echo ""
cat <<EOF
# Arduino UNO WiFi Rev2
SUBSYSTEMS=="tty", ENV{ID_REVISION}=="03eb", ENV{ID_MODEL_ID}=="2145", MODE="0666", ENV{ID_MM_DEVICE_IGNORE}="1", ENV{ID_MM_CANDIDATE}="0"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="03eb", ATTRS{idProduct}=="2145", MODE:="0666", ENV{ID_MM_DEVICE_IGNORE}="1"

# Arduino Nano Every
SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0058", MODE:="0666"
EOF
}

if [ "$EUID" -ne 0 ]
  then echo "Please run as root"
  exit
fi

megaAVR_rules > /etc/udev/rules.d/60-arduino-megaAVR.rules

# reload udev rules
echo "Reload rules..."
udevadm control --reload-rules
udevadm trigger

