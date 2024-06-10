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

if [ "$EUID" -ne 0 ]; then
  if [ -e "${PWD}/post_install.sh" ]; then
    echo
    echo "You might need to configure permissions for uploading."
    echo "To do so, run the following command from the terminal:"
    echo "sudo \"${PWD}/post_install.sh\""
    echo
  else
    # Script was executed from another path. It is assumed this will only occur when user is executing script directly.
    # So it is not necessary to provide the command line.
    echo "Please run as root"
  fi

  exit
fi

megaAVR_rules > /etc/udev/rules.d/60-arduino-megaAVR.rules

# reload udev rules
echo "Reload rules..."
udevadm control --reload-rules
udevadm trigger

