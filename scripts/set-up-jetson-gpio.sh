#!/bin/bash

groupadd -f -r gpio
usermod -a -G gpio root
mkdir -p /etc/udev/rules.d/
rules_dir=$(find .. | grep 99-gpio.rules)
cp $rules_dir /etc/udev/rules.d/
/lib/systemd/systemd-udevd --daemon && sudo udevadm control --reload-rules && sudo udevadm trigger