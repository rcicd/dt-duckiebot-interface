#!/bin/bash

cd /app

sudo /app/set-up-jetson-gpio.sh

. install/setup.sh && $DEFAULT_LAUNCH