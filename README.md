# This repo is the driver pack designed for duckiebot hardware, it supports Jetson Nano version, as well as the RaspberryPI version.
## All Python dependencies are mentioned in dependencies-py3.txt (non-duckietown packages only) and dependencies-py3.dt.txt (duckietown packages only)

1. Jetson.GPIO==2.0.20                    -- GPIO management library for Jetson Nano
2. RPi.GPIO==0.7.1                        -- GPIO management library for RaspberryPI
3. luma.oled==3.13.0                      -- I2C oled display library
4. adafruit-circuitpython-mpu6050==1.2.4  -- MPU (Motion Processing Unit) interface library
5. colorir==2.0.1                         -- Palettes and colors manager library
6. Pillow==9.5.0                          -- Image processing library
7. dt-vl53l0x==0.0.1                      -- Proprietary ToF interface library

## dt-duckiebot-interface

Status:
[![Build Status](http://build-arm.duckietown.org/job/Docker%20Autobuild%20-%20dt-duckiebot-interface/badge/icon.svg)](http://build-arm.duckietown.org/job/Docker%20Autobuild%20-%20dt-duckiebot-interface/)
[![Docker Hub](https://img.shields.io/docker/pulls/duckietown/dt-duckiebot-interface.svg)](https://hub.docker.com/r/duckietown/dt-duckiebot-interface)

Repository containing all the necessary drivers to start sensors and actuators.
It should not contain any high-level functionality.


### How to launch manually

```$ docker -H <Hostname>.local run --name duckiebot-interface -v /data:/data --privileged --network=host -dit --restart unless-stopped -e ROBOT_TYPE=<ROBOT_TYPE> duckietown/duckiebot-interface:daffy-arm32v7```

By default, `ROBOT_TYPE` is duckiebot, and you can set it to watchtower or traffic_light if you use them.
