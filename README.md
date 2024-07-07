# dt-duckiebot-interface

This is a duckietown repo that contains package of the driver pack designed for duckiebot hardware ported from ros to ros2

Here is a list of packages that are used to run the duckiebot hardware:
1. **adafruit_drivers**:
    - is used by other packages in repo to communicate with the hardware
    - **Status**: ported
2. **button_driver**:
    - is used to handle shutdown behavior triggered by pressing the button
    - **Status**: not tested
3. **camera_driver**:
    - publishes camera images to `image/compressed` topic in CompressedImage format
    - publishes camera info to `camera_info` topic in CameraInfo format
    - **Status**: ported
4. **display_driver**:
    - uses `fragments` topic to display info the screen
    - **Status**: not tested
5. **display_renderers**:
    - uses `fragments` topic to display info the screen
    - contains `health_renderer_node.py` that renders health info on the screen fetched from duckiebot health API (`http://<vehicle>.local/health/`)
    - contains `network_renderer_node.py` that renders network info on the screen
    - contains `robot_info_renderer_node.py` that renders robot info on the screen
    - Fragments with the info should be added to Robot HTTP API to show it in dashboard
    - **Status**: not tested
6. **duckiebot_interface**:
    - contains file `all_drivers.launch` that launches all the drivers
    - **Status**: ported
7. **duckietown_msgs**:
    - contains modified duckietown messages and services for the duckiebot hardware
    - Can be rewritten to remove the dependency on duckietown
    - **Status**: ported, need revision as the package is being built for about 35 minutes in GitHub Actions
8. **hat_driver**:
    - contains python scripts to communicate with the hat
    - **Status**: not tested
9. **imu_driver**:
    - publishes Imu messages to `data` topic
    - publishes Temperature messages to `temperature` topic
    - **Status**: not tested
10. **joystick**:
    - **Status**: not tested
11. **led_driver**:
    - is subscribed to `led_pattern` topic that receives LedPattern messages to set the led pattern
    - **Status**: not tested
12. **robot_http_api**:
    - uses 8090 port for duckiebot (see [this file](packages/robot_http_api/include/dt_robot_rest_api/constants.py) with ports constants)
    - flask server that provides API to get robot car status, initiate emergency stop and shutdown 
    - health and network state should be added to the API
    - **Status**: ported
13. **tof_driver**:
    - publishes Range messages to `range` topic
    - publishes [DisplayFragment](packages/duckietown_msgs/msg/DisplayFragment.msg) messages to `fragments` topic
    - **Status**: bug detected, segfault when trying to access sensor
14. **traffic_light**:
    - **Status**: not tested
15. **utils**:
    - Contains helper functions (not ros2-specific) for the drivers
    - **Status**: created, works properly
16. **wheel_encoder**:
    - is subscribed to `wheels_cmd_executed` topic to get feedback on the wheels speed
    - publishes to `tick` topic to get the wheel encoder ticks in [WheelEncoderStamped](packages/duckietown_msgs/msg/WheelEncoderStamped.msg) format
    - **Status**: ported
17. **wheels_driver**:
    - gets WheelsCmdStamped messages from `wheels_cmd` topic to set wheels speed to certain values
    - publishes to `wheels_cmd_executed` topic to get feedback on the wheels speed
    - is subscribed to `emergency_stop` topic to stop the wheels when the [BoolStamped](packages/duckietown_msgs/msg/BoolStamped.msg) message is sent
    - **Status:** ported

# This repo contains ros package of the driver pack designed for duckiebot hardware, it supports Jetson Nano version, as well as the RaspberryPI version. 
## All Python dependencies are mentioned in dependencies-py3.txt (non-duckietown packages only) and dependencies-py3.dt.txt (duckietown packages only)

1. Jetson.GPIO==2.0.20                    -- GPIO management library for Jetson Nano
2. RPi.GPIO==0.7.1                        -- GPIO management library for RaspberryPI
3. luma.oled==3.13.0                      -- I2C oled display library
4. adafruit-circuitpython-mpu6050==1.2.4  -- MPU (Motion Processing Unit) interface library
5. colorir==2.0.1                         -- Palettes and colors manager library
6. Pillow==9.5.0                          -- Image processing library
7. dt-vl53l0x==0.0.1                      -- Proprietary ToF interface library

## dt-duckiebot-interface


Repository containing all the necessary drivers to start sensors and actuators.
It should not contain any high-level functionality.


### How to launch manually

```$ docker run --privileged -v /var/run/avahi-daemon/socket:/var/run/avahi-daemon/socket -v /tmp/argus_socket:/tmp/argus_socket -v /data:/data --network=host --runtime nvidia -v /tmp/.X11-unix/:/tmp/.X11-unix --name duckiebot-ros2-interface -dit --restart unless-stopped spgc/duckietown_autonomous_driving:ros2-interface```
