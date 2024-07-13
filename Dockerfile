FROM nvcr.io/nvidia/l4t-base:r36.2.0
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    nvidia-opencv-dev \
    && rm -rf /var/lib/apt/lists/* \
    && apt-get clean

#INSTALL ROS2 iron

RUN apt update && apt install curl -y
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt upgrade -y

ENV TZ=Europe/Nicosia
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN apt-get update && apt-get install -y tzdata

RUN apt install ros-iron-desktop -y

RUN set -eux; \
       key='C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654'; \
       export GNUPGHOME="$(mktemp -d)"; \
       gpg --batch --keyserver keyserver.ubuntu.com --recv-keys "$key"; \
       mkdir -p /usr/share/keyrings; \
       gpg --batch --export "$key" > /usr/share/keyrings/ros2-latest-archive-keyring.gpg; \
       gpgconf --kill all; \
       rm -rf "$GNUPGHOME"

#RUN echo "deb [ signed-by=/usr/share/keyrings/ros2-latest-archive-keyring.gpg ] http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

#RUN apt-get update && apt-get install -y --no-install-recommends \
#    ros-iron-ros-core=0.10.0-3* \
#    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

ENV ROS_DISTRO iron
# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# Port for bot health status
ENV HEALTH_PORT 8085

RUN apt update && apt-get update
RUN apt-get -q install -y bc build-essential bzip2 can-utils curl freeglut3-dev git gnupg2 i2c-tools lsb-release python3 python3-dev python3-pip software-properties-common tmux vim wget nano

COPY ./10-default.list /etc/ros/rosdep/sources.list.d/10-default.list

RUN rosdep update --rosdistro $ROS_DISTRO
RUN apt update && apt-get update

RUN apt-get -y update && apt-get -q install -y udev
RUN apt-get -y update && apt-get -q install -y ros-iron-tf2-ros ros-iron-joy
RUN pip3 -q install  flask Flask-Cors dt-vl53l0x==1.0.0 Jetson.GPIO==2.0.20 luma.oled==3.13.0 adafruit-circuitpython-mpu6050==1.2.4 colorir==2.0.1 Pillow==9.5.0 requests setuptools

# Nadia's
ENV SOURCE_DIR /app/duckiebot_interface
RUN apt update && apt-get update
ENV SOURCE_REPO duckiebot_interface
ENV ROS2_SOURCE /opt/ros/iron/setup.sh
ENV ROBOT_HARDWARE jetson_nano
ENV ROBOT_CONFIGURATION DB21M
ENV VEHICLE_NAME example_robot
ENV PROJECT_NAME example_project
ENV DEFAULT_LAUNCH /app/duckiebot_interface/launchers/default.sh
ENV ROS_DOMAIN_ID 0
ENV PARALLEL_WORKERS 4
COPY assets/dt-commons/packages /app/duckiebot_interface/dt-common/packages
COPY assets/dt-commons/assets/bin /usr/local/bin/
COPY assets/dt-commons/assets/entrypoint.sh /entrypoint.sh
COPY assets/dt-commons/assets/environment.sh /environment.sh
COPY ./launchers /app/duckiebot_interface/launchers
COPY ./packages /app/duckiebot_interface/packages
COPY ./assets/etc/ld.so.conf.d/nvidia-tegra.conf /etc/ld.so.conf.d/nvidia-tegra.conf
COPY ./assets/usr/share/fonts/*.ttf /usr/share/fonts/
COPY ./scripts /app/
WORKDIR /app

#RUN cat /etc/ros/rosdep/sources.list.d/10-default.list

#RUN apt-cache policy | grep universe
#
#RUN apt-get update
#
#RUN apt install -y ros-iron-ros2-control ros-iron-ros2-controllers ros-iron-controller-manager

RUN rosdep install -q -y -i --from-path . --rosdistro iron -r --skip-keys="gazebo_ros gazebo_ros2_control"

WORKDIR /app/.
RUN source ${ROS2_SOURCE} && colcon build --parallel-workers ${PARALLEL_WORKERS}
WORKDIR /app
ENTRYPOINT ["./entrypoint.sh"]