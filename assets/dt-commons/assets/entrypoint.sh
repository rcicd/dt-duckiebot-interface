#!/bin/bash

# constants
CONFIG_DIR=/data/config
ROBOT_TYPE_FILE=${CONFIG_DIR}/robot_type
ROBOT_CONFIGURATION_FILE=${CONFIG_DIR}/robot_configuration
ROBOT_HARDWARE_FILE=${CONFIG_DIR}/robot_hardware

echo "==> Entrypoint"

# if anything weird happens from now on, STOP
set -e

# reset health
echo ND >/health

# get container ID
DT_MODULE_INSTANCE=$(dt-get-container-id)
export DT_MODULE_INSTANCE

debug() {
    if [ "${DEBUG}" = "1" ]; then
        echo "  DEBUG: $1"
    fi
}

info() {
    echo "   INFO: $1"
}

warning() {
    echo "WARNING: $1"
}

error() {
    echo "  ERROR: $1"
}

is_nethost() {
    netmode=$(dt-get-network-mode)
    [[ "${netmode}" = "HOST" ]]
}

configure_vehicle() {
    # check the mandatory arguments
    VEHICLE_NAME_IS_SET=1
    if [ ${#VEHICLE_NAME} -le 0 ]; then
        VEHICLE_NAME_IS_SET=0
        VEHICLE_NAME=$(hostname)
        info "The environment variable VEHICLE_NAME is not set. Using '${VEHICLE_NAME}'."
    fi
    export VEHICLE_NAME="${VEHICLE_NAME}"

    # super user configuration
    if [ "${UID}" -eq "0" ]; then
        # we are root
        netwarnings=0

        # check optional arguments
        if [ ${#VEHICLE_IP} -ne 0 ]; then
            info "The environment variable VEHICLE_IP is set to '${VEHICLE_IP}'. Adding to /etc/hosts."
            {
                echo "${VEHICLE_IP} ${VEHICLE_NAME} ${VEHICLE_NAME}.local" >>/etc/hosts
            } || {
                warning "Failed writing to /etc/hosts. Will continue anyway."
                netwarnings=1
            }
        fi

        # configure hosts
        if [ "${VEHICLE_NAME_IS_SET}" -eq "0" ]; then
            # vehicle name not set (assume vehicle is localhost)
            {
                echo "127.0.0.1 localhost ${VEHICLE_NAME} ${VEHICLE_NAME}.local" >>/etc/hosts
            } || {
                warning "Failed writing to /etc/hosts. Will continue anyway."
                netwarnings=1
            }
        fi

        # configure (fake) mDNS
        {
            echo "127.0.0.1 localhost $(hostname) $(hostname).local" >>/etc/hosts
        } || {
            warning "Failed writing to /etc/hosts. Will continue anyway."
            netwarnings=1
        }

        if [ "${netwarnings}" -eq "1" ]; then
            info "Network configured (with warnings)."
        else
            info "Network configured successfully."
        fi
    else
        warning "Running in unprivileged mode, container's network will not be configured."
    fi

    # robot_type
    if [ ${#ROBOT_TYPE} -le 0 ]; then
        if [ -f "${ROBOT_TYPE_FILE}" ]; then
            ROBOT_TYPE=$(cat "${ROBOT_TYPE_FILE}")
            debug "ROBOT_TYPE[${ROBOT_TYPE_FILE}]: '${ROBOT_TYPE}'"
            export ROBOT_TYPE
        else
            warning "robot_type file does not exist. Using 'duckiebot' as default type."
            export ROBOT_TYPE="duckiebot"
        fi
    else
        info "ROBOT_TYPE is externally set to '${ROBOT_TYPE}'."
    fi

    # robot_configuration
    if [ ${#ROBOT_CONFIGURATION} -le 0 ]; then
        if [ -f "${ROBOT_CONFIGURATION_FILE}" ]; then
            ROBOT_CONFIGURATION=$(cat "${ROBOT_CONFIGURATION_FILE}")
            debug "ROBOT_CONFIGURATION[${ROBOT_CONFIGURATION_FILE}]: '${ROBOT_CONFIGURATION}'"
            export ROBOT_CONFIGURATION
        else
            warning "robot_configuration file does not exist."
            export ROBOT_CONFIGURATION="__NOTSET__"
        fi
    else
        info "ROBOT_CONFIGURATION is externally set to '${ROBOT_CONFIGURATION}'."
    fi

    # robot_hardware
    if [ ${#ROBOT_HARDWARE} -le 0 ]; then
        if [ -f "${ROBOT_HARDWARE_FILE}" ]; then
            ROBOT_HARDWARE=$(cat "${ROBOT_HARDWARE_FILE}")
            debug "ROBOT_HARDWARE[${ROBOT_HARDWARE_FILE}]: '${ROBOT_HARDWARE}'"
            export ROBOT_HARDWARE
        else
            warning "robot_hardware file does not exist."
            export ROBOT_HARDWARE="__NOTSET__"
        fi
    else
        info "ROBOT_HARDWARE is externally set to '${ROBOT_HARDWARE}'."
    fi
}


configure_python() {
    # make the code discoverable by python
    for d in $(find "${SOURCE_DIR}" -mindepth 1 -maxdepth 1 -type d); do
        if [ -d "${d}/packages" ]; then
            debug " > Adding ${d}/packages to PYTHONPATH"
            export PYTHONPATH="${d}/packages:${PYTHONPATH}"
        fi
    done
}

configure_ROS() {
    # check if ROS_MASTER_URI is set
    ROS_MASTER_URI_IS_SET=0
    if [ -n "${ROS_MASTER_URI-}" ]; then
        ROS_MASTER_URI_IS_SET=1
        info "Forcing ROS_MASTER_URI=${ROS_MASTER_URI}"
    fi

    # check if ROS_HOSTNAME is set
    ROS_HOSTNAME_IS_SET=0
    if [ -n "${ROS_HOSTNAME-}" ]; then
        ROS_HOSTNAME_IS_SET=1
        info "Forcing ROS_HOSTNAME=${ROS_HOSTNAME}"
    fi

    # check if ROS_IP is set
    ROS_IP_IS_SET=0
    if [ -n "${ROS_IP-}" ]; then
        ROS_IP_IS_SET=1
        info "Forcing ROS_IP=${ROS_IP}"
    fi

    # constants
    ROS_SETUP=(
        "/opt/ros/${ROS_DISTRO}/setup.bash"
        "${SOURCE_DIR}/install/setup.bash"
        "${SOURCE_DIR}/setup.bash"
    )

    # setup ros environment
    for ROS_SETUP_FILE in "${ROS_SETUP[@]}"; do
        if [ -f "${ROS_SETUP_FILE}" ]; then
            source "${ROS_SETUP_FILE}"
        fi
    done

    # configure ROS_HOSTNAME
    if [ "${ROS_HOSTNAME_IS_SET}" -eq "0" ]; then
        if is_nethost; then
            # configure ROS_HOSTNAME
            MACHINE_HOSTNAME="$(hostname).local"
            debug "Detected '--net=host', setting ROS_HOSTNAME to '${MACHINE_HOSTNAME}'"
            export ROS_HOSTNAME=${MACHINE_HOSTNAME}
        fi
    fi

    # configure ROS_IP
    if [ "${ROS_IP_IS_SET}" -eq "0" ]; then
        if ! is_nethost; then
            # configure ROS_IP
            CONTAINER_IP=$(hostname -I 2>/dev/null | cut -d " " -f 1)
            debug "Detected '--net=bridge', setting ROS_IP to '${CONTAINER_IP}'"
            export ROS_IP=${CONTAINER_IP}
        fi
    fi

    # configure ROS MASTER URI
    if [ "${ROS_MASTER_URI_IS_SET}" -eq "0" ]; then
        export ROS_MASTER_URI="http://${VEHICLE_NAME}.local:11311/"
    fi
}

configure_workspaces() {
    IFS="," read -ra USER_WORKSPACES <<< "${DT_USER_WORKSPACES:-}"
    for USER_WS in "${USER_WORKSPACES[@]}"; do
        if [ "${USER_WS}" == "" ]; then
            continue
        fi
        WS_DIR="${USER_WS_DIR}/${USER_WS}"
        if [ -d "${WS_DIR}" ]; then
            debug "Analyzing workspace candidate '${WS_DIR}'..."
            USER_WS_SETUP_FILE="${WS_DIR}/devel/setup.bash"
            if [ -f "${USER_WS_SETUP_FILE}" ]; then
                debug "Sourcing workspace '${WS_DIR}'"
                source "${USER_WS_SETUP_FILE}" --extend
            else
                warning "Workspace '${USER_WS}' is not built!"
            fi
        else
            error "Workspace '${USER_WS}' not found!"
        fi
    done
}

configure_user() {
    # impersonate UID
    if [ "${IMPERSONATE_UID:-}" != "" ]; then
        echo "Impersonating user with UID: ${IMPERSONATE_UID}"
        usermod -u ${IMPERSONATE_UID} ${DT_USER_NAME}
        export DT_USER_UID=${IMPERSONATE_UID}
    fi
    # impersonate GID
    if [ "${IMPERSONATE_GID:-}" != "" ]; then
        echo "Impersonating group with GID: ${IMPERSONATE_GID}"
        groupmod -g ${IMPERSONATE_GID} ${DT_USER_NAME}
        export DT_GROUP_GID=${IMPERSONATE_GID}
    fi
}

# configure
debug "=> Setting up vehicle..."
configure_vehicle
debug "<= Done!"

debug "=> Setting up user..."
configure_user
debug "<= Done!"

debug "=> Setting up PYTHONPATH..."
configure_python
debug "<= Done!"

debug "=> Setting up ROS environment..."
configure_ROS
debug "<= Done!"

debug "=> Setting up workspaces..."
configure_workspaces
debug "<= Done!"

# mark this file as sourced
DT_ENTRYPOINT_SOURCED=1
export DT_ENTRYPOINT_SOURCED

# if anything weird happens from now on, CONTINUE
set +e

echo "<== Entrypoint"

# exit if this file is just being sourced
if [ "$0" != "${BASH_SOURCE[0]}" ]; then
    return
fi

# reuse DT_LAUNCHER as CMD if the var is set and the first argument is `--`
if [ ${#DT_LAUNCHER} -gt 0 ] && [ "$1" == "--" ]; then
    shift
    exec bash -c "dt-launcher-$DT_LAUNCHER $*"
else
    exec "$@"
fi
