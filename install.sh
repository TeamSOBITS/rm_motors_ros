#!/bin/bash

echo "╔══╣ Setup: RM Motors ROS (STARTING) ╠══╗"

CRT_DIR=`pwd`

# Install RUST
# - Reference: https://www.rust-lang.org/tools/install
sudo apt-get update
sudo apt install -y curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

echo "source $HOME/.cargo/env" >> ~/.bashrc
source "$HOME/.cargo/env"

cd rm_motors_hw/rm_motors_can
cargo install cargo-expand
cargo build --release
cd $CRT_DIR

# Setup CAN transport
# - Reference: https://wiki.st.com/stm32mpu/wiki/How_to_set_up_a_SocketCAN_interface
sudo apt-get update
sudo apt-get install -y iproute2 can-utils

# Install a udev rule (not systemd-networkd) so can0 auto-reconfigures on every
# e-stop power cycle, including from inside a Docker container without systemd.
SCRIPT_DIR=$(dirname "$(realpath "$0")")
sudo cp "$SCRIPT_DIR"/80-can.rules /etc/udev/rules.d/

# One-shot config so can0 works immediately, without replugging the adapter.
if ip link show can0 &> /dev/null; then
    sudo ip link set can0 down
    sudo ip link set can0 type can bitrate 1000000
    sudo ip link set can0 txqueuelen 65536
    sudo ip link set can0 up
fi


# Download ROS packages
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-urdf \
    ros-$ROS_DISTRO-controller-manager \
    ros-$ROS_DISTRO-forward-command-controller \
    ros-$ROS_DISTRO-joint-state-broadcaster \
    ros-$ROS_DISTRO-joint-trajectory-controller \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-ros2controlcli \
    ros-$ROS_DISTRO-ros2launch \
    ros-$ROS_DISTRO-rviz2 \
    ros-$ROS_DISTRO-xacro


echo "╚══╣ Setup: RM Motors ROS (FINISHED) ╠══╝"
