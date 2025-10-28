#!/bin/bash

echo "╔══╣ Setup: RM Motors ROS (STARTING) ╠══╗"

CRT_DIR=`pwd`

# Install RUST
# - Reference: https://www.rust-lang.org/tools/install
sudo apt-get update
sudo apt install -y curl
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh

echo "source $HOME/.cargo/env" >> ~/.bashrc
source $HOME/.bashrc

cd rm_motors_hw/rm_motors_can
cargo install cargo-expand
cargo build --release
cd $CRT_DIR

# Setup CAN transport
# - Reference: https://wiki.st.com/stm32mpu/wiki/How_to_set_up_a_SocketCAN_interface
sudo apt-get update
sudo apt-get install -y can-utils
sudo ip link set can0 type can bitrate 1000000 dbitrate 2000000 fd on
sudo ip link set can0 up


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
