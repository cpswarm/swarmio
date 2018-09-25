[general]
unpack=true
cleanup=true
bootstrap=ubuntu ubuntu-security ubuntu-updates ubuntu-backports ros
aptsources=ubuntu ubuntu-security ubuntu-updates ubuntu-backports ros

[ubuntu]
packages=uuid uuid-dev libsystemd-dev build-essential
source=@MULTISTRAP_SOURCE@
keyring=ubuntu-keyring
suite=@MULTISTRAP_UBUNTU_VERSION@
components=main universe multiverse

[ubuntu-security]
source=@MULTISTRAP_SOURCE@
suite=@MULTISTRAP_UBUNTU_VERSION@-security
components=main universe multiverse

[ubuntu-updates]
source=@MULTISTRAP_SOURCE@
suite=@MULTISTRAP_UBUNTU_VERSION@-updates
components=main universe multiverse

[ubuntu-backports]
source=@MULTISTRAP_SOURCE@
suite=@MULTISTRAP_UBUNTU_VERSION@-backports
components=main universe multiverse

[ros]
packages=ros-@MULTISTRAP_ROS_VERSION@-ros-base
source=http://packages.ros.org/ros/ubuntu
suite=@MULTISTRAP_UBUNTU_VERSION@
