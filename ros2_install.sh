# ROS2 one-line install
# Apache License 2.0
# Copuright (c) 2020, Yoon Ho Seol.

#!/bin/bash #following lines are written for bash; execute this file as bash scrip

echo ""
echo "[Note] Target OS version  >>> Ubuntu 18.04.x (bionic) or Ubuntu Xenial Xerus (16.04)" # Xenial Xerus would be deleted after Crystal 
echo "[Note] Target ROS version >>> ROS2 Dashing Diademata"
# echo "[Note] Target ROS version >>> ROS2 Foxy Fitzroy"
echo "[Note] Catkin workspace   >>> $HOME/catkin_ws"
echo ""
echo "PRESS [ENTER] TO CONTINUE THE INSTALLATION"
echo "IF YOU WANT TO CANCEL, PRESS [CTRL] + [C]"
read

# Added - Check Ubuntu version
echo "[Check ubuntu version]"
case $version in
  "bionic" | "xenial")
  ;;
  *)
    echo "ERROR: This script will only work on Ubuntu Bionic Beaver (18.04) and Xenial Xerus (16.04). Exit." # Xenial Xerus would be deleted after Crystal 
    exit 0
esac

echo "[Set the target OS, ROS version and name of catkin workspace]"
name_os_version=${name_os_version:="bionic"}
name_ros_version=${name_ros_version:="dashing"}
# name_ros_version=${name_ros_version:="foxy"}
# name_catkin_workspace=${name_catkin_workspace:="catkin_ws"} # catkin deleted

echo "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

echo "[Install build environment, the chrony, ntpdate and set the ntpdate]"
sudo apt-get install -y chrony ntpdate build-essential
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${name_os_version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -z "$roskey" ]; then
  sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi

echo "[Check the ROS keys]"
roskey=`apt-key list | grep "Open Robotics"`
if [ -n "$roskey" ]; then
  echo "[ROS key exists in the list]"
else
  echo "[Failed to receive the ROS key, aborts the installation]"
  exit 0
fi

echo "[Update the package lists and upgrade them]"
sudo apt-get update -y
sudo apt-get upgrade -y

echo "[Install the ros-desktop-full and all rqt plugins]"
sudo apt-get install -y ros-$name_ros_version-desktop-full ros-$name_ros_version-rqt-*

echo "[Initialize rosdep]"
sudo sh -c "rosdep init"
rosdep update

echo "[Environment setup and getting rosinstall]"
source /opt/ros/$name_ros_version/setup.sh
sudo apt-get install -y python-rosinstall

# echo "[Make the catkin workspace and test the catkin_make]"  # catkin deleted
# mkdir -p $HOME/$name_catkin_workspace/src
# cd $HOME/$name_catkin_workspace/src
# catkin_init_workspace
# cd $HOME/$name_catkin_workspace
# catkin_make


# Added - Setup Locale, Sources, & Install packages
echo "[Setup Locale]"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[Setup ROS2 apt repository]"
sudo apt update -y && sudo apt install curl gnupg2 lsb-release -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

echo "[Install ROS2 Dashing packages]"
sudo apt update -y
sudo apt install ros-dashing-desktop -y
source /opt/ros/dashing/setup.bash
sudo apt install python3-argcomplete -y


# Added - Edit bashrc
echo "[Setup bashrc; ROS evironment]"
sh -c "echo \"alias eb='nano ~/.bashrc'\" >> ~/.bashrc"  # alias commands for bash
sh -c "echo \"alias sb='source ~/.bashrc'\" >> ~/.bashrc"
sh -c "echo \"alias gs='git status'\" >> ~/.bashrc"
sh -c "echo \"alias gp='git pull'\" >> ~/.bashrc"
# sh -c "echo \"alias cw='cd ~/$name_catkin_workspace'\" >> ~/.bashrc"  # catkin deleted
# sh -c "echo \"alias cs='cd ~/$name_catkin_workspace/src'\" >> ~/.bashrc"
# sh -c "echo \"alias cm='cd ~/$name_catkin_workspace && catkin_make'\" >> ~/.bashrc"

sh -c "echo \"source /opt/ros/$name_ros_version/setup.bash\" >> ~/.bashrc"
# sh -c "echo \"source ~/$name_catkin_workspace/devel/setup.bash\" >> ~/.bashrc"  # catkin deleted

# sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"  # master, roscore deleted
# sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"

source $HOME/.bashrc


# Added - Install Turtlesim
echo "[Install turtlesim]"
sudo apt install ros-dashing-turtlesim -y


echo "[Complete!!!]"
exit 0
