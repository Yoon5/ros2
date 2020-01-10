# ROS2 one-line install
# Yoon Ho Seol

set -x

function usage {
    # Print out usage of this script.
    echo >&2 "usage: $0 [catkin workspace name (default:catkin_ws)] [ROS distro (default: bionic)"
    echo >&2 "          [-h|--help] Print help message."
    exit 0
}
# Parse command line. If the number of argument differs from what is expected, call `usage` function.
OPT=`getopt -o h -l help -- $*`
if [ $# != 2 ]; then
    usage
fi
eval set -- $OPT
while [ -n "$1" ] ; do
    case $1 in
        -h|--help) usage ;;
        --) shift; break;;
        *) echo "Unknown option($1)"; usage;;
    esac
done

name_catkinws=$1
name_catkinws=${name_catkinws:="catkin_ws"}
name_ros_distro=$2
name_ros_distro=${name_ros_distro:="bionic"}

version=`lsb_release -sc`

echo "[Checking the ubuntu version]"
case $version in
  "bionic")
  ;;
  *)
    echo "ERROR: This script will only work on Ubuntu Bionic(18.03). Exit."
    exit 0
esac

echo "[Update & upgrade the package]"
sudo apt-get update
sudo apt-get upgrade

echo "[Check the 18.04.3 TLS issue]"
relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
if [ "$relesenum" = "18.04.3" ]
then
  echo "Your ubuntu version is $relesenum"
  echo "Intstall the libgl1-mesa-dev-lts-utopic package to solve the dependency issues for the ROS installation specifically on $relesenum"
  sudo apt-get install -y libgl1-mesa-dev-lts-utopic
else
  echo "Your ubuntu version is $relesenum"
fi

echo "[Installing chrony and setting the ntpdate]"
sudo apt-get install -y chrony
sudo ntpdate ntp.ubuntu.com

echo "[Add the ROS repository]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Download the ROS keys]"
roskey=`apt-key list | grep "ROS builder"`
if [ -z "$roskey" ]; then
  wget --quiet https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
fi

echo "[Update & upgrade the package]"
sudo apt-get update
sudo apt-get upgrade

echo "[Installing ROS]"
sudo apt-get install -y ros-$name_ros_distro-desktop-full ros-$name_ros_distro-rqt-*

echo "[rosdep init and python-rosinstall]"
sudo sh -c "rosdep init"
rosdep update
. /opt/ros/$name_ros_distro/setup.sh
sudo apt-get install -y python-rosinstall

echo "[Making the catkin workspace and testing the catkin_make]"
mkdir -p ~/$name_catkinws/src
cd ~/$name_catkinws/src
catkin_init_workspace
cd ~/$name_catkinws/
catkin_make

echo "[Setting the ROS evironment]"
sh -c "echo \"source /opt/ros/$name_ros_distro/setup.bash\" >> ~/.bashrc"
sh -c "echo \"source ~/$name_catkinws/devel/setup.bash\" >> ~/.bashrc"
sh -c "echo \"export ROS_MASTER_URI=http://localhost:11311\" >> ~/.bashrc"
sh -c "echo \"export ROS_HOSTNAME=localhost\" >> ~/.bashrc"


echo "[=================]"

echo "[Set the Locale]"
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "[Add ROS2 Repository]"
sudo apt update -y && sudo apt install curl gnupg2 lsb-release -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=amd64,arm64] http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

echo "[Install ROS2 Dashing Desktop]"
sudo apt update -y
sudo apt install ros-dashing-desktop -y
source /opt/ros/dashing/setup.bash
sudo apt install python3-argcomplete -y

echo "[Set the ROS2 Environment]"
sh -c "echo \"source /opt/ros/dashing/setup.bash\" >> ~/.bashrc"

echo "[Install Turtlesim]"
sudo apt install ros-dashing-turtlesim -y


echo "[=================]"
echo "[Complete!!!]"

exec bash

exit 0
