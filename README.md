
This repository is based on the following repositories, considering some changes in the codes to have compatibility with ubuntu 20.4 and with Optitrack's Motion Capture system and in addition control algorithms applied to individual nano quadrotors and in swarm will be added.


*** Repository of the original unmodified codes

crazyflie_ros

https://github.com/whoenig/crazyflie_ros

vrpn

https://github.com/ros-drivers/vrpn_client_ros


## Requirements

### Installing ROS 
#### Setup your sources.list

Setup your computer to accept software from packages.ros.org.
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
#### Set up your keys
```
sudo apt install curl -y
```
```
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
#### Installation
First, make sure your Debian package index is up-to-date:
```
sudo apt-get update
```
now you can install Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages
```
sudo apt install ros-noetic-desktop-full
```
#### Environment setup
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
#### Dependencies for building packages
Up to now you have installed what you need to run the core ROS packages. To create and manage your own ROS workspaces, there are various tools and requirements that are distributed separately. For example, rosinstall is a frequently used command-line tool that enables you to easily download many source trees for ROS packages with one command.

To install this tool and other dependencies for building ROS packages, run:

```
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```
##### Initialize rosdep
Before you can use many ROS tools, you will need to initialize rosdep. rosdep enables you to easily install system dependencies for source you want to compile and is required to run some core components in ROS. If you have not yet installed rosdep, do so as follows.
```
sudo apt install python3-rosdep
```
With the following, you can initialize rosdep.
```
sudo rosdep init
rosdep update
```

### Install dependencies.
To be able to execute the programs it is necessary to install the following dependencies, executing the following commands in the console
```
sudo apt-get install ros-noetic-vrpn-client-ros
sudo apt-get install ros-noetic-joy
```
## Create a catkin workspace
```
mkdir -p ~/crazyflie_ws/src
cd crazyflie_ws/src
catkin_init_workspace
```
With these commands a new workspace called Crazyflie_ws has been created

## Installation

Clone the package into your catkin workspace (in src folder): 
```
cd
cd crazyflie_ws/src
git clone --recursive https://github.com/juliordzcer/crazyflie_ros.git
cd crazyflie_ros
git submodule init
git submodule update
cd
```

Use `catkin_make` on your workspace to compile.
```
cd crazyflie_ws
catkin_make
```

Finally run the following command in terminal
```
echo "source ~/crazyflie_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Usage

There are five packages included: crazyflie, crazyflie_description, crazyflie_controller, crazyflie_demo and vrpn_client_ros.
Note that the below description might be slightly out-of-date, as we continue merging the Crazyswarm and crazyflie_ros.

### Crazyflie

This package contains the driver. In order to support multiple Crazyflies with a single Crazyradio, there is crazyflie_server (communicating with all Crazyflies) and crazyflie_add to dynamically add Crazyflies.
The server does not communicate to any Crazyflie initially, hence crazyflie_add needs to be used.

### Crazyflie_demo

This package contains a set of examples to quickly get started with Crazyflie.

To follow a trajectory using a crazyflie:
```
roslaunch crazyflie_demo Individual.launch uri:=radio://0/100/2M
```
where uri specifies the uri of your Crazyflie.

To start the path of two agents:
```
roslaunch crazyflie_demo LiderVirtual.launch
```
You can modify the crazyflie uri parameters as well as the VRPN parameters in the Multiagent.launch launch file.
located in the crazyflie_demo/launch folder

For multiple Crazyflies, make sure all Crazyflies have a different address.
Crazyflies sharing a dongle should use the same channel and data rate for the best performance.
Performance degrades with number of Crazyflies per dongle due to bandwidth limitations, however successfully tested to use 3 CF per Crazyradio.
### Vrpn_client_ros
This package contains the code for the external motion capture system, which has been modified to change the frames sent by the Optitrack system.
To know the position of a rigid body execute the following command

```
roslaunch vrpn_client_ros sample.launch server:=<ip>
rostopic echo /vrpn client node/<rigid body name>/pose
```

## ROS Features

### Parameters

The launch file supports the following arguments:
* uri: Specifier for the crazyflie, e.g. radio://0/80/2M
* tf_prefix: tf prefix for the crazyflie frame(s)
* roll_trim: Trim in degrees, e.g. negative if flie drifts to the left
* pitch_trim: Trim in degrees, e.g. negative if flie drifts forward

See http://wiki.bitcraze.se/projects:crazyflie:userguide:tips_and_tricks for details on how to obtain good trim values.

