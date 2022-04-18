# Automatic Tower Crane

TBD: description

# Setup
This project is developed and verified under ubuntu 18.04 and ROS Melodic.

## 1. ROS install
### 1.1 Setup your sources.list
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```
### 1.2  Set up your keys
```
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

### 1.3 Installation
```
sudo apt update
sudo apt install ros-melodic-desktop-full
```

### 1.4 Setup ROS Environment
```
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 1.5 Setup ROS Dependencies
```
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install python-rosdep
```


## 2. Clone this project to the ROS environment
```
mkdir -p ~/catkin_ws/src
git clone https://github.com/YimengZhu/auto_crane.git
```

## 3. Build the package
```
cd ~/catkin_ws/
catkin_make
source devel/setup.sh    
```

# Usage

## 1. Create a tower crane running instance
```
roslaunch auto_crane demo.launch
```
The rviz tool will started up and the tower crane model will be running in the environment.

## 2. Planning the path
```
roslaunch auto_crane plan.launch
```
This will output the example.waypoints.txt file, which contains the sampled waypoint for the motion controller.
