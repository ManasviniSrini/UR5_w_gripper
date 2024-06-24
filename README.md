This repository contains all the necessary files to set up and use a UR5 robot with a 2F-85 Robotiq gripper using MoveIt. The gripper is controlled using RTDE (Real-time data exchange).

Follow the instructions below to clone the repository, build the workspace, and run the complete setup.

**Note:** This setup was tested on ROS Noetic and Melodic.

Before starting, ensure you have the following:

- ROS Noetic installed
- Catkin workspace created
- UR5 robot and Robotiq gripper hardware
- Ensure robot is powered on.

## Dependencies

Make sure you have the following dependencies installed:

- `curl`
- `python3-rosdep`
- `python3-rosinstall`
- `python3-rosinstall-generator`
- `python3-wstool`
- `build-essential`
- `git`
- `rviz`
- `ur-robot-driver`
- `moveit`

**Note:** Any other missing dependencies can be installed using this command:
`sudo apt install ros-${ROS_DISTRO}-<package-name>`

## Installing ROS Noetic

Follow the steps below to install ROS Noetic on Ubuntu 20.04.

### Setup your sources.list
`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
### Setup your sources.list
```bash
sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```
### Installation
```bash
sudo apt update
sudo apt install ros-noetic-desktop-full
```
### Environment setup
`source /opt/ros/noetic/setup.bash`


### Dependencies for building packages:
To install rosinstall and other dependencies for building ROS packages, run:

`sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential`

### Intialize rosdep:

```bash
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

## Setting up the Catkin Workspace

### Create a Catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
```
### Initialize the workspace:
```bash
catkin_make
source devel/setup.bash
```
### Clone the repository:
```bash
cd ~/catkin_ws/src
git clone <repository-url>
```
### Building the workspace:
Once you have cloned the repository, navigate back to the Catkin workspace root and build the workspace:
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
## Running the Visualization
### To visualize the UR5 robot with the Robotiq gripper in Rviz, run this command:
`roslaunch ur_gripper_moveit_config demo.launch`

## Calibration
An existing calibration file of the UR5 arm is located in the calibration_data file, but in case you need to calibrate again, use the following command:

`roslaunch ur_calibration calibration_correction.launch robot_ip:=<ROBOT_IP> target_filename:="<TARGET_PATH>/calibration.yaml"`
where <ROBOT_IP> is the ip address of the robot.

## Controlling the UR5 Robot with MoveIt

### Initial Setup

Before running the UR5 robot, ensure the following settings in the installation file on the teach pendant:

- **Profinet and Ethernet/IP**: These should be disabled.
- **External Control**: Set this to the host IP address and use a custom port (default is 50002).
- **URCaps**: Ensure that the External Control URCap is installed.

Before any control of the robot can happen, a program containing the External Control program node must be running on the robot.

To create the program, follow these steps:

1. In the **Program Robot** tab on the teach pendant, create an empty program.
2. Navigate to **Structure -> URCaps** and select **External Control**.
3. Ensure the program tree on the left lists **Control by <HOST_IP>**.
4. Save the program and then load it.

The robot is now ready to receive and execute external commands from MoveIt.

### Bringing up the UR5 Robot and RViz Visualization

1. Open a terminal window, navigate to the root of your workspace, and run the following command to launch the UR5 robot driver:

    ```bash
    roslaunch ur_robot_driver ur5_bringup.launch robot_ip:=<ROBOT_IP> kinematics_config:=<TARGET_PATH>/calibration_data/calibration.yaml
    ```

    Once the robot driver is started, press play on the External Control Program we created. Inside the ROS terminal running the driver you should see the output `Robot ready to receive control commands.`

2. In a second terminal window, run the following command to launch the MoveIt planning and execution node:

    ```bash
    roslaunch ur5_moveit_config moveit_planning_execution.launch
    ```

3. Open a third terminal window and run the following command to launch RViz:

    ```bash
    roslaunch ur5_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur5_moveit_config)/launch/moveit.rviz
    ```

You should get an RViz window showing the robot overlaid with an orange version of the robot. You can now change the pose of the robot by dragging the blue ball around. Click on **plan** once you're satisfied with the pose and then **execute** to move the real robot.


## Controlling the Gripper using RTDE:

The code to control the Robotiq gripper using RTDE is already included in the repository
### Running the Gripper Control Script
Open a new terminal and run the python script to control the gripper. Replace the IP address given with the IP address of the robot.
`python3 control_gripper.py`
**Note:** Make sure that the python version is >=3 when running the script.

### Example Usage of the control_gripper.py:
The control_gripper.py script allows you to control the gripper by sending commands to open and close it. 

#### Open the gripper:
`gripper.move_and_wait_for_pos(0, 255, 255)  # Move to open position`

#### Close the gripper:
`gripper.move_and_wait_for_pos(255, 255, 255)  # Move to closed position`
