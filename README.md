# Multi URSim control interface using Moveit!
This repo creates a interface to control multiple ur5e arms using Moveit!

## Setup
### Install Universal_Robots_ROS_Driver
Follow the instructions [here](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver) to install the ROS driver for UR arms.

### The interface package
Clone and build this repo in desired catkin_ws.
### URSim
#### Create docker container for URSim
```bash
docker pull universalrobots/ursim_e-series
cd multi_ursim_interface/dockerursim
docker build -t my/dockerursim .
docker network create --subnet=192.168.56.0/24 ursim_net
```

#### Change Firewall settings
```bash
sudo ufw status # if active
sudo ufw allow 50001 # reverse_port for robot 1
sudo ufw allow 50002 # script_sender_port for robot 1
sudo ufw allow 50003 # trajectory_port for robot 1
sudo ufw allow 50011 # reverse_port for robot 2
sudo ufw allow 50012 # script_sender_port for robot 2
sudo ufw allow 50013 # trajectory_port for robot 2
```
Allows the URSim inside docker container to access host via specific ports, specified in the [launch file](ursim_control/launch/single_ur5e_control.launch).

## Execution 

```bash
docker run --rm -it -p 5900:5900 -p 6080:6080 --net=ursim_net --ip 192.168.56.101 --add-host=host.docker.internal:host-gateway --name ursim_robot_1 my/dockerursim
```
To  view the GUI: http://localhost:6080/vnc.html?host=localhost&port=6080

For the second URSim, change the robot ip and port mappings:
```bash
docker run --rm -it -p 5901:5900 -p 6081:6080 --net=ursim_net --ip 192.168.56.102 --add-host=host.docker.internal:host-gateway --name ursim_robot_2 my/dockerursim
```
To  view the GUI: http://localhost:6081/vnc.html?host=localhost&port=6081

Remember to set the correct **Host IP** and **Custom port** (set to corresponding script_sender_port) in URSim's **Installation -> URCaps -> External Control** for each robot.

To run the interface for both robots:
```bash
roslaunch ursim_control single_ur5e_control.launch robot_name:=robot1 robot_ip:=192.168.56.101 reverse_port:=50001 script_sender_port:=50002 trajectory_port:=50003
```
Follow the instructions to start the robot, if you see 
```
Robot connected to reverse interface. Ready to receive control commands.
```
in terminal, the robot is ready to be controlled.

Try publishing a pose topic to /robot1/robot_cmd to test IK control.