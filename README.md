# Reactive Robot (⚡ 🤖)

## Mission

The goal of this project is to create a reactive robot that can navigate in a world where there is a question mark shaped wall. The robot should be able to navigate from the start point (rounded area) till the "bottom" of question mark wall, following without hitting it.

```

      ???????     
    ??:::::::??   
  ??:::::::::::?  
 ?:::::????:::::? 
 ?::::?    ?::::? 
 ?::::?  O  ?::::?
 ??????     ?::::?
           ?::::? 
          ?::::?  
         ?::::?   
        ?::::?    
       ?::::?     
       ?::::?     
       ??::??     
        ????      
         X -> End point
```
## Project Structure
6 directories, 17 files

```bash 
├── CMakeLists.txt
├── README.md
├── config
│   ├── gaz_ros2_ctl_use_sim.yaml
│   ├── gazebo_params.yaml
│   └── my_controllers.yaml
├── description
│   ├── gazebo_control.xacro
│   ├── inertial_macros.xacro
│   ├── lidar.xacro
│   ├── robot.urdf.xacro
│   ├── robot_core.xacro
│   └── ros2_control.xacro
├── launch
│   ├── launch_sim.launch.py
│   └── rsp.launch.py
├── package.xml
├── scripts
│   └── follow_wall.py
└── worlds
    ├── empty.world
    └── question_mark_wall.world
```

## Installation and Build

This project use the version 2 of ROS (Robot Operating System) and the Gazebo simulator. To install ROS 2, follow the instructions on the [ROS 2 Installation Guide](https://docs.ros.org/en/foxy/Installation.html).

Beside ROS 2, We will need some other packages:

```bash
    $ sudo apt install ros-foxy-gazebo-ros-pkgs ros-foxy-joint-state-publisher ros-foxy-joint-state-publisher-gui row-foxy-xacro gazebo_ros twis_mux
```

Create a workspace and clone the project:

```bash
    $ mkdir -p ~/ros2_ws/src
    $ cd ~/ros2_ws/src
    $ git clone https://github.com/JoseMarshall/reactivebot_one.git
```

Build the project:

```bash
    $ cd ~/ros2_ws
    $ colcon build --symlink-install
```

## Usage

After creating the workspace (on the example we are considering that the workspace name is ros2_ws) and building the project, change the directory to the workspace and source the setup file:

```bash
    $ cd ~/ros2_ws
    $ source install/local_setup.bash
```

Run the following command to start the robot on the simulation world:

```bash
    $ ros2 launch reactive_robot launch_sim.launch.py
```

To start moving the robot and make it follow the wall, run the script:

```bash
    $ python3 scripts/follow_wall.py
```

If you want to run the robot without the simulation, run the following command:

```bash
    $ ros2 launch reactive_robot rsp.launch.py
```
