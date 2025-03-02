# FRA532 LAB1

<p align="center"><img src="images/gazebo_purepursuit.gif" alt="Overview .gif" /></p>

<!-- TABLE OF CONTENTS -->
## Table of Contents
<ol>
    <li>
        <a href="#about-the-project">About The Project</a>
    </li>
    <li>
        <a href="#getting-started">Getting Started</a>
        <ul>
        <li><a href="#prerequisites">Prerequisites</a></li>
            <ul>
                <li><a href="#python-packages">Python packages</a></li>
                <li><a href="#ros2-packages">ROS2 packages</a></li>
            </ul>
        <li><a href="#installation">Installation</a></li>
        </ul>
    </li>
    <li><a href="#usage">Usage</a></li>
    <ul>
        <li><a href="#launch-the-project">Launch the project</a></li>
        <li><a href="#service-call-in-this-project">Service call in this project</a></li>
        <li><a href="#teleop_twist_keyboard">Teleop twist keyboard</a></li>
    </ul>
    <li><a href="#features">Features</a></li>
    <li><a href="#contact">Contact</a></li>
</ol>

<!-- ABOUT THE PROJECT -->
## About The Project

This project is in FRA532(Mobile Robotics) class at **FIBO** that teach about Mobile robotics. So **LAB1** is the one of class's lab that have 3 sub-labs that are mobile robot kinematics, path tracking controller, and state estimator.

<p align="center"><img src="images/robot_initial_pose.png" alt="Mobile robot kinematics" /></p>

<p align="center"><img src="images/robot_initial_pose.png" alt="Path tracking controller" /></p>

<p align="center"><img src="images/robot_initial_pose.png" alt="State estimator" /></p>

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

To use this project. You need to have all of prerequisites for this project.

#### Python packages

⚠️ **Warning:** Make sure you have python version >= 3.6 already.

*   numpy

    ```
    pip3 install numpy
    ```

*   matplotlib

    ```
    pip3 install matplotlib
    ```

* tf_transformations

    ```
    pip3 install tf_transformations
    ```

* pyyaml

    ```
    pip3 install pyyaml
    ```

#### ROS2 packages

⚠️ **Warning:** Make sure you have ROS2 humble and gazebo classic already.

* robot state publisher

    ```
    sudo apt install ros-humble-robot-state-publisher
    ```

* ros2 control

    ```
    sudo apt install ros-humble-ros2-control
    ```

* ros2 controllers

    ```
    sudo apt install ros-humble-ros2-controllers
    ```

* gazebo ros2 control

    ```
    sudo apt install ros-humble-gazebo-ros2-control
    ```

### Installation

Follow the command below to dowload and install package.

1.  Go to home directory

    ```
    cd
    ```

2.  Clone the repository and change directory to workspace

    ```
    git clone https://github.com/itsnotthetim/FRA532_LAB1.git
    cd FRA532_LAB1
    ```

3.  Build & Source the packages

    ```
    colcon build
    source install/setup.bash
    ```

<p align="right">(<a href="#fra532-lab1">back to top</a>)</p>

<!-- USAGE -->
## Usage

⚠️ **Warning:** Before use this project you need to `source ~/FRA532_LAB1/install/setup.bash` and `source /opt/ros/humble/setup.bash` everytime that you open new terminal. If you want to make sure that 2 path has been source everytime when open new terminal you can follow the command below and next time you open new terminal .bashrc will source everything you write on that file.

```
echo "source ~/FRA532_LAB1/install/setup.bash" >> ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Launch the project

```
ros2 launch ack_description sim.launch.py
```

After launch the project, rviz2 and gazebo window will show up on your screen with ackermann robot in gazebo world like this picture below.

<p align="center"><img src="images/launch_sim.png" alt="After launch project" /></p>

### Run the controller node

To use controller for path tracking in lab 1.2 and 1.3, you can use this command below to start the controller node.

```
ros2 run ackermann_controller <controller_node>
```

Example: To run the pure pursuit controller you can use this command below.

```
ros2 run ackermann_controller pure_pursuit_controller.py
```

This step will explain more in Lab <a href="#lab-1.2">1.2</a> section.

<p align="right">(<a href="#fra532-lab1">back to top</a>)</p>