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
        <li><a href="#run-the-controller-node">Run the controller node</a></li>
    </ul>
    <li><a href="#kinematics-of-mobile-robot">Kinematics of Mobile robot</a></li>
        <ul>
            <li><a href="#1-robot-model">Robot model</a></li>
            <li><a href="#2-inverse-kinematics-models">Inverse Kinematics Models</a></li>
            <li><a href="#3-forward-kinematics-models">Forward Kinematics Models</a></li>
            <li><a href="#4-model-selection-guide">Model selection guide</a></li>
        </ul>
    <li><a href="#path-tracking-controller">Path tracking controller</a></li>
    <li><a href="#state-estimator">State estimator</a></li>
</ol>

<!-- ABOUT THE PROJECT -->
## About The Project

This project is in FRA532(Mobile Robotics) class at **FIBO** that teach about Mobile robotics. So **LAB1** is the one of class's lab that have 3 sub-labs that are kinematics of mobile robot, path tracking controller, and state estimator.

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

⚠️  **Warning:**    Before use this project you need to `source ~/FRA532_LAB1/install/setup.bash` and `source /opt/ros/humble/setup.bash` everytime that you open new terminal. If you want to make sure that 2 path has been source everytime when open new terminal you can follow the command below and next time you open new terminal .bashrc will source everything you write on that file.

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

This step will explain more in <a href="#path-tracking-controller">Lab 1.2</a> section.

<p align="right">(<a href="#fra532-lab1">back to top</a>)</p>

## Kinematics of Mobile robot

This section provided robot model with inverse and forward kinematics for simulate, control, and calculate odometry of robot.

### 1. Robot model

#### 1.1 Robot description

Robot description is the part that have to use for visualize in rviz and simulate in gazebo. You can check our robot description <a href="src/ack_description/urdf/robot.xacro">here</a>.

<p align="center"><img src="images/robot_model.png" alt="Robot model" /></p>

This model is based on intruction of this lab, see below.

<p align="center"><img src="images/model_instruction.png" alt="Model instuction" /></p>

Additionaly, for <a href="#31-yaw-rate-model">Yaw-Rate Model</a> we add imu sensor frame (yellow cylinder on the head) to simulate imu sensor and link it into gazebo.

#### 1.2 Transformation

<p align="center"><img src="images/tf.png" alt="Robot model" /></p>

The transformation tree is:

* base_link
    * chassis
        * left_rear_wheel_link
        * left_steer_link
            * left_front_wheel_link
        * right_rear_wheel_link
        * right_steer_link
            * right_front_wheel_link
        * imu_link

And after we create robot odometry transformation tree will change into this:

* world
    * odom
        * base_link
            * chassis
                * left_rear_wheel_link
                * left_steer_link
                    * left_front_wheel_link
                * right_rear_wheel_link
                * right_steer_link
                    * right_front_wheel_link
                * imu_link

If we use `view frame` of `tf2_tools`, the result is a image below.

<p align="center"><img src="images/tf_viewframe.png" alt="tf_viewframe" /></p>

### 2. Inverse Kinematics Models

#### 2.1 Bicycle Model

#### 2.2 No-Slip Condition Model

### 3. Forward Kinematics Models

#### 3.1 Yaw-Rate Model

#### 3.2 Single-Track Model

#### 3.3 Double-Track Model

### 4. Model Selection Guide

<p align="right">(<a href="#fra532-lab1">back to top</a>)</p>

## Path tracking controller

<p align="right">(<a href="#fra532-lab1">back to top</a>)</p>

## State estimator

<p align="right">(<a href="#fra532-lab1">back to top</a>)</p>