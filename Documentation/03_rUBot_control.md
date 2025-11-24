# **3. ROS2 rUBot Mecanum Control**

The objectives of this chapter are:
- control in virtual environment 
- control with real robot

We have created different activities in this section:
- Robot performances
    - Keyboard control
    - Python programming control
- Autonomous control with obstacle avoidance
- Robot Wall follower

The final model represents the real robots we will use in the laboratory:
- rUBot mecanum custom made robot
- LIMO commercial mecanum robot


| <img src="./Images/01_Setup/01_rubot_pi.jpg" width="270"/> | <img src="./Images/01_Setup/02_Limo.png" width="250"/> |<img src="./Images/01_Setup/rosbot_xl.png" width="260"/> |
|:--:|:--:|:--:|
| **rUBot** | **LIMO** | **ROSbot** |

**Bibliography:**
- TheConstruct: Build Your First ROS2 Based Robot https://www.robotigniteacademy.com/courses/309
- LIMO repository: https://github.com/agilexrobotics/limo_ros2/tree/humble
- LIMO Doc: https://github.com/agilexrobotics/limo_pro_doc/blob/master/Limo%20Pro%20Ros2%20Foxy%20user%20manual(EN).md


## **3.1. Robot performances**

We will need to create a new package. This is already done, but if you want to do it from scratch:
```shell
ros2 pkg create --build-type ament_python my_robot_control --dependencies rclpy std_msgs sensor_msgs geometry_msgs nav_msgs
cd ..
colcon build
```

### **1.1. Kinematics model of mecanum robot**
The rUBot mecanum is based on a 4-wheels and has a Mecanum-drive kinematics model. We have first to analyse its kinematics model to properly control it.

Wheeled mobile robots may be classified in two major categories, holonomic (omnidirectional) and nonholonomic. 
- **Nonholonomic mobile robots**, such as conventional cars, employ conventional wheels, which prevents cars from moving directly sideways.
- **Holonomic mobile robots**, such as mecanum cars, employ omni or mecanum wheels, which allow lateral and diagonal movements

The rUBot mecanum corresponds to a Kinematic model for Holonomic Mecanum wheeled robot:

Omnidirectional wheeled mobile robots typically employ either omni wheels or mecanum wheels, which are typical wheels augmented with rollers on their outer circumference. These rollers spin freely and they allow sideways sliding while the wheel drives forward or backward without slip in that direction.

The **different movements** our car can perform are:
![](./Images/03_Control/01_mecanum_movements.png)

The **forces** involved define the robot linear and angular movement:
![](./Images/03_Control/02_mecanum_forces.png)

The **Forward Kinematics** equations are defined below:
![](./Images/03_Control/03_mecanum_fkine.png)

where

- Vi: Linear speed of the wheels.
- ωdi: Angular speed of the wheels.
- Vir: Tangential speed of the rollers.
- ul: Linear velocity of the system on the X axis.
- uf: Linear velocity of the system on the Y axis.
- ω: Speed of rotation of the system on the Z axis.
- a: Distance from the center of the robot to the axis of rotation of the wheel.
- b: Distance from the center of the robot to the center of the width of the wheel.

>(see [Lynch & Park, 2017] for a complete derivation of this model).

In the **Inverse Kinematics** we want to apply a robot movement defined by:
- a linear and angular velocity using a Twist message type published in a /cmd_vel topic. 
- we need to calculate the 4 wheel speeds needed to obtain this robot velocity

This is defined by the following expressions:
![](./Images/03_Control/04_mecanum_ikine.png)

To obtain the **Odometry** we use the information of (uf,ul,w) and Gazebo plugin calculates the POSE of our robot.

The analytical expressions are explained graphically in the picture:
![](./Images/03_Control/05_mecanum_odom.png)

In the case of real mecanum robot this is calculated by the robot driver as an arduino program in arduino-mega platform.

### **1.2. Robot control**

We will first drive the robot with speciffic Twist message.

We can control the movement of our robot programatically in python creating a "/my_robot_control_node" node

We will do it first in virtual environment and later with the real robot.

**Virtual environment**

A first simple control program is created to move the robot according to a speciffic Twist message.

- We first bringup our robot (rubot/rubot_mecanum.urdf) in a speciffic world on a desired POSE:
```shell
ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml x0:=1.0 y0:=1.0 yaw0:=1.8 robot:=rubot/rubot_mecanum.urdf custom_world:=square3m_walls.world
```
![](./Images/03_Control/06_bringup_sw.png)

- We will create now a first robot control python file "my_robot_control.py" to define a rubot movement with linear and angular speed during a time td
- Because we will use parameters, review in the file:
    - Declare Parametes
    - Get Parameters
- We have to add in "setup.py" the entry point corresponding to the created node and the executable name after compilation process

    ```python
        entry_points={
            'console_scripts': [
                'my_robot_control_exec = my_robot_control.my_robot_control:main',
            ],
        },
    ```
- Create "launch" folder
- Install the launch and config folders modifying the "setup.py" file
- Add dependency on the ros2launch package in "package.xml":
    ```shell
    <exec_depend>ros2launch</exec_depend>
    ```
- Create specific launch file "my_robot_control.launch.xml" or "my_robot_control.launch.py" to launch the node and python file created above
- The parameter values can be updated:
    - In the node with the "declare parameter"
    - In the launch file with the parameter values
    - as arguments in command-line
- Compile again and execute (I have changed the time duration as argument in command-line):
    ```
    ros2 launch my_robot_control my_robot_control.launch.xml vx:=0.0 vy:=0.2 td:=5.0
    ```
    > Change parameter values to verify different movements

**Real robot**

The same simple control program created in virtual environment to move the robot is used for the real robot.

- We first bringup our real robot. Remember that this is already done when you power on the robot.

    ![](./Images/03_Control/08_bringup.png)

- We control the robot with the same node created for virtual environment:
    ``` shell
    ros2 launch my_robot_control my_robot_control.launch.xml vx:=0.0 vy:=0.2 w:=0.0 td:=5.0
    ```
To properly control your real rUBot, we have a very usefull Lidar sensor to detect obstacles and avoid collisions.

**Activity: Lidar test**

The LIDAR sensor we are using in our rUBot mecanum robot is a RPLIDAR A1 with the following specifications:
- angle_min: -3.141593 (rad)
- angle_max: 3.141593 (rad)
- angle_increment: 0.008727 (rad)  -> 720 laser beams

- Verify if the Lidar model you have specified in `rubot_mecanum.urdf` file has the same speciffications as the real one. Modify it if necessary.
- Bringup the corrected rUBot mecanum model:
    ```shell
    ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml x0:=1.0 y0:=-0.5 yaw0:=0.0 robot:=rubot/rubot_mecanum.urdf custom_world:=square3m_walls.world
    ````
- A node is created to verify the Lidar readings:
    ````shell
    ros2 launch my_robot_control my_robot_lidar_test.launch.xml
    ````
    - Are the Lidar readings correct?
    - what do you think it could hapen?

The objectives of this activity are:
- Identify the Lidar specifications in the urdf file and correct it if necessary
- Create a new `my_robot_lidar_test_rUBot.launch.xml` and `my_robot_lidar_test_rUBot.py` file to verify the proper Lidar readings
- Launch the `my_robot_lidar_test_rUBot.launch.xml` file and show:
    - The minimum distance and angle to a wall detected by the Lidar
    - The distances at 0º, 90º and -90º with respect to the robot front

Upload a pdf file with a picture including:
- Gazebo bringup where you can see the robot in a speciffic POSE in the world
- terminal running the `my_robot_lidar_test_rUBot.launch.xml` with distances readings

**Lab Session: rUBot control and Lidar test**

The objectives of this lab session are:
- Verify the proper Lidar readings in your rUBot mecanum robot in Gazebo simulation
- Create a new `my_robot_control_lidar.launch.xml` and `my_robot_control_lidar.py` file to move the robot with a desired Twist message until a distance of 30cm in the movement direction to wall is detected
- Launch the `my_robot_control_lidar.launch.xml` file and show:
    - The robot moving until a the distance of 30cm to a wall is detected
    - The distance and angle to a wall detected by the Lidar
- Verify first in Gazebo virtual environment and later with the real robot

## **2. Driving self-control using Lidar sensor**

We will use now the created world to test the autonomous navigation with obstacle avoidance performance. 

The algorithm description functionality, created in "my_robot_selfcontrol.py" file,is:
- The created node makes the robot go forward.
    - LIDAR is allways searching the closest distance and the angle
    - when this distance is lower than a threshold, the robot goes backward with angular speed in the oposite direction of the minimum distance angle.

Let's verify first this behaviour in virtual environment

**VIRTUAL environment**

We have to launch the "my_robot_selfcontrol.launch.xml" file in the "my_robot_control" package.
```shell
ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml x0:=1.0 y0:=1.0 yaw0:=1.8 robot:=rubot/rubot_mecanum.urdf custom_world:=square3m_walls.world
ros2 launch my_robot_control my_robot_selfcontrol.launch.xml time_to_stop:=10.0
```
>- Verify in rviz if you have to change the fixed frame to "odom" frame
>- You can test the behaviour when tunning the parameters defined

**Activity: rUBot self-control**

The objective of this activity is to modify the code to move the robot in Holonomic way, for exemple:
-  When the minimum distance is in the right side move the robot over the left side

Design the code using the Holonomic robot performances, and upload:
- the file "my_robot_selfcontrol_holonomic.py"
- a video of the current behaviour in your designed world

**REAL robot**

We have to launch the same `my_robot_selfcontrol.launch.xml` file designed for Virtual environment.
```shell
ros2 launch my_robot_control my_robot_selfcontrol.launch.xml time_to_stop:=10.0
```
>Verify if you have used the number of laser beams of your Lidar included in your rUBot!

**Lab Session: rUBot selfcontrol**

The objectives of this lab session are:
- Verify the designed self-control behaviour in your rUBot mecanum robot in Gazebo simulation
- Validate the same behaviour with the real robot

## **3. Wall Follower**

We have created a Wall_Follower strategy based on the reading distances from LIDAR in the ranges: front, front-right, right and back-right, and perform a specific actuation in function of the minimum distance readings.

The algorith is based on laser ranges test and speciffic actuations on each region: 
![](./Images/03_Control/10_lidar_rg.png)

**VIRTUAL environment**

We have to launch the "my_robot_wallfollower.launch.xml" file in the "my_robot_control" package.
```shell
ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml x0:=1.0 y0:=1.0 yaw0:=1.8 robot:=rubot/rubot_mecanum.urdf custom_world:=square3m_walls.world
ros2 launch my_robot_control my_robot_wallfollower.launch.xml time_to_stop:=50.0
```
>- You can test the behaviour when tunning the parameters defined

**Activity: rUBot wall-follower**

The objective of this activity is to modify the code to move the robot in Holonomic way, for exemple:
-  When the minimum distance is in the front side move the robot over the left side
- Whem the minimum distance is in the front-right side move the robot over the front-left side
- When the minimum distance is in the right side move the robot forward and maintain its orientation parallel to the wall
- When the minimum distance is in the back-right side move the robot over the front-right side
- When the minimum distance is in the back side move the robot over the right side

Design the code using the Holonomic robot performances, and upload:
- the file "my_robot_wallfollower_holonomic.py"
- a video of the current behaviour in your designed world

**REAL robot**

We have to launch the same "my_robot_selfcontrol.launch.xml" file designed for Virtual environment.
```shell
ros2 launch my_robot_control my_robot_wallfollower.launch.xml time_to_stop:=50.0
```
>The robot is not working as expected because the number of laser beams is not 720 as in simulation!

**Lab Session: rUBot Wall Follower**

The objectives of this lab session are:
- Verify the designed wall-follower behaviour in your rUBot mecanum robot in Gazebo simulation
- Validate the same behaviour with the real robot

## **4. Go to POSE**

We have created a Go2Pose node based on the Odometry information to reach a desired POSE in the environment. 

```shell
ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml x0:=1.0 y0:=1.0 yaw0:=1.8 robot:=rubot/rubot_mecanum.urdf custom_world:=square3m_walls.world
ros2 launch my_robot_control my_robot_go2pose.launch.xml 
```