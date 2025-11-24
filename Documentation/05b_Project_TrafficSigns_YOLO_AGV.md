# Project for Traffic Signal Detection with YOLO

We will describe the Computer Vision based method to identify the Traffic Sign.

Training models: 
- https://roboflow.com/
- https://github.com/ultralytics/ultralytics
- https://docs.ultralytics.com/es/usage/python/#how-do-i-train-a-custom-yolo-model-using-my-dataset

For this project we have created a new package "my_robot_ai_identification" where we have used 2 strategies to perform signal identification:
- Keras with tensorflow
- YOLO 

## **1. ROS2 packages installation**

The needed Installation for YOLO identification is only to install "ultralytics" on the ROS2 Humble environment. Open a terminal and type:
````shell
pip install ultralytics
pip3 uninstall numpy
pip3 install "numpy<2.0"
````

## **2. Robot Navigation**

To proceed with the signal identification we first bringup the robot and navigate from initial pose to final target.

- Bringup the robot
    - In simulation:
    ````shell
    ros2 launch my_robot_bringup my_robot_bringup_sw.launch.xml use_sim_time:=true x0:=0.5 y0:=-1.5 yaw0:=1.57 robot:=rubot/rubot_mecanum.urdf custom_world:=square4m_sign.world
    ````
    >Important: Include a traffic signal in the world. When using "square4m_sign.world" you can change the sign model on line 30 changing the traffic sign model name
    - In real robot LIMO the bringup is already made when turned on

- Generate a map
    - In simulation:
        ````shell
        ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=true
        ````
    - In real robot (rUBot or LIMO):
        ````shell
        ros2 launch my_robot_cartographer cartographer.launch.py use_sim_time:=false
        ````
- Save the map in my_robot_navigation2/map folder with:
    ````shell
    cd src/Navigation_Projects/my_robot_navigation2/map/
    ros2 run nav2_map_server map_saver_cli -f map_square4m_sign
    ````
- Navigate using the Map:
    - In simulation:
        ````bash
        ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=true map_file:=map_square4m_sign.yaml params_file:=limo_sw.yaml
        ````
        >For LIMO: We use `limo_sw.yaml` file. In case we want to priorize the lidar data from odometry data we will use `limo_sw_lidar.yaml`. Equivalent names are found for rUBot real robot.

        ![](./Images/07_Yolo/10_nav_sw.png)
    - In the case of real robot:
        - Because the bringup is done without the LIMO robot model. The only frames available are
            - odom: as a ``base_frame_id``
            - base_link: as the ``robot_base_frame``
        - We have to create "LIMO_real.yaml" file in "param" folder correcting base_frame_id: "odom" (instead of base_footprint)
        ````shell
        ros2 launch my_robot_navigation2 navigation2_robot.launch.py use_sim_time:=false map_file:=map_square4m_sign.yaml params_file:=limo_real.yaml
        ````
        > Be sure the Odometry message is zero when starting the navigation.

## **3. Signal prediction**

You can make a prediction of the signal that the robot find on its path to target pose:
- for 1 test image (use ``picture_prediction_yolo.py``). 
- for video images from robot camera when moving to target (use ``rt_prediction_yolo.py``)

**Software** test in Gazebo: 
- Use the ``limo_rt_prediction_yolo.py`` after the navigation node is launched.
    ````shell
    ros2 run my_robot_ai_identification limo_rt_prediction_yolo_exec
    ````
    > You have to verify the model path to '/home/user/ROS2_rUBot_mecanum_ws/src/AI_Projects/my_robot_ai_identification/models/yolov8n_custom.pt

    > If you are using rUBot robot change the exec to `rubot_rt_prediction_yolo_exec`

    > Verify also the the camera topic if you are using rUBot or Limo

- To see the image with prediction on RVIZ2, select a new Image message on topic /inference_result
    ![](./Images/07_Yolo/11_prediction_sw.png)
    ![](./Images/07_Yolo/11_prediction_sw2.png)

**Hardware** Test in real LIMO robot:
- If you want to execute on Limo robot, you have to install on the Limo robot container:
    ````shell
    apt update
    apt install python3-pip
    pip install ultralytics
    # needed numpy version compatible
    pip3 uninstall numpy
    pip3 install "numpy<2.0"
    #
    apt install git
    git clone https://github.com/manelpuig/ROS2_rUBot_mecanum_ws.git
    source /opt/ros/humble/setup.bash
    apt install python3-colcon-common-extensions
    apt install build-essential
    colcon build
    source install/setup.bash
    ros2 run my_robot_ai_identification rt_prediction_yolo_exec
    ````
    
- Run The real-time prediction:
    ````shell
    ros2 run my_robot_ai_identification limo_rt_prediction_yolo_exec
    ````
    
## **5. Robot actuation after prediction**

Consider:
- Predefined map positions of traffic signs. These coordinates MUST be in the same reference frame as the robot's odometry (/odom) or /map (depending on your setup).
- If YOLO detects a sign AND the robot is within "front_distance" parameter value of the corresponding sign position, then the robot reacts.
- Specify the sign positions in a `yolo_signs.yaml` file as follows:
    ```yaml
    sign_positions:
      STOP:      [2.30, -1.00]   # STOP sign position (x, y)
      Ceda:      [5.00,  0.20]   # Yield (Ceda el paso)
      Derecha:   [7.10,  1.40]   # Turn right
      Izquierda: [7.10, -1.40]   # Turn left
      Prohibido: [9.00,  0.00]   # No entry / forbidden
    ````
- To launch the robot actuation node after prediction, use:
    ````shell
    ros2 launch my_robot_ai_identification rubot_navigation_yolo.launch.py
    ````