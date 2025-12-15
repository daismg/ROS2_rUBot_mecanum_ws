# ROS 2 Humble – Lab Network Setup with CycloneDDS Unicast

## 1. Network topology and ROS 2 setup

This document describes the network setup of a teaching lab with a mobile robot and a control PC, both running ROS 2 Humble and CycloneDDS.

### 1.1 Physical network

- **WiFi router (mobile router / hotspot)**

  - Provides DHCP and fixed IP addresses in the `192.168.1.x` network.
- **Robot 1**

  - Hardware: Raspberry Pi 4
  - OS: Ubuntu Server 22.04
  - ROS: ROS 2 Humble
  - IP address (fixed): `192.168.1.14`
- **Control PC**

  - Hardware: PC with Ubuntu 22.04
  - ROS: ROS 2 Humble
  - IP address (fixed): `192.168.1.15`

Both machines are connected to the same WiFi router and are in the same Layer-2 network.

### 1.2 ROS 2 environment

The Ubuntu22.04 installations is performed, either with:

- External SSD USB Disc with Ubuntu 22.04 and ROS2 Humble installed.

- or Docker container (on PC Unix) with ROS2 Humble installed.

In each case the communicacion is ensured by configuring CycloneDDS peer-to-peer unicast configuration.


## 2. CycloneDDS peer-to-peer unicast configuration
To make the system robust and independent of multicast quirks and startup order, we explicitly configure CycloneDDS to use unicast peer discovery between the PC and the robot.

With this configuration:

- Each machine (PC or rUBot) uses its own fixed IP address for DDS.

- Each machine is given the other machine as a peer (explicit unicast address).

- Multicast can still be allowed, but discovery no longer depends on it.

We will:

- Store configuration files inside the ROS 2 workspace:
    ````text
    ~/Desktop/ROS2_rUBot_mecanum_ws/config/cyclonedds_pc.xml
    ~/ROS2_rUBot_mecanum_ws/config/cyclonedds_robot.xml
    ````
- CycloneDDS configuration file for the PC `cyclonedds_pc.xml` 
    - NetworkInterfaceAddress: usually the IP (192.168.1.15). This ensures DDS uses the correct network interface.

    - Peers: Here we tell the PC to directly contact the robot at 192.168.1.14 via unicast.

- CycloneDDS configuration file for the robot. `cyclonedds_robot.xml`
    - NetworkInterfaceAddress: ensures the robot uses the WiFi network interface associated with 192.168.1.14.

    - Peers tells the robot to directly talk to the PC at 192.168.1.15.

- Update `.bashrc` to use these configs: We now need to tell CycloneDDS to load these XML files using the CYCLONEDDS_URI environment variable.

    - On the PC: Edit the `.bashrc` for user student and verify it contains the following lines:

        ````bash
        source /opt/ros/humble/setup.bash
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
        source ~/Desktop/ROS2_rUBot_mecanum_ws/install/setup.bash
        cd ~/Desktop/ROS2_rUBot_mecanum_ws
        export GAZEBO_MODEL_PATH=~/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
        export QT_QPA_PLATFORM=xcb           # Best for RVIZ2
        export ROS_DOMAIN_ID=1               # group/domain ID
        export ROS_LOCALHOST_ONLY=0          # allow communication with other machines
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI=file:///~/Desktop/ROS2_rUBot_mecanum_ws/config/cyclonedds_pc.xml
        ````
    - On the robot:

        ````bash
        source /opt/ros/humble/setup.bash
        source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
        source /home/ubuntu/ROS2_rUBot_mecanum_ws/install/setup.bash
        cd /home/ubuntu/ROS2_rUBot_mecanum_ws
        export GAZEBO_MODEL_PATH=/home/ubuntu/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
        export QT_QPA_PLATFORM=xcb           # Best for RVIZ2
        export ROS_DOMAIN_ID=1               # group/domain ID
        export ROS_LOCALHOST_ONLY=0          # allow communication with other machines
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
        export CYCLONEDDS_URI=file:///home/ubuntu/ROS2_rUBot_mecanum_ws/config/cyclonedds_robot.xml
        ````
        > You will have to connect to the robot via SSH to edit its `.bashrc`.
        > Sometimes you will have to regenerate the keys to connect to the raspberry pi. Type on a `cmd`terminal on the PC:
        ````bash
        ssh-keygen -R rUBot_IP_ADDRESS
        ````

- If something still looks wrong:

    - Restart the ROS 2 daemon on the PC:

        ````bash
        ros2 daemon stop
        ros2 topic list   # this will implicitly restart the daemon
        ````
        >If the unicast configuration is correct and the IP addresses are reachable, this setup should be very robust against router multicast issues and startup order.

## 3. Time Synchronization

Make sure PC and Robot (Raspberry Pi) have the same system time using built-in NTP.


Matching time helps with:

- TF transforms

- Logging

- Sensor fusion

- Debugging timestamped data

(ROS 2 discovery does NOT depend on clock sync, but TF and logs do.)

### On the PC (Ubuntu 22) and robot (Raspberry Pi Ubuntu 22):

- Check current time sync status
    ````bash
    timedatectl status
    ````

- It should show:

    - System clock synchronized: yes

    - NTP service: active

    - Time zone: Europe/Madrid (or your timezone)

- If not, enable NTP:
    ````bash
    # Enable automatic NTP time synchronization
    sudo timedatectl set-ntp true
    # Set your timezone
    sudo timedatectl set-timezone Europe/Madrid
    # Check again
    timedatectl status
    ````


### On Docker containers

No extra configuration needed.

Containers inherit the host’s system clock, so:

- If the PC clock is correct → the container clock is correct.

- Mounting /etc/localtime is only for correct timezone display, not time sync.

    ````yaml
    volumes:
    - /etc/localtime:/etc/localtime:ro   # timezone only (not time sync)
    ````

## 4. ROS2 environment on Docker based computers

Computers with DualBoot (Windows-Linux) we need to use a Docker based setup to run ROS2 Humble.

We have created a `ros2-humble-biorobub.zip` file containing:
- Dockerfile: with all the ROS2 Humble packages needed and some tools like VSCode installed.
- docker-compose.yml: to launch the container with the proper configuration.
- cyclonedds_pc.xml: CycloneDDS unicast configuration file for the PC.
- cyclonedds_robot.xml: CycloneDDS unicast configuration file for the robot.
- entrypointpc.sh: entrypoint script to setup the environment variables when the container starts.

First of all, a proper Docker Image has been created with the custom configuration on Dockerfile and uploaded to my DockerHub account.
- In the folder where your Dockerfile is:
    ````bash
    docker build -t ros2-humble-biorobub-pc:latest .
    ````
- Tag the image for Docker Hub
    ````bash
    docker tag ros2-humble-biorobub-pc:latest manelpuig/ros2-humble-biorobub-pc:latest
    ````
- Login to Docker Hub from terminal and follow instructions to autenticate
    ````bash
    docker login
    ````
- Push the image to Docker Hub
    ````bash
    docker push manelpuig/ros2-humble-biorobub-pc:latest
    ````
- Once the image is pushed properly to my Dockerhub account, I have to delete the local image tag:
    ````bash
    docker rmi ros2-humble-biorobub-pc:latest
    ````
Students in the lab they only need to:
- Unzip the `ros2-humble-biorobub.zip` file in a `~/Desktop/rob` folder on Linux PC
- review on:
    - `docker-compose.yml` file: `ROS_DOMAIN_ID` variable to match your robot.
    - `cyclonedds_pc.xml` file: IPs to match your PC and robot.
    - `cyclonedds_robot.xml` file: IPs to match your robot and PC.
- Open a terminal in the `~/Desktop/rob/ros2-humble-biorobub` folder and run:
    ````bash
    xhost +local:root            # allow X11 for graphs in container
    cd ~/Desktop/ros2-humble-biorobub
    docker-compose up -d
    docker exec -it pc_humble bash
    code .                     # open VSCode inside the container
    ros2 topic list
    ````
- Open `.bashrc` file inside the container and verify it contains:
    ````bash
    source /opt/ros/humble/setup.bash
    source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
    source ~/Desktop/ROS2_rUBot_mecanum_ws/install/setup.bash
    cd ~/Desktop/ROS2_rUBot_mecanum_ws
    export GAZEBO_MODEL_PATH=~/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
    export QT_QPA_PLATFORM=xcb           # Best for RVIZ2
    export ROS_DOMAIN_ID=1               # group/domain ID
    export ROS_LOCALHOST_ONLY=0          # allow communication with other machines
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file:///~/Desktop/ROS2_rUBot_mecanum_ws/config/cyclonedds_pc.xml
    ````

- To stop the container:
    ````bash
    docker-compose down
    ````
- To see the Images and Containers:
    ````bash
    docker ps -a               # containers
    docker images              # images
    ````

You are ready to work with ROS2 Humble on Docker!