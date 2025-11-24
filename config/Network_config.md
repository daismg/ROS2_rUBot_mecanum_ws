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

You can verify connectivity with:

```bash
ping 192.168.1.14    # from the PC to the robot
ping 192.168.1.15    # from the robot to the PC
````
### 1.2 ROS 2 environment (.bashrc)
Both the robot and the PC use a similar .bashrc configuration (user: student):

````bash
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /home/student/Desktop/ROS2_rUBot_mecanum_ws/install/setup.bash
cd /home/student/Desktop/ROS2_rUBot_mecanum_ws

export ROS_DOMAIN_ID=1               # group/domain ID
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GAZEBO_MODEL_PATH=/home/student/Desktop/ROS2_rUBot_mecanum_ws/src/my_robot_bringup/models:$GAZEBO_MODEL_PATH
export QT_QPA_PLATFORM=xcb           # Best for RVIZ2
export ROS_LOCALHOST_ONLY=0          # allow communication with other machines
````

With this configuration, both machines:

- Share the same ROS domain (ROS_DOMAIN_ID=1).

- Use CycloneDDS as the RMW implementation.

- Are allowed to communicate over the network (ROS_LOCALHOST_ONLY=0).

## 2. Why discovery sometimes fails (multicast vs unicast)
ROS 2 with CycloneDDS uses DDS discovery to automatically find publishers and subscribers across machines. By default, discovery is based on UDP multicast in the given ROS domain:

- Each participant sends multicast "I am here" messages.

- Other participants in the same domain listen to these multicast messages and add the new participant to their peer list.

In this lab setup, the following issue is observed:

- Case A (works):

    - Robot is powered on first.

    - Robot bringup is started (nodes are running).

    - PC is powered on afterwards.

    - When ROS 2 commands (e.g. ros2 topic list) are launched on the PC, it discovers the robot's topics correctly.

- Case B (problematic):

    - PC is powered on first, ROS 2 environment is used (daemon starts, DDS participants created).

    - Robot is powered on later and bringup is started.

    - The PC can ping the robot (192.168.1.14), but does not see any ROS 2 topics from the robot.

    - Only after rebooting the PC (or restarting the ROS 2 daemon/DDS participants) the robot's topics become visible.

This is typically caused by:

- The WiFi router handling multicast in an unreliable way.

- The order of startup of CycloneDDS participants and the router’s multicast behavior.

- The ROS 2 daemon on the PC not properly detecting new participants under some network conditions.

In short: IP connectivity is fine (ping works), but DDS multicast discovery is unreliable, depending on who starts first.

## 3. Solution: CycloneDDS peer-to-peer unicast configuration
To make the system robust and independent of multicast quirks and startup order, we explicitly configure CycloneDDS to use unicast peer discovery between the PC and the robot.

With this configuration:

- Each machine uses its own fixed IP address for DDS.

- Each machine is given the other machine as a peer (explicit unicast address).

- Multicast can still be allowed, but discovery no longer depends on it.

We will:

- Store configuration files inside the ROS 2 workspace:

````text
/home/student/Desktop/ROS2_rUBot_mecanum_ws/config/cyclonedds_pc.xml
/home/ubuntu/ROS2_rUBot_mecanum_ws/config/cyclonedds_robot.xml
````
- Set CYCLONEDDS_URI in .bashrc to point to these files.

## 4. Unicast configuration
On both the PC and the robot, we assume the ROS 2 workspace is:

- Create a config directory inside the workspace:

- CycloneDDS configuration file on the PC (/home/student/Desktop/ROS2_rUBot_mecanum_ws/config/cyclonedds_pc.xml) with the following content:

    ````xml
    <?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
    <Domain id="any">
        <General>
        <!-- Bind CycloneDDS to the PC's IP address -->
        <NetworkInterfaceAddress>192.168.1.15</NetworkInterfaceAddress>
        <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
        <!-- Explicitly add the robot as a unicast peer -->
        <Peers>
            <Peer address="192.168.1.14"/>
        </Peers>
        </Discovery>
    </Domain>
    </CycloneDDS>
    ````
    Notes:

    NetworkInterfaceAddress: Can be the IP (192.168.1.15) or an interface name (e.g. wlp2s0). This ensures DDS uses the correct network interface.

    Peers: Here we tell the PC to directly contact the robot at 192.168.1.14 via unicast.

- CycloneDDS configuration file on the robot. On the robot (192.168.1.14), at /home/ubuntu/ROS2_rUBot_mecanum_ws/config/cyclonedds_robot.xml, with the following content:

    ````xml
    <?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS>
    <Domain id="any">
        <General>
        <!-- Bind CycloneDDS to the robot's IP address -->
        <NetworkInterfaceAddress>192.168.1.14</NetworkInterfaceAddress>
        <AllowMulticast>true</AllowMulticast>
        </General>
        <Discovery>
        <!-- Explicitly add the PC as a unicast peer -->
        <Peers>
            <Peer address="192.168.1.15"/>
        </Peers>
        </Discovery>
    </Domain>
    </CycloneDDS>
    ````
    Again:

    NetworkInterfaceAddress ensures the robot uses the WiFi network interface associated with 192.168.1.14.

    Peers tells the robot to directly talk to the PC at 192.168.1.15.

- Updating .bashrc to use these configs: We now need to tell CycloneDDS to load these XML files using the CYCLONEDDS_URI environment variable.

    - On the PC: Edit the .bashrc for user student and add:

        ````bash
        export CYCLONEDDS_URI=file:///home/student/Desktop/ROS2_rUBot_mecanum_ws/config/cyclonedds_pc.xml
        ````
        >Make sure this line is after the RMW_IMPLEMENTATION=rmw_cyclonedds_cpp line.

    - On the robot:

        ````bash
        export CYCLONEDDS_URI=file:///home/ubuntu/ROS2_rUBot_mecanum_ws/config/cyclonedds_robot.xml
        ````

## 5. How to use and test
- Start the robot:

- Boot the PC.

    - Log in as student.

    - Open a terminal:

        ````bash
        source ~/.bashrc
        ros2 topic list
        ````
        >You should now see the robot’s topics even if the PC was powered on before or after the robot. The order should no longer matter.

- If something still looks wrong:

    - Restart the ROS 2 daemon on the PC:

        ````bash
        ros2 daemon stop
        ros2 topic list   # this will implicitly restart the daemon
        ````
        >If the unicast configuration is correct and the IP addresses are reachable, this setup should be very robust against router multicast issues and startup order.

## 6. Adapting this setup to other robots/PCs
If you replicate this setup for multiple robots and PCs in the lab:

Each robot and PC must have:

- A unique IP address (e.g., 192.168.1.14, 192.168.1.24, etc.).

- Matching ROS_DOMAIN_ID for each robot–PC pair or group.

- For each pair:

    - Create a dedicated CycloneDDS XML file on each side.

    - Set NetworkInterfaceAddress to each machine’s IP.

    - Add the peer(s) in the <Peers> section with the corresponding IPs.


This approach keeps the lab configuration clear, reproducible, and independent of the router’s multicast behavior.