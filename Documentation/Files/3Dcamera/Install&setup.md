# Install Intel RealSense D435/i

This camera could be installed on:
- PC Ubuntu 22 with ROS2 Humble
- Raspberrypi4 Ubuntu 22 with ROS2 Humble

The installation process in **PC Ubuntu 22**:

```bash
sudo apt install -y git cmake build-essential \
  libssl-dev libusb-1.0-0-dev libudev-dev pkg-config \
  libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev 

mkdir -p ~/software && cd ~/software
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
sudo ./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DFORCE_RSUSB_BACKEND=true -DBUILD_EXAMPLES=true
make -j$(nproc)
sudo make install
````
Increase the `usbfs_memory` to 512:
```bash
echo 'options usbcore usbfs_memory_mb=512' | sudo tee /etc/modprobe.d/usbfs_memory.conf
sudo reboot
```
To test if the installation was successful, connect the camera and run:
```bash
realsense-viewer
```

To install the wrapper ROS2:
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-realsense2-description
````
The launch file with default parameters:
```bash
ros2 launch realsense2_camera rs_launch.py \
  rgb_camera.color_profile:=640x480x15 \
  depth_module.depth_profile:=640x360x15 \
  pointcloud.enable:=false
```
When camera is on **raspberrypi**, the installation instructions are equivalent, but you have to:
- add a patch to it kernel (after step `cd librealsense`):
  ````bash
  cd ~/software/librealsense
  sudo ./scripts/setup_udev_rules.sh
  sudo ./scripts/patch-realsense-ubuntu-lts-hwe.sh
  ````
- Increase the `usbfs_memory` to 512:
  - open the file:
    ```bash
    sudo nano /boot/firmware/cmdline.txt
    ````
  - add at the end of line:
    ````bash
    usbcore.usbfs_memory_mb=512
    ````
  - reboot the raspberrypi
- change to a more stable firmware version for D435/i: version
`5.12.15.50`. This can be done in the "Device" tab -> "Update Firmware".

- Use the launch file with custom parameters:
  ```bash
  ros2 launch realsense2_camera rs_launch.py \
    initial_reset:=true \
    enable_color:=true \
    enable_depth:=true \
    align_depth:=true \
    enable_sync:=false \
    pointcloud.enable:=false \
    depth_module.depth_profile:=640x360x15 \
    rgb_camera.color_profile:=640x480x30 \
    enable_infra1:=false enable_infra2:=false
  ```

# Install Orbbec DaBai

This camera could be installed on:
- PC Ubuntu 22 with ROS2 Humble
- Raspberrypi4 Ubuntu 22 with ROS2 Humble

The installation process is very simple in both cases.

## PC Ubuntu 22 with ROS2 Humble

Install dependencies:
```bash
sudo apt install libgflags-dev nlohmann-json3-dev libgoogle-glog-dev \
     ros-humble-image-transport ros-humble-camera-info-manager \
     ros-humble-image-publisher libusb-1.0-0-dev libeigen3-dev
```
Clone the ROS2 package in your workspace:
```bash
cd ~/ROS2_rUBot_mecanum_ws/src
git clone --branch main https://github.com/orbbec/OrbbecSDK_ROS2.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```
Define the UDEV rules:
```bash
cd src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo bash install_udev_rules.sh
sudo udevadm control --reload-rules && sudo udevadm trigger
```
Launch:
```bash
ros2 launch orbbec_camera dabai.launch.py
```
Or with custom parameters:
```bash
ros2 launch orbbec_camera dabai.launch.py \
  color_width:=640  color_height:=480  color_fps:=15 \
  depth_width:=640  depth_height:=480  depth_fps:=15 \
  enable_point_cloud:=false
```

Best cameras:
- https://www.orbbec.com/products/tof-camera/femto-bolt/
- https://www.orbbec.com/products/tof-camera/femto-mega/

## Raspberrypi4 Ubuntu 22 with ROS2 Humble

Download the SDK driver for ARM64 from Orbbec site: 
- https://github.com/orbbec/OrbbecSDK/releases

- Copy the contents of SDK driver on home custom folder:
```bash
mkdir -p ~/orbbec_sdk
cp -r OrbbecViewer_v1.10.27_202509260950_arm64_release/* ~/orbbec_sdk/
````
- Execute the viewer to test the camera:
```bash
cd ~/orbbec_sdk
./OrbbecViewer
````
- Create udev rules (to run without sudo):
```bash
cd ~/orbbec_sdk/script
chmod +x install_udev_rules.sh
sudo ./install_udev_rules.sh
```

Install from source the ROS2 wrapper: https://github.com/orbbec/ros2_orbbec_camera.git
- Install ROS2 wrapper as in PC Ubuntu 22 section and launch with:
```bash
mkdir -p ~/ros2_orbbec_ws/src
cd ~/ros2_orbbec_ws/src
git clone https://github.com/orbbec/ros2_orbbec_camera.git
cd ..
colcon build --symlink-install
source install/setup.bash
```
- Launch with:
```bash
ros2 launch ros2_orbbec_camera dabai_a.launch.py
ros2 launch orbbec_camera gemini2.launch.py
ros2 launch orbbec_camera astra2.launch.py
```

# Program test
To test if the camera is working properly, you can:
- Start the camera:
  ```bash
  ros2 launch orbbec_camera dabai.launch.py
  ```
- review the topics:
  ```bash
  ros2 topic list
  ```
- review the info of image topics:
  ```bash
  ros2 topic info /camera/color/image_raw/compressed
  ros2 topic info /camera/depth/image_rect_raw/compressedDepth
  ```
- review the hz and bw of image topics:
  ```bash
  ros2 run topic_tools hz /camera/color/image_raw/compressed
  ros2 run topic_tools bw /camera/color/image_raw/compressed
  ros2 run topic_tools hz /camera/depth/image_rect_raw/compressedDepth
  ros2 run topic_tools bw /camera/depth/image_rect_raw/compressedDepth
  ```
- Create a simple Python node that subscribes to the topics:
    - /camera/color/image_raw/compressed
    - /camera/depth/image_rect_raw/compressedDepth
- Execute this node:
  ```bash
  chmod +x compressed_image_subscriber.py
  ./compressed_image_subscriber.py
  ```
- Use `rqt_image_view` to visualize the topics:
  ```bash
  ros2 run rqt_image_view rqt_image_view
  ``` 
  - Select topic: `/camera/color/image_raw` → Transport: `compressed`

  - Select topic: `/camera/depth/image_rect_raw` → Transport: `compressedDepth`
- Verify the clocks are sync:
  ```bash
  timedatectl
  ```