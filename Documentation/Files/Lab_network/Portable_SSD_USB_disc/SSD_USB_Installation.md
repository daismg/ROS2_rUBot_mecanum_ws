# SSD portable disc installation guide

We have installed:
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill

We need to install manually:
- Gazebo11
- wifi driver 

## Usefull packages installation

To install gazebo11 on Ubuntu 22.04, follow these steps:

```bash
sudo apt update
sudo apt install -y gazebo libgazebo-dev ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins ros-humble-gazebo-ros2-control
```
Also is important to install:
```bash
sudo apt install ros-humble-robot-state-publisher ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui ros-humble-xacro
```

## WiFi driver

The best option is to use a WiFi adapter compatible with Linux.

We have used: BrosTrend Adaptador WiFi Linux 1200Mbps, WiFi USB Linux (https://www.amazon.es/dp/B07FCNP2VL)


### Low-cost: TP-Link AC600 (RTL8811AU/RTL8821CU) Driver Installation on Ubuntu 22.04

An interesting low-cost alternative is TP-Link AC600.

This guide explains how to install the **TP-Link Archer T2U Nano AC600** Wi-Fi adapter driver on Ubuntu 22.04.  
The chipset is **Realtek RTL8811AU / RTL8821CU**, which is not supported by default.

### 1. Prepare the system

Make sure you have the required tools to compile DKMS drivers:

```bash
sudo apt update
sudo apt install -y build-essential dkms git linux-headers-$(uname -r)
```

### 2. Clone the correct repository

The following repository supports **RTL8811AU/RTL8821CU** chipsets:

```bash
cd ~
git clone https://github.com/morrownr/8821cu-20210916.git
cd 8821cu-20210916
```

### 3. Install the driver

Run the installation script:

```bash
sudo ./install-driver.sh
```

- If asked about editing driver options → answer **`N` (No)**.  
- Options can be edited later manually if needed.

### 4. Configure driver options (recommended)

To disable USB power saving and avoid disconnections, add:

```bash
echo "options 8821cu rtw_power_mgnt=0 rtw_enusbss=0 rtw_ips_mode=0" | \
  sudo tee /etc/modprobe.d/8821cu.conf
```

### 5. Load the module and restart services

```bash
sudo modprobe -r 8821cu || true
sudo modprobe 8821cu
sudo systemctl restart NetworkManager
```

If the interface does not appear, unplug and replug the Wi-Fi dongle.

### 6. Verify the interface

```bash
ip link             # should show 'wlan0' or 'wlx...' interface
nmcli dev status    # should show 'wifi' as available
nmcli dev wifi list # should list available Wi-Fi networks
```

### 7. Connect to a Wi-Fi network

You can connect via Network Manager GUI (top-right menu)  
or via terminal:

```bash
nmcli dev wifi connect "SSID_NAME" password "SSID_PASSWORD"
```

### 8. Check your IP address

```bash
hostname -I
ip a
```

Your Wi-Fi interface should have an IP like `192.168.1.xxx`.

### 9. Uninstall the driver (if needed)

If you want to remove the driver:

```bash
cd ~/8821cu-20210916
sudo ./remove-driver.sh
```


✅ With this, the TP-Link AC600 dongle should work correctly on Ubuntu 22.04 with full Wi-Fi connectivity.

## Install Docker

We have created a `install_docker.sh` to automatically:
- Updates the system package index.
- Installs required dependencies (ca-certificates, curl, gnupg).
- Adds Docker’s official GPG key.
- Sets up the Docker repository.
- Installs Docker Engine and plugins.
- Installs the standalone docker-compose binary.
- Verifies both installations.

To run the script:
```bash
chmod +x install_docker.sh
./install_docker.sh
````
After installation, you can verify Docker and Docker Compose are installed correctly by running:
```bash
docker --version
docker-compose --version
```

## Reset student user

Every year course is convenient to reset the user `student` to its initial state.
- Make executable the scripts:
  ````bash
  chmod 700 ~/Desktop/reset_student.sh
  chmod 700 ~/Desktop/reset_student.desktop
  ````
- Ensure the files are owned by the user biorob:
  ````bash
  chown biorob:biorob ~/Desktop/reset_student.sh ~/Desktop/reset_student.desktop
  ````
- Execute the script:
````bash
sudo ~/Desktop/reset_student.sh
````
> if PASSWORD="", you will have to type the new pass for user `student`
> If you prefer the launcher `reset_student.desktop`, you double-click and you will have to type your admin password.


## **Logout github student account**
 In order to remove git and github credentials from the `student` user, you can type:
````shell
git config --global --unset user.name 
git config --global --unset user.email 
git config --global --unset credential.helper 

rm -f ~/.git-credentials
git credential-cache exit 
gh auth logout -h github.com -y 
````
> This can be done with the instruction: `rm -f ~/.gitconfig ~/.git-credentials`
- Remember to 'Sign out' in VS Code: Settings → Accounts → Sign out from GitHub