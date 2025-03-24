# Tappy
tappy! The autonomous typing robot!

# Realsense Setup

Follow the steps described in the Realsense-ROS installation guide: [Realsense-ROS Guide](https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#installation-on-ubuntu). 

The example steps below are for installing on an Ubuntu 22.04 Linux install running on VMWare Fusion.

## Install the Intel® RealSense™ SDK 2.0
This SDK is needed to interface with the Realsense natively in Linux. The Realsense-ROS wrapper utilizes this SDK to be able to use the camera in ROS. 

## Installing the packages:
- Register the server's public key:
```
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
```

- Make sure apt HTTPS support is installed:
`sudo apt-get install apt-transport-https`

- Add the server to the list of repositories:
```
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
```

- Install the libraries (see section below if upgrading packages):  
  `sudo apt-get install librealsense2-dkms`  
  `sudo apt-get install librealsense2-utils`  
  The above two lines will deploy librealsense2 udev rules, build and activate kernel modules, runtime library and executable demos and tools.  

Reconnect the Intel RealSense depth camera and run: `realsense-viewer` to verify the installation.

Verify that the kernel is updated :    
`modinfo uvcvideo | grep "version:"` should include `realsense` string

## Install the Realsense-ROS Wrapper: 
This ROS package will be used to visualize and interact with Realsense camera streams in ROS. 

- Install the ros-specific realsense packages:
```
sudo apt install ros-humble-realsense2-*
```

## Passing Realsense into VM (USB Cameras with macOS Virtual Machines)
Since our team is working in a VM, it is vital that we pass the Realsense into the VM and bypassing the native machine. 

### **NOTE: Passing the Realsense through as a Virtual Camera (using the VMWare GUI) makes it unrecognizable to the Realsense SDK!**

This is because the Realsense is passed through as a Generic Virtual USB Camera type which is imcompatible with the Realsense firmware. 


It is possible to directly pass your camera through by editing the VMware preferences file in a terminal. **Shut down any VMs**, then do the following (assuming on a Mac):

```bash
cd ~/Library/Preferences/VMware\ Fusion
nano preferences
```

At the end of the file, add this line:
```
vusbcamera.passthrough = "TRUE"
```

Save and close the preferences file. 

Restart VMWare and try to launch the realsense in ROS!

## Launch Realsense-ROS Node
With **ros2 launch**:
```
ros2 launch realsense2_camera rs_launch.py
```