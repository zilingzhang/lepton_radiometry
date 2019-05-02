# lepton_radiometry
simple python ROS wrapper for flir lepton 3.5 radiometry, publish raw sensor data (16bit per pixel, resolution 0.01K, start from absolute zero) and gray scale image (grayscale range clamp by max and min temp in a frame)
To run lepton driver and ROS wrapper only:
~~~~
rosrun lepton_radiometry uvc-radiometry.py
~~~~

## Udev rule that allows ROS accesss to the thermal camera	
~~~~
sudo sh -c "echo 'SUBSYSTEMS==\"usb\", ATTRS{idVendor}==\"1e4e\", ATTRS{idProduct}==\"0100\", SYMLINK+=\"pt1\", GROUP=\"usb\", MODE=\"666\"' > /etc/udev/rules.d/99-pt1.rules"
~~~~

# Pre-requisites for Kinect 1 integration

## ROS kinetic and turtlebot dependencies
~~~~
sudo apt-get install ros-kinetic-turtlebot*
~~~~
## kinect 1 dependencies

### 0. Pre-requsites
~~~~
sudo apt-get install git build-essential python libusb-1.0-0-dev freeglut3-dev openjdk-8-jdk
sudo apt-get install doxygen graphviz mono-complete
~~~~

### 1. Install OpenNI
~~~~
cd ~/
mkdir kinect
cd ~/kinect
git clone https://github.com/OpenNI/OpenNI.git
cd OpenNI
git checkout Unstable-1.5.4.0
cd Platform/Linux/CreateRedist
chmod +x RedistMaker
./RedistMaker
cd ../Redist/OpenNI-Bin-Dev-Linux-[xxx]  
# (where [xxx] is your architecture and this particular OpenNI release. In my case, this is x64)
sudo ./install.sh
~~~~

As is mentioned in the referenced guide, the Kinect sensor won't work out of the box with OpenNI. Avin2 has created a module that allows the Kinect Controller to work with OpenNI.
	
~~~~
cd ~/kinect
git clone https://github.com/avin2/SensorKinect
cd SensorKinect
cd Platform/Linux/CreateRedist
chmod +x RedistMaker
./RedistMaker
cd ../Redist/Sensor-Bin-Linux-[xxx] 
(where [xxx] is your architecture and this particular OpenNI release. In my case this was x64-v5.1.2.1)

chmod +x install.sh
sudo ./install.sh
~~~~
EDIT: Be careful not to run this more than once, otherwise you might run into problems. If you do need to change something, you can go to the ./install and run
~~~~
sudo ./install.sh -u
~~~~
which will uninstall this. Don't just go taking the axe to it by deleting everything.

### 2. Install Kinetic OpenNI

There was one guide I found that did the following step a different way, linked here. This may have worked for Indigo, but it didn't work for my case. The following worked fine.
~~~~
sudo apt-get install ros-kinetic-openni*
~~~~

### 3. Install NITE

NITE is now proprietary, but there are still links to older versions around the internet. A copy at Github https://github.com/zilingzhang/NITE-Bin-Dev-Linux-v1.5.2.23. Untar/zip this in the /kinect folder:
	
~~~~
cd ~/kinect 
NITE-Bin-Dev-Linux-x64-v1.5.2.23
sudo ./install.sh
~~~~

### 4. Install openni_tracker package

This is the ROS package that actually tracks your skeleton and broadcasts it (NOT publishes) as a series of transforms.
~~~~
cd ~/catkin_ws/src
git clone https://github.com/ros-drivers/openni_tracker.git
~~~~

### 5. Remake Catkin Workspace
~~~~
cd ~/catkin_ws
catkin_make
catkin_make install
~~~~
Now you're done!

### 6. To test run

If you don't already know, ctrl + alt + t gives you a new terminal windows. ctrl + shift + t gives you a new tab in that terminal window. ROS terminals tend to get out of hand and I absolutely detest the way that the alt + tab switches between windows of different types in Ubuntu. Labelling your terminals also goes a long way to getting around nicely. Granted, if you're a super n00b, please note that for each of the following lines, you're going to want to have a new terminal window/tab.

~~~~
roscore
roslaunch openni_launch openni.launch
rosrun openni_tracker openni_tracker
~~~~

# Installation
cd to your catkin workspace
~~~~
cd ~/catkin_ws/src
git clone https://github.com/zilingzhang/lepton_radiometry/
cd ~/catkin_ws
catkin_make
~~~~

# Running
This launch file will start 4 packages: turtlebot bringup_minimal, openni_tracker, lepton_radiometry, rviz
~~~~
roslaunch lepton_radiometry start.launch
~~~~

