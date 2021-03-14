# Download Instructions

Easy Manipulation Deployment (EMD) package revolves arounds several dependencies which includes the Moveit2, Pointcloud (PCL) and Flexible Collision (FCL) libraries.   

Steps listed below provides a step-by-step installation of the aforementioned libraries.

## Installing libraries for EMD
--------------------
## #1 Moveit2 Library

### Installing Moveit2 Library
```
(Open a new terminal window)

mkdir -p ~/moveit2_ws/src
cd ~/moveit2_ws

curl --output moveit2.repos \
  https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos

vcs import src < moveit2.repos 
cd src/moveit2/ && git checkout 2499a72f7388a371905eaef72685fcfaae04335a && cd ~/moveit2_ws
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"
colcon build
source install/setup.bash
```
----------------
## #2 PCL library

### Installing PCL Library
```
(Open a new terminal tab)

git clone https://github.com/PointCloudLibrary/pcl.git
cd pcl && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4
sudo make -j4 install
```
----------------
## #3 FCL library (Has dependencies to be installed)

Libccd and Eigen are dependencies required for the FCL library. Steps below are for those respective libraries.

### FCL library dependencies ###

### Libccd
```
(Open a new terminal tab)

git clone https://github.com/danfis/libccd.git 
cd ~/libccd
mkdir build && cd build
cmake -G "Unix Makefiles" ..
sudo make  
sudo make install 
```
### Eigen
```
(Open a new terminal tab)

sudo apt install libeigen3-dev
```
### Installing FCL Library
```
(Open a new terminal tab)

git clone https://github.com/flexible-collision-library/fcl.git
cd ~/fcl
mkdir build && cd build
cmake ..
sudo make 
sudo make install
```
------------------
### Step below is only required for slides ## 
### If you are reading this on github skip this next portion ##

Installing EMD package
```
(Open a new terminal window)

source /opt/ros/foxy/setup.bash
mkdir ~/full_emd_ws/
cd ~/full_emd_ws/
git clone https://github.com/Macloserpants/emd_training.git
colcon build
source install/setup.bash
```
-------------

** IMPORTANT NOTE **

Path to usr/local/lib may not be in user’s path, resulting in problems with dependencies when launching grasp_planner.launch.py.
Steps below are to resolve those issues. 

(Open a new terminal tab)

echo $PATH (Double check if /usr/local/lib is in $PATH, If is not present in $PATH, follow the steps below to enable it into path)

```
cd /etc/ld.so.conf.d
nano 99local.conf
*Key in "/usr/local/lib" in conf file* 
ldconfig
echo $PATH (to check if /usr/local/lib is in path)
```
