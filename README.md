# Download Instructions

**Easy Manipulation Deployment** (EMD) package revolves arounds several dependencies which includes the **Moveit2**, **Pointcloud** (PCL) and **Flexible Collision** (FCL) libraries.   

Steps listed below provides a **step-by-step installation** of the aforementioned libraries.

## Installing libraries for EMD
--------------------
## #1 Moveit2 Library

### Installing Moveit2 Library
```bash
# (Open a new terminal window)

mkdir -p ~/moveit2_ws/src && cd ~/moveit2_ws
curl --output moveit2.repos https://raw.githubusercontent.com/ros-planning/moveit2/main/moveit2.repos
vcs import src < moveit2.repos
cd ~/moveit2_ws/src/moveit2/ && git checkout 2499a72f7388a371905eaef72685fcfaae04335a && cd ~/moveit2_ws
source /opt/ros/foxy/setup.bash
rosdep install --from-paths src --ignore-src -yr --rosdistro "${ROS_DISTRO}"
colcon build && source install/setup.bash
```
----------------
## #2 PCL library

### Installing PCL Library
```bash
# (Open a new terminal tab)
cd $HOME
git clone https://github.com/PointCloudLibrary/pcl.git && cd pcl && git reset --hard ea4d0b62d184349dbf4c245d9e79c71d070ff9fe
cd pcl && mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j4 && sudo make -j4 install
```
----------------
## #3 FCL library (Has dependencies to be installed)

Libccd and Eigen are dependencies required for the FCL library. Steps below are for those respective libraries.

### FCL library dependencies ###

### Libccd
```bash
# (Open a new terminal tab)
cd $HOME
git clone https://github.com/danfis/libccd.git && cd ~/libccd && git reset --hard 7931e764a19ef6b21b443376c699bbc9c6d4fba8
mkdir build && cd build
cmake -G "Unix Makefiles" ..
sudo make && sudo make install
```

### Eigen
```bash
# (Open a new terminal tab)

sudo apt-get install -y libeigen3-dev
```
### Installing FCL Library
```bash
(Open a new terminal tab)

git clone https://github.com/flexible-collision-library/fcl.git && cd ~/fcl && git reset --hard f4018b4e451f7850081a8c254dc4c0779bd9061f
mkdir build && cd build
cmake ..
sudo make && sudo make install
```
------------------
### Step below is only required for **slides** ##
### If you are reading this on github,  **skip** this next portion ##

Installing EMD package.
```bash
# (Open a new terminal window)

source /opt/ros/foxy/setup.bash
mkdir ~/full_emd_ws/ && cd ~/full_emd_ws/
git clone https://github.com/Macloserpants/emd_training.git --depth 1
colcon build && source install/setup.bash
```
-------------

**IMPORTANT NOTE**

Path to `usr/local/lib` may not be in user’s path, resulting in problems with dependencies when launching grasp_planner.launch.py.
Steps below are to resolve those issues.

```bash
# (Open a new terminal tab)

# (Double check if /usr/local/lib is in $PATH, If is not present in $PATH, follow the steps below to enable it into path)
echo $PATH

cd /etc/ld.so.conf.d
nano 99local.conf

# *Key in "/usr/local/lib" in conf file*
ldconfig

# (To check if /usr/local/lib is in path)
echo $PATH
```
