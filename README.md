## The implementation is currently under preparation and will be released upon acceptance.

# Multicam-VINS

**Videos:**

## 1. Prerequisites
1.1 **ROS and Ubuntu**
- Ubuntu 20.04
- ROS Noetic: [ROS Installation](https://wiki.ros.org/noetic/Installation/Ubuntu)
- Additional ROS pacakge
```
sudo apt-get install ros-YOUR_DISTRO-cv-bridge ros-YOUR_DISTRO-tf ros-YOUR_DISTRO-message-filters ros-YOUR_DISTRO-image-transport
```

1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html), remember to **make install**.

1.3. **Sophus**
```
git clone https://github.com/strasdat/Sophus.git
cd Sophus
git checkout 1.22.10
mkdir build && cd build
cmake .. -DSOPHUS_USE_BASIC_LOGGING=ON
make
sudo make install
```

## 2. Build Multicam_VINS on ROS1
Clone the repository and catkin_make:
```
    cd ~/Multicam_VINS_IRL/src
    git clone https://github.com/DuyTPresent/Multicam_VINS_IRL
    cd ../
    catkin_make -j4
    source devel/setup.bash
```

## 3. Run Multicam_VINS
```
    cd ~/Multicam_VINS_IRL
    roslaunch multicam_vins Multicam_VINS.launch 
    rosbag play <Your path to dataset>/<Your bag file>.bag 
```
## 4. Datasets
4.1 Download the dataset at: 

## 5. Calibration Camera and IMU
5.1 Calibration IMU

5.2 Calibration Camera 

5.3 Calibration IMU with Camera

## 6. Acknowledgements
We use [ceres solver](http://ceres-solver.org/) for non-linear optimization and a generic [camera model](https://github.com/hengli/camodocal).
