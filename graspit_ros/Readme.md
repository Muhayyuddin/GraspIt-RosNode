# Installation Instructions
To install the graspit_ros package first we need to install the graspit by following the instructions on the 
[graspit homepage](https://graspit-simulator.github.io/build/html/installation_linux.html). For simplicity the same instructions 
are given below. 

**Dependencies**
```
sudo apt-get install libqt4-dev
sudo apt-get install libqt4-opengl-dev
sudo apt-get install libqt4-sql-psql
sudo apt-get install libcoin80-dev
sudo apt-get install libsoqt4-dev
sudo apt-get install libblas-dev
sudo apt-get install liblapack-dev
sudo apt-get install libqhull-dev
sudo apt-get install libeigen3-dev
```
**Graspit Installation**
```
git clone git@github.com:graspit-simulator/graspit.git
cd graspit
export GRASPIT=$PWD
mkdir build
cd build
cmake ..
make -j5
sudo make install 
```
Once the graspit is installed using the default instructions given on the 
[graspit homepage](https://graspit-simulator.github.io/build/html/installation_linux.html) (the instructions are written above)
**Setting Up Environment Variable** 
We can add the graspit environment variable to the *bashrc* file using the following command
```
echo "export GRASPIT=<directory that contains model and world folder>" >> ~/.bashrc    
source ~/.bashrc
```
**Example** 
```
echo "export GRASPIT=/home/muhayyuddin/Software/Graspit/graspit">>~/.bashrc
source ~/.bashrc
```

or to use the demos provided in the current package:
```
echo "export GRASPIT=/home/muhayyuddin/Software/GraspIt-RosNode/graspit_ros/demos">>~/.bashrc
source ~/.bashrc
```



It will permanently setup the graspit environment variable.
## Compiling the Graspit_ros Node
We have all set to compile the graspit_ros node. Below are the instructions to compile it.
```
mkdir -p graspit_ws/src
cd graspit_ws
catkin_make
source devel/setup.bash
cd src
git clone https://github.com/Muhayyuddin/GraspIt-RosNode.git
cd ..
catkin_make
```

## Launching the demo
You can edit the demo.launch file to set your defaults world and output paths, or write them when calling roslaunch:
```
roslaunch graspit_ros demo.launch world_file_path="/your/path/GraspIt-RosNode/graspit_ros/models/worlds/Kitchen_pal_gripper_cup_simple.xml" keepMaxPlanningResults=5 output_path:=/your/path/GraspIt-RosNode/graspit_ros/RESULTS"
```

