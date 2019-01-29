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
```
Once the graspit is installed using the default instructions given on the 
[graspit homepage](https://graspit-simulator.github.io/build/html/installation_linux.html) (the instructions are written above)
