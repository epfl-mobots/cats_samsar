<!-- ![CATS2 logo](cats-bleu.jpg?raw=true) -->

# CATS2

Released under LGPL3, see [license.txt](license.txt) for details.

CATS2 goes for **C**ontrol **A**nd **T**racking **S**oftware.
It serves to monitor animal-robot experiments and to control robots in closed-loop.
CATS2 is built within the frame of ASSISIbf project (http://assisi-project.eu).

## Prerequisites

CATS2 was tested with Ubuntu 14.04 and Ubuntu 16.04.

CATS2 depends on the following packages
* gcc 4.9 or newer
* SDL2
* CMake 3.2 or newer
* Qt5.6 or newer
* gstreamer-0.10
* qt-gstreamer-0.10.3
* OpenCV 3.x (with dependecies)
* Boost
* ZeroMQ
* protobuf
* [dashel](https://github.com/aseba-community/dashel)
* [enki](https://github.com/enki-community/enki)
* [aseba](https://github.com/gribovskiy/aseba)

## Compile CATS2 under Ubuntu 14.04

### Install SDL2
    sudo apt-get install libsdl2-2.0-0 libsdl2-dbg libsdl2-dev 

### Install Boost
    sudo apt-get install libboost-dev

### Install ZeroMQ
Check here 
http://zeromq.org/intro:get-the-software and here
https://gist.github.com/cdjhlee/b8e3c927a01b0948b42d
for more details

### Additional dependencies for Ubuntu 16.04
    sudo apt-get install libboost-all-dev libprotoc-dev libzmqpp-dev libeigen3-dev

### Install gcc4.9 or newer 

    sudo add-apt-repository ppa:ubuntu-toolchain-r/test
    sudo apt-get update
    sudo apt-get install g++-4.9

Setup the switch between the new gcc4.9 and the previous version installed if
necessary (https://askubuntu.com/questions/26498/choose-gcc-and-g-version)

### Install Qt5.6 or newer 
Can be downloaded here https://info.qt.io/download-qt-for-application-development but the registration with Qt is needed

### Compile CMake or newer 

    sudo apt install ncurses* (required by cmake to create ccmake)
    wget https://cmake.org/files/v3.8/cmake-3.8.0.tar.gz
    tar -xvf cmake-3.8.0.tar.gz
    cd cmake-3.8.0/
    ./bootstrap --qt-gui
    make -j24
    make install

### Install gstreamer-0.10 and qt-gstreamer-0.10.3 
Use this link https://gstreamer.freedesktop.org/src/qt-gstreamer/ to download the qt-gstreamer-0.10.3 source files. 
When running cmake add -DQT_VERSION=5 flag

### Compile OpenCV with dependecies

Please use OpenCV 3.x to compile CATS without problems. Version 3.2.0 is suggested for both the opencv and opencv_contrib packages.

    git clone https://github.com/opencv/opencv_contrib.git
    git clone https://github.com/opencv/opencv.git
    cd opencv
    mkdir build
    cd build
    cmake .. -DOPENCV_EXTRA_MODULES_PATH="path-to-opencv-contrib"
    make -j24
    make install

### Install or compile from sources dashel, enki and aseba

### Compile CATS2

    git clone https://github.com/gribovskiy/CATS2.git
    cd CATS2
    git submodule init
    git submodule update
    mkdir build
    cd build
    cmake ..
    make -j24

## Run CATS2

This line will launch CATS2 with the video4linux drivers and input device "1":

    ./cats -mc v4l 1 -c <path-to-configuration-file>

Other than **v4l** you can use **vf** (to use a video file) or **if** (to use a still image),
in this case you need to provide the path to the corresponding file.

If you don't have any camera connected, but you would like to have a look on the
user interface of CATS2, then you can try the following command:

    ./cats -bc if ../../../config/camera-calibration/epfl-setup-180/cats2-180-angle-below-camera.png -mc if ../../../config/setup/epfl-two-rooms.png -c ../../../config/cats2-epfl-setup.xml

## Configuration files

Configuration files store parameters used by submodules of CATS2, such as the
camera calibration files, parameters of tracking routines, or parameters of the
robots' controllers. Examples of the configuration files can be found in
[config](config).
