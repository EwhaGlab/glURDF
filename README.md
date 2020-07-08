# glURDF
<img src="./img/demo_talos.gif" width="600">

glURDF is a graphical interface for visualizing robot model using URDF and OpenGL.
This repository is based on [learnopengl](https://learnopengl.com), extended to read [urdf](http://wiki.ros.org/urdf) files.
glURDF supports not only visualizing the robot model but also animating the robot motion with given robot configurations.

Currently, the robot description files given in this repository is from [fetch_ros](https://github.com/fetchrobotics/fetch_ros.git), and [talos-data](https://github.com/stack-of-tasks/talos-data.git).

*Tested on both **Ubuntu 16.04** and **18.04**.*

## Dependencies
glURDF depends on multiple packages and libraries. Please follow the procedure below to make sure you have installed all the dependencies.
- System tools
  ```sh
  sudo apt-get install g++ cmake git
  ```
- packages for graphical interfaces
  ```sh
  sudo apt-get install libsoil-dev libglm-dev libassimp-dev libglew-dev libglfw3-dev libxinerama-dev libxcursor-dev libxi-dev libboost-all-dev
  ```
- packages for reading urdf files

  In order to read urdf files, one has to install the urdfdom package which can come either along ROS library or be installed as a standalone library. Either of them works fine. Below is the procedure for the second option. 

  [urdfdom](https://github.com/ros/urdfdom) depends on both [console_bridge](https://github.com/ros/console_bridge) and [urdfdom_headers](https://github.com/ros/urdfdom_headers). Follow all the command lines below to install all three packages.

  ```sh
  git clone git://github.com/ros/console_bridge.git && cd console_bridge
  mkdir build && cd build
  cmake ..
  make
  sudo make install
  ```
  ```sh
  git clone git://github.com/ros/urdfdom_headers && cd urdfdom_headers
  mkdir build && cd build
  cmake ..
  make
  sudo make install
  ```
  ```sh
  git clone git://github.com/ros/urdfdom && cd urdfdom
  mkdir build && cd build
  cmake ..
  make
  sudo make install
  ```

## Setup
To compile, it is recommended to create a separate build directory:
```sh
mkdir build && cd build
cmake ..
make 
```

## Run
To visualize the robot, run the script below:
```sh
cd build/bin
./glURDF
```
To run the demo of the talos robot motion, run the script below:
```sh
cd build/bin
./demo_talos
```

Some keyboard inputs are available:
- **WASD**: camera movement
- **Q**: shadow ON/OFF
- **space bar**: (only for the demo)replay the motion
