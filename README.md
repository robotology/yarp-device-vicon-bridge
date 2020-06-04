# `yarp-vicon-bridge`

## Maintainers
This repository is maintained by:

| | |
|:---:|:---:|
| [<img src="https://github.com/Nicogene.png" width="40">](https://github.com/niNicogenecogene) | [@Nicogene](https://github.com/Nicogene) |

## Description
The `yarp-vicon-bridge` is a [YARP device](https://www.yarp.it/note_devices.html) working as interface between a Vicon setup and YARP. 

Using this device, it is possible to retrieve the Vicon markers information in the form of transformation matrices, i.e. a 3D vector for translation and a quaternion for the rotation. 

## Dependencies

- Get YARP from the [official GitHub repository](https://github.com/robotology/yarp.git).
- Get ViconDataStreamSDK from the [official Vicon website](https://www.vicon.com/products/software/datastream-sdk).

## Installation 

### Linux
- Setup the `~/.bash_aliases`:
  ```bash
  export ViconSDK_DIR=location/of/the/ViconSDK/libraries
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ViconSDK_DIR}
  ```
- Configure and make:
  ```bash
  $ git clone https://github.com/robotology/yarp-vicon-bridge.git
  $ cd yarp-vicon-bridge
  $ mkdir build && cd build
  $ cd build 
  $ ccmake ..
  $ make install
  ```
- Check if the device has been installed correctly:
  ```bash
  yarpdev --list | grep vicon
  ```

### Windows
- Setup the environment variables:
  ```bash
    setx ViconSDK_DIR "location/of/the/ViconSDK/libraries"
    setx PATH=%PATH%;ViconSDK_DIR
  ```
  :bulb: You can also set the environment variables using a GUI called [Rapid Environment Editor](https://www.rapidee.com/en/about).
  
- Clone this repository:
  ```bash
  git clone https://github.com/robotology/yarp-vicon-bridge.git
  ```
- Configure and build the project using the [CMake GUI](https://cmake.org/runningcmake/) (:warning: set x64 for generator).
- Build the project either using Visual Studio or the command line:
  ```bash
  cmake --build . --config Release --target install
  ```
  
## How to run the `yarp-vicon-bridge`

:warning: Vicon Nexus software must be up and running for being able to use the `yarp-vicon-bridge`.

### Using the Vicon setup

- Run a `yarpserver` in a terminal.
- Open a second terminal and start a `transformServer` device:
  ```bash
  yarpdev --device transformServer --ROS::enable_ros_publisher 0 --ROS::enable_ros_subscriber 0
  ```
- Open a third terminal and start a `yarp-vicon-bridge` device:
  ```bash
  yarpdev --device vicon --hostname "192.168.10.2"
  ```
  :warning: The default `hostname` is `192.168.10.2` (see [Vicon Network Cards Settings](https://docs.vicon.com/display/Connect/Configuring+network+card+settings#Configuringnetworkcardsettings-IPAddresses)) but some network test might be needed to find out the port where the Vicon output is actually streamed.

### Using the Vicon setup and R1
- Make sure that `roscore` is running on `r1-base`.
- Make sure that `yarpserver --ros` is running on `r1-base`.
- Open a terminal and type: 
  ```bash
  $ rostopic echo \tf
  ```
- Open another terminal and start a `transformServer` device:
  ```bash
  $ yarpdev --from tfServer.ini
  ```
where `tfServer.ini` is:
  ```
  device  transformServer
  transforms_lifetime 0.500
  [ROS]
  enable_ros_publisher 1
  enable_ros_subscriber 0
  ```
- Open another terminal and start a `yarp-vicon-bridge` device:
  ```bash
  $ yarpdev --device vicon --hostname "192.168.10.2"
  ```
  :warning: The default `hostname` is `192.168.10.2` (see [Vicon Network Cards Settings](https://docs.vicon.com/display/Connect/Configuring+network+card+settings#Configuringnetworkcardsettings-IPAddresses)) but some network test might be needed to find out the port where the Vicon output is actually streamed.

