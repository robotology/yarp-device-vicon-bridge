# How to use YarpViconBridge
Installation on Linux
---------------------
- Getting Yarp
```
$ git clone https://github.com/robotology/yarp.git
$ git checkout devel
$ cd yarp 
$ mkdir build && cd build
$ ccmake ..
$ make
$ sudo make install
$ sudo ldconfig
```
- Get ViconDataStreamSDK from the [official  Vicon website](https://www.vicon.com/products/software/datastream-sdk)

⚠️ Use the `1.7.1` version of the SDK, the `1.8.0` has some issues.

- Setup the `~/.bash_aliases`
```
export ViconSDK_ROOT=location/of/the/ViconSDK/libraries
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${ViconSDK_ROOT}
```

- Configure and make

```
$ git clone https://github.com/robotology/yarp-vicon-bridge.git
$ cd yarp-vicon-bridge
$ mkdir build && cd build
$ cd build 
$ ccmake ..
$ make
```
System setup
------------
- Make sure the cameras are connected to the switch
- Make sure that the pc where **Nexus** is running has the port 192.168.10.1 (bottom network card) connected to the switch (blue cable), and the black
ethernet cable plugged in the top Network Card(check that the other side is connected to the *"r1 router"*).

Run YarpViconBridge with R1 setup
---------------------------------
- Make sure that `roscore` is running on `r1-base`
- Make sure that `yarpserver --ros` is running on `r1-base`
- Open a terminal and type: 
```
$ rostopic echo \tf
```
- Open another terminal and launch a `transformServer`, e.g.:
```
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
- Open another terminal and then in `YarpViconBridge/build`
```
$ yarpdev --device vicon --hostname 192.168.100.153
```
