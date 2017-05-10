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

- Export `ViconSDK_ROOT` variable in `~/.bash_aliases`
```
$ export ViconSDK_ROOT=location/of/the/ViconSDK/libraries
```

- Configure and make

```
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
- Open antoher terminal and then in `YarpViconBridge/build`
```
$ ./YarpViconBridge --hostname 192.168.100.153
```
