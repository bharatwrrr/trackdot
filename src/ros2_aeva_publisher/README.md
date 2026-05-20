# Aeva ROS2 API Publisher #

### Overview of the program ###

This program demonstrates how to use ROS2 build of the Aeva API to publish ROS2 messages from the sensor.

### Program dependencies ###

- Aeva API Debian package `Atlas_API_<VERSION>_<ARCH>.deb`  
- ROS2 (recommended distribution ROS2 Foxy Fitzroy)

### Steps to build and run the program ###

* Install the provided Aeva API Debian package on your system.
```bash
sudo dpkg -i Atlas_API_<VERSION>_<ARCH>.deb
```

* __IMPORTANT:__ Please note that this example uses the ROS2 build of the Aeva API and not the standard Aeva API.



* Navigate to the `ros2_examples` directory (which is our ROS2 workspace directory) and run the following command.
```bash
colcon build --packages-up-to ros2_aeva_publisher
```
This builds the ROS2 package with the name `ros2_aeva_publisher`.

* Source the `setup.bash` file from the `install` directory.
```bash
source install/setup.bash
```

* Run the following command from the workspace directory. The program takes the sensor IP address and IDs as the inputs.\n* Note that the SENSOR_NAME shouldn't begin with a number, since it's used to derive the message topic. ROS2 topics can't start with a number.
```bash
LD_LIBRARY_PATH=/opt/aeva/atlas-api/ros2/library:$LD_LIBRARY_PATH ros2 run ros2_aeva_publisher ros2_aeva_publisher SENSOR_IP_0 SENSOR_NAME_0 [... SENSOR_IP_N SENSOR_NAME_N]
```
