# vl53l0x_driver

## Simple vl53l0x range sensor ROS driver

More information on the Medium page: https://medium.com/robotics-weekends/ros-node-for-vl53l0x-range-sensor-56edbe5d9172

Sensor measurement data is converted into simple binary protocol and is sent to the ROS node via USB connection.  \


## Arduino and sensor protocol
Every message, sent from Arduino, starts with 0xAD byte, followed by message type byte, and ends with \r\n (0x0D, 0x0A) sequence. There are 4 message types:

 * 0xEF — initialization failed;
 * 0xE0 — initialization was successful, no errors;
 * 0x1D — range message, followed by two bytes of measured range;
 * 0xEA — out of range.

![Serial message format diagram](https://miro.medium.com/max/700/1*Dudnb_vWuisHHcNYTOh8iQ.png)

The firmware for Arduino can be found in **arduino/vl53l0x_ros** directory. The firmware requires **Adafruit_VL53L0X** library.

The wiring diagram is very simple - just connect the sensor to the Arduino’s I2C bus:

![Sensor schema](https://miro.medium.com/max/700/1*-2CR795WKqKxqfK0PiRzng.png)

1. Load the arduino program onto the Arduino (e.g. Arduino IDE). Remember to install he library used (Adafruit_VL53L0X)


 ## Docker
Make sure that Docker is installed following the official [Docker Installation Instructions](https://docs.docker.com/engine/install/). It is advisable to also follow the [Linux Post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/).


1. Pull the docker image:
```bash
./docker-pull.sh
```

2. Run the docker container specifiying a shared folder path between the host and the container:
```bash
./docker-run.sh /path/to/shared/folder
```

if you are already in your /vl53l0x_driver folder just type:
```bash
./docker-run.sh .
```

3. To open more terminals in the running container:

```bash
docker exec -it vl53l0x_driver bash
```

4. To restart an stopped container after a PC reboot (then do exec):

```bash
docker start vl53l0x_driver
```

## Building the image (if needed)
Build image
1. cd to the folder where the dockerfile is located (in this repos root folder)
```bash
docker build . -t lucasmogsan/vl53l0x_driver:latest
```
2. The following is optional if you want to push to dockerhub:
```bash
docker tag lucasmogsan/vl53l0x_driver:latest docker.io/lucasmogsan/vl53l0x_driver:latest
```

```bash
docker push docker.io/lucasmogsan/vl53l0x_driver:latest
```


## ROS node
**vl53l0x_node** ROS node publishes sensor_msgs::Range message into **range_data** topic. Node has several optional configuration parameters:
 * port - which usb port to use. Default: /dev/ttyUSB0
 * baud_rate - transmission speed between sensor and node. Default: 57600
 * frame_id - tf frame ID of the sensor. Default: ir_range
 * firmware_version - reserved. Not used as for now.


1. When in the container remember to make and soruce
```bash
catkin_make
source devel/setup.bash
```

2. Test both gui and connection works by:
```bash
roslaunch vl53l0x_driver test.launch 
```

3. It could happen that your port name differs from mine (for example /dev/ttyUSB0 or /dev/ttyACM1). In this case you have to change port param (in the launch-file) to the port name on your computer.





## Links

 * More information on the Medium page - https://medium.com/robotics-weekends/ros-node-for-vl53l0x-range-sensor-56edbe5d9172
 * Case 3D STL files — https://www.thingiverse.com/thing:4324054
 * VL53L0X datasheet — https://www.st.com/resource/en/datasheet/vl53l0x.pdf
 * Range message documentation — https://docs.ros.org/melodic/api/sensor_msgs/html/msg/Range.html
