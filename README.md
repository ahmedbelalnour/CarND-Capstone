## Individual submission
E-mail address: ahmedbelalnour@gmail.com

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).
This project build an autonomous car that is tested in Carla simulator built by Udacity.
The car follows the way points of the road to keep it in the same center lane.
The car should follow the traffic rules as it should stop when the traffic light is red and begin to move
smoothly when the traffic light is green.
The acceleration and jerk should not exceed 10 m/s2, and 10 m/s3 respectively.
The code is implemented in Python2.
The project consists of many ROS nodes so the project is built using ROS operating system. 
I tried to use the [tensor flow detection API coco mobilenet by NVIDIA](https://github.com/tensorflow/models/tree/master/research/object_detection) that contains a pre trained model and can detect objects including
the traffic signs.
My GPU capabilities is not so great and when I use the workspace the results are not good in real time either.
Thats why I decided to use computer vision techniques to detect and classify the traffic signs.
Now I will cover the [rubric points](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/project) and describe how I addressed each one of them in details.

### Code correctness
The code runs correctly using the launch file `launch/styx.launch`.
No compilation errors exist.

### Project size
The repository size is more less than 2GB allowed for the project sbumission.

### Smoothly follows the way points
The car smoothly follows the way points by implementing the PID controller node which is the exact copy of 
the code in the [PID controller project](https://github.com/ahmedbelalnour/CarND-PID-Control-Project) after translating the CPP code into Python code.
I used the following parameters:
 - Kp: 0.219
 - Ki: 0.002
 - Kd: 13.94
The implementation of the PID controller can be found in `ros\src\twist_controller\pid.py`
The values are passed to the PID controller module in `ros\src\twist_controller\twist_controller.py`

### Acceleration and Jerk does not exceed 10
The acceleration and jerk should not exceed 10 m/s2, and 10 m/s3 respectively.
That is done by setting the maximum acceleration of the car to 0.2 and the brake is done when applying 700 N*m which can be found in `ros\src\twist_controller\twist_controller.py`.

### Car maximum speed
The car follows the maximum speed set in the software inside `ros\src\waypoint_loader\waypoint_loader.py`.
The speed is given to the software in KM/H but it is displayed in the simulator in MPH.

### Stops at traffic light
The software modules responsible for this task are: `ros\src\tl_detector\tl_detector.py`, and `ros\src\tl_detector\light_classification\tl_classifier.py`.
They give the waypoint updater node the location and the state of the nearest traffic sign in the forward direction.
If the state of the traffic sign can not be detected (Unkown state), then a recovery mechanism is to assume it as a red state.
To save the resources and the power of the ECU, the system uses the detection module only before the traffic sign locations by 100 meters.
The software classifies the traffic signs in the detected image by comparing the red, yellow and green pixels inside the traffic sign portion of the image.

### Stop and restart the PID controller depending on the state of `dbw_enabled`
The software does not run if the `manual` box is checked.
This feature is granted by creating and following the way points in the waypoint updater node if and only if the `dbw_enabled` signal is on.
The same is true for the PID controller which is reset when the `dbw_enabled` signal is not active inside the `twist_controller` node.

### Publish throttle, steering and brake commands at 50HZ
The `dbw_node` is responsible for publishing the throttle, steering and brake commads for the car at 50HZ by executing this ROS node 50 times per second.
The values of the throttle, steering and brake are controlled by the controller in `twist_controller` node.

Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
