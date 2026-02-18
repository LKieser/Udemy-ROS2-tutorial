# ROS2 Basics Tutorial

## Overview

This code is all written as part of a tutorial on ROS2. The tutorial can be found here: [ROS 2 for Beginners](https://www.udemy.com/course/ros2-for-beginners/)

The goal of this tutorial, as described on the course website, is to teach ROS2 Nodes, Topics, Services, Parameters, Launch Files, etc.
This tutorial will be a step by step guide into building a complete ROS2 application in both Python and C++

Because this tutorial is in Python and C++, I will include the code for both in this `README` file and will denote that with either writing Python or C++ above it.


## Setup

First create a folder for the ROS2 workspace. Inside this folder create a `/src` directory. 
Then in this `/src` directory build a package:

#### Python
```bash
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```
#### C++
```bash
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclpy
```

Back in the ROS2 workspace folder build

#### Python 
```bash
colcon build --packages-select my_py_pkg
```
#### C++
```bash
colcon build --packages-select my_cpp_pkg
```

This command must only be run in the ROS2 workspace folder as it will create multiple directories the first time it is used and when it doesn't recognize those directories in the current directory.

## Building a Node

Nodes are the primary building block in ROS2 architecture. Each node is used for one distinct purpose. Nodes can then communicate with each other using various methods.

### Python

When a new node is created, it must be added to the [setup.py](src/my_py_pkg/setup.py) file in the `console_scripts` list with the following example code:

```
"py_node = my_py_pkg.my_first_node:main"
```

- `py_node` is now the name of this node
- `my_py_pkg.my_first_node` is the relative path to the node file
- `main` is the function that will be run

This file is then be made executable with the following code
```
chmode +x my_first_node
```
The following three steps will then be repeated anytime to run a node after building or editing:

1. `colcon build --packages-select my_py_pkg`
- Builds the files in the package
- **WARNING**: Must be run in the ROS2 workspace folder

2. `source ~/.bashrc` or if in the ROS2 workspace folder `source /install/setup.bash` 
- These commands will clear the terminal history

3. `ros2 run my_py_pkg py_node`
- Runs the name of the node created in the `setup.py` file

Only in Python, step 1 can be modified to be:
```
colcon build --packages-select my_py_pkg --symlink-install
```
- `--symlink-install` enables continuous editing on the file so that in the future steps 1 and 2 can be skipped after editing the file

#### Example
- [template_python_node.py](src/my_py_pkg/my_py_pkg/template_python_node.py)
    - This specific node can be used as a template for any new python node

### C++

When a new node is created, any new packages as well as the name of the folder must be added to the [CMakeLists.txt](src/my_cpp_pkg/CMakeLists.txt) file with the following code:

```CMake
find_package(ament_cmake REQUIRED)              # add new packages
find_package(rclcpp REQUIRED)

add_executable(cpp_node src/my_first_node.cpp)  # add new node name + relative path
ament_target_dependencies(cpp_node rclcpp)      # add new node name + packages

install(TARGETS 
  cpp_node                                      # add new node name
  DESTINATION lib/${PROJECT_NAME}
)
```
The following three steps will then be repeated anytime to run a node after building or editing:

1. `colcon build --packages-select my_cpp_pkg`
- Builds the files in the package
- **WARNING**: Must be run in the ROS2 workspace folder

2. `source ~/.bashrc` or if in the ROS2 workspace folder `source /install/setup.bash` 
- These commands will clear the terminal history

3. `ros2 run my_cpp_pkg py_node`
- Runs the name of the node created in the `CMakeLists.txt` file

#### Example
- [template_cpp_node.cpp](src/my_cpp_pkg/src/template_cpp_node.cpp)
    - This specific node can be used as a template for any new cpp node

## Helpful Command Line Tools for Nodes

- `ros2 node list` -- this will show any currently running nodes
- `ros2 node info /<currently_running_node>` -- gives detailed information on the node specified
- `--ros-args -r __node:=<new_name>` -- added at the end of the ros2 run command to change the name of a node temporarily

---
- `rqt_graph` -- opens a window that can visually represent the node connections
- `ros2 interface show <example_interface>` -- shows the data types in an interface
---

## Topics

A topic is a named bus over which nodes exchange messages. It is used to send information only one way. It's key features include
- Publisher sends information
- Subscriber receives information
- Unidirectional data streams
- Anonymous
- A topic has a message type that the publisher and subscriber must match
- A node can have many publishers/subscribers for many different topics

The flow goes as follows:

Publisher(s) -> Topic(s) -> Subscriber(s)

- See a publisher in the example [robot_news_station.py](src/my_py_pkg/my_py_pkg/robot_news_station.py) or [robot_news_station.cpp](src/my_cpp_pkg/src/robot_news_station.cpp)
- See a Subscriber in the example [smartphone.py](src/my_py_pkg/my_py_pkg/smartphone.py) or [smartphone.cpp](src/my_cpp_pkg/src/smartphone.cpp)

### Helpful Command Line Tools for Topics
- `ros2 topic list` -- shows any currently running topics
- `ros2 topic info /<currently_running_topic>` -- gives detailed information on the topic specified
- `ros2 topic echo /<currently_running_topic>` -- prints into the terminal the data stream in the topic
- `--ros-args -r <topic_name>:=<new_name>` -- added at the end of the ros2 run command to change the name of a topic temporarily
    - Must change the topic name for both the publisher and subscriber
- `ros2 topic hz /<currently_running_topic>` -- number of messages being sent in hz
- `ros2 topic bw /<currently_running_topic>` -- how much data is being sent
- `ros2 topic pub -r 5 /<topic_name> <data_type> "{data: 'Hello from the terminal'}"` -- this creates a publisher in the terminal
    - `5` is the frequency it will publish at

### Bags
Bags are used to save data from a topic during a test so that it can be replayed later and worked on without actually running another experiment

- First create a new folder to put the data in then run the following command

    - `ros2 bag record -o <test> /<topic_name>` -- record all data running through that topic
    - `-o test` -- name the file created
    - Can add on additional topic by listing them at the end so that you can record multiple topics at once
- `ros2 bag play <test>` --play the bagged data back
- `ros2 bag info <test>` --get extra data on the bag

## Services
ROS2 services are used when it is needed to both send and receive information in one node. They are typically used when that node will need to take an action so services are named with a verb. To decide between a Service and a Topic as yourself, "Am I just sending some data, or do I expect a response after I send the message?"
- Clients make a *request* for information from a Server
- Servers receive a Client *request* and send the desired data back in a *response*
- Services can use one message type for *requests* and one message type for *responses*
- Their can only be **one** Server but many Clients

The flow goes as follows:

Server <--> Service <--> Client(s)

- See a Server in the example [add_two_ints_server.py](src/my_py_pkg/my_py_pkg/add_two_ints_server.py) or [add_two_ints_server.cpp](src/my_cpp_pkg/src/add_two_ints_server.cpp)
- See a Client in the examples [add_two_ints_no_oop.py](src/my_py_pkg/my_py_pkg/add_two_ints_client_no_oop.py) and [add_two_ints.py](src/my_py_pkg/my_py_pkg/add_two_ints_client.py) or [add_two_ints_no_oop.cpp](src/my_cpp_pkg/src/add_two_ints_client_no_oop.cpp) and [add_two_ints.cpp](src/my_cpp_pkg/src/add_two_ints_client.cpp)

### Helpful Command Line Tools for Services
- `ros2 service list` -- this will show any currently running services
- `ros2 service type /<currently_running_node>` -- gives detailed information on the service specified
- `--ros-args -r /<current_service_name>=<new_name>` -- added at the end of the ros2 run command to change the name of a service temporarily
- Write to a Service on the command line
    1. `ros2 service type /<currently_running_node>` -- find out the type
    2. `ros2 interface show <example_interface>` -- shows the data types in the interface
    3. `ros2 service call /<service_name> <example_interface> "{a: 100, b: 200}"` -- sends a request from the command line

## Custom Interfaces
Custom Interfaces are the actual "letter" that holds the information going to and from Topics and Services
- Topics: use `msg` type that holds data it will send
- Services: use `srv` type that holds two kinds of data: `request` on top and `response` below it

Here are three important online sources for custom interfaces:
1. [ROS2 Documentation](https://docs.ros.org/en/rolling/Concepts/Basic/About-Interfaces.html) -- This site has a table of all ROS2 data types that can be used
2. [Example Interfaces](https://github.com/ros2/example_interfaces/tree/rolling/msg) -- This site holds all the example_interfaces that can be used for ROS2
3. [Common Interfaces](https://github.com/ros2/common_interfaces/tree/rolling/std_srvs/srv) -- This site holds pre-made common interfaces to use for robotics applications

### Setup the interface
It is always good to create a new package for the interfaces. Do this with
```
ros2 pkg create <my_robot_interfaces>
```
- This will automatically create a C++ pkg
- Can immediately remove the un-needed folders with
```
rm -r include/ src/
```
Need to add code for custom interface into the [package.xml](src/my_robot_interfaces/package.xml)
```xml
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```
Can then go to the [CmakeLists.txt](src/my_robot_interfaces/CMakeLists.txt) file and delete lines 10-24 of test code. After this, paste the following lines in the dependencies:
```CMake
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "your custom interfaces will be here"
  "one per line"
  "no comma for separating lines"
 )
```
### Build the interface
In the created interface package that is named `my_robot_interfaces` in this example, run
```
mkdir msg
```
Then create an interface with
```
touch <HardwareStatus.msg>
```
- Name must end with .msg
- Name must have no spaces
- Name is separated by CamelCasing
- Example interface can be found in [HardwareStatus.msg](src/my_robot_interfaces/msg/HardwareStatus.msg)
