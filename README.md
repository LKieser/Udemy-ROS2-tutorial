# ROS2 Basics Tutorial

- [ROS2 Basics Tutorial](#ros2-basics-tutorial)
  - [Overview](#overview)
  - [Setup](#setup)
      - [Python](#python)
      - [C++](#c)
      - [Python](#python-1)
      - [C++](#c-1)
  - [Building a Node](#building-a-node)
    - [Python](#python-2)
      - [Example](#example)
    - [C++](#c-2)
      - [Example](#example-1)
  - [Helpful Command Line Tools for Nodes](#helpful-command-line-tools-for-nodes)
  - [Topics](#topics)
    - [Helpful Command Line Tools for Topics](#helpful-command-line-tools-for-topics)
    - [Bags](#bags)
  - [Services](#services)
    - [Helpful Command Line Tools for Services](#helpful-command-line-tools-for-services)
  - [Custom Interfaces](#custom-interfaces)
    - [Setup the interface](#setup-the-interface)
    - [Build and use the interface for Topics](#build-and-use-the-interface-for-topics)
    - [Helpful Command Line Tools for Interfaces](#helpful-command-line-tools-for-interfaces)
  - [Ros2 Parameters](#ros2-parameters)
    - [Helpful Parameter Commands](#helpful-parameter-commands)
    - [Parameter Yaml File](#parameter-yaml-file)
  - [Launch Files](#launch-files)
    - [Setup the launch file workspace](#setup-the-launch-file-workspace)
    - [Build a launch file](#build-a-launch-file)
    - [Additional launch file functionality](#additional-launch-file-functionality)
      - [Remapping](#remapping)
      - [Parameters](#parameters)
      - [Namespaces](#namespaces)
  - [If there are any arguments in the code that come after this be sure they don't include a `/` in front of them or they will not have the same name](#if-there-are-any-arguments-in-the-code-that-come-after-this-be-sure-they-dont-include-a--in-front-of-them-or-they-will-not-have-the-same-name)


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
This is where each new interface will be added after it is made
### Build and use the interface for Topics
In the created interface package that is named `my_robot_interfaces` in this example, run
```
mkdir msg
```
Then create an interface with
```
touch <HardwareStatus.msg>
```
- Name:
    - must end with .msg
    - must have no spaces
    - is separated by CamelCasing
    - Example interface can be found in [HardwareStatus.msg](src/my_robot_interfaces/msg/HardwareStatus.msg)
- Use in Python:
    - Add as dependency as seen in [package.xml](src/my_py_pkg/package.xml)
    - Call in at the top of the file as seen in [hardware_status_publisher.py](src/my_py_pkg/my_py_pkg/hardware_status_publisher.py)
- Use in C++:
    - Add the path to the interface into the vscode workspace folder [c_cpp_properties.json](.vscode/c_cpp_properties.json)
    - Add the dependency to [package.xml](src/my_cpp_pkg/package.xml)
    - Add to `find_package` in [CMakeLists.txt](src/my_cpp_pkg/CMakeLists.txt)
    - Add the #include at the top of the file as seen in [hardware_status_publisher.cpp](src/my_cpp_pkg/src/hardware_status_publisher.cpp)

- **NOTE**: Make a custom service the same way but replace `msg` with `srv`

### Helpful Command Line Tools for Interfaces
This command shows the data types in a specific interface
```
ros2 interface show <example_interface/msg/example>
```
This command shows every interface in the system
```
ros2 interface list
```
This command shows the custom interfaces in one folder
```
ros2 interface package <my_robot_interfaces>
```
These commands can be used to see what interface is being used on a running node or topic
- `ros2 node list` then `ros2 node info /<running_node>`
- `ros2 topic list` then `ros2 topic info /<running_topic>`
- The following examples were used for a custom interface, topic, and service call [LED_panel.py](src/my_py_pkg/my_py_pkg/LED_panel.py) and [battery_node.py](src/my_py_pkg/my_py_pkg/battery_node.py)


## Ros2 Parameters
ROS2 parameter are used to set values at run time. Typically these values would be settings for the node that may need to change so they can't be hardcoded. A parameter is specific to each node. For each parameter, in the code:
- Declare the parameter
- Get the parameter value
- Example in [number_publisher.py](src/my_py_pkg/my_py_pkg/number_publisher.py) and [number_publisher.cpp](src/my_cpp_pkg/src/number_publisher.cpp)

### Helpful Parameter Commands
This command shows the available parameters
```
ros2 param list
```
Use this to run the parameter. You can run multiple parameters with one line as seen here
```
ros2 run my_py_pkg number_publisher --ros-args -p number:=3 -p timer_period:=0.5
```
Use this to see the current value of the parameter
```
ros2 param get /<currently_running_node> <param_you_want_details_on>
```
Use this to set a new default parameter value in the terminal. If this is done the code in the file will need to be modified to add a `parameter_callback` as seen in [number_publisher.py](src/my_py_pkg/my_py_pkg/number_publisher.py)
```
ros2 param set /<currently_running_node> <param_you_want_details_on> <new value>
```

### Parameter Yaml File
Yaml files are used to run may parameters at once through a file. Create a folder and then a file that we will call `number_params.yaml` and it will look like this
```yaml
# first 2 lines are needed in every yaml file
/Number_Publisher: # node name
  ros__parameters: # use 2 spaces
    number: 5      # use another 2 spaces and can now list all params if desired
    timer_period: 0.7
```
Run this yaml file with
```bash
ros2 run my_py_pkg <example_node> --ros-args --params-file ~/<example_folder>/<number_params.yaml>
```
If it is needed, a second node can be added in the Yaml file. This node must be called by a different name. To use this node with it's parameter settings the ros2 run command must be given a --ros-args and node name switch to match the Yaml file's name.

## Launch Files
Launch files allow you to start a bunch of nodes all at once from one file. This is useful when you have a lot of nodes and don't want to run them one at a time in a terminal
### Setup the launch file workspace
While a launch file could be made in any pkg it is standard to make a separate pkg to do hold the launch files. This pkg can be placed in the ros2_ws/src directory
```
ros2 pkg create <robot_name>_bringup
```
Then it is good to get rid of the unused folders made by the pkg create command and add the launch folder
```
rm -r include/ src/
```
```
mkdir launch
```
In the pkg now go to the [CMakeLists.txt](src/my_robot_bringup/CMakeLists.txt) file and add the following code
```
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)
```
### Build a launch file
- All launch file code will go between the `<launch></launch>`. This can be seen here in [number_app.launch.xml](src/my_robot_bringup/launch/number_app.launch.xml). This file is placed in the `launch` directory. An example of the bare bones of a launch file are included here:
```xml
<launch>
    <!--list the package name and then the node executable name just like you would for a ros2 run command-->
    <node pkg="my_py_pkg" exec="number_publisher" /> 
    <node pkg="my_cpp_pkg" exec="number_counter" />
</launch>
```
- All pkgs being used in the launch file must be included in the dependencies in [package.xml](src/my_robot_bringup/package.xml)
- Once finished run:
```
colcon build --packages-select <robot_name>_bringup
```
```
source install/setup.bash
```
```
ros2 launch <robot_name>bringup number_app.launch.xml 
```
- A python launch file can be used to do the same thing as xml if needed as seen in [number_app.launch.py](src/my_robot_bringup/launch/number_app.launch.py) but it is way harder to use Python so always use xml. Sometimes there may be a node that can only be run through Python and if this the case, you can simply run that specific node in Python and then include it in the xml file where all the rest of the code will be run as seen in [number_app_from_python.xml](src/my_robot_bringup/launch/number_app_from_python.launch.xml)
  
### Additional launch file functionality
#### Remapping
To rename the node use this
```xml
name="my_number_publisher"
```
To rename the topic use this
```xml
<remap from="/number" to="/my_number" />
```
#### Parameters
Can run parameters directly in xml with
```xml
<param name="number" value="6" />
```
Can also run parameters through yaml file.
- First make a `config` directory in the `<robot_name>_bringup` pkg
- Then add this config directory to the [CMakeLists.txt](src/my_robot_bringup/CMakeLists.txt) file
- Finally use this to include the yaml file that was placed in config:
```xml
<param from="$(find-pkg-share <robot_name>_bringup)/config/<yaml_file.yaml>" />
```
#### Namespaces
Adding a namespace to a node changes every name inside to add the namespace before it. This can be done on the command line:
```
ros2 run my_cpp_pkg number_publisher --ros-ars -r __ns:=/test
```
- The node name is now `/test/number_publisher` and the topic name is `/test/number` and so on
- If you provide a leading slash to either of these in the files code like `/number` then namespace will not change that topic when it changes the others
- Namespaces can also be changed in the launch file with this:
```
namespace="/abc"
```
If there are any arguments in the code that come after this be sure they don't include a `/` in front of them or they will not have the same name
---
All of these can be seen in the example file [number_app.launch.xml](src/my_robot_bringup/launch/number_app.launch.xml)