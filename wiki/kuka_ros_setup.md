# Kuka ROS Tutorial - Setup

## Preparation

This system requires 2 computers (Windows + Ubuntu).

- Sunrise Cabinet: controller box of robot arm
- ROS machine: the Ubuntu computer with ROS, iiwa_stack
- Sunrise Workbench: workbench running on the Windows computer, works as a middleman

You need to get the IP address of:

- Sunrise Cabinet (e.g. `192.168.10.118` or `192.168.10.116`)
- ROS machine (run `ifconfig` in terminal to get the IP)

## Reference

This guide is based on [iiwa's wiki](https://github.com/IFL-CAMP/iiwa_stack/wiki), with some part adapted to RROS Lab's environment

## Setup  

### Steps

1. Connect an ethernet cable between the **X66 port** of the Sunrise Cabinet and your **ROS** machine
2. By default the **Sunrise** Cabinet IP address is `192.168.10.118`. Setup the network interface on the **ROS** machine to be in the same subnet. (e.g. `192.168.10.101`). Ping the **Sunrise** Cabinet from your **ROS** machine to check that everything is fine.
3. [Clone, build and setup iiwa_stack on your ROS machine.](#ros-machine-side)
4. [Clone and setup a Sunrise Project with iiwa_stack on your SUNRISE Cabinet.](#sunrise-workbench-side)

### ROS Machine side

1. **Install ROS NOETIC** (if not already there) as described at [here](http://wiki.ros.org/melodic/Installation/Ubuntu).
   It's also a good idea to install the python catkin tools
   `sudo apt-get install python-catkin-tools`

2. **Setup the ROS IP** :
Open .bashrc file  
`gedit ~/.bashrc &`
and append these two lines at the end
`export ROS_IP=xxx.xxx.xxx.xxx`
`export ROS_MASTER_URI=http://$ROS_IP:11311`
Where *xxx.xxx.xxx.xxx* is the **IP address** of your **ROS** machine, which we previously set to be under the same subnet of your **SUNRISE Cabinet** (e.g. 192.168.10.101).  
(after adding above two lines in .bashrc you can only run `roscore` with your computer connected to the subnet, if you want to run `roscore` in other network, i.e., your IP address changed, you need to comment out these two lines and do the step bellow)  
Then run
`source ~/.bashrc`

3. **Setup ROS workspace**
run command  
`sh scripts/kuka_env_setup.sh`  
**NOTE** : this command will create a new ros workspace in directory `~/` if you want set your workspace somewhere else, skip this step; otherwise, do this step and skip the steps bellow to [Sunrise side](#sunrise-workbench-side)

4. **Clone iiwa_stack repository to your workspace** (you can omit the first 2 commands if you already have one) :
`mkdir -p iiwa_stack_ws/src && cd ..`
`catkin_init_workspace`
`git clone https://github.com/RROS-Lab/iiwa_stack_cam.git src/iiwa_stack_cam`

5. **Download the dependencies** :
`rosdep install --from-paths src --ignore-src -r -y`

6. **Build the workspace** :  
`catkin build`  
if you failed with error message:
*Unable to find either executable 'empy' or Python module 'em'...  
try installing the package 'python-empy'*  
try:  ([Ref](https://github.com/ysl208/iRoPro/issues/59))  
`catkin build -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.7m`  

7. **Source the workspace** :
`source devel/setup.bash`

### Sunrise Workbench Side

1. **Basic Sunrise Project Setup** :  

   - Within **Sunrise Workbench**, create a **Sunrise project** or load an existing one from the controller (default IP: 172.31.1.147).
   - Open the **StationSetup.cat** file
   - Select the **Software** tab
   - Enable the Servo Motion packages:
     - Direct Servo Motion Extension
     - Smart Servo Motion Extension
     - Smart Servo Linear Motion Extension
   - Save and apply changes(you might need to reinstall the project)

2. **Add `iiwa_stack` (more precisely the content of `iiwa_ros_java`) to the Sunrise project** :

    From now on, for simplicity, we will use two **pseudo** shell variables:
      - **IIWA_STACK_DIR**: the path to the root of the `iiwa_stack` project, you probably cloned it into this folder.
      - **SUNRISE_PROJECT_DIR**: the path to the root of your **Sunrise project** (somewhere in your file system under your **Sunrise Workspace**)

    Remember, these are **NOT** real variables, we use them just to shorten things, use the full path!

      - Copy the content of `$IIWA_STACK_DIR\iiwa_ros_java\src` inside the `src` folder of the **Sunrise project**.
      - Copy the folder `$IIWA_STACK_DIR\iiwa_ros_java\ROSJavaLib` into the root of the **Sunrise project**.
      - Inside **Sunrise Workbench** select all the files inside *ROSJavaLib*, right click and choose *Build Path* -> Add to Build Path...

3. **Setup the ProcessData Configuration** :  

    In your **Sunrise Project**, open the file `src/RoboticsAPI.data.xml` and modify its content to

    ```xml
    <?xml version="1.0" encoding="UTF-8" standalone="no"?>
    <RoboticsAPIData version="3">
        <world>
            <gravitation x="0.0" y="0.0" z="9.81"/>
        </world>
        <processDataContainer>
            <processData dataType="java.lang.String" defaultValue="iiwa" displayName="Robot Name" editableOnHmi="true" id="robot_name" value="iiwa"/>
            <processData dataType="java.lang.String" defaultValue="192.168.10.101" displayName="ROS Master IP" editableOnHmi="true" id="master_ip" value="192.168.10.101"/>
            <processData dataType="java.lang.String" defaultValue="11311" displayName="ROS Master Port" editableOnHmi="false" id="master_port" value="11311"/>
            <processData dataType="java.lang.String" defaultValue="192.168.10.118" displayName="Robot IP" editableOnHmi="false" id="robot_ip" value="192.168.10.118" visibleOnHmi="false"/>
            <processData dataType="java.lang.Boolean" defaultValue="false" displayName="Enable NTP" editableOnHmi="true" id="ntp" value="false"/>
            <processData dataType="java.lang.Boolean" defaultValue="false" displayName="Enable Debug Output" editableOnHmi="true" id="debug" value="false"/>
        </processDataContainer>
    </RoboticsAPIData>
    ```

    **You need to check the following items, if they do not match your configuration, change them**
    - ROS Master IP (the IP of your ROS machine)
    - Robot IP (the IP of your Sunrise Cabinet)

    e.g., if the IP of your Sunrise Cabinet is `192.168.10.116`. Inside the `processDataContainer` field, you need to change  

    ```xml
    <processData dataType="java.lang.String" defaultValue="192.168.10.118" displayName="Robot IP" editableOnHmi="false" id="robot_ip" value="192.168.10.118" visibleOnHmi="false"/>
    ```

    to:  

    ```xml
    <processData dataType="java.lang.String" defaultValue="192.168.10.118" displayName="Robot IP" editableOnHmi="false" id="robot_ip" value="192.168.10.116" visibleOnHmi="false"/>
    ```

    This enables you to dynamically reconfigure some parameters that are used in the **Sunrise Application** without having to resynchronize it to the **Sunrise Cabinet**. These options will be available on the **Process Data** window of the **SmartPad**.

    If your `RoboticsAPI.data.xml` file contains or will contain something else, adjust it accordingly to your needs maintaining the items above inside the `processDataContainer` field.

4. **DONE** :  
Whatever option you choose you should now have a **Sunrise project** without any error.
You can now *install* it (*Station Setup* -> *Installation*) and then synchronize it.

## Run Application on Teaching Pendant

1. Switch Operating Mode to Auto Mode  
    - turn Emergency Stop button (red button on the top of teaching pendant) clockwise to release it
    - turn the little key on teaching pendant clockwise
    - click AUT
    - turn back the key
1. Run ROSSmartServo  
    - click the Application button on the screen
    - choose ROSSmartServo
    - press green button "|>" on the left side
1. Run ROS Master Node on the Ubuntu Machine
    - run `roscore`
1. Enjoy ROS on KUKA~
    - **IF ANYTHING GOES WRONG, PRESS EMERGENCY STOP BUTTON IMMEDIATELY**
    - try `rostopic list` to see if there are iiwa topics
1. Stop ROSSmartServo Application before Turning off ROS Master Node
    - if you turn off ROS Master Node before ROSSmartServo application, you may need to restart the kuka controller

## ❗ **IMPORTANT** ❗

Have you already set your **Safety Configuration**? If not, you should!  
Mind that every time you change your **Safety Configuration** you need to reinstall the project.
