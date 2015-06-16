#Python based Robot Interactive Development Environment (PyRIDE) for ROS/PR2

##Introduction
Python based Robot Interactive Development Environment (PyRIDE) is a middleware that provides
a self-contained development environment for rapid interactive programming of robot skills 
and behaviours using Python scripting language. PyRIDE functions as a software integration 
tool that aggregates disparate robot software modules and exposes their functionalities to an
embedded Python interpreter engine through a unified programming interface. Robot programmers
can access the embedded Python engine and the unified robot function programming interface
using a remote interactive shell service. One can code, perform experiments/test, debug 
programs interactively in the same way as using the standard Python interactive interpreter.
Programs can transit seamlessly from development to deployment with a bootstrap mechanism.
PyRIDE also provides remote user level accesses of the robot functionalities, e.g. real-time
robot camera image data, through a client-server mechanism. You can view a demonstration video 
[here](https://www.youtube.com/watch?v=0DTB62lm8z4).

Finally, PyRIDE is written in portable C++ and can be ported to various robot platform with 
relative ease. Currently, PyRIDE runs on NAO, ROS/PR2 and iOS/ROMO platforms. This repository 
contains PyRIDE source code for ROS/PR2 platform.

##Compile source code
###Prerequisites
PyRIDE on ROS/PR2 uses the standard catkin build system. It requires a full PR2 Hydro 
installation on a Ubuntu Linux system. In particular, make sure you have the following
packages installed on your system:

* ros-hydro-moveit-resources
* ros-hydro-sound-play
* libccrtp-dev

You need to set up a catkin workspace and check out the latest code from this repository
into the source directory of a working catkin workspace. Then, do:

```
catkin_make pyride_pr2
catkin_make install
```

PyRIDE should be compiled and installed under the catkin install directory along with
all necessary dependent libraries installed.

##Generate PyRIDE on PR2 API documentation
You can use doxygen to generate the latest API documentation of PyRIDE on PR2. Simply run:

```
doxygen pyride.dox
```
under the `pyride_pr2` directory. The API documentation will be generated under `doc` 
sub-directory. **NOTE**: The API documentation is incomplete at this stage. You can find the latest doxygen generated documentation at http://uts-magic-lab.github.io/pyride_pr2

##Launch PyRIDE on PR2
Make sure you have started up a full PR2 simulation or on a PR2 robot and sourced appropriate
catkin overlay. Do:

```
roslaunch pyride_pr2 pyride.launch
```

**NOTE** fully functioning PyRIDE depends on several subsystems such as MoveIt!, 2D navigation
stack. If you want to use functionalities related to these subsystems, make sure these 
subsystems are operating before launching PyRIDE.

##Access embedded interactive Python console
PyRIDE contains a fully functional Python intepreter. One can access its interactive remote shell
via
```
telnet host 27005
```
where ```host``` is the robot/machine PyRIDE is running on. You can turn off the remote shell
access by setting **RemotePythonAccess** tag to disable in ```pyrideconfig.xml``` configuration
file.
 
##Load user-developed Python scripts
PyRIDE on PR2 can automatically load and execute Python scripts on its start up. It searches
for a ```py_main.py``` under a predefined script directory and executes its ```main()``` function.
Check ```scripts/py_main.py``` under this repository for further details. A working application
example that receives and sends messages to a twitter account is also provided.

**NOTE** You can specify the directory where you want PyRIDE load Python scripts under ```script_dir```
parameter in the ```pyride.launch``` file.

##Logging
PyRIDE uses its own logging system. You can find the log output under ```.ros/pyride_pr2*.log```.

##Choose a suitable inverse kinematic engine
PyRIDE natively supports MoveIt! as well as S-PR2/Magiks. You can choose either inverse
kinematic package to drive PR2 arms in the task space (with odometry_combined as the default reference
frame). To support adoption and further development of S-PR2/Magiks, PyRIDE selects S-PR2 as its default
inverse kinematic engine. However, you still need to manually install all required dependencies and 
clone the entire Magiks repository under the script directory before running PyRIDE. Since S-PR2 runs 
entirely under PyRIDE. You have direct access to its advanced features and the vast array of library 
functions within the package.

Currently, PyRIDE provides only basic level support for MoveIt! and methods relate to MoveIt! have not
been fully tested. To enable MoveIt! under PyRIDE, you need to do the following steps:

1. Set ```use_move_it``` parameter in the ```pyride.launch``` to true.
2. Launch MoveIt! before running PyRIDE.
3. Call ```py_main.iksResolver.useMoveIt()``` method in PyRIDE.

Note that regardless which inverse kinematic engine is selected, you can move a PR2 arm use the same 
```PyPR2.moveArmTo``` method. Check API documentation for further details. Another important issue you
need to keep in mind is that the reference frames used in MoveIt! and S-PR2 are slightly different. You
cannot switch between MoveIt! and S-PR2 transparently without adjust your target position and orientation
calculations. We will improve the wrapper functions so that you can transit between the two seamlessly in
the future.

##Third-party ROS node integration with PyRIDE
PyRIDE for PR2 already provides some of the most commonly used functionalities on PR2 by integrating the
respective ROS nodes into the PyRIDE framework. You may want to integrate additional third party ROS node
with PyRIDE. PyRIDE supports two approaches for ROS node integration:

1. "Deep" integration: This approach requires modifications of PyRIDE source code. ```PR2ProxyManager```
class is the main class that is responsible for communicating with foreign ROS nodes and providing C++ based
wrapper functions to interface with these node. You should create necessary subscriber or publishers to the
node in the ```initWithNodeHandle``` method and the corresponding clean up code in the ```fini``` method.
You should write necessary callback functions for your subscriber and command methods for your publisher (or
action client) as methods of ```PR2ProxyManager```. ```PyPR2Module``` class provides wrapper functions to 
expose C++ methods (usually) of ```PR2ProxyManager``` class to the embedded Python engine. It uses the
standard embedded Python programming approach and APIs. See source code further details and examples.
**NOTE** Future version of PyRIDE may provide a better designed plugin mechanism for deep ROS node integration.

2. "Shallow" integration: This approach represents a simpler way of sending messages from an external ROS node
to PyRIDE. PyRIDE constantly listens on a ```/pyride_pr2/node_status``` topic. A ROS node can publish 
```NodeStatus``` (see ```msg/NodeStatus.msg``` for message structure) messages to this topic, PyRIDE will 
automatically forward the received messages to ```PyPR2.onNodeStatusUpdate``` Python callback function in the
embedded Python environment. See ```py_main.py``` for further details on this callback function.

##Limited use of ```rospy``` under PyRIDE environment
In ideal circumstance, PyRIDE should be used as a full replacement for ```rospy``` with custom integration of third
party ROS modules. However, it is possible to use ```rospy``` in a limited way under PyRIDE to publish messages
to other ROS node. Use the following example code snippet to create an *addon* node and publish messages.

```
import sys
sys.path.append('/opt/ros/hydro/lib/python2.7/dist-packages') #add rospy path into the system path
import rospy
from std_msgs.msg import String
rospy.init_node('pyride_talker',disable_signals=True)
pub = rospy.Publisher( 'pyride_msg', String, queue_size = 5 )
pub.publish('this is a test message')
```

**NOTE** that you should **not** use ```rospy.spin()``` under PyRIDE.

