#Python based Robot Inteactive Development Environment (PyRIDE) for ROS/PR2

##Introduction
Python based Robot Interactive Development Environment (PyRIDE) is a middleware that provides
a self-contained development environment for rapid interactive programming of robot skills 
and behaviours using Python scripting language. PyRIDE functions as a software integration 
tool that aggregates disparate robot software modules and exposes their functionalities to an
embedded Python intepreter engine through a unified programmming interface. Robot programmers
can access the embedded Python engine and the unified robot function programming interface
using a remote interactive shell service. One can code, perform experiments/test, debug 
programs interactively in the same way as using the standard Python interactive interpreter.
Programs can transit seamlessly from development to deployment with a bootstrap mechanism.
PyRIDE also provides remote user level accesses of the robot functionalities, e.g. real-time
robot camera image data, through a client-server mechanism. You can view a tech demo 
[here](http://https://www.youtube.com/watch?v=0DTB62lm8z4).

Finally, PyRIDE is written in portable C++ and can be ported to various robot platform with 
relative ease. Currently, PyRIDE runs on NAO, ROS/PR2 and iOS/ROMO platforms. This repository 
contains PyRIDE source code for ROS/PR2 platform.

##Compile source code
PyRIDE on ROS/PR2 uses the standard catkin build system. It requires a full PR2 Hydro 
installation on a Ubuntu linux system. You need to set up a catkin workspace and check out
the latest code from the repository into the source directory of the catkin workspace. Then,
 do:

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
sub-directory. **NOTE**: The API documentation is incomplete at this stage.

##Lauch PyRIDE on PR2
Make sure you have started up a full PR2 simulation or on a PR2 robot and sourced appropriate
catkin overlay. Do:

```
roslaunch pyride_pr2 pyride.launch
```

***NOTE** fully functioning PyRIDE depends on several subsystems such as MoveIt!, 2D navigation
stack. If you want to use functionalities related to these subsystems, make sure these 
subsystems are operating before launching PyRIDE.

##Logging
PyRIDE uses its own logging system. You can find the log output under ```.ros/pyride_pr2*.log```.



