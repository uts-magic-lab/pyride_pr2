cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
LINK_DIRECTORIES(${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_boost_directories()
rosbuild_add_executable(${PROJECT_NAME} src/AppConfigManager.cpp
                                        src/PyRideCommon.cpp
                                        src/jdatabufferdst.cpp
                                        src/DeviceController.cpp
                                        src/ServerDataProcessor.cpp
                                        src/PyRideNetComm.cpp
                                        src/PythonServer.cpp
                                        src/PyPR2Server.cpp
                                        src/PyModuleStub.cpp
                                        src/PR2ProxyManager.cpp
                                        src/PyPR2Module.cpp
                                        src/VideoObject.cpp
                                        src/main.cpp)

add_definitions(-DROS_BUILD -DUSE_ENCRYPTION)
include_directories( include /usr/include/python2.7 )
target_link_libraries(${PROJECT_NAME} ccrtp1 ccgnu2 crypto python2.7 celt0 jpeg tinyxml )

#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(example ${PROJECT_NAME})
