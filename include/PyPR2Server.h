/*
 *  PyPR2Server.h
 *  PyPR2Server
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef PyPR2Server_h_DEFINED
#define PyPR2Server_h_DEFINED

#include <stdio.h>
#include <string.h>
#include <sstream>
#include <sys/types.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>
#include <netdb.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <pyride_common_msgs/NodeStatus.h>

#include "ServerDataProcessor.h"

namespace pyride {

using namespace ros;

class PyPR2Server : public PyRideExtendedCommandHandler {
public:
  PyPR2Server();
  virtual ~PyPR2Server();
  bool init();
  void fini();
  
  void continueProcessing();
  
  void nodeStatusCB( const pyride_common_msgs::NodeStatusConstPtr & msg );

  void stopProcess();

private:
  bool isRunning_;
  NodeHandle * hcNodeHandle_;
  Subscriber nodeStatusSub_;

  VideoDeviceList activeVideoDevices_;
  AudioDeviceList activeAudioDevices_;

  bool initVideoDevices();
  void finiVideoDevices();
  
  bool initAudioDevices();
  void finiAudioDevices();

  void notifySystemShutdown();

  bool executeRemoteCommand( PyRideExtendedCommand command, int & retVal,
                            const unsigned char * optionalData = NULL,
                            const int optionalDataLength = 0 );

  void cancelCurrentOperation();
  
};
}; // namespace pyride
#endif // PyPR2Server_h_DEFINED
