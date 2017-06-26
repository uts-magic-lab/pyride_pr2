/*
 *  VideoObject.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef VIDEOOJECT_H
#define VIDEOOJECT_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <ccrtp/rtp.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>

#include "DeviceController.h"

using namespace std;
using namespace ros;
using namespace ost;

namespace pyride {

class VideoObject : public VideoDevice {
public:
  VideoObject( DeviceInfo & info );
  
  bool initDevice();
  void finiDevice();

  void takeSnapshot( const VideoDeviceDataHandler * dataHandler );

  void continueProcessing( const sensor_msgs::ImageConstPtr& msg );

private:  
  NodeHandle * imgSubNode_;
  NodeHandle priImgNode_;
  image_transport::ImageTransport imgTrans_;
  image_transport::Subscriber imgSub_;
  sensor_msgs::ImageConstPtr imgMsgPtr_;

  boost::thread * streaming_data_thread_;

  boost::mutex mutex_;
  boost::condition_variable imageCon_;

  CallbackQueue imgQueue_;

  AsyncSpinner * procThread_;

  bool takeSnapShot_;
  
  bool getDefaultVideoSettings();
  //void getUDPSourcePorts();
  
  bool initWorkerThread();
  void finiWorkerThread();
  
  void doImageStreaming();
};
} // namespace pyride
#endif // VIDEOOJECT_H
