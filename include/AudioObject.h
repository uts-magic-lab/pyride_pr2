/*
 *  AudioObject.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef AUDIOOJECT_H
#define AUDIOOJECT_H

#include <string>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <ccrtp/rtp.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <audio_common_msgs/AudioData.h>

#include "DeviceController.h"

using namespace std;
using namespace ros;
using namespace ost;

namespace pyride {

class AudioObject : public AudioDevice {
public:
  AudioObject();
  
  bool initDevice();
  void finiDevice();

private:
  NodeHandle priAudioNode_;
  Subscriber * audioSub_;

  CallbackQueue audioQueue_;

  AsyncSpinner * procThread_;

  bool setDefaultAudioParameters();
  //void getUDPSourcePorts();
  
  bool initWorkerThread();
  void finiWorkerThread();
  
  void doAudioStreaming( const audio_common_msgs::AudioDataConstPtr& msg );
};
} // namespace pyride
#endif // AUDIOOJECT_H
