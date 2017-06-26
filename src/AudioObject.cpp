/*
 *  AudioObject.cpp
 *  PyRIDE
 *
 *  Created by Xun Wang on 9/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/select.h>

#include "PyRideCommon.h"
#include "AudioObject.h"

namespace pyride {

AudioObject::AudioObject() :
  AudioDevice(),
  procThread_( NULL ),
  audioSub_( NULL )
{
  priAudioNode_.setCallbackQueue( &audioQueue_ );
}

bool AudioObject::initDevice()
{
  if (isInitialised_)
    return isInitialised_;

  if (!this->setDefaultAudioParameters()) {
    ERROR_MSG( "Unable to set default audio settings.\n" );
    return false;
  }

  procThread_ = new AsyncSpinner( 1, &audioQueue_ );
  procThread_->start();

  packetStamp_ = 0;
  clientNo_ = 0;
  isInitialised_ = true;

  INFO_MSG( "Robot audio is successfully initialised.\n" );

  return true;
}

bool AudioObject::initWorkerThread()
{
  if (!isInitialised_)
    return false;

  if (!procThread_) {
    ERROR_MSG( "Unable to ceate thread to grab audio data" );
    return false;
  }

  audioSub_ = new ros::Subscriber( priAudioNode_.subscribe( "audio", 1, &AudioObject::doAudioStreaming, this ) );

  return true;
}

void AudioObject::finiWorkerThread()
{
  if (audioSub_) {
    audioSub_->shutdown();
    delete audioSub_;
    audioSub_ = NULL;
  }
}
  
void AudioObject::doAudioStreaming( const audio_common_msgs::AudioDataConstPtr& msg )
{
  this->processAndSendAudioData( (short*)(&msg->data[0]), msg->data.size() / 2 / aSettings_.channels );
}

bool AudioObject::setDefaultAudioParameters()
{
  int val = 0;
  ros::param::param( "/audio_stream/channels", val, 1 );
  aSettings_.channels = (char)val;
  ros::param::param( "/audio_stream/samplerate", val, 16000 );
  aSettings_.sampling = (short)val;
  ros::param::param( "/audio_stream/depth", val, 16 );
  aSettings_.samplebytes = (char)(val / 8);
  return this->setProcessParameters();
}

void AudioObject::finiDevice()
{
  if (!isInitialised_)
    return;

  if (isStreaming_) {
    isStreaming_ = false;
    this->finiWorkerThread();
  }
  
  procThread_->stop();
  delete procThread_;
  procThread_ = NULL;

  packetStamp_ = 0;

  isInitialised_ = false;
}
} // namespace pyride
