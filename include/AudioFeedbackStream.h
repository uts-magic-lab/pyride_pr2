/*
 *  AudioFeedbackStream.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 23/06/2017
 *
 */

#ifndef AUDIO_FEEDBACK_STREAM_H
#define AUDIO_FEEDBACK_STREAM_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

#include <ros/ros.h>

#include <celt/celt.h>

#include "RTPDataReceiver.h"

namespace pyride {

using namespace pyride_remote;
using namespace ros;

class AudioFeedbackStream
{
public:
  static AudioFeedbackStream * instance();
  ~AudioFeedbackStream();

  void initWithNode( NodeHandle * nodeHandle );

  void addClient();
  void removeClient();

private:
  int clientNo_;
  bool isRunning_;

  Publisher audioPub_;
  NodeHandle * mCtrlNode_;

  RTPDataReceiver * dataStream_;

  CELTMode * celtMode_;
  CELTDecoder * audioDecoder_;

  boost::thread * streaming_data_thread_;

  static AudioFeedbackStream * s_pAudioFeedbackStream;

  AudioFeedbackStream();

  bool start();
  void stop();

  void grabAndDispatchAudioStreamData();
};

} // namespace pyride

#endif /* AUDIO_FEEDBACK_STREAM_H */
