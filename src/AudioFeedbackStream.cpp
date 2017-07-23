/*
 *  AudioFeedbackStream.cpp
 *  PyRIDE
 *
 *  Created by Xun Wang on 13/06/2017
 *
 */

#include <sys/time.h>
#include <audio_common_msgs/AudioData.h>

#include "AudioFeedbackStream.h"

namespace pyride {

using namespace pyride_remote;
using namespace ros;

AudioFeedbackStream * AudioFeedbackStream::s_pAudioFeedbackStream = NULL;

static bool __verbose = false;
static const int kMaxCachedAudioData = 128 * PYRIDE_AUDIO_FRAME_SIZE;

AudioFeedbackStream::AudioFeedbackStream() :
  clientNo_( 0 ),
  isRunning_( false ),
  mCtrlNode_( NULL ),
  dataStream_( NULL ),
  celtMode_( NULL ),
  audioDecoder_( NULL )
{
  streaming_data_thread_ = NULL;
  pyExtension_ = NULL;
}

AudioFeedbackStream::~AudioFeedbackStream()
{
  this->stop();
}

AudioFeedbackStream * AudioFeedbackStream::instance()
{
  if (!s_pAudioFeedbackStream) {
    s_pAudioFeedbackStream = new AudioFeedbackStream();
  }
  return s_pAudioFeedbackStream;
}

void AudioFeedbackStream::initWithNode( NodeHandle * nodeHandle, PyModuleExtension * extension )
{
  mCtrlNode_ = nodeHandle;
  audioPub_ = mCtrlNode_->advertise<audio_common_msgs::AudioData>( "pyride/audio_feedback", 1 );
  pyExtension_ = extension;
}

void AudioFeedbackStream::addClient()
{
  clientNo_++;

  if (!isRunning_) {
    INFO_MSG( "Start the feedback streaming service.\n" );
    this->start();

    if (pyExtension_) {
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();

      PyObject * arg = Py_BuildValue( "(O)", Py_True );

      pyExtension_->invokeCallback( "onAudioFeedback", arg );

      Py_DECREF( arg );

      PyGILState_Release( gstate );
    }
  }
}

void AudioFeedbackStream::removeClient()
{
  if (clientNo_ <= 0) {
    ERROR_MSG( "AudioFeedbackStream::removeClient: no existing client available.\n" );
    return;
  }
  clientNo_--;

  if (clientNo_ == 0) {
    INFO_MSG( "No more client audio, stop the feedback streaming service.\n" );
    this->stop();

    if (pyExtension_) {
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();

      PyObject * arg = Py_BuildValue( "(O)", Py_False );

      pyExtension_->invokeCallback( "onAudioFeedback", arg );

      Py_DECREF( arg );

      PyGILState_Release( gstate );
    }
  }
}

bool AudioFeedbackStream::start()
{
  if (isRunning_) {
    WARNING_MSG( "Audio feedback streaming service is already running.\n" );
    return false;
  }

  if (!mCtrlNode_) {
    ERROR_MSG( "Audio feedback streaming service is not initialised.\n" );
    return false;
  }

  celtMode_ = celt_mode_create( PYRIDE_AUDIO_SAMPLE_RATE, PYRIDE_AUDIO_FRAME_SIZE, NULL );

  if (!celtMode_) {
    ERROR_MSG( "Unable to create encoding mode.\n" );
    return false;
  }

  audioDecoder_ = celt_decoder_create_custom( celtMode_, 1, NULL );

  dataStream_ = new RTPDataReceiver();
  dataStream_->init( PYRIDE_VIDEO_STREAM_BASE_PORT + 6, true );

  isRunning_ = true;

  streaming_data_thread_ = new boost::thread( &AudioFeedbackStream::grabAndDispatchAudioStreamData, this );

  return true;
}

void AudioFeedbackStream::stop()
{
  if (!isRunning_)
    return;

  isRunning_ = false;

  if (streaming_data_thread_) {
    streaming_data_thread_->join();
    delete streaming_data_thread_;
    streaming_data_thread_ = NULL;
  }

  if (audioDecoder_) {
    celt_decoder_destroy( audioDecoder_ );
    audioDecoder_ = NULL;
  }
  if (celtMode_) {
    celt_mode_destroy( celtMode_ );
    celtMode_ = NULL;
  }

  if (dataStream_) {
    delete dataStream_;
    dataStream_ = NULL;
  }
}

void AudioFeedbackStream::grabAndDispatchAudioStreamData()
{
  unsigned char * rawData = NULL;
  unsigned char * data = NULL;
  int dataSize = 0, rawDataSize = 0;
  bool dataSizeChanged = false;
  int audioFrames = 0;
  int decodedSize = 0;

  if (!dataStream_)
    return;

  unsigned char * audioData = new unsigned char[kMaxCachedAudioData * sizeof(short)];

  while (isRunning_) {
    int gcount = 0;

    do {
      rawDataSize = dataStream_->grabData( &rawData, dataSizeChanged );

      data = rawData;
      dataSize = rawDataSize;
      gcount++;
      usleep( 100 ); // 1ms
    } while (dataSize == 0 && gcount < 10);

    audio_common_msgs::AudioData msg;

    if (dataSize == 0 || dataSize % PYRIDE_AUDIO_BYTES_PER_PACKET != 0) {
      continue;
    }

    // double ts = double(now.tv_sec) + (double(now.tv_usec) / 1000000.0);

    audioFrames = dataSize / PYRIDE_AUDIO_BYTES_PER_PACKET;

    //DEBUG_MSG("Got audio frames %d.\n", audioFrames );

    for (int i = 0;i < audioFrames;i++) {
      celt_decode( audioDecoder_, data+i*PYRIDE_AUDIO_BYTES_PER_PACKET, PYRIDE_AUDIO_BYTES_PER_PACKET,
          (short *)(audioData) + i * PYRIDE_AUDIO_FRAME_SIZE, PYRIDE_AUDIO_FRAME_SIZE );
    }
    decodedSize = audioFrames * PYRIDE_AUDIO_FRAME_SIZE * sizeof( short );

    msg.data.resize( decodedSize );
    memcpy( &msg.data[0], audioData, decodedSize );

    audioPub_.publish( msg );
  }

  delete [] audioData;
}

} // namespace pyride
