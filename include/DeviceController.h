/*
 *  DeviceController.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef DEVICE_CONTROLLER_H
#define DEVICE_CONTROLLER_H

#ifdef __cplusplus

#include <ccrtp/rtp.h>
#include <cc++/config.h>
#include <vector>
#include <string>
#if defined ROS_BUILD || defined IOS_BUILD
#include <celt/celt.h>
#else
#include <celt.h>
#endif
#include "PyRideCommon.h"
#include "jdatabufferdst.h"

#ifdef ROS_BUILD
#define PYRIDE_SNAPSHOT_SAVE_DIRECTORY     "/removable/recordings/cameras"
#else
#define PYRIDE_SNAPSHOT_SAVE_DIRECTORY     "/home/nao/recordings/cameras"
#endif

namespace pyride {

using namespace std;
using namespace ost;

typedef struct {
  std::string deviceID;
  std::string deviceName;
  std::string deviceLabel;
  bool shouldBeActive;
  int index;
} DeviceInfo;

typedef std::vector< DeviceInfo *> DeviceInfoList;

class DeviceController 
{
public:
  DeviceController();
  virtual ~DeviceController();

  virtual bool initDevice() { return false; }
  virtual void finiDevice() {}
  virtual bool start() { return true; }
  virtual bool stop() { return true; }
  bool isInitialised() { return isInitialised_; }
  bool isStreaming() { return isStreaming_; }

  DeviceInfo & deviceInfo() { return devInfo_; }
  std::string & deviceLabel() { return devInfo_.deviceLabel; }

protected:
  bool isInitialised_;
  bool isStreaming_;
  int clientNo_;
  long packetStamp_;

  DeviceInfo devInfo_;
  
  RTPSession * streamSession_;
  
  bool getUDPSourcePorts( short & dataport, short & ctrlport );
  void dispatchData( const unsigned char * data, const int datalength );
};

class AudioDevice : public DeviceController
{
public:
  AudioDevice( int nofAudioChans );
  virtual ~AudioDevice();
  
  virtual bool start( struct sockaddr_in & cAddr, short cDataPort = PYRIDE_VIDEO_STREAM_BASE_PORT + 2 );
  virtual bool stop( struct sockaddr_in & cAddr, short cDataPort = PYRIDE_VIDEO_STREAM_BASE_PORT + 2 );
  
  void getAudioSettings( AudioSettings & settings );

protected:  
  AudioSettings aSettings_;

  virtual bool initWorkerThread() = 0;
  virtual void finiWorkerThread() = 0;

  void processAndSendAudioData( const signed short * data, const int nofSamples );

private:
  CELTMode * celtMode_;
  CELTEncoder * audioEncoder_;
  
  int nofAudioChans_;
  int nofEncodedFrames_;
  unsigned char * encodedAudio_;
};

class VideoDeviceDataHandler {
public:
  VideoDeviceDataHandler() {}
  virtual ~VideoDeviceDataHandler() {}

  virtual void onSnapshotImage( const string & imageName ) {}
};

class VideoDevice : public DeviceController
{
public:
  VideoDevice();
  virtual ~VideoDevice();

  virtual bool start( struct sockaddr_in & cAddr, short cDataPort = PYRIDE_VIDEO_STREAM_BASE_PORT );
  virtual bool stop( struct sockaddr_in & cAddr, short cDataPort = PYRIDE_VIDEO_STREAM_BASE_PORT );

  virtual void takeSnapshot( const VideoDeviceDataHandler * dataHandler ) {}
  
  void getVideoSettings( VideoSettings & settings );
  void setImageFormat( ImageFormat format ) { vSettings_.format = format; }

protected:
  VideoSettings vSettings_;
  VideoDeviceDataHandler * dataHandler_;

  virtual bool getDefaultVideoSettings() { return true; }
  virtual bool initWorkerThread() = 0;
  virtual void finiWorkerThread() = 0;

  void processAndSendImageData( const unsigned char * imageData, const int imageDataSize, ImageFormat format );
  void saveToJPEG( const unsigned char * imageData, const int imageDataSize, ImageFormat format );

  void setProcessParameters();
  
private:  
  unsigned char * outBuffer_;
  int outBufferSize_;
  
  struct jpeg_compress_struct cinfo_;
  struct jpeg_error_mgr jerr_;
  int imageWidth_;
  int imageHeight_;
  int imageSize_;

  bool YUV422ToRGB24( unsigned char * dstData, int dstDataSize, const unsigned char * srcData, const int srcDataSize );
  bool YUV422To24( unsigned char * dstData, int dstDataSize, const unsigned char * srcData, const int srcDataSize );
  bool RGBAToRGB24( unsigned char * dstData, int dstDataSize, const unsigned char * srcData, const int srcDataSize );
  bool BGRAToRGB24( unsigned char * dstData, int dstDataSize, const unsigned char * srcData, const int srcDataSize );

  int compressToJPEG( const unsigned char * imageData, const int imageDataSize,
                     unsigned char * & compressedData );
  int compressToHalf( const unsigned char * imageData, const int imageDataSize,
                     unsigned char * & compressedData );
  bool processImageData( const unsigned char * rawData, const int rawDataSize,
                    unsigned char * & outData, int & outDataSize,
                    ImageFormat format, bool compress );
};

typedef std::vector< AudioDevice * > AudioDeviceList;
typedef std::vector< VideoDevice * > VideoDeviceList;

} // namespace pyride

#endif // __cplusplus

#endif // DEVICE_CONTROLLER_H

