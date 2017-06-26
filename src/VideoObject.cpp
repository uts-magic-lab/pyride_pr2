/*
 *  VideoObject.cpp
 *  PyRIDE
 *
 *  Created by Xun Wang on 9/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <time.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "PyRideCommon.h"
#include "VideoObject.h"

namespace pyride {

static inline long timediff_usec( timespec start, timespec end )
{
  if ((end.tv_nsec - start.tv_nsec) < 0) {
    return (end.tv_sec-start.tv_sec - 1) * 1E06 + (1E09 + end.tv_nsec - start.tv_nsec) / 1E03;
  }
  else {
    return (end.tv_sec - start.tv_sec) * 1E06 + (end.tv_nsec - start.tv_nsec) / 1E03;
  }
}

VideoObject::VideoObject( DeviceInfo & devInfo ) :
  imgTrans_( priImgNode_ ),
  procThread_( NULL ),
  takeSnapShot_( false )
{
  devInfo_ = devInfo;
  priImgNode_.setCallbackQueue( &imgQueue_ );
}

bool VideoObject::initDevice()
{
  if (isInitialised_)
    return isInitialised_;

  if (devInfo_.deviceID.empty()) {
    ERROR_MSG( "Invalid device info." );
    return false;
  }

  // get default video settings
  if (!this->getDefaultVideoSettings()) {
    return false;
  }

  procThread_ = new AsyncSpinner( 1, &imgQueue_ );
  procThread_->start();

  packetStamp_ = 0;
  clientNo_ = 0;
  isInitialised_ = true;

  INFO_MSG( "Robot camera %s is successfully initialised.\n", devInfo_.deviceLabel.c_str() );

  return true;
}

bool VideoObject::initWorkerThread()
{
  if (!procThread_) {
    ERROR_MSG( "Unable to ceate thread to grab images from camera %s.\n",
              devInfo_.deviceLabel.c_str() );
    return false;
  }
  image_transport::TransportHints hints( "compressed" );
  imgSub_ = imgTrans_.subscribe( devInfo_.deviceID, 1, &VideoObject::continueProcessing, this,
		  hints );
  
  streaming_data_thread_ = new boost::thread( &VideoObject::doImageStreaming, this );

  return true;
}

void VideoObject::finiWorkerThread()
{
  if (streaming_data_thread_) {
    streaming_data_thread_->join();
    delete streaming_data_thread_;
    streaming_data_thread_ = NULL;
    imgSub_.shutdown();
  }
}
  
void VideoObject::doImageStreaming()
{
  cv_bridge::CvImagePtr cv_ptr;
  //long tick = 0;
  timespec time1, time2;
  long interval = long(1.0 / (double)vSettings_.fps * 1E6);
  long proctime = 0;

  while (isStreaming_) {
    //tick = cv::getTickCount();
    clock_gettime( CLOCK_MONOTONIC, &time1 );
    {
      boost::mutex::scoped_lock lock( mutex_ );
      while (!imgMsgPtr_) {
        // prevent this thread goes into infinite loop and
        // still respond to external control, if the input data
        // stream is dead.
        if (isStreaming_)
          imageCon_.wait( lock );
        else
          break;
      }
      try {
        cv_ptr = cv_bridge::toCvCopy( imgMsgPtr_, sensor_msgs::image_encodings::RGB8 );
      }
      catch (cv_bridge::Exception & e) {
        imgMsgPtr_.reset();
        ROS_ERROR( "Unable to convert image message to mat." );
        continue;
      }
    }
    processAndSendImageData( cv_ptr->image.data, cv_ptr->image.total()*cv_ptr->image.elemSize(), RGB );
    
    if (takeSnapShot_) {
      saveToJPEG( cv_ptr->image.data, cv_ptr->image.total()*cv_ptr->image.elemSize(), RGB );
      takeSnapShot_ = false;
    }
    clock_gettime( CLOCK_MONOTONIC, &time2 );
    proctime = timediff_usec( time1, time2 );
    //printf( "processing time %li usec\n",  proctime);
    if (interval > proctime)
      usleep( interval - proctime );

    //usleep( long((1.0 / (double)vSettings_.fps - double(cv::getTickCount() - tick) / cv::getTickFrequency()) * 1E6) );
  }
}

void VideoObject::continueProcessing( const sensor_msgs::ImageConstPtr& msg )
{
  // assume we cannot control the framerate (i.e. default 30FPS)
  boost::mutex::scoped_lock lock( mutex_, boost::try_to_lock );
  
  if (lock) {
    imgMsgPtr_ = msg;

    if (takeSnapShot_ && !isStreaming_) {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::RGB8 );
      saveToJPEG( cv_ptr->image.data, cv_ptr->image.total()*cv_ptr->image.elemSize(), RGB );
      takeSnapShot_ = false;
      imgSub_.shutdown();
    }
    else {
      imageCon_.notify_one();
    }
  }
}

void VideoObject::takeSnapshot( const VideoDeviceDataHandler * dataHandler )
{
  dataHandler_ = (VideoDeviceDataHandler *)dataHandler;

  takeSnapShot_ = true;
  if (!isStreaming_) {
    imgSub_ = imgTrans_.subscribe( devInfo_.deviceID, 1,
                                  &VideoObject::continueProcessing, this );
  }
}

bool VideoObject::getDefaultVideoSettings()
{
  vSettings_.fps = 10;
  vSettings_.format = RGB;
  vSettings_.resolution = 2; // 640x480
  vSettings_.reserved = 0;
  
  this->setProcessParameters();
  
  return true;
}

void VideoObject::finiDevice()
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
