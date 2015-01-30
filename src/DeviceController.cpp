//
//  DeviceController.cpp
//  PyRIDE
//
//  Created by Xun Wang on 14/02/13.
//
//

#include "DeviceController.h"

namespace pyride {

static const int VideoStreamID = 101;
  
// a very stupid hack on port count
static int DeviceLocalCount = 0;

static inline unsigned char clamp( int pixel )
{
  if (pixel > 255)
    return 255;
  else if (pixel < 0)
    return 0;
  
  return pixel;
}

DeviceController::DeviceController()
{
  isInitialised_ = false;
  devInfo_.deviceID = "";
  devInfo_.deviceName = "";
  devInfo_.deviceLabel = "";
  devInfo_.shouldBeActive = false;
  devInfo_.index = 0;
  
  clientNo_ = 0;
  isStreaming_ = false;
  packetStamp_ = 0L;
  // this is to prevent a bad crash that occurs when we create a series of
  // RTPSessions with default setting in a quick sucession (on iOS)

  int myport = DeviceLocalCount++ * 2 + 1000 + PYRIDE_VIDEO_STREAM_BASE_PORT;
  //printf( "random port %d\n", myport );
  streamSession_ = new RTPSession( InetHostAddress( "0.0.0.0" ), myport );
}

DeviceController::~DeviceController()
{
  if (streamSession_) {
    delete streamSession_;
    streamSession_ = NULL;
  }
}
  
void DeviceController::dispatchData( const unsigned char * data, const int datalength )
{
  streamSession_->sendImmediate( packetStamp_++, data, datalength );
  // purge any data received from the clients (they are probably hole puncher)
  const AppDataUnit * adu = streamSession_->getData( streamSession_->getFirstTimestamp() );
  if (adu) {
    delete adu;
  }
}

bool DeviceController::getUDPSourcePorts( short & dataport, short & ctrlport )
{
  dataport = ctrlport = 0;

  if (!streamSession_)
    return false;

  // create UDP sockets
  SOCKET_T  controlSocket;
  SOCKET_T  dataSocket;
  struct sockaddr_in cAddr;
  struct sockaddr_in dAddr;
  struct sockaddr_in xAddr;

  bool found = false;
  
  int xLen = sizeof( xAddr );
  int maxFD = 0, localMaxFD, readLen;
  fd_set  masterFDSet;
  fd_set readyFDSet;
  
  FD_ZERO( &masterFDSet );
  unsigned char dataBuffer[200];
  unsigned char testData[] = "test";
  
  if (!streamSession_->addDestination( InetHostAddress( "0.0.0.0" ), PYRIDE_CONTROL_PORT )) {
    //ERROR_MSG( "getUDPSourcePorts:: unable to add loop source." );
    return found;
  }
  if ((controlSocket = socket( AF_INET, SOCK_DGRAM, 0 ) ) == INVALID_SOCKET ||
      (dataSocket = socket( AF_INET, SOCK_DGRAM, 0 ) ) == INVALID_SOCKET)
  {
    //ERROR_MSG( "getUDPSourcePorts:: unable to create UDP sockets." );
    return found;
  }
  
  cAddr.sin_family = AF_INET;
  cAddr.sin_addr.s_addr = INADDR_ANY;
  cAddr.sin_port = htons( PYRIDE_CONTROL_PORT + 1 );
  
  dAddr.sin_family = AF_INET;
  dAddr.sin_addr.s_addr = INADDR_ANY;
  dAddr.sin_port = htons( PYRIDE_CONTROL_PORT );
  
  if (bind( controlSocket, (struct sockaddr *)&cAddr, sizeof( cAddr ) ) < 0 ||
      bind( dataSocket, (struct sockaddr *)&dAddr, sizeof( dAddr ) ) < 0)
  {
    //ERROR_MSG( "getUDPSourcePorts: unable to bind to network interface." );
    close( controlSocket );
    close( dataSocket );
    return found;
  }
  
  maxFD = max( maxFD, controlSocket );
  FD_SET( controlSocket, &masterFDSet );
  maxFD = max( maxFD, dataSocket );
  FD_SET( dataSocket, &masterFDSet );
  
  while (!found) {
    streamSession_->sendImmediate( packetStamp_++, testData, 4 );
    
    FD_ZERO( &readyFDSet );
    memcpy( &readyFDSet, &masterFDSet, sizeof( masterFDSet ) );
    localMaxFD = maxFD;
    
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100ms
    select( localMaxFD + 1, &readyFDSet, NULL, NULL, &timeout );
    
    if (FD_ISSET( controlSocket, &readyFDSet )) {
      readLen = recvfrom( controlSocket, dataBuffer, 200,
                         0, (sockaddr *)&xAddr, (socklen_t *)&xLen );
      dataport = xAddr.sin_port;
    }
    if (FD_ISSET( dataSocket, &readyFDSet )) {
      readLen = recvfrom( dataSocket, dataBuffer, 20,
                         0, (sockaddr *)&xAddr, (socklen_t *)&xLen );
      ctrlport = xAddr.sin_port;
    }
    found = (dataport && ctrlport);
  }
  close( controlSocket );
  close( dataSocket );
  streamSession_->forgetDestination( InetHostAddress( "0.0.0.0" ),
                                    PYRIDE_CONTROL_PORT );
  return true;
  //DEBUG_MSG( "Got local source control/data port (%d, %d).", ntohs( dataport ),
  //          ntohs( ctrlport ) );
}

bool VideoDevice::YUV422ToRGB24( unsigned char * dstData, int dstDataSize, const unsigned char * srcData, const int srcDataSize )
{
  unsigned char * dataPtr = dstData;
  unsigned char * srcDataPtr = (unsigned char *)srcData;
  unsigned char u, v;
  int i;
  int newDataSize = srcDataSize + (srcDataSize >> 1);
  unsigned char * dend = dataPtr + dstDataSize;
  
  if (newDataSize != dstDataSize) // destination buffer is not what expected
    return false;
  
  while (dataPtr < dend) {
    u = srcDataPtr[1];
    v = srcDataPtr[3];
    
    for (i = 0; i < 2; i++) {
      *dataPtr++ = clamp( (int) *srcDataPtr + 1.140 * ((int) v - 128) );
      *dataPtr++ = clamp( (int) *srcDataPtr - 0.396 * ((int) u - 128) - 0.581 * ((int) v - 128) );
      *dataPtr++ = clamp( (int) *srcDataPtr + 2.029 * ((int) u - 128) );
      srcDataPtr += 2;
    }
  }
  return true;
}

bool VideoDevice::YUV422To24( unsigned char * dstData, int dstDataSize, const unsigned char * srcData, const int srcDataSize )
{
  unsigned char * dataPtr = dstData;
  unsigned char * srcDataPtr = (unsigned char *)srcData;
  //unsigned char y1, y2, u, v;
  //int i;
  int newDataSize = srcDataSize + (srcDataSize >> 1);
  unsigned char * dend = dataPtr + dstDataSize;
  
  if (newDataSize != dstDataSize) // destination buffer is not what expected
    return false;
  
  while (dataPtr < dend) {
    /*
     y1 = srcDataPtr[0];
     y2 = srcDataPtr[2];
     u = srcDataPtr[1];
     v = srcDataPtr[3];
     */
    *dataPtr++ = srcDataPtr[0];
    *dataPtr++ = srcDataPtr[1];
    *dataPtr++ = srcDataPtr[3];
    
    *dataPtr++ = srcDataPtr[2];
    *dataPtr++ = srcDataPtr[1];
    *dataPtr++ = srcDataPtr[3];
    
    srcDataPtr += 4;
  }
  return true;
}

bool VideoDevice::RGBAToRGB24( unsigned char * dstData, int dstDataSize, const unsigned char * srcData, const int srcDataSize )
{
  unsigned char * dataPtr = dstData;
  unsigned char * srcDataPtr = (unsigned char *)srcData;
  
  int newDataSize = srcDataSize * 3 / 4;
  unsigned char * dend = dataPtr + dstDataSize;
  
  if (newDataSize != dstDataSize) {// destination buffer is not what expected
    return false;
  }
  
  while (dataPtr < dend) {
    dataPtr[0] = srcDataPtr[0]; //(*srcDataPtr >> 16) & 0xff;
    dataPtr[1] = srcDataPtr[1];//(*srcDataPtr >> 8) & 0xff;
    dataPtr[2] = srcDataPtr[2];//*srcDataPtr & 0xff;

    dataPtr += 3;
    srcDataPtr += 4;
  }
  return true;
}

bool VideoDevice::BGRAToRGB24( unsigned char * dstData, int dstDataSize, const unsigned char * srcData, const int srcDataSize )
{
  unsigned char * dataPtr = dstData;
  unsigned char * srcDataPtr = (unsigned char *)srcData;
  
  int newDataSize = srcDataSize * 3 / 4;
  unsigned char * dend = dataPtr + dstDataSize;
  
  if (newDataSize != dstDataSize) {// destination buffer is not what expected
    return false;
  }
  
  while (dataPtr < dend) {
    dataPtr[0] = srcDataPtr[2]; //(*srcDataPtr >> 16) & 0xff;
    dataPtr[1] = srcDataPtr[1];//(*srcDataPtr >> 8) & 0xff;
    dataPtr[2] = srcDataPtr[0];//*srcDataPtr & 0xff;
    
    dataPtr += 3;
    srcDataPtr += 4;
  }
  return true;
}

VideoDevice::VideoDevice() :
  dataHandler_( NULL ),
  outBuffer_( NULL ),
  outBufferSize_( 0 ),
  imageWidth_( 320 ),
  imageHeight_( 240 ),
  imageSize_( 320 * 240 )
{
  
  vSettings_.fps = 5;
  vSettings_.format = RGB;
  vSettings_.resolution = 1; // 320x240
  vSettings_.reserved = 0;
  this->setProcessParameters();

  streamSession_->setPayloadFormat( DynamicPayloadFormat( VideoStreamID, 90000 ) );
  
  DualRTPUDPIPv4Channel * dataChan = streamSession_->getDSO();
  SOCKET_T sendSock = dataChan->getSendSocket();
  
  int optval = 50000;
  setsockopt( sendSock, SOL_SOCKET, SO_SNDBUF, (int *)&optval, sizeof( int ) );
  //streamSession_->setSessionBandwidth( 100000 );
  streamSession_->startRunning();
  this->getUDPSourcePorts( vSettings_.dataport, vSettings_.ctrlport );
}

VideoDevice::~VideoDevice()
{
  if (outBuffer_) { // initialised before
    delete [] outBuffer_;
    outBuffer_ = NULL;
    outBufferSize_ = 0;
  }

  jpeg_databuffer_free( &cinfo_ );
  jpeg_destroy_compress( &cinfo_ );
}

void VideoDevice::setProcessParameters()
{
  if (vSettings_.resolution >= 0 && vSettings_.resolution < 3) {
    imageWidth_ = kSupportedCameraQuality[(int)vSettings_.resolution].width;
    imageHeight_ = kSupportedCameraQuality[(int)vSettings_.resolution].height;
    imageSize_ = imageWidth_ * imageHeight_;
  }

  if (outBuffer_) { // initialised before
    delete [] outBuffer_;
    outBuffer_ = NULL;
    outBufferSize_ = 0;

    jpeg_databuffer_free( &cinfo_ );
    jpeg_destroy_compress( &cinfo_ );
  }
  outBufferSize_ = imageWidth_ * imageHeight_ * 3;
  outBuffer_ = new unsigned char[outBufferSize_];    

  cinfo_.err = jpeg_std_error( &jerr_ );
  jpeg_create_compress( &cinfo_ );
  jpeg_databuffer_dest( &cinfo_ );
  cinfo_.image_width = imageWidth_;
  cinfo_.image_height = imageHeight_;
  cinfo_.input_components = 3;
  cinfo_.in_color_space = JCS_RGB;
  jpeg_set_defaults( &cinfo_ );
  jpeg_set_quality( &cinfo_, kCompressionRate[vSettings_.resolution], TRUE );
}

bool VideoDevice::start( struct sockaddr_in & cAddr, short cDataPort )
{
  bool retVal = false;
  
  if (!isInitialised_)
    return retVal;
  
  if (streamSession_->addDestination( InetHostAddress( cAddr.sin_addr ), cDataPort )) {
    clientNo_++;
    retVal = true;
  }
  
  if (isStreaming_)
    return retVal;
  
  isStreaming_ = true;

  if (!this->initWorkerThread()) {
    isStreaming_ = false;
    return isStreaming_;
  }

  return retVal;
}

bool VideoDevice::stop( struct sockaddr_in & cAddr, short cDataPort )
{
  bool retVal = false;
  
  if (!isInitialised_ || !isStreaming_)
    return retVal;
  
  if (streamSession_->forgetDestination( InetHostAddress( cAddr.sin_addr ), cDataPort )) {
    clientNo_--;
    retVal = true;
  }

  isStreaming_ = (clientNo_ > 0);
  
  if (!isStreaming_) {
    this->finiWorkerThread();
  }
  return retVal;
}

void VideoDevice::getVideoSettings( VideoSettings & settings )
{
  memcpy( (void *)&settings, (void *)&vSettings_, sizeof( VideoSettings ) );
}

bool VideoDevice::processImageData( const unsigned char * rawData, const int rawDataSize,
                          unsigned char * & outData, int & outDataSize,
                          ImageFormat format, bool compress )
{
  unsigned char * myDataPtr = NULL;
  int myDataSize = 0;

  switch (format) {
    case RAW:
      if (vSettings_.format == RAW) {
        if (!YUV422To24( outBuffer_, outBufferSize_, rawData, rawDataSize ))
          return false;
        myDataPtr = outBuffer_;
        myDataSize = outBufferSize_;
      }
      else if (vSettings_.format == RGB) {
        if (!YUV422ToRGB24( outBuffer_, outBufferSize_, rawData, rawDataSize ))
          return false;
        myDataPtr = outBuffer_;
        myDataSize = outBufferSize_;
      }
      else {
        return false;
      }
      break;
    case RGB:
      if (vSettings_.format != RGB) {
        return false;
      }
      myDataPtr = (unsigned char *)rawData;
      myDataSize = rawDataSize;
      break;
    case RGBA:
      if (vSettings_.format == RGB) {
        if (!RGBAToRGB24( outBuffer_, outBufferSize_, rawData, rawDataSize ))
          return false;
        myDataPtr = outBuffer_;
        myDataSize = outBufferSize_;
      }
      else {
        return false;
      }
      break;
    case PROCESSED:
      if (vSettings_.format != PROCESSED) {
        return false;
      }
      myDataPtr = (unsigned char *)rawData;
      myDataSize = rawDataSize;
      break;
    default:
      return false;
      break;
  }

  if (compress) {
    if (format == PROCESSED) {
      outDataSize = compressToHalf( myDataPtr, myDataSize, outData );
    }
    else {
      outDataSize = compressToJPEG( myDataPtr, myDataSize, outData );
    }
  }
  else {
    outData = myDataPtr;
    outDataSize = myDataSize;
  }
  return (outDataSize > 0);
}

int VideoDevice::compressToJPEG( const unsigned char * imageData, const int imageDataSize, unsigned char * & compressedData )
{
  JSAMPROW rowPtr[1];
  unsigned char * data = (unsigned char *)imageData;
  jpeg_start_compress( &cinfo_, TRUE );
  
  int rowStride = imageWidth_ * 3;
  while (cinfo_.next_scanline < (size_t)imageHeight_) {
    /* jpeg_write_scanlines expects an array of pointers to scanlines.
     * Here the array is only one element long, but you could pass
     * more than one scanline at a time if that's more convenient.
     */
    rowPtr[0] = &data[cinfo_.next_scanline * rowStride];
    (void) jpeg_write_scanlines( &cinfo_, rowPtr, 1 );
  }
  
  jpeg_finish_compress( &cinfo_ );
  
  int compressedDataSize;
  compressedData = get_jpeg_data_and_size( &cinfo_, &compressedDataSize );
  
  return compressedDataSize;
}

void VideoDevice::processAndSendImageData( const unsigned char * imageData, const int imageDataSize, ImageFormat format )
{
  int pDataSize = 0;
  unsigned char * pData = NULL;

  if (processImageData( imageData, imageDataSize, pData, pDataSize, format, true )) {
    this->dispatchData( pData, pDataSize );
    //printf( "dispatch size %d\n", pDataSize );
  }
}
  
void VideoDevice::saveToJPEG( const unsigned char * imageData, const int imageDataSize, ImageFormat format )
{
  int pDataSize = 0;
  unsigned char * pData = NULL;
  
  if (!processImageData( imageData, imageDataSize, pData, pDataSize, format, false )) {
    return;
  }
  //save to file
  FILE * outfile = NULL;
  char filename[200];
  
  struct timeval now;
  gettimeofday( &now, NULL );
  struct tm * lt = localtime( &(now.tv_sec) );
  
  sprintf( filename, "%s/%s_snapshot_%02d%02d%02d_%02d%02d%02d.jpg", PYRIDE_SNAPSHOT_SAVE_DIRECTORY, devInfo_.deviceLabel.c_str(),
          1900+lt->tm_year, lt->tm_mon + 1, lt->tm_mday, lt->tm_hour, lt->tm_min, lt->tm_sec );
  
  if ((outfile = fopen( filename, "wb" )) == NULL) {
    ERROR_MSG( "Unable to save a snapshot!\n" );
    return;
  }
  // DO a second compression
  struct jpeg_compress_struct cinfo;
  struct jpeg_error_mgr jerr;
  
  cinfo.err = jpeg_std_error( &jerr );
  jpeg_create_compress( &cinfo );
  cinfo.image_width = imageWidth_;
  cinfo.image_height = imageHeight_;
  cinfo.input_components = 3;
  cinfo.in_color_space = cinfo_.in_color_space;
  jpeg_set_defaults( &cinfo );
  jpeg_set_quality( &cinfo, 70, TRUE );
  jpeg_stdio_dest( &cinfo, outfile );
  
  JSAMPROW rowPtr[1];
  unsigned char * data = pData;
  
  jpeg_start_compress( &cinfo, TRUE );
  
  int rowStride = imageWidth_ * 3;
  while (cinfo.next_scanline < (size_t)imageHeight_) {
    /* jpeg_write_scanlines expects an array of pointers to scanlines.
     * Here the array is only one element long, but you could pass
     * more than one scanline at a time if that's more convenient.
     */
    rowPtr[0] = &data[cinfo.next_scanline * rowStride];
    (void) jpeg_write_scanlines( &cinfo, rowPtr, 1 );
  }
  
  jpeg_finish_compress( &cinfo );
  fclose( outfile );
  jpeg_destroy_compress( &cinfo );

  if (dataHandler_) {
    dataHandler_->onSnapshotImage( string( filename ) );
    dataHandler_ = NULL;
  }
}

int VideoDevice::compressToHalf( const unsigned char * imageData, const int imageDataSize, unsigned char * & compressedData )
{
  if (imageDataSize != imageSize_)
    return 0;
  
  int halfSize = imageSize_ >> 1;
  
  unsigned char * imageDataPtr = (unsigned char *)imageData;
  
  for (int i = 0; i < halfSize; i++) {
    outBuffer_[i] = imageDataPtr[0] << 4 | (imageDataPtr[1] & 0xf);
    imageDataPtr += 2;
  }
  
  compressedData = outBuffer_;
  
  return halfSize;
}

AudioDevice::AudioDevice( int nofAudioChans ) :
  nofAudioChans_( 1 )
{
  aSettings_.channels = nofAudioChans_;
  aSettings_.sampling = PYRIDE_AUDIO_SAMPLE_RATE;
  aSettings_.reserved = 0;

  celtMode_ = celt_mode_create( PYRIDE_AUDIO_SAMPLE_RATE, PYRIDE_AUDIO_FRAME_SIZE, NULL );
  
  if (nofAudioChans > 0 && nofAudioChans < 5)
    nofAudioChans_ = nofAudioChans;
  
  audioEncoder_ = celt_encoder_create_custom( celtMode_, nofAudioChans_, NULL );
  
  nofEncodedFrames_ = 32;
  
  encodedAudio_ = new unsigned char[PYRIDE_AUDIO_BYTES_PER_PACKET*nofEncodedFrames_];

  streamSession_->setPayloadFormat( StaticPayloadFormat( sptPCMU ) );
  streamSession_->startRunning();
  this->getUDPSourcePorts( aSettings_.dataport, aSettings_.ctrlport );
}

AudioDevice::~AudioDevice()
{
  if (encodedAudio_) {
    delete [] encodedAudio_;
    encodedAudio_ = NULL;
  }

  celt_encoder_destroy( audioEncoder_ );
  celt_mode_destroy( celtMode_ );

  nofEncodedFrames_ = 0;
}

bool AudioDevice::start( struct sockaddr_in & cAddr, short cDataPort )
{
  bool retVal = false;
  
  if (!isInitialised_)
    return retVal;
  
  if (streamSession_->addDestination( InetHostAddress( cAddr.sin_addr ), cDataPort )) {
    clientNo_++;
    retVal = true;
  }
  
  if (isStreaming_)
    return retVal;
  
  isStreaming_ = true;

  if (!this->initWorkerThread()) {
    isStreaming_ = false;
    return isStreaming_;
  }
  
  return retVal;
}

bool AudioDevice::stop( struct sockaddr_in & cAddr, short cDataPort )
{
  bool retVal = false;
  
  if (!isInitialised_ || !isStreaming_)
    return retVal;
  
  if (streamSession_->forgetDestination( InetHostAddress( cAddr.sin_addr ), cDataPort )) {
    clientNo_--;
    retVal = true;
  }
  
  isStreaming_ = (clientNo_ > 0);
  
  if (!isStreaming_) {
    this->finiWorkerThread();
  }
  return retVal;
}

void AudioDevice::getAudioSettings( AudioSettings & settings )
{
  memcpy( (void *)&settings, (void *)&aSettings_, sizeof( AudioSettings ) );
}

void AudioDevice::processAndSendAudioData( const signed short * data, const int nofSamples )
{
  int dataFrames = nofSamples / PYRIDE_AUDIO_FRAME_SIZE;
  if (nofEncodedFrames_ < dataFrames) {
    nofEncodedFrames_ = dataFrames;
    delete [] encodedAudio_;
    encodedAudio_ = new unsigned char[PYRIDE_AUDIO_BYTES_PER_PACKET*nofEncodedFrames_];
  }
  
  signed short * audioDataPtr = (short *)data;
  unsigned char * encodedDataPtr = encodedAudio_;
  
  int elen = 0;
  
  if (dataFrames > 0) {
    for (int i = 0;i < dataFrames; i++) {
      elen = celt_encode( audioEncoder_, audioDataPtr, PYRIDE_AUDIO_FRAME_SIZE,
                         encodedDataPtr, PYRIDE_AUDIO_BYTES_PER_PACKET );
      encodedDataPtr += elen;
      audioDataPtr += PYRIDE_AUDIO_FRAME_SIZE * nofAudioChans_;
    };
    this->dispatchData( encodedAudio_, encodedDataPtr - encodedAudio_ );
  }
}

} // namespace pyride