//
//  PyRideNetComm.h
//  PyRIDE
//
//  Created by Xun Wang on 10/05/10.
//  Copyright 2010 GalaxyNetwork. All rights reserved.
//

#ifndef PyRideNetComm_h_DEFINED
#define PyRideNetComm_h_DEFINED

#include "PyRideCommon.h"

#ifdef __cplusplus

#ifdef WIN32
#include <winbase.h>
#include <process.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <netdb.h>
#include <netinet/in.h>
#include <pthread.h>
#endif

#ifndef PYRIDE_REMOTE_CLIENT
#include "DeviceController.h"
#endif

namespace pyride {

class PyRideNetComm;

typedef struct {
  PyRideNetComm * mainObj;
  void * timerObj;
} timerExecuteData;

class RobotDataHandler
{
public:
#ifdef PYRIDE_REMOTE_CLIENT
  RobotDataHandler() { clientID_ = 0; }
  virtual ~RobotDataHandler() {}

  virtual void setImageFormat( const char cID, ImageFormat format ) = 0;
#else
  RobotDataHandler() { clientID_ = 0; telemetryClients_ = 0; }
  virtual const RobotInfo & defaultRobotInfo() = 0;
#endif

  char clientID() { return clientID_; }

protected:
  char clientID_;
  virtual void onTimer( const long tID ) {}
  virtual void onTimerLapsed( const long tID ) {}

#ifdef PYRIDE_REMOTE_CLIENT
  virtual void onRobotCreated( const char cID, const int ipAddr, const RobotInfo * rinfo,
                            const VideoSettings * vsettings, const AudioSettings * asettings,
                            const unsigned char * optLabel,
                            const int optLabelLength ) = 0;
  virtual void onRobotDestroyed( const char cID ) = 0;
  virtual void onRobotTelemetryData( const char cID, const RobotPose * pose, const FieldObject * objects = NULL,
                          const int nofObjs = 0 ) = 0;
  virtual void onTelemetryStreamStart( const char cID ) = 0;
  virtual void onTelemetryStreamStop( const char cID ) = 0;
  virtual void onImageStreamStart( const char cID ) = 0;
  virtual void onImageStreamStop( const char cID ) = 0;
  virtual void onImageFormatChange( const char cID, ImageFormat format ) = 0;
  virtual void onVideoSwitchChange( const char cID, const VideoSettings * vsettings ) = 0;
  virtual void onExtendedCommandResponse( const char cID, const PyRideExtendedCommand command,
                                         const unsigned char * optionalData = NULL,
                                         const int optionalDataLength = 0 ) = 0;
                                         
  virtual void onOperationalData( const char cID, const int status,
                                 const unsigned char * optionalData = NULL,
                                 const int optionalDataLength = 0 ) = 0;
#else
  int telemetryClients_;
  
  virtual void onTelemetryStreamControl( bool isStart ) {};

  virtual bool executeRemoteCommand( const unsigned char * commandData, const int dataLength ) = 0;
  virtual void cancelCurrentOperation() = 0;
  virtual bool onUserLogOn( const unsigned char * authCode, SOCKET_T fd, struct sockaddr_in & addr ) { return false; }
  virtual void onUserLogOff( SOCKET_T fd ) {}
#endif

private:
#ifdef PYRIDE_REMOTE_CLIENT
#else
  void setTelemetryClients( int nofClients ) { telemetryClients_ = nofClients; }
#endif

  friend class PyRideNetComm;
};

class PyRideNetComm
{
public:
  PyRideNetComm( RobotDataHandler * pDataHandler );
  ~PyRideNetComm();

#ifdef PYRIDE_REMOTE_CLIENT
  void init();
#else
  void init( const VideoDeviceList & videoObjs,
            const AudioDeviceList & audioObjs );
#endif
  void fini();

  void startProcessing();
  void continuousProcessing();
  void processTimer( void * data );
  void stopProcessing();

#ifdef PYRIDE_REMOTE_CLIENT
  void discoverRobots();
  bool logonToRobot( const char * host, const unsigned char * authCode );
  void disconnectRobots();
  void startTelemetryStream( const char cID = 0 );
  void stopTelemetryStream();
  void startCameraImageStream( const char cID );
  void stopCameraImageStream( const char cID );
  void cancelCurrentOperation( const char cID );
  void issueExtendedCommand( const char cID, const PyRideExtendedCommand command,
                            const unsigned char * optionalData , const int optionalDataLength );
  void setImageFormat( const char cID, ImageFormat format );
  void switchCamera( const char cID, const char vID );
#else
  void declareRobot( const RobotInfo * robotInfo );
  void disconnectConsoles();
  void dispatchTelemetryData( const unsigned char * data, const int size );
  void dispatchStatusData( const unsigned char * data, const int size );
  void takeCameraSnapshot( const VideoDeviceDataHandler * dataHandler, bool allCameras = false );
  
  void blockRemoteExclusiveControl( bool isYes );
#endif

  void disconnectClientWithFD( SOCKET_T fd );
  bool getIDFromIP( int & addr );
  bool getMyIPAddress( int & addr );
  long addTimer( int initialTime, long repeats = 0, int interval = 1 );
  void delTimer( long tID );
  bool isTimerRunning( long tID );
  long totalTimers() { return timerCount_; }
  void delAllTimers();

private:
  struct SocketDataBufferInfo {
    unsigned char * bufferedData;
    int expectedDataLength;
    int bufferedDataLength;
  };
  typedef struct sClientItem {
    SOCKET_T fd;
    char cID;
    struct sockaddr_in addr;
    struct SocketDataBufferInfo dataInfo;
    bool pushData;
#ifndef PYRIDE_REMOTE_CLIENT
    bool pushImage;
    AudioDevice * activeAudioObj;
    VideoDevice * activeVideoObj;
#endif
    struct sClientItem * pNext;
  } ClientItem;

  typedef struct sTimerObj {
    long tID;
    long remainCount; // -1 for infinite
    int interval;
    time_t nextTrigTime;
    bool isExecuting;
#ifdef WIN32
    HANDLE timerThread;
#else
    pthread_t timerThread;
#endif
    struct sTimerObj * pNext;
  } TimerObj;

#ifdef WIN32
  CRITICAL_SECTION t_criticalSection_;
  CRITICAL_SECTION timer_criticalSection_;
#else
  pthread_mutex_t t_mutex_;
  pthread_mutex_t timer_mutex_;
  pthread_mutexattr_t t_mta;
#endif

  RobotDataHandler * pDataHandler_;

#ifndef PYRIDE_REMOTE_CLIENT
#ifdef WIN32
  HANDLE runThread_;
#else
  pthread_t runThread_;
#endif
  VideoDeviceList * activeVideoObjs_;
  AudioDeviceList * activeAudioObjs_;
  ClientItem * exclusiveCtrlClient_;
#endif

  struct sockaddr_in  sAddr_;
  struct sockaddr_in  bcAddr_;

  SOCKET_T  udpSocket_;
  SOCKET_T  tcpSocket_;

  unsigned char * dgramBuffer_;
  unsigned char * clientDataBuffer_;
  unsigned char * dispatchDataBuffer_;
  
  ClientItem * clientList_;
  
  TimerObj * timerList_;
  TimerObj * lastTimer_;
  
  long timerCount_;
  long nextTimerID_;

  int     maxFD_;
  fd_set  masterFDSet_;

  bool  netCommEnabled_;
  bool  keepRunning_;

  PyRideNetComm();
  bool initUDPListener();
  bool initTCPListener();

  void processIncomingData( fd_set * readyFDSet );
  void checkTimers();
  void processUDPInput( const unsigned char * recBuffer, int recBytes, struct sockaddr_in & cAddr );
  void processDataInput( ClientItem * client, const unsigned char * receivedMesg,
                        const int receivedBytes );
#ifdef PYRIDE_REMOTE_CLIENT
  void processTelemetryData( ClientItem * client, const unsigned char * commandData,
                            int dataLen );
  void processOperationalData( ClientItem * client, const unsigned char * commandData,
                            int dataLen );
  void processRobotResponse( ClientItem * client, int subcommand, 
                            const unsigned char * commandData, int dataLen );
#else
  void declareRobot( const RobotInfo * robotInfo, ClientItem * client );
  void processConsoleCommand( ClientItem * client, int subcommand, 
                            const unsigned char * commandData, int dataLen );
  int calcTelemetryClients();
#endif

  void clientDataSend( const int command, const int subcommand = 0, 
                         const unsigned char * optionalData = NULL,
                         const int optionalDataLen = 0,
                         ClientItem * client = NULL, bool useUDP = false, bool forceAll = false );

  bool messageValidation( const unsigned char * receivedMesg, const int receivedBytes, char & cID,
                         char & command, char & subcommand );
  void disconnectClient( ClientItem * client, bool sendNotification = false );
  
  void cleanupClient( ClientItem * client );

  ClientItem * createTCPTalker( struct sockaddr_in & cAddr );
  ClientItem * findClientFromClientList( struct sockaddr_in & cAddr );
  ClientItem * findClientFromClientList( const char cID );
  ClientItem * findClientFromClientList( SOCKET_T fd );
  ClientItem * addFdToClientList( const SOCKET_T & fd, struct sockaddr_in & cAddr );
  void initIPAddresses();
  bool isNonExclusiveCommand( PyRideExtendedCommand cmd );
};
} // namespace pyride
#endif // __cplusplus
#endif  // PyRideNetComm_h_DEFINED
