//
//  PyRideNetComm.cpp
//  PyRIDE
//
//  Created by Xun Wang on 10/05/10.
//  Copyright 2010 GalaxyNetwork. All rights reserved.
//
#ifdef WIN32
#pragma warning( disable: 4068 4018 )
#endif

#ifndef WIN32
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/sysctl.h>
#include <sys/ioctl.h>
#include <errno.h>
#include <sys/time.h>
#define max( a, b ) (a > b) ? a : b

#endif

#include <openssl/sha.h>

#include "PyRideNetComm.h"

namespace pyride {

static int kTCPMSS = 1024;

#ifdef WIN32
unsigned __stdcall robotmodule_thread( void * processor )
#else
void * robotmodule_thread( void * processor )
#endif
{
  ((PyRideNetComm *)processor)->continuousProcessing();
  return NULL;
}

#ifdef WIN32
unsigned __stdcall robottimer_thread( void * exeData )
#else
void * robottimer_thread( void * exeData )
#endif
{
  timerExecuteData * threadData = (timerExecuteData *)exeData;
  PyRideNetComm * process = threadData->mainObj;
  void * data = threadData->timerObj;
  delete threadData;

  process->processTimer( data );
  return NULL;
}

PyRideNetComm::PyRideNetComm( RobotDataHandler * pDataHandler ) :
  pDataHandler_( pDataHandler ),
#ifndef PYRIDE_REMOTE_CLIENT
  runThread_( (pthread_t)NULL ),
  activeVideoObjs_( NULL ),
  activeAudioObjs_( NULL ),
#endif
  udpSocket_( INVALID_SOCKET ),
  tcpSocket_( INVALID_SOCKET ),
  dgramBuffer_( NULL ),
  clientDataBuffer_( NULL ),
  dispatchDataBuffer_( NULL ),
  clientList_( NULL ),
  timerList_( NULL ),
  lastTimer_( NULL ),
  timerCount_( 0 ),
  nextTimerID_( 1 ),
  maxFD_( 0 ),
  netCommEnabled_( false ),
  keepRunning_( false )
{
#ifdef WIN32
  InitializeCriticalSection( &t_criticalSection_ );
  InitializeCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutexattr_init( &t_mta );
  pthread_mutexattr_settype( &t_mta, PTHREAD_MUTEX_RECURSIVE );
  pthread_mutex_init( &t_mutex_, &t_mta );
  pthread_mutex_init( &timer_mutex_, &t_mta );
#endif
}

PyRideNetComm::~PyRideNetComm()
{
#ifdef WIN32
  DeleteCriticalSection( &t_criticalSection_ );
  DeleteCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_destroy( &timer_mutex_ );
  pthread_mutex_destroy( &t_mutex_ );
  pthread_mutexattr_destroy( &t_mta );
#endif
}

void PyRideNetComm::startProcessing()
{
  if (keepRunning_)
    return;
  
  keepRunning_ = true;

#ifndef PYRIDE_REMOTE_CLIENT
#ifdef WIN32
  runThread_ = (HANDLE)_beginthreadex( NULL, 0, &robotmodule_thread, this, 0, NULL );
  if (runThread_ == 0) {
    ERROR_MSG( "Unable to create thread to mainloop.\n" );
    return;
  }
#else
  if (pthread_create( &runThread_, NULL, robotmodule_thread, this ) ) {
    ERROR_MSG( "Unable to create thread to mainloop.\n" );
    return;
  }
#endif
#endif
}

void PyRideNetComm::stopProcessing()
{
  if (!keepRunning_)
    return;
  
  keepRunning_ = false;

#ifndef PYRIDE_REMOTE_CLIENT
#ifdef WIN32
    WaitForSingleObject( runThread_, INFINITE );;
    CloseHandle( runThread_ );
#else
  pthread_join( runThread_, NULL ); // allow thread to exit
#endif
  runThread_ = (pthread_t)NULL;
#endif
  this->delAllTimers();
}

#ifdef PYRIDE_REMOTE_CLIENT
void PyRideNetComm::init()
#else
void PyRideNetComm::init( const VideoDeviceList & videoObjs,
                            const AudioDeviceList & audioObjs )
#endif
{
  if (netCommEnabled_)
    return;

#ifdef USE_ENCRYPTION
  endecryptInit();
#endif

  FD_ZERO( &masterFDSet_ );

  initIPAddresses();
#ifdef NO_AUTO_DISCOVERY
  netCommEnabled_ = true;
#else
  netCommEnabled_ = initUDPListener();
  netCommEnabled_ &= initTCPListener();
#endif

#ifndef PYRIDE_REMOTE_CLIENT
  exclusiveCtrlClient_ = NULL;
  // assume all video and audio objects have been initialised.
  activeVideoObjs_ = (VideoDeviceList *)&videoObjs;
  activeAudioObjs_ = (AudioDeviceList *)&audioObjs;
  if (activeVideoObjs_->size() == 0) {
    WARNING_MSG( "No active video devices!\n" );
  }
#endif

  if (clientDataBuffer_ == NULL)
    clientDataBuffer_ = new unsigned char[PYRIDE_DEFAULT_BUFFER_SIZE];

  if (dispatchDataBuffer_ == NULL)
    dispatchDataBuffer_ = new unsigned char[PYRIDE_MSG_BUFFER_SIZE];

  if (netCommEnabled_) {
    startProcessing();
  }
  else {
    ERROR_MSG( "PyRideNetComm::enableNetComm: Unable to initialise network "
              "communication layer!\n" );
  }
}

bool PyRideNetComm::initUDPListener()
{
  if ((udpSocket_ = socket( AF_INET, SOCK_DGRAM, 0 ) ) == INVALID_SOCKET) {
    ERROR_MSG( "PyRideNetComm::initUDPListener: unable to create UDP socket.\n" );
    return false;
  }
  // setup broadcast option
  int turnon = 1;
#ifdef USE_MULTICAST
  char ttl = 30;
  if (setsockopt( udpSocket_, IPPROTO_IP, IP_MULTICAST_TTL, (char *)&ttl, sizeof( ttl ) ) < 0) {
    ERROR_MSG( "PyRideNetComm::initUDPListener: failed to set multicast TTL on UDP socket.\n" );
#ifdef WIN32
    closesocket( udpSocket_ );
#else
    close( udpSocket_ );
#endif
    return false;
  }
  if (setsockopt( udpSocket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&multiCastReq_, sizeof( multiCastReq_ ) ) < 0) {
    ERROR_MSG( "PyRideNetComm::initUDPListener: failed to join multicast group on UDP socket.\n" );
    close( udpSocket_ );
    return false;
  }
#else // !USE_MULTICAST
  if (setsockopt( udpSocket_, SOL_SOCKET, SO_BROADCAST, (char *)&turnon, sizeof( turnon ) ) < 0) {
    ERROR_MSG( "PyRideNetComm::initUDPListener: failed to enable broadcast on UDP socket.\n" );
#ifdef WIN32
    closesocket( udpSocket_ );
#else
    close( udpSocket_ );
#endif
    return false;
  }
#endif

  if (setsockopt( udpSocket_, SOL_SOCKET, SO_REUSEADDR, (char *)&turnon, sizeof( turnon ) ) < 0) {
    ERROR_MSG( "PyRideNetComm::initUDPListener: failed to enable reuse address on UDP socket.\n" );
#ifdef WIN32
    closesocket( udpSocket_ );
#else
    close( udpSocket_ );
#endif
    return false;
  }

#ifdef SO_REUSEPORT
  setsockopt( udpSocket_, SOL_SOCKET, SO_REUSEPORT, (char *)&turnon, sizeof( turnon ) );
#endif

  if (bind( udpSocket_, (struct sockaddr *)&sAddr_, sizeof( sAddr_ ) ) < 0) {
    ERROR_MSG( "PyRideNetComm::initUDPListener: unable to bind to network interface.\n" );
#ifdef WIN32
    closesocket( udpSocket_ );
#else
    close( udpSocket_ );
#endif
    return false;
  }

  maxFD_ = max( maxFD_, udpSocket_ );
  FD_SET( udpSocket_, &masterFDSet_ );

  if (dgramBuffer_ == NULL)
    dgramBuffer_ = new unsigned char[PYRIDE_DEFAULT_BUFFER_SIZE];

  return true;
}

bool PyRideNetComm::initTCPListener()
{
  if ((tcpSocket_ = socket( AF_INET, SOCK_STREAM, 0 )) == INVALID_SOCKET) {
    ERROR_MSG( "PyRideNetComm::initTCPListener: unable to create TCP socket.\n" );
    return false;
  }

  int turnon = 1;
  if (setsockopt( tcpSocket_, SOL_SOCKET, SO_REUSEADDR, (char *)&turnon, sizeof( turnon ) ) < 0) {
    ERROR_MSG( "PyRideNetComm::initTCPListener: failed to enable reuse addr option on TCP socket.\n" );
#ifdef WIN32
    closesocket( tcpSocket_ );
#else
    close( tcpSocket_ );
#endif
    return false;
  }

  if (bind( tcpSocket_, (struct sockaddr *)&sAddr_, sizeof( sAddr_ ) ) < 0) {
    ERROR_MSG( "PyRideNetComm::initTCPListener: unable to bind to network interface.\n" );
#ifdef WIN32
    closesocket( tcpSocket_ );
#else
    close( tcpSocket_ );
#endif
    return false;
  }
  
  if (listen( tcpSocket_, 5 ) < 0) {
    ERROR_MSG( "PyRideNetComm::initTCPListener: unable to listen for incoming data.\n" );
#ifdef WIN32
    closesocket( tcpSocket_ );
#else
    close( tcpSocket_ );
#endif
    return false;
  }

  INFO_MSG( "Listening on TCP port %d for remote access.\n", PYRIDE_CONTROL_PORT );
  maxFD_ = max( maxFD_, tcpSocket_ );
  FD_SET( tcpSocket_, &masterFDSet_ );

  return true;
}

void PyRideNetComm::processIncomingData( fd_set * readyFDSet )
{
  SOCKET_T fd = INVALID_SOCKET;

#ifndef NO_AUTO_DISCOVERY
  struct sockaddr_in cAddr;
  int cLen = sizeof( cAddr );

  if (netCommEnabled_) {
    if (FD_ISSET( udpSocket_, readyFDSet )) {
      int readLen = (int)recvfrom( udpSocket_, dgramBuffer_, PYRIDE_DEFAULT_BUFFER_SIZE,
        0, (sockaddr *)&cAddr, (socklen_t *)&cLen );
      if (readLen <= 0) {
        ERROR_MSG( "PyRideNetComm::continuousProcessing: error accepting "
          "incoming UDP packet. error %d\n", errno );
      }
      else {
        processUDPInput( dgramBuffer_, readLen, cAddr );
      }
    }
    if (FD_ISSET( tcpSocket_, readyFDSet )) {
      // accept incoming TCP connection and read the stream
      fd = accept( tcpSocket_, (sockaddr *)&cAddr, (socklen_t *)&cLen );
      if (fd != INVALID_SOCKET) {
        addFdToClientList( fd, cAddr );
      }
      else if (errno != ECONNABORTED) {
        ERROR_MSG( "PyRideNetComm::continuousProcessing: error accepting "
               "incoming TCP connection error = %d\n", errno );
      }
    }
  }
#endif
#ifdef WIN32
  EnterCriticalSection( &t_criticalSection_ );
#else
  pthread_mutex_lock( &t_mutex_ );
#endif

  ClientItem * clientPtr = clientList_;
  ClientItem * prevClientPtr = clientPtr;
  while (clientPtr) {
    fd = clientPtr->fd;
    if (fd != INVALID_SOCKET && FD_ISSET( fd, readyFDSet )) {
#ifdef WIN32
      int readLen = recv( fd, (char*)clientDataBuffer_, kTCPMSS, 0 );
#else
      int readLen = (int)read( fd, clientDataBuffer_, kTCPMSS );
#endif
      if (readLen <= 0) {
        if (readLen == 0) {
          INFO_MSG( "Socket connection %d closed.\n", fd );
        }
        else {
          ERROR_MSG( "PyRideNetComm::continuousProcessing: "
                    "error reading data stream on %d error = %d.\n", fd, errno );
        }
        disconnectClient( clientPtr );
      }
      else {
        unsigned char * dataPtr = clientDataBuffer_;

        do {
          if (*dataPtr == PYRIDE_MSG_INIT && clientPtr->dataInfo.expectedDataLength == 0) { // new message
            if (readLen < 4) {
              ERROR_MSG( "PyRideNetComm::continuousProcessing: "
                        "invalid data packet in stream on %d (too small).\n", fd );
              break;
            }
            dataPtr++;
            short dataCount = 0;
            memcpy( &dataCount, dataPtr, sizeof( short ) ); dataPtr += sizeof( short );
            readLen -= 3;
            if (dataCount < 0 || dataCount > PYRIDE_MSG_BUFFER_SIZE) { // encryption buffer size
              ERROR_MSG( "PyRideNetComm::continuousProcessing: "
                        "invalid data size in stream on %d.\n", fd );
              break;
            }
            else if (dataCount > (readLen - 1)) { // the message needs multiple reads
              clientPtr->dataInfo.bufferedDataLength = readLen;
              memcpy( clientPtr->dataInfo.bufferedData, dataPtr, clientPtr->dataInfo.bufferedDataLength );
              clientPtr->dataInfo.expectedDataLength = dataCount - clientPtr->dataInfo.bufferedDataLength;
              readLen = 0;
            }
            else if (*(dataPtr + dataCount) == PYRIDE_MSG_END) { // valid message
              this->processDataInput( clientPtr, dataPtr, dataCount );
              readLen -= (dataCount + 1);
              dataPtr += (dataCount + 1);
            }
            else {
              ERROR_MSG( "PyRideNetComm::continuousProcessing: "
                        "invalid data packet in stream on %d.\n", fd );
              break;
            }
          }
          else if (clientPtr->dataInfo.expectedDataLength > 0) { // patch up cached data
            unsigned char * cachedPtr = clientPtr->dataInfo.bufferedData + clientPtr->dataInfo.bufferedDataLength;
            if (clientPtr->dataInfo.expectedDataLength > (readLen - 1)) { // continue to cache
              memcpy( cachedPtr, dataPtr, readLen );
              clientPtr->dataInfo.bufferedDataLength += readLen;
              clientPtr->dataInfo.expectedDataLength -= readLen;
              readLen = 0;
            }
            else if (*(dataPtr + clientPtr->dataInfo.expectedDataLength) == PYRIDE_MSG_END) { // valid message
              memcpy( cachedPtr, dataPtr, clientPtr->dataInfo.expectedDataLength );
              this->processDataInput( clientPtr, clientPtr->dataInfo.bufferedData,
                                     clientPtr->dataInfo.bufferedDataLength + clientPtr->dataInfo.expectedDataLength );
              readLen -= (clientPtr->dataInfo.expectedDataLength + 1);
              if (readLen > 0) {
                dataPtr += (clientPtr->dataInfo.expectedDataLength + 1);
              }
              clientPtr->dataInfo.bufferedDataLength = 0;
              clientPtr->dataInfo.expectedDataLength = 0;
            }
            else {
              ERROR_MSG( "PyRideNetComm::continuousProcessing: "
                        "unexpected data fragment in data stream on %d.\n", fd );
              clientPtr->dataInfo.bufferedDataLength = 0;
              clientPtr->dataInfo.expectedDataLength = 0;
              break;
            }
          }
          else {
            ERROR_MSG( "PyRideNetComm::continuousProcessing: "
                      "invalid data stream on %d.\n", fd );
            clientPtr->dataInfo.bufferedDataLength = 0;
            clientPtr->dataInfo.expectedDataLength = 0;
            break;
          }
        } while (readLen > 0);
      }
    }
    if (clientPtr->fd == INVALID_SOCKET) { // client has been disconnected
      if (clientPtr == clientList_) { // deletion of first node
        clientList_ = clientPtr->pNext;
        prevClientPtr = clientList_;
        delete clientPtr;
        clientPtr = clientList_;
      }
      else {
        prevClientPtr->pNext = clientPtr->pNext;
        delete clientPtr;
        clientPtr = prevClientPtr->pNext;
      }
    }
    else {
      prevClientPtr = clientPtr;
      clientPtr = clientPtr->pNext;
    }
  }
#ifdef WIN32
  LeaveCriticalSection( &t_criticalSection_ );
#else
  pthread_mutex_unlock( &t_mutex_ );
#endif
}

void PyRideNetComm::processTimer( void * data )
{
  TimerObj * mytimer = (TimerObj *)data;
  mytimer->isExecuting = true;
  if (pDataHandler_) {
    pDataHandler_->onTimer( mytimer->tID );
  }
  mytimer->isExecuting = false;
}

void PyRideNetComm::continuousProcessing()
{
  int maxFD = 0;
  fd_set readyFDSet;

  while (keepRunning_) {
    FD_ZERO( &readyFDSet );
    memcpy( &readyFDSet, &masterFDSet_, sizeof( masterFDSet_ ) );
    maxFD = maxFD_;

    struct timeval timeout; 
    timeout.tv_sec = 0;
    timeout.tv_usec = 500000; // 500ms

    select( maxFD + 1, &readyFDSet, NULL, NULL, &timeout );

    this->checkTimers();
    this->processIncomingData( &readyFDSet );
  }
}

void PyRideNetComm::processUDPInput( const unsigned char * recBuffer, int recBytes, struct sockaddr_in & cAddr )
{
  unsigned char * message = NULL;
  int messageSize = 0;
  
#ifdef USE_ENCRYPTION
  if (decryptMessage( (unsigned char *)recBuffer, (int)recBytes, (unsigned char **)&message, (int *)&messageSize ) != 1) {
    WARNING_MSG( "Unable to decrypt incoming messasge.\n" );
    return;
  }
#else
  message = (unsigned char *)recBuffer;
  messageSize = recBytes;
#endif

  // initialise a new TCP connection
  char command, subcommand, cID;
  if (!messageValidation( message, messageSize, cID, command, subcommand ))
    return;

  switch (command) {
#ifdef PYRIDE_REMOTE_CLIENT
    case ROBOT_DISCOVERY:
      break;
    case ROBOT_DECLARE:
#else
    case ROBOT_DECLARE:
      break;
    case ROBOT_DISCOVERY:
#endif
    {
      ClientItem * newClient = NULL;
      if ((newClient = createTCPTalker( cAddr )) != NULL) { // process udp data
        processDataInput( newClient, recBuffer, recBytes );
      }
      else {
        char cAddrStr[INET_ADDRSTRLEN];
        inet_ntop( AF_INET, &cAddr.sin_addr.s_addr, cAddrStr, INET_ADDRSTRLEN );
        ERROR_MSG( "PyRideNetComm::processUDPInput: Unable to "
          "create connection to %s:%d.\n", cAddrStr, ntohs( cAddr.sin_port ) );
      }
    }
      break;
    default:
      ERROR_MSG( "PyRideNetComm::processUDPInput()"
        "Unknown negotiation message\n" );
  }
}

PyRideNetComm::ClientItem * PyRideNetComm::createTCPTalker( struct sockaddr_in & cAddr )
{
  ClientItem * found = findClientFromClientList( cAddr );

  if (found)
    return found;

  SOCKET_T mySocket = socket( AF_INET, SOCK_STREAM, 0 );

  if (mySocket == INVALID_SOCKET)
    return NULL;

  if (connect( mySocket, (struct sockaddr *)&cAddr, sizeof( cAddr ) ) < 0) {
#ifdef WIN32
    closesocket( mySocket );
#else
    close( mySocket );
#endif
    return NULL;
  }
  return addFdToClientList( mySocket, cAddr );
}

void PyRideNetComm::disconnectClientWithFD( SOCKET_T fd )
{
  ClientItem * client = findClientFromClientList( fd );
  if (client) {
    disconnectClient( client, false );
  }
}

void PyRideNetComm::fini()
{
  stopProcessing();

  if (netCommEnabled_) {
#ifndef PYRIDE_REMOTE_CLIENT
    // assume all audio and video objects will be finalised by parent level code.
    if (activeVideoObjs_) {
      activeVideoObjs_ = NULL;
    }
    if (activeAudioObjs_) {
      activeAudioObjs_ = NULL;
    }
#endif
    disconnectClient( NULL, false );

#ifdef USE_MULTICAST
    setsockopt( udpSocket_, IPPROTO_IP, IP_DROP_MEMBERSHIP, (char *)&multiCastReq_, sizeof( multiCastReq_ ) );
#endif

#ifdef WIN32
    closesocket( udpSocket_ );
    closesocket( tcpSocket_ );
#else
    close( udpSocket_ );
    close( tcpSocket_ );
#endif

    netCommEnabled_ = false;
  }    

  if (dgramBuffer_) {
    delete [] dgramBuffer_;
    dgramBuffer_ = NULL;
  }
  
  if (clientDataBuffer_) {
    delete [] clientDataBuffer_;
    clientDataBuffer_ = NULL;
  }
  if (dispatchDataBuffer_) {
    delete [] dispatchDataBuffer_;
    dispatchDataBuffer_ = NULL;
  }

  maxFD_ = 0;
  FD_ZERO( &masterFDSet_ );

#ifdef USE_ENCRYPTION
  endecryptFini();
#endif
}

#pragma mark message processing
void PyRideNetComm::processDataInput( ClientItem * client, const unsigned char * receivedMesg,
                                      const int receivedBytes )
{
  unsigned char * message = NULL;
  int messageSize = 0;
  
#ifdef USE_ENCRYPTION
  if (decryptMessage( (unsigned char *)receivedMesg, (int)receivedBytes, (unsigned char **)&message, (int *)&messageSize ) != 1) {
    WARNING_MSG( "Unable to decrypt incoming messasge.\n" );
    return;
  }
#else
  message = (unsigned char *)receivedMesg;
  messageSize = receivedBytes;
#endif

  char command, subcommand, cID;
  
  if (!messageValidation( message, messageSize, cID, command, subcommand )) {
    ERROR_MSG( "PyRide:processControlCommand: corrupted control data.\n" );
    return;
  }
  unsigned char * commandData = (unsigned char *)message + PYRIDE_MSG_HEADER_SIZE;
  int commandDataLen = messageSize - PYRIDE_MSG_MIN_LENGTH;
  switch (command) {
#ifdef PYRIDE_REMOTE_CLIENT
    case ROBOT_DECLARE:
      if (client->cID) {
        WARNING_MSG( "Duplicate NAO declaration message. Client %d with (new) ID %d. Ignore.\n",
                    client->cID, cID );
      }
      else {
        client->cID = cID;
        client->pushData = true; // on console, we broadcast command to every connected NAO.
        /*
        char addressStr[50];
        inet_ntop( AF_INET, &client->addr.sin_addr.s_addr, addressStr, INET_ADDRSTRLEN );
        DEBUG_MSG( "receive robot declare %s\n", addressStr );
         */
        if (pDataHandler_ && commandDataLen >= (sizeof( RobotInfo ) + 1)) {
          // DEBUG_MSG( "correct declare structure\n" );
          RobotInfo rinfo;
          VideoSettings vsettings;
          AudioSettings asettings;
          memset( &vsettings, 0, sizeof(VideoSettings) );
          memset( &asettings, 0, sizeof(AudioSettings) );

          unsigned char * dataPtr = commandData;
          unsigned char * optionalLabels = NULL;

          memcpy( &rinfo, dataPtr, sizeof( RobotInfo ));
          dataPtr += sizeof( RobotInfo );
          int optLabelLength = commandDataLen - sizeof( RobotInfo );

          if (rinfo.nofaudios > 0) {
            memcpy( &asettings, dataPtr, sizeof( AudioSettings ));
            dataPtr += sizeof( AudioSettings );
            optLabelLength -= sizeof( AudioSettings ) ;
          }
          if (rinfo.nofcams > 0) {
            memcpy( &vsettings, dataPtr, sizeof( VideoSettings ));
            optionalLabels = dataPtr + sizeof( VideoSettings );
            optLabelLength -= sizeof( VideoSettings );
          }
          pDataHandler_->onRobotCreated( cID, client->addr.sin_addr.s_addr, &rinfo, &vsettings, &asettings, optionalLabels, optLabelLength );
        }
      }
      break;
    case ROBOT_TELEMETRY:
      if (client->cID != cID) {
        ERROR_MSG( "Robot telemetry data contains incorrect Robot ID. Ignore.\n" );
      }
      else {
        processTelemetryData( client, commandData, commandDataLen );
      }
      break;
    case ROBOT_STATUS:
      if (client->cID != cID) {
        ERROR_MSG( "Robot status data contains incorrect Robot ID. Ignore.\n" );
      }
      else {
        processOperationalData( client, commandData, commandDataLen );
      }
      break;
      break;
    case CLIENT_RESPONSE:
      if (client->cID != cID) {
        ERROR_MSG( "Robot response contains incorrect Robot ID. Ignore.\n" );
      }
      else {
        processRobotResponse( client, subcommand, commandData, commandDataLen );
      }
      break;
#else
    case ROBOT_DISCOVERY:
    {
      if (client->cID) {
        WARNING_MSG( "Duplicate remote console declaration message. Console %d with (new) ID %d. Ignore.",
                    client->cID, cID );
        break;
      }
      if (subcommand == USER_AUTH) {
        if (commandDataLen != SHA256_DIGEST_LENGTH) {
          disconnectClient( client, false );
          break;
        }
        if (!pDataHandler_ || !pDataHandler_->onUserLogOn( commandData, client->fd, client->addr )) {
          disconnectClient( client, false );
          break;
        }
      }
      client->cID = cID;
      if (activeVideoObjs_->size() > 0) {
        client->activeVideoObj = activeVideoObjs_->at( 0 );
      }
      if (activeAudioObjs_->size() > 0) {
        client->activeAudioObj = activeAudioObjs_->at( 0 );
      }
      this->declareRobot( &(pDataHandler_->defaultRobotInfo()), client );
    }
      break;
    case CLIENT_COMMAND:
      if (client->cID != cID) {
        ERROR_MSG( "Console command contains incorrect Console ID. Ignore.\n" );
      }
      else {
        processConsoleCommand( client, subcommand, commandData, commandDataLen );
      }
      break;
#endif
    case CLIENT_SHUTDOWN:
      if (client->cID != cID) {
        ERROR_MSG( "Invalid shutdown message. Ignore.\n" );
      }
      else {
        disconnectClient( client );
      }
      break;
    default:
      ERROR_MSG( "PyRide:processControlCommand: unknown control command %d.\n", command );
  }
}

void PyRideNetComm::disconnectClient( ClientItem * client, bool sendNotification )
{
  if (sendNotification) { // let the other end to close down the connection first
    unsigned char mesg[PYRIDE_MSG_MIN_LENGTH];
    mesg[0] = PYRIDE_MSG_INIT;
    mesg[1] = PYRIDE_PROTOCOL_VERSION;
    if (pDataHandler_)
      mesg[2] = pDataHandler_->clientID();
    else
      mesg[2] = 0;
    mesg[3] = CLIENT_SHUTDOWN << 4;
    mesg[4] = PYRIDE_MSG_END;

    unsigned char * outputData = NULL;
    int outputLength = 0;

#ifdef USE_ENCRYPTION
    if (encryptMessage( mesg, PYRIDE_MSG_MIN_LENGTH, &outputData, &outputLength ) != 1) {
      return;
    }
#else
    outputData = mesg;
    outputLength = PYRIDE_MSG_MIN_LENGTH;
#endif
    
    unsigned char * dataPtr = dispatchDataBuffer_;
    *dataPtr++ = PYRIDE_MSG_INIT;
    short opl = (short) outputLength;
    memcpy( dataPtr, &opl, sizeof( short ) );
    dataPtr += sizeof( short );
    memcpy( dataPtr, outputData, outputLength );
    dataPtr += outputLength;
    *dataPtr = PYRIDE_MSG_END;
    outputLength += (2 + sizeof( short ));

#ifdef WIN32
    EnterCriticalSection( &t_criticalSection_ );
#else
    pthread_mutex_lock( &t_mutex_ );
#endif
    if (client) {
      if (client->fd != INVALID_SOCKET)
#ifdef WIN32
        send( client->fd, (char*)dispatchDataBuffer_, outputLength, 0 );
#else
        write( client->fd, dispatchDataBuffer_, outputLength );
#endif
    }
    else {
      ClientItem * fdPtr = clientList_;
      while (fdPtr) {
        if (fdPtr->fd != INVALID_SOCKET)
#ifdef WIN32
          send( fdPtr->fd, (char*)dispatchDataBuffer_, outputLength, 0 );
#else
          write( fdPtr->fd, dispatchDataBuffer_, outputLength );
#endif
        fdPtr = fdPtr->pNext;
      }
    }
#ifdef WIN32
    LeaveCriticalSection( &t_criticalSection_ );
#else
    pthread_mutex_unlock( &t_mutex_ );
#endif
  }
  else {
#ifdef WIN32
    EnterCriticalSection( &t_criticalSection_ );
#else
    pthread_mutex_lock( &t_mutex_ );
#endif
    if (client) {
      if (client->fd != INVALID_SOCKET) {
        cleanupClient( client );
#ifdef WIN32
        closesocket( client->fd );
#else
        close( client->fd );
#endif
        FD_CLR( client->fd, &masterFDSet_ );
        client->fd = INVALID_SOCKET; // reset fd. this is also a flag for client item removal
#ifndef PYRIDE_REMOTE_CLIENT
        if (exclusiveCtrlClient_ == client) {
          exclusiveCtrlClient_ = NULL;
          unsigned char data = NORMAL_CONTROL;
          this->clientDataSend( ROBOT_STATUS, 0, &data, 1, NULL, false, true );
        }
#endif
      }
    }
    else { // disconnect all clients
      ClientItem * fdPtr = clientList_;
      while (fdPtr) {
        if (fdPtr->fd != INVALID_SOCKET) {
          cleanupClient( fdPtr );
#ifdef WIN32
          closesocket( fdPtr->fd );
#else
          close( fdPtr->fd );
#endif
          FD_CLR( fdPtr->fd, &masterFDSet_ );
        }
        ClientItem * tmpPtr = fdPtr;
        fdPtr = fdPtr->pNext;
        delete tmpPtr;
      }
      clientList_ = NULL;
#ifndef PYRIDE_REMOTE_CLIENT
      exclusiveCtrlClient_ = NULL;
#endif
    }
#ifdef WIN32
    LeaveCriticalSection( &t_criticalSection_ );
#else
    pthread_mutex_unlock( &t_mutex_ );
#endif
  }
#ifndef PYRIDE_REMOTE_CLIENT
  pDataHandler_->setTelemetryClients( this->calcTelemetryClients() );
#endif
}

void PyRideNetComm::cleanupClient( ClientItem * client )
{
  if (client && pDataHandler_) {
    if (client->dataInfo.bufferedData) {
      delete [] client->dataInfo.bufferedData;
      client->dataInfo.bufferedData = NULL;
      client->dataInfo.bufferedDataLength = client->dataInfo.expectedDataLength = 0;
    }
#ifdef PYRIDE_REMOTE_CLIENT
    pDataHandler_->onRobotDestroyed( client->cID );
#else
    client->pushData = false;
    if (client->pushImage) {
      if (client->activeVideoObj) {
        client->activeVideoObj->stop( client->addr, PYRIDE_VIDEO_STREAM_BASE_PORT );
      }
      if (client->activeAudioObj) {
        client->activeAudioObj->stop( client->addr, PYRIDE_VIDEO_STREAM_BASE_PORT + 2 );
      }
      client->pushImage = false;
    }
    pDataHandler_->onUserLogOff( client->fd );
#endif
  }
}

bool PyRideNetComm::messageValidation( const unsigned char * receivedMesg, const int receivedBytes, char & cID,
                                         char & command, char & subcommand )
{
  cID = command = subcommand = 0;
  
  //DEBUG_MSG( "Received %d bytes, header as %X|%X|%X|%X%X\n", receivedBytes, 
  //receivedMesg[0], receivedMesg[1], receivedMesg[2], receivedMesg[3], receivedMesg[4]);
  
  if (receivedBytes < PYRIDE_MSG_MIN_LENGTH ||
      receivedMesg[0] != PYRIDE_MSG_INIT ||
      receivedMesg[1] != PYRIDE_PROTOCOL_VERSION ||
      receivedMesg[receivedBytes-1] != PYRIDE_MSG_END)
  {
    return false;
  }
  
  cID = receivedMesg[2];
  command = (receivedMesg[3] >> 4) & 0x0f;
  subcommand = receivedMesg[3] & 0x0f;
  return true;
}

#ifdef PYRIDE_REMOTE_CLIENT
void PyRideNetComm::discoverRobots()
{
  this->clientDataSend( ROBOT_DISCOVERY, 0, NULL, 0, NULL, true );
}

bool PyRideNetComm::logonToRobot( const char * host, const unsigned char * authCode )
{
  if (!host || !authCode) {
    return false;
  }

  unsigned long saddr = 0;
  struct hostent * hostInfo = gethostbyname( host ); // try resolve name first
  if (!hostInfo) {
#ifdef WIN32
    saddr = inet_addr( host );
    if (saddr == INADDR_NONE)
#else
    if (inet_pton( AF_INET, host, &saddr ) != 1)
#endif
      return false;
  }

  struct sockaddr_in cAddr;
  cAddr.sin_family = AF_INET;
  if (hostInfo) {
    memcpy( (char *)&cAddr.sin_addr, hostInfo->h_addr, hostInfo->h_length );
  }
  else {
#ifdef WIN32
    cAddr.sin_addr.s_addr = saddr;
#else
    cAddr.sin_addr.s_addr = (in_addr_t)saddr;
#endif
  }
  cAddr.sin_port = htons( PYRIDE_CONTROL_PORT );

  ClientItem * client = this->createTCPTalker( cAddr );
  if (client) {
    this->clientDataSend( ROBOT_DISCOVERY, USER_AUTH, authCode, SHA256_DIGEST_LENGTH, client );
    return true;
  }
  return false;
}

void PyRideNetComm::disconnectRobots()
{
  this->disconnectClient( NULL, true );
}

void PyRideNetComm::startTelemetryStream( const char cID )
{
  if (cID == 0) {
    this->clientDataSend( CLIENT_COMMAND, TELEMETRY_START );
  }
  else {
    ClientItem * client = findClientFromClientList( cID );
    if (client) {
      this->clientDataSend( CLIENT_COMMAND, TELEMETRY_START, NULL, 0, client );
    }
  }
}

void PyRideNetComm::stopTelemetryStream()
{
  this->clientDataSend( CLIENT_COMMAND, TELEMETRY_STOP );
}

void PyRideNetComm::startCameraImageStream( const char cID )
{
  ClientItem * client = findClientFromClientList( cID );
  if (client) {
    this->clientDataSend( CLIENT_COMMAND, VIDEO_START, NULL, 0, client );
  }
}

void PyRideNetComm::stopCameraImageStream( const char cID )
{
  ClientItem * client = findClientFromClientList( cID );
  if (client) {
    this->clientDataSend( CLIENT_COMMAND, VIDEO_STOP, NULL, 0, client );
  }
}

void PyRideNetComm::cancelCurrentOperation( const char cID )
{
  ClientItem * client = findClientFromClientList( cID );
  if (client) {
    this->clientDataSend( CLIENT_COMMAND, CANCEL_CUR_OP, NULL, 0, client );
  }
}

void PyRideNetComm::setImageFormat( const char cID, ImageFormat format )
{
  if (cID == 0) {
    this->clientDataSend( CLIENT_COMMAND, VIDEO_FORMAT, (unsigned char *)&format, 1 );
  }
  else {
    ClientItem * client = findClientFromClientList( cID );
    if (client) {
      this->clientDataSend( CLIENT_COMMAND, VIDEO_FORMAT, (unsigned char *)&format, 1, client );
    }
  }
}

void PyRideNetComm::switchCamera( const char cID, const char vID )
{
  if (cID == 0) {
    this->clientDataSend( CLIENT_COMMAND, VIDEO_SWITCH, (unsigned char *)&vID, 1 );
  }
  else {
    ClientItem * client = findClientFromClientList( cID );
    if (client) {
      this->clientDataSend( CLIENT_COMMAND, VIDEO_SWITCH, (unsigned char *)&vID, 1, client );
    }
  }
}

void PyRideNetComm::issueExtendedCommand( const char cID, const PyRideExtendedCommand command,
                                                 const unsigned char * optionalData , const int optionalDataLength )
{
  unsigned char * data = new unsigned char[optionalDataLength+1];
  data[0] = (unsigned char) command;
  if (optionalDataLength > 0) {
    memcpy( data+1, optionalData, optionalDataLength );
  }

  if (cID == 0) {
    this->clientDataSend( CLIENT_COMMAND, CUSTOM_COMMAND, data, optionalDataLength + 1 );
  }
  else {
    ClientItem * client = findClientFromClientList( cID );
    if (client) {
      this->clientDataSend( CLIENT_COMMAND, CUSTOM_COMMAND, data, optionalDataLength + 1, client );
    }
  }
  delete [] data;
}

void PyRideNetComm::processRobotResponse( ClientItem * client, int subcommand, const unsigned char * commandData, int dataLen )
{
  switch (subcommand) {
    case TELEMETRY_START:
      if (pDataHandler_) {
        pDataHandler_->onTelemetryStreamStart( client->cID );
      }
      break;
    case TELEMETRY_STOP:
      if (pDataHandler_) {
        pDataHandler_->onTelemetryStreamStop( client->cID );
      }
      break;
    case VIDEO_START:
      if (pDataHandler_) {
        pDataHandler_->onImageStreamStart( client->cID );
      }
      break;
    case VIDEO_STOP:
      if (pDataHandler_) {
        pDataHandler_->onImageStreamStop( client->cID );
      }
      break;
    case VIDEO_FORMAT:
      if (pDataHandler_) {
        pDataHandler_->onImageFormatChange( client->cID, (ImageFormat)(*commandData) );
      }
      break;
    case VIDEO_SWITCH:
      if (pDataHandler_) {
        pDataHandler_->onVideoSwitchChange( client->cID, (VideoSettings *)(commandData) );
      }
      break;
    case CUSTOM_COMMAND:
      if (dataLen >= 1 && pDataHandler_) {
        PyRideExtendedCommand command = (PyRideExtendedCommand)commandData[0];
        unsigned char * optionData = (unsigned char *)commandData + 1;
        pDataHandler_->onExtendedCommandResponse( client->cID, command, optionData, dataLen - 1 );
      }
      break;
    default:
      ERROR_MSG( "PyRide::processRobotResponse unknow subcommand %d\n", subcommand );
      break;
  }
}

void PyRideNetComm::processTelemetryData( ClientItem * client, const unsigned char * data, int dataLen )
{
  if (!pDataHandler_) {
    return;
  }
  
  int objDataLen = dataLen - sizeof( RobotPose );

  if (objDataLen < 0) {
    ERROR_MSG( "Corrupted telemetry data Robot id %d.\n", client->cID );
    return;
  }
  else if (objDataLen == 0) { // we have RobotPose data
    pDataHandler_->onRobotTelemetryData( client->cID, (RobotPose *)data );
  }
  else {
    if (objDataLen % sizeof( FieldObject ) == 0) { // we have observed field objects
      pDataHandler_->onRobotTelemetryData( client->cID, (RobotPose *)data, (FieldObject *)(data+sizeof( RobotPose )),
                                     objDataLen / sizeof( FieldObject ) );
      
    }
    else {
      ERROR_MSG( "Corrupted telemetry data Robot id %d.\n", client->cID );
      return;
    }
  }

}
void PyRideNetComm::processOperationalData( ClientItem * client, const unsigned char * data, int dataLen )
{
  if (!pDataHandler_) {
    return;
  }
  
  if (dataLen < 1) {
    ERROR_MSG( "Corrupted Robot operational data Robot id %d.\n", client->cID );
    return;
  }
  char status = data[0];
  unsigned char * optionalData = (unsigned char *)data + 1;
  pDataHandler_->onOperationalData( client->cID, (int) status, optionalData, dataLen - 1 );

}
#else //!PYRIDE_REMOTE_CLIENT
#pragma mark Robot robot side implementation
void PyRideNetComm::declareRobot( const RobotInfo * robotInfo )
{
  this->declareRobot( robotInfo, NULL );
}

void PyRideNetComm::declareRobot( const RobotInfo * robotInfo, ClientItem * client )
{
  RobotInfo rinfo;
  
  rinfo.pose.x = robotInfo->pose.x;
  rinfo.pose.y = robotInfo->pose.y;
  rinfo.pose.theta = robotInfo->pose.theta;
  rinfo.type = robotInfo->type;
  rinfo.status = exclusiveCtrlClient_ ? EXCLUSIVE_CONTROL : NORMAL_CONTROL;
  rinfo.nofcams = 0;
  rinfo.nofaudios = 0;

  VideoDevice * activeCam = NULL;
  AudioDevice * activeAudio = NULL;

  int dataSize = sizeof( RobotInfo );
  
  if (activeVideoObjs_) {
    rinfo.nofcams = (char)activeVideoObjs_->size();
  }
  
  if (activeVideoObjs_) {
    rinfo.nofaudios = (char)activeAudioObjs_->size();
  }
  
  if (rinfo.nofcams > 0) {
    activeCam = activeVideoObjs_->at( 0 );
    dataSize += sizeof( VideoSettings );
    for (size_t i = 0; i < activeVideoObjs_->size(); i++) {
      dataSize += activeVideoObjs_->at( i )->deviceLabel().length() + 1;
    }
  }

  if (rinfo.nofaudios > 0) {
    activeAudio = activeAudioObjs_->at( 0 );
    dataSize += sizeof( AudioSettings );
  }

  unsigned char * declareData = new unsigned char[dataSize];
  unsigned char * dataPtr = declareData;
  
  //Combine pDataHandler and the RobotPose data to be sent, modify the length of the data as well in the argument
  memcpy( dataPtr, (void *)&rinfo, sizeof(RobotInfo) ); dataPtr += sizeof( RobotInfo );

  if (rinfo.nofaudios > 0) {
    AudioSettings asettings;
    activeAudio->getAudioSettings( asettings );
    memcpy( dataPtr, (void *)&asettings, sizeof( AudioSettings ) );
    dataPtr += sizeof( AudioSettings );
  }

  if (rinfo.nofcams > 0) {
    VideoSettings vsettings;
    activeCam->getVideoSettings( vsettings );
    memcpy( dataPtr, (void *)&vsettings, sizeof( VideoSettings ) );
    dataPtr += sizeof( VideoSettings );
    VideoDevice * cam = NULL;
    for (size_t i = 0; i < activeVideoObjs_->size(); i++) {
      cam = activeVideoObjs_->at( i );
      *dataPtr++ = (char) cam->deviceLabel().length();
      memcpy( dataPtr, cam->deviceLabel().c_str(),
             cam->deviceLabel().length() );
      dataPtr += cam->deviceLabel().length();
    }
  }

  if (client) {
    this->clientDataSend( ROBOT_DECLARE, 0, declareData, dataSize, client );
  }
  else {
    this->clientDataSend( ROBOT_DECLARE, 0, declareData, dataSize, NULL, true );
  }
  delete [] declareData;
}

void PyRideNetComm::disconnectConsoles()
{
  this->disconnectClient( NULL, false );
}

void PyRideNetComm::processConsoleCommand( ClientItem * client, int subcommand, const unsigned char * commandData, int dataLen )
{
  unsigned char retData = 1;
  
  switch (subcommand) {
    case TELEMETRY_START:
      if (pDataHandler_) {
        client->pushData = true;
        pDataHandler_->setTelemetryClients( this->calcTelemetryClients() );
      }
      break;
    case TELEMETRY_STOP:
      if (pDataHandler_) {
        client->pushData = false;
        pDataHandler_->setTelemetryClients( this->calcTelemetryClients() );
      }
      break;
    case VIDEO_START:
      if (client->activeVideoObj) {
        if (client->activeVideoObj->start( client->addr, PYRIDE_VIDEO_STREAM_BASE_PORT )) {
          client->pushImage = true;
          if (client->activeAudioObj) {
            client->activeAudioObj->start( client->addr, PYRIDE_VIDEO_STREAM_BASE_PORT + 2);
          }
        }
      }
      break;
    case VIDEO_STOP:
      if (pDataHandler_) {
        if (client->activeVideoObj && client->activeVideoObj->stop( client->addr, PYRIDE_VIDEO_STREAM_BASE_PORT )) {
          client->pushImage = false;
          if (client->activeAudioObj) {
            client->activeAudioObj->stop( client->addr, PYRIDE_VIDEO_STREAM_BASE_PORT + 2 );
          }
        }
      }
      break;
    case VIDEO_FORMAT:
      if (dataLen == 1 && client->activeVideoObj) {
        client->activeVideoObj->setImageFormat( (ImageFormat)(*commandData) );
        retData = *commandData;
      }
      break;
    case VIDEO_SWITCH:
      if (dataLen == 1 && activeVideoObjs_) {
        size_t vid = (char)(*commandData);
        if (vid >= activeVideoObjs_->size()) {
          ERROR_MSG( "Video selection ID out of range.\n" );
          break;
        }
        if (client->activeVideoObj != activeVideoObjs_->at( vid )) {
          VideoDevice * oldVObj = client->activeVideoObj;
          client->activeVideoObj = activeVideoObjs_->at( vid );
          if (oldVObj->stop( client->addr, PYRIDE_VIDEO_STREAM_BASE_PORT )) {
            // previous video were streaming, so keep this way on the new video object
            client->activeVideoObj->start( client->addr, PYRIDE_VIDEO_STREAM_BASE_PORT );
          }
          VideoSettings vsettings;
          client->activeVideoObj->getVideoSettings( vsettings );
          clientDataSend( CLIENT_RESPONSE, subcommand, (unsigned char *)&vsettings,
                         sizeof( VideoSettings ), client );
          return;
        }
      }
      break;
    case CUSTOM_COMMAND:
      if (dataLen >= 1 && pDataHandler_) {
        // more hacks to enable execlusive robot control
        unsigned char retData[2];
        PyRideExtendedCommand cmd = (PyRideExtendedCommand) commandData[0];
        retData[0] = cmd;
        if (cmd == EXCLUSIVE_CTRL_REQUEST) {
          if (exclusiveCtrlClient_) {
            retData[1] = EXCLUSIVE_CTRL_REJECT;
          }
          else {
            exclusiveCtrlClient_ = client;
            retData[1] = 1;
            clientDataSend( CLIENT_RESPONSE, subcommand, (unsigned char *)&retData, 2, client );
            unsigned char data = EXCLUSIVE_CONTROL;
            this->clientDataSend( ROBOT_STATUS, 0, &data, 1, NULL, false, true );
            return;
          }
        }
        else if (cmd == EXCLUSIVE_CTRL_RELEASE) {
          if (exclusiveCtrlClient_ == client) {
            exclusiveCtrlClient_ = NULL;
            retData[1] = 1;
            clientDataSend( CLIENT_RESPONSE, subcommand, (unsigned char *)&retData, 2, client );
            unsigned char data = NORMAL_CONTROL;
            this->clientDataSend( ROBOT_STATUS, 0, &data, 1, NULL, false, true );
            return;
          }
          else {
            retData[1] = EXCLUSIVE_CTRL_REJECT;
          }
        }
        else if (isNonExclusiveCommand( cmd ) || exclusiveCtrlClient_ == client) {
          retData[1] = (unsigned char)pDataHandler_->executeRemoteCommand( commandData, dataLen );
        }
        else {
          retData[1] = EXCLUSIVE_CTRL_REJECT;
        }
        clientDataSend( CLIENT_RESPONSE, subcommand, (unsigned char *)&retData, 2, client );
        return;
      }
      break;
    case CANCEL_CUR_OP:
      if (pDataHandler_) {
        pDataHandler_->cancelCurrentOperation();
      }
      break;
    default:
      retData = 0;
      ERROR_MSG( "PyRide::processConsoleCommand unknow subcommand %d\n", subcommand );
      break;
  }
  clientDataSend( CLIENT_RESPONSE, subcommand, (unsigned char *)&retData, 1, client );
}

void PyRideNetComm::dispatchTelemetryData( const unsigned char * data, const int size )
{
  this->clientDataSend( ROBOT_TELEMETRY, 0, data, size );
}

void PyRideNetComm::dispatchStatusData( const unsigned char * data, const int size )
{
  this->clientDataSend( ROBOT_STATUS, 0, data, size );
}

void PyRideNetComm::takeCameraSnapshot( const VideoDeviceDataHandler * dataHandler, bool allCameras )
{
  if (!activeVideoObjs_) {
    return;
  }

  if (allCameras) {
    for (size_t i = 0; i < activeVideoObjs_->size(); i++) {
      VideoDevice * cam = activeVideoObjs_->at( i );
      cam->takeSnapshot( dataHandler );
    }
  }
  else {
    VideoDevice * cam = activeVideoObjs_->at( 0 );
    cam->takeSnapshot( dataHandler );
  }
}
  
void PyRideNetComm::blockRemoteExclusiveControl( bool isYes )
{
  if (isYes) {
    if (exclusiveCtrlClient_) {
      if (exclusiveCtrlClient_->fd == INVALID_SOCKET) {
        // already locked
        return;
      }
      else {
        // someone has the exclusive lock.
        unsigned char data = EXCLUSIVE_CONTROL_OVERRIDE;
        this->clientDataSend( ROBOT_STATUS, 0, &data, 1, exclusiveCtrlClient_ );
      }
    }
    else {
      unsigned char data = EXCLUSIVE_CONTROL;
      this->clientDataSend( ROBOT_STATUS, 0, &data, 1, NULL, false, true );
    }
    ClientItem * clientLock = new ClientItem; // dummy item for locking
    clientLock->fd = INVALID_SOCKET;
    clientLock->cID = 0;
    clientLock->pushData = false;
    clientLock->pNext = NULL;
    exclusiveCtrlClient_ = clientLock;
  }
  else {
    if (exclusiveCtrlClient_ && exclusiveCtrlClient_->fd == INVALID_SOCKET) {
      delete exclusiveCtrlClient_;
      exclusiveCtrlClient_ = NULL;
      unsigned char data = NORMAL_CONTROL;
      this->clientDataSend( ROBOT_STATUS, 0, &data, 1, NULL, false, true );
    }
  }
}

#endif

void PyRideNetComm::clientDataSend( const int command, const int subcommand,
                                      const unsigned char * optionalData,
                                      const int optionalDataLen,
                                      ClientItem * client, bool useUDP, bool forceAll )
{
  if (client && client->fd == INVALID_SOCKET)
    return;
  
  int replyLength = PYRIDE_MSG_MIN_LENGTH + optionalDataLen;  
  unsigned char * replyMesg = new unsigned char[replyLength];
  unsigned char * msgPtr = replyMesg;

  *msgPtr++ = PYRIDE_MSG_INIT;
  *msgPtr++ = PYRIDE_PROTOCOL_VERSION;
  if (pDataHandler_)
    *msgPtr++ = pDataHandler_->clientID();
  else
    *msgPtr++ = 0;

  *msgPtr++ = (command << 4) | (subcommand & 0xf);

  if (optionalDataLen) {
    memcpy( (void *)msgPtr, (void *)optionalData, optionalDataLen );
    msgPtr += optionalDataLen;
  }
  *msgPtr = PYRIDE_MSG_END;
#ifdef WIN32
  char * outputData = NULL;
#else
  unsigned char * outputData = NULL;
#endif
  int outputLength = 0;

#ifdef USE_ENCRYPTION
  if (encryptMessage( replyMesg, replyLength, (unsigned char**)&outputData, &outputLength ) != 1) {
    delete [] replyMesg;
    return;
  }
#else
  outputData = replyMesg;
  outputLength = replyLength;
#endif

#ifdef WIN32
  EnterCriticalSection( &t_criticalSection_ );
#else
  pthread_mutex_lock( &t_mutex_ );
#endif

  if (useUDP) {
    sendto( udpSocket_, outputData, outputLength, 0, (struct sockaddr *)&bcAddr_, sizeof( bcAddr_ ) );
  }
  else {
    // YAH (Yet Another Hack)
    // encapulate entire message into a simple container so that we can
    // correctly parse the message in a TCP stream at the other end. This
    // is due to some of our data packets are too small and underlying TCP
    // stack keep coalesce them together. We have to do this properly now.
    unsigned char * dataPtr = dispatchDataBuffer_;
    *dataPtr++ = PYRIDE_MSG_INIT;
    short opl = (short) outputLength;
    memcpy( dataPtr, &opl, sizeof( short ) );
    dataPtr += sizeof( short );
    memcpy( dataPtr, outputData, outputLength );
    dataPtr += outputLength;
    *dataPtr = PYRIDE_MSG_END;
    outputLength += (2 + sizeof( short ));

    if (client) {
#ifdef WIN32
      send( client->fd, dispatchDataBuffer_, outputLength, 0 );
#else
      //DEBUG_MSG( "sending total packet size %d, data count %d\n", outputLength, opl );
      write( client->fd, dispatchDataBuffer_, outputLength );
#endif
    }
    else {
      ClientItem * cPtr = clientList_;
      while (cPtr) {
        if (cPtr->fd != INVALID_SOCKET && (forceAll || cPtr->pushData)) {
#ifdef WIN32
          send( cPtr->fd, dispatchDataBuffer_, outputLength, 0 );
#else
          write( cPtr->fd, dispatchDataBuffer_, outputLength );
#endif
        }
        cPtr = cPtr->pNext;
      }
    }
  }

#ifdef WIN32
  LeaveCriticalSection( &t_criticalSection_ );
#else
  pthread_mutex_unlock( &t_mutex_ );
#endif

  delete [] replyMesg;
}

//helper method to manage TCP fd list
PyRideNetComm::ClientItem * PyRideNetComm::findClientFromClientList( struct sockaddr_in & cAddr )
{
  ClientItem * found = NULL;
  ClientItem * fdPtr = clientList_;
  while (fdPtr) {
    if (!memcmp( &(fdPtr->addr), &cAddr, sizeof( struct sockaddr_in ) )) {
      found = fdPtr;
      break;
    }
    fdPtr = fdPtr->pNext;
  }
  return found;
}

PyRideNetComm::ClientItem * PyRideNetComm::findClientFromClientList( SOCKET_T fd )
{
  ClientItem * found = NULL;
  ClientItem * fdPtr = clientList_;
  while (fdPtr) {
    if (fdPtr->fd == fd) {
      found = fdPtr;
      break;
    }
    fdPtr = fdPtr->pNext;
  }
  return found;
}

PyRideNetComm::ClientItem * PyRideNetComm::findClientFromClientList( const char cID )
{
  ClientItem * found = NULL;
  ClientItem * fdPtr = clientList_;
  while (fdPtr) {
    if (fdPtr->cID == cID) {
      found = fdPtr;
      break;
    }
    fdPtr = fdPtr->pNext;
  }
  return found;
}

PyRideNetComm::ClientItem * PyRideNetComm::addFdToClientList( const SOCKET_T & fd, struct sockaddr_in & cAddr )
{
  ClientItem * newClient = new ClientItem;
  
  newClient->fd = fd;
  newClient->dataInfo.bufferedData = new unsigned char[PYRIDE_MSG_BUFFER_SIZE];
  newClient->dataInfo.bufferedDataLength = 0;
  newClient->dataInfo.expectedDataLength = 0;
  newClient->addr = cAddr;
  newClient->cID = 0;
  newClient->pushData = false;
#ifndef PYRIDE_REMOTE_CLIENT
  newClient->pushImage = false;
  newClient->activeVideoObj = NULL;
  newClient->activeAudioObj = NULL;
#endif
  newClient->pNext = NULL;

#ifdef WIN32
  EnterCriticalSection( &t_criticalSection_ );
#else
  pthread_mutex_lock( &t_mutex_ );
#endif

  if (clientList_) {
    ClientItem * fdPtr = clientList_;
    while (fdPtr->pNext) fdPtr = fdPtr->pNext;
    fdPtr->pNext = newClient;
  }
  else {
    clientList_ = newClient;
  }
  FD_SET( fd, &masterFDSet_ );
  maxFD_ = max( fd, maxFD_ );

#ifdef WIN32
  LeaveCriticalSection( &t_criticalSection_ );
#else
  pthread_mutex_unlock( &t_mutex_ );
#endif

  /*
  int turnon = 1;
#ifndef PYRIDE_REMOTE_CLIENT // this option seem to cause issue on PR2 return status message
  if (setsockopt( fd, IPPROTO_TCP, TCP_NODELAY, (char *) &turnon, sizeof(int)) < 0) {
    ERROR_MSG( "Unable to set TCP_NODELAY to socket %d\n", fd );
  }
#endif

  turnon = kTCPMSS;
  if (setsockopt( fd, IPPROTO_TCP, TCP_MAXSEG, (char *) &turnon, sizeof(int)) < 0) {
    ERROR_MSG( "Unable to set TCP MSS to socket %d\n", fd );
  }
  */
  return newClient;
}

#ifndef PYRIDE_REMOTE_CLIENT
int PyRideNetComm::calcTelemetryClients()
{
  int clients = 0;
  ClientItem * clientPtr = clientList_;
  while (clientPtr) {
    if (clientPtr->pushData) {
      clients++;
    }
    clientPtr = clientPtr->pNext;
  }
  return clients;
}
#endif

void PyRideNetComm::initIPAddresses()
{
  sAddr_.sin_family = AF_INET;
  sAddr_.sin_addr.s_addr = INADDR_ANY;
  sAddr_.sin_port = htons( PYRIDE_CONTROL_PORT );

  bcAddr_.sin_family = AF_INET;
  inet_pton( AF_INET, PYRIDE_BROADCAST_IP, &bcAddr_.sin_addr.s_addr );
  bcAddr_.sin_port = htons( PYRIDE_CONTROL_PORT );
  
#ifdef USE_MULTICAST
  multiCastReq_.imr_multiaddr.s_addr = bcAddr_.sin_addr.s_addr;
  multiCastReq_.imr_interface.s_addr = INADDR_ANY;
#endif  
}

bool PyRideNetComm::getMyIPAddress( int & addr )
{
  addr = 0;
#ifdef WIN32
  if (tcpSocket_ == INVALID_SOCKET) {
    return false;
  }
  else {
    char * buf = NULL;
    DWORD dwBytesRet = 0;
    
    int rc = WSAIoctl( tcpSocket_, SIO_ADDRESS_LIST_QUERY, NULL,
                      0, NULL, 0, &dwBytesRet, NULL, NULL );
    
    if (rc == SOCKET_ERROR && GetLastError() == WSAEFAULT) { // retrieve output buffer size
      // Allocate the necessary size
      buf = (char *)HeapAlloc( GetProcessHeap(), 0, dwBytesRet );
      if (buf != NULL) {
        rc = WSAIoctl( tcpSocket_, SIO_ADDRESS_LIST_QUERY, NULL, 0,
                      buf, dwBytesRet, &dwBytesRet, NULL, NULL );
        if (rc != SOCKET_ERROR) {
          SOCKET_ADDRESS_LIST * slist = (SOCKET_ADDRESS_LIST *)buf;
          if (slist->iAddressCount > 0) {
            struct sockaddr_in localAddr;
            memcpy( (char *)&localAddr, slist->Address[0].lpSockaddr,
                   slist->Address[0].iSockaddrLength );
            addr = htonl(localAddr.sin_addr.s_addr);
          }
        }
      }
    }
    if (buf) HeapFree( GetProcessHeap(), 0, buf );
  }
  return (addr != 0);
#else
  int nofinf = 8;
  struct ifconf myifconf;
  
  myifconf.ifc_len = nofinf * sizeof( struct ifreq );
  myifconf.ifc_req = new struct ifreq[nofinf];
  SOCKET_T mySock = socket( AF_INET, SOCK_STREAM, 0 );
  if (ioctl( mySock, SIOCGIFCONF, &myifconf ) == -1) {
    ERROR_MSG( "PyRideNetComm::getIDFromIP: ioctl SIOCGIFCONF "
              "failed\n" );
    delete [] myifconf.ifc_req;
    close( mySock );
    return false;
  }
  
  struct ifreq * ifr = myifconf.ifc_req;
  for (;(char *)ifr < (char *)myifconf.ifc_req + myifconf.ifc_len; ++ifr) {
    if (ioctl( mySock, SIOCGIFFLAGS, ifr ) == -1) {
      continue;  /* failed to get flags, skip it */
    }
    if ((ifr->ifr_flags & IFF_UP) &&
        !(ifr->ifr_flags & IFF_LOOPBACK))
    { // found an live non-loopback interface
      ioctl( mySock, SIOCGIFADDR, ifr );
      sockaddr_in localAddr;
      memcpy( &localAddr, (struct sockaddr_in *)&ifr->ifr_addr,
             sizeof( struct sockaddr_in ) );
      addr = htonl(localAddr.sin_addr.s_addr);
      delete [] myifconf.ifc_req;
      close( mySock );
      return true;
    }
  }
  delete [] myifconf.ifc_req;
  close( mySock );
  return false;
#endif
}

bool PyRideNetComm::getIDFromIP( int & myid )
{
  int addr = 0;
  myid = 0;

  if (getMyIPAddress( addr )) {
    myid = addr & 0xf;
    return true;
  }
  return false;
}

inline bool PyRideNetComm::isNonExclusiveCommand( PyRideExtendedCommand cmd )
{
  for (int i = 0; i < NonExcmdSize; i++) {
    if (cmd == NonExclusiveExtendedCommands[i]) {
      return true;
    }
  }
  return false;
}

#pragma marker timer implementation
long PyRideNetComm::addTimer( int initialTime, long repeats, int interval )
{
  if (initialTime <= 0 || repeats < -1 || interval <= 0) {
    return -1; // invalid. no timer is created.
  }

  struct timeval now;

#ifdef WIN32
  win_gettimeofday( &now, NULL );
#else
  gettimeofday( &now, NULL );
#endif

  TimerObj * newTimer = new TimerObj;
  newTimer->pNext = NULL;
  newTimer->remainCount = (repeats == 0) ? 1 : repeats;
  newTimer->isExecuting = false;
  newTimer->interval = interval;
#ifdef WIN32
  newTimer->timerThread = (HANDLE)NULL;
#else
  newTimer->timerThread = (pthread_t)NULL;
#endif
  newTimer->nextTrigTime = now.tv_sec + initialTime;

#ifdef WIN32
  EnterCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_lock( &timer_mutex_ );
#endif
  newTimer->tID = nextTimerID_++;
  timerCount_ ++;

  if (lastTimer_) {
    lastTimer_->pNext = newTimer;
  }
  else { // the timer list must be empty
    timerList_ = newTimer;
  }
  lastTimer_ = newTimer;
#ifdef WIN32
  LeaveCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_unlock( &timer_mutex_ );
#endif

  return newTimer->tID;
}

void PyRideNetComm::delTimer( long tID )
{
  if (tID <= 0)
    return;

#ifdef WIN32
  EnterCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_lock( &timer_mutex_ );
#endif

  TimerObj * timerPtr = timerList_;
  TimerObj * prevTimerPtr = timerPtr;
  while (timerPtr) {
    if (timerPtr->tID == tID) {
      if (timerPtr->isExecuting) { // allow the timer finishing its execution
#ifdef WIN32
      TerminateThread( timerPtr->timerThread, 1 );
      CloseHandle( timerPtr->timerThread );
#else
        pthread_cancel( timerPtr->timerThread );
#endif
      }
      if (timerPtr == timerList_) {
        timerList_ = timerPtr->pNext;
        prevTimerPtr = timerList_;
        delete timerPtr;
        timerPtr = timerList_;
      }
      else {
        prevTimerPtr->pNext = timerPtr->pNext;
        delete timerPtr;
        timerPtr = prevTimerPtr->pNext;
      }
      timerCount_--;
    }
    else {
      prevTimerPtr = timerPtr;
      timerPtr = timerPtr->pNext;
    }
  }
  lastTimer_ = prevTimerPtr;
#ifdef WIN32
  LeaveCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_unlock( &timer_mutex_ );
#endif
}

bool PyRideNetComm::isTimerRunning( long tID )
{
  if (tID <= 0)
    return false;

#ifdef WIN32
  EnterCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_lock( &timer_mutex_ );
#endif

  TimerObj * timerPtr = timerList_;
  while (timerPtr) {
    if (timerPtr->tID == tID) {
#ifdef WIN32
      LeaveCriticalSection( &timer_criticalSection_ );
#else
      pthread_mutex_unlock( &timer_mutex_ );
#endif
      return true;
    }
    timerPtr = timerPtr->pNext;
  }
#ifdef WIN32
  LeaveCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_unlock( &timer_mutex_ );
#endif
  return false;
}

void PyRideNetComm::delAllTimers()
{
#ifdef WIN32
  EnterCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_lock( &timer_mutex_ );
#endif

  TimerObj * timerPtr = timerList_;
  TimerObj * tmpPtr = NULL;

  while (timerPtr) {
    tmpPtr = timerPtr;
    timerPtr = timerPtr->pNext;
    if (tmpPtr->isExecuting) { // allow the timer finishing its execution
#ifdef WIN32
      TerminateThread( tmpPtr->timerThread, 1 );
      CloseHandle( tmpPtr->timerThread );
#else
      pthread_cancel( tmpPtr->timerThread ); // TODO should install proper cancellation cleanup routine.
#endif
    }
    delete tmpPtr;
  }
  timerCount_ = 0;
  nextTimerID_ = 1;

#ifdef WIN32
  LeaveCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_unlock( &timer_mutex_ );
#endif
}

void PyRideNetComm::checkTimers()
{
  struct timeval now;

#ifdef WIN32
  win_gettimeofday( &now, NULL );
#else
  gettimeofday( &now, NULL );
#endif

#ifdef WIN32
  EnterCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_lock( &timer_mutex_ );
#endif

  TimerObj * timerPtr = timerList_;
  TimerObj * prevTimerPtr = timerPtr;

  while (timerPtr) {
    if (timerPtr->nextTrigTime <= now.tv_sec) {
      if (timerPtr->remainCount != -1) {
        timerPtr->remainCount--;
      }
      // should fire the timer
      if (timerPtr->isExecuting) {
        ERROR_MSG( "Timer %u is still executing while it is called again! Skip\n", (int)timerPtr->tID );
      }
      else {
        timerExecuteData * ted = new timerExecuteData;
        ted->timerObj = timerPtr;
        ted->mainObj = this;
#ifdef WIN32
        timerPtr->timerThread = (HANDLE)_beginthreadex( NULL, 0, &robottimer_thread, (void*)ted, 0, NULL );
        if (timerPtr->timerThread == 0) {
          ERROR_MSG( "Unable to create a timer thread.\n" );
          delete ted;
        }
#else
        if (pthread_create( &(timerPtr->timerThread), NULL, robottimer_thread, (void *)ted ) ) {
          ERROR_MSG( "Unable to create a timer thread.\n" );
          delete ted;
        }
        else if(pthread_detach( timerPtr->timerThread )) {
          ERROR_MSG( "Unable to detach a timer thread.\n" );
        }
#endif
      }
      if (timerPtr->remainCount == 0) { // remove the timer
        if (pDataHandler_) {
          pDataHandler_->onTimerLapsed( timerPtr->tID );
        }
        if (timerPtr == timerList_) {
          timerList_ = timerPtr->pNext;
          prevTimerPtr = timerList_;
          delete timerPtr;
          timerPtr = timerList_;
        }
        else {
          prevTimerPtr->pNext = timerPtr->pNext;
          delete timerPtr;
          timerPtr = prevTimerPtr->pNext;
        }
        timerCount_--;
      }
      else {
        timerPtr->nextTrigTime = now.tv_sec + timerPtr->interval;
      }
    }
    else {
      prevTimerPtr = timerPtr;
      timerPtr = timerPtr->pNext;
    }
  }
  lastTimer_ = prevTimerPtr;
#ifdef WIN32
  LeaveCriticalSection( &timer_criticalSection_ );
#else
  pthread_mutex_unlock( &timer_mutex_ );
#endif
}
} // namespace pyride
