//
//  ServerDataProcessor.cpp
//  PyRIDE
//
//  Created by Xun Wang on 10/05/10.
//  Copyright 2010 GalaxyNetwork. All rights reserved.
//
#include <math.h>
#include "ServerDataProcessor.h"

namespace pyride {

ServerDataProcessor * ServerDataProcessor::s_pServerDataProcessor = NULL;

ServerDataProcessor * ServerDataProcessor::instance()
{
  if (!s_pServerDataProcessor)
    s_pServerDataProcessor = new ServerDataProcessor();
  return s_pServerDataProcessor;
}

ServerDataProcessor::ServerDataProcessor() :
  RobotDataHandler(),
  pNetComm_( NULL )
{
  defaultRobotInfo_.pose.x = 0.0;
  defaultRobotInfo_.pose.y = 0.0;
  defaultRobotInfo_.pose.theta = 0.0;
  defaultRobotInfo_.type = UNKNOWN;
  defaultRobotInfo_.nofcams = defaultRobotInfo_.nofaudios = 0;

  setTeamMemberID( 1, BlueTeam );
  cmdHandlerList_.clear();
}

ServerDataProcessor::~ServerDataProcessor()
{
}

void ServerDataProcessor::init( const VideoDeviceList & videoObjs, const AudioDeviceList & audioObjs )
{
  if (!pNetComm_) {
    pNetComm_ = new PyRideNetComm( this );
    pNetComm_->init( videoObjs, audioObjs );
  }
  activeVideoObjs_ = (VideoDeviceList *)&videoObjs;
}

void ServerDataProcessor::fini()
{
  this->removeCommandHandler();

  if (pNetComm_) {
    pNetComm_->fini();
    pNetComm_ = NULL;
  }
}

void ServerDataProcessor::addCommandHandler( PyRideExtendedCommandHandler * cmdHandler )
{
  if (cmdHandler) {
    cmdHandlerList_.push_back( cmdHandler );
  }
}

void ServerDataProcessor::removeCommandHandler( PyRideExtendedCommandHandler * cmdHandler )
{
  if (cmdHandler) {
    PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
    while ((*iter != cmdHandler) && iter != cmdHandlerList_.end()) { iter++; }
    if (iter != cmdHandlerList_.end()) {
      cmdHandlerList_.erase( iter );
    }
    if (!cmdHandlerList_.empty()) {
      return;
    }
  }
  else {
    cmdHandlerList_.clear();
  }
}

void ServerDataProcessor::discoverConsoles()
{
  if (pNetComm_) {
    pNetComm_->declareRobot( &defaultRobotInfo_ );
  }
}

void ServerDataProcessor::disconnectConsoles()
{
  if (pNetComm_) {
    pNetComm_->disconnectConsoles();
  }
}

void ServerDataProcessor::disconnectConsole( SOCKET_T fd )
{
  if (pNetComm_) {
    pNetComm_->disconnectClientWithFD( fd );
  }
}

void ServerDataProcessor::setTeamMemberID( int number, TeamColour team )
{
  if (number < 1 || number > 10) { // artificial max team number
    ERROR_MSG( "Invalid team number %d\n", number );
    return;
  }
  clientID_ = (number & 0xf) << 4 | team;
}

void ServerDataProcessor::setTeamColour( TeamColour team )
{
  clientID_ = (clientID_ & 0xf0) | team;
}

void ServerDataProcessor::updateRobotTelemetryWithDefault()
{
  ObservedObjects objects;
  this->updateRobotTelemetry( defaultRobotInfo_.pose, objects );
}

void ServerDataProcessor::updateRobotTelemetry( float x, float y, float heading )
{
  if (telemetryClients_ <= 0)
    return;

  RobotPose pose;
  ObservedObjects objects;
  
  pose.x = x; pose.y = y; pose.theta = heading;
  this->updateRobotTelemetry( pose, objects );
}

void ServerDataProcessor::updateRobotTelemetry( RobotPose & pose, ObservedObjects & objects )
{
  if (telemetryClients_ <= 0)
    return;
  
  if (isnan( pose.x ) || isnan( pose.y ))
    return;

  if (isnan( pose.theta ))
    pose.theta = 0.0;

  int dataSize = sizeof( RobotPose ) + sizeof( FieldObject ) * objects.size();
  unsigned char * dataBuf = new unsigned char[dataSize];
  unsigned char * dataPtr = dataBuf;

  memcpy( dataPtr, (void *)&pose, sizeof( RobotPose ) ); dataPtr += sizeof( RobotPose );
  for (size_t i = 0; i < objects.size(); i++) {
    memcpy( dataPtr, (void *)&objects[i], sizeof( FieldObject ) );
    dataPtr += sizeof( FieldObject );
  }
  pNetComm_->dispatchTelemetryData( dataBuf, dataSize );
  delete [] dataBuf;
}

void ServerDataProcessor::blockRemoteExclusiveControl( bool isyes )
{
  if (pNetComm_) {
    pNetComm_->blockRemoteExclusiveControl( isyes );
  }
}
  

int ServerDataProcessor::getMyIPAddress()
{
  int addr = 0;
  if (pNetComm_) {
    pNetComm_->getMyIPAddress( addr );
  }
  return addr;
}

void ServerDataProcessor::takeCameraSnapshot( bool takeAllCamera )
{
  if (pNetComm_) {
    pNetComm_->takeCameraSnapshot( this, takeAllCamera );
  }
}

void ServerDataProcessor::updateOperationalStatus( RobotOperationalState status,
                                                        const char * optionalData,
                                                        const int optionalDataLength )
{
  if (optionalDataLength < 0)
    return;
  
  unsigned char * data = new unsigned char[optionalDataLength+1];
  data[0] = (unsigned char) status;
  memcpy( data+1, optionalData, optionalDataLength );

  pNetComm_->dispatchStatusData( data, optionalDataLength + 1 );
  delete [] data;
}

long ServerDataProcessor::addTimer( int initialTime, long repeats, int interval )
{
  if (repeats < 0) {
    return pNetComm_->addTimer( initialTime, -1, interval );
  }
  return pNetComm_->addTimer( initialTime, repeats, interval );
}

void ServerDataProcessor::delTimer( long tID )
{
  pNetComm_->delTimer( tID );
}

bool ServerDataProcessor::isTimerRunning( long tID )
{
  return pNetComm_->isTimerRunning( tID );
}

long ServerDataProcessor::totalTimers()
{
  return pNetComm_->totalTimers();
}

void ServerDataProcessor::delAllTimers()
{
  pNetComm_->delAllTimers();
}

bool ServerDataProcessor::executeRemoteCommand( const unsigned char * commandData,
                                                   const int dataLength )
{
  if (cmdHandlerList_.empty())
    return false;

  //INFO_MSG( "command data received %d\n", dataLength );
  PyRideExtendedCommand command = (PyRideExtendedCommand) commandData[0];
  unsigned char * data = (unsigned char *)commandData + 1;
  for (PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
       iter != cmdHandlerList_.end(); iter++)
  {
    if ((*iter)->executeRemoteCommand( command, data, dataLength - 1 )) { // data has already been consumed, the handler does not wish other handlers to process the data again.
      break;
    }
  }
  return true;
}

void ServerDataProcessor::cancelCurrentOperation()
{
  for (PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
       iter != cmdHandlerList_.end(); iter++)
  {
    (*iter)->cancelCurrentOperation();
  }
}

int ServerDataProcessor::activeVideoObjectList( std::vector<std::string> & namelist )
{
  namelist.clear();
  int vsize = activeVideoObjs_->size();

  for (int i = 0; i < vsize ; ++i) {
    DeviceInfo & info = activeVideoObjs_->at( i )->deviceInfo();
    namelist.push_back( info.deviceID );
  }
  return vsize;
}

bool ServerDataProcessor::dispatchVideoDataTo( int vidObjID, struct sockaddr_in & cAddr, short port, bool todispath )
{
  int vsize = activeVideoObjs_->size();
  
  if (vidObjID < 0 || vidObjID >= vsize) {
    return false;
  }
  VideoDevice * device = activeVideoObjs_->at( vidObjID );
  if (todispath) {
    device->start( cAddr, port );
  }
  else {
    device->stop( cAddr, port );
  }
  return true;
}

void ServerDataProcessor::onTimer( const long timerID )
{
  for (PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
       iter != cmdHandlerList_.end(); iter++)
  {
    (*iter)->onTimer( timerID );
  }
}

void ServerDataProcessor::onTimerLapsed( const long timerID )
{
  for (PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
       iter != cmdHandlerList_.end(); iter++)
  {
    (*iter)->onTimerLapsed( timerID );
  }
}

bool ServerDataProcessor::onUserLogOn( const unsigned char * authCode, SOCKET_T fd, struct sockaddr_in & addr )
{
  bool retVal = false;

  std::string username;
  if (AppConfigManager::instance()->signInUserWithPassword( authCode, fd, addr, username )) {
    for (PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
         iter != cmdHandlerList_.end(); iter++)
    {
      retVal |= (*iter)->onUserLogOn( username );
    }
    return retVal;
  }
  else {
    return false;
  }
}

void ServerDataProcessor::onUserLogOff( SOCKET_T fd )
{
  std::string username;
  if (AppConfigManager::instance()->signOutUser( fd, username )) {
    for (PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
         iter != cmdHandlerList_.end(); iter++)
    {
      (*iter)->onUserLogOff( username );
    }
  }
}

void ServerDataProcessor::onTelemetryStreamControl( bool isStart )
{
  for (PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
       iter != cmdHandlerList_.end(); iter++)
  {
    (*iter)->onTelemetryStreamControl( isStart );
  }
}

void ServerDataProcessor::onSnapshotImage( const string & imageName )
{
  for (PyRideExtendedCommandHandlerList::iterator iter = cmdHandlerList_.begin();
       iter != cmdHandlerList_.end(); iter++)
  {
    (*iter)->onSnapshotImage( imageName );
  }
}

// helper functions
} // namespace pyride
