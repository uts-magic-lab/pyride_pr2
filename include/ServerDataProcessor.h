//
//  ServerDataProcessor.h
//  PyRIDE
//
//  Created by Xun Wang on 10/05/10.
//  Copyright 2010 GalaxyNetwork. All rights reserved.
//

#ifndef ServerDataProcessor_h_DEFINED
#define ServerDataProcessor_h_DEFINED

#include <vector>
#include "PyRideNetComm.h"

using namespace std;

namespace pyride {

typedef std::vector< FieldObject > ObservedObjects;

class ServerDataProcessor;

class PyRideExtendedCommandHandler : public VideoDeviceDataHandler
{
protected:
  virtual bool executeRemoteCommand( PyRideExtendedCommand command, 
                                    const unsigned char * optinalData = NULL,
                                    const int optionalDataLength = 0 ) = 0;
  virtual void cancelCurrentOperation() = 0;
  virtual bool onUserLogOn( const unsigned char * authCode, SOCKET_T fd, struct sockaddr_in & addr ) { return false; }
  virtual void onUserLogOff( SOCKET_T fd ) {}
  virtual void onTimer( const long timerID ) {}
  virtual void onTimerLapsed( const long timerID ) {}

  virtual void onTelemetryStreamControl( bool isStart ) {};
  
  friend class ServerDataProcessor;
};

class ServerDataProcessor : public RobotDataHandler, VideoDeviceDataHandler
{
public:
  static ServerDataProcessor * instance();
  ~ServerDataProcessor();

  void init( const VideoDeviceList & videoObjs, const AudioDeviceList & audioObjs );
  void fini();
  void addCommandHandler( PyRideExtendedCommandHandler * cmdHandler = NULL );
  void removeCommandHandler( PyRideExtendedCommandHandler * cmdHandler = NULL );
  
  void discoverConsoles();
  void disconnectConsole( SOCKET_T fd );
  void disconnectConsoles();

  void setClientID( const char clientID ) { clientID_ = clientID; }
  void setTeamMemberID( int number = 1, TeamColour team = BlueTeam );
  void setTeamColour( TeamColour team );
  void updateRobotTelemetry( float x, float y, float heading );
  void updateRobotTelemetryWithDefault();
  void updateRobotTelemetry( RobotPose & pos, ObservedObjects & objects );

  void blockRemoteExclusiveControl( bool isyes );
  
  void takeCameraSnapshot( bool takeAllCamera );
  int getMyIPAddress();
  
  int activeVideoObjectList( std::vector<std::string> & namelist );
  bool dispatchVideoDataTo( int vidObjID, struct sockaddr_in & cAddr,
                            short port, bool todispath );
  
  void updateOperationalStatus( RobotOperationalState status, const char * optionalData = NULL,
                                  const int optionalDataLength = 0 );  
  TeamColour teamColour() { return (TeamColour)(clientID_ & 0xf); }
  int teamMemberID() { return (clientID_ >> 4); }
  const RobotInfo & defaultRobotInfo() { return defaultRobotInfo_; }
  void setDefaultRobotInfo( const RobotType rtype, const RobotPose & pose )
  {
    defaultRobotInfo_.type = rtype;
    defaultRobotInfo_.pose = pose;
  }

  long addTimer( int initialTime, long repeats = 0, int interval = 1 );
  void delTimer( long tID );
  void delAllTimers();
  long totalTimers();
  bool isTimerRunning( long tID );

private:
  typedef std::vector<PyRideExtendedCommandHandler *> PyRideExtendedCommandHandlerList;
  PyRideNetComm * pNetComm_;
  PyRideExtendedCommandHandlerList cmdHandlerList_;
  VideoDeviceList * activeVideoObjs_;

  RobotInfo defaultRobotInfo_;
  
  static ServerDataProcessor * s_pServerDataProcessor;

  ServerDataProcessor();

  // RobotDataHandler
  bool executeRemoteCommand( const unsigned char * commandData, const int dataLength );
  void cancelCurrentOperation();
  bool onUserLogOn( const unsigned char * authCode, SOCKET_T fd, struct sockaddr_in & addr );
  void onUserLogOff( SOCKET_T fd );
  void onTimer( const long timerID );
  void onTimerLapsed( const long timerID );
  
  void onTelemetryStreamControl( bool isStart );
  void onSnapshotImage( const string & imageName );
};
} // namespace pyride
#endif  // ServerDataProcessor_h_DEFINED
