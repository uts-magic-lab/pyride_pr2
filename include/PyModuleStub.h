//
//  PyModuleStub.h
//  PyRIDE
//
//  Created by Xun Wang on 8/04/11.
//  Copyright 2011 Galaxy Network. All rights reserved.
//
#ifndef PyModuleStub_h_DEFINED
#define PyModuleStub_h_DEFINED

#include "ServerDataProcessor.h"
#ifdef IOS_BUILD
#include "iOSAppConfigManager.h"
#else
#include "AppConfigManager.h"
#endif

// Python redefine the following definition
// and cause compiler warning.
#ifdef _POSIX_C_SOURCE
#undef _POSIX_C_SOURCE
#endif

#ifdef _XOPEN_SOURCE
#undef _XOPEN_SOURCE
#endif

#include <Python.h>

namespace pyride {

class PyOutputWriter
{
public:
  virtual void write( const char * msg ) = 0;
  virtual void broadcastMessage( const char * data ) = 0;
  virtual PyObject * mainScript() = 0;
  virtual ~PyOutputWriter() {}
};

class PyModuleExtendedCommandHandler;

class PyModuleExtension
{
public:
  PyModuleExtension( const char * name );
  virtual ~PyModuleExtension();
  
  std::string & name() { return name_; }

  PyObject * init( PyOutputWriter * pow );
  void invokeCallback( const char * fnName, PyObject * arg );
  void write( const char * str );
  
  void sendTeamMessage( const char * mesg );
  void setTeamColour( TeamColour teamColour );

  void fini();
  char clientID() const { return clientID_; }
  void clientID( char cID );

protected:
  char clientID_;
  std::string name_;
  
  PyOutputWriter * pow_;
  PyObject * pPyModule_;
  PyModuleExtendedCommandHandler * pyModuleCommandHandler_;
  
  virtual PyObject * createPyModule() = 0;
  void swapCallbackHandler( PyObject * & master, PyObject * newObj );
  void invokeCallbackHandler( PyObject * & cbObj, PyObject * arg );
};
  
class PyModuleExtendedCommandHandler : public PyRideExtendedCommandHandler
{
public:
  PyModuleExtendedCommandHandler( PyModuleExtension * pyExtModule = NULL );
  
private:
  bool executeRemoteCommand( PyRideExtendedCommand command,
                            const unsigned char * optionalData = NULL,
                            const int optionalDataLength = 0 );
  void cancelCurrentOperation();
  
  bool onUserLogOn( const unsigned char * authCode, SOCKET_T fd, struct sockaddr_in & addr );
  void onUserLogOff( SOCKET_T fd );
  void onTimer( const long timerID );
  void onTimerLapsed( const long timerID );
  
  void onSnapshotImage( const string & name );

  PyModuleExtension * pyExtModule_;
};

extern std::vector<long> g_PyModuleTimerList;

} // namespace pyride

#endif // PyModuleStub_h_DEFINED
