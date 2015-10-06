//
//  PyModuleStub.cpp
//  PyRIDE
//
//  Created by Xun Wang on 16/12/13.
//  Copyright 2013 Galaxy Network. All rights reserved.
//
#include "PyModuleStub.h"

namespace pyride {

std::vector<long> g_PyModuleTimerList;

//#pragma mark PyModuleExtension implmentation
PyModuleExtension::PyModuleExtension( const char * name ) :
  clientID_( 0 ),
  pow_( NULL ),
  pPyModule_( NULL ),
  pyModuleCommandHandler_( NULL )
{
  name_ = name;
}

PyModuleExtension::~PyModuleExtension()
{
  this->fini();
}

PyObject * PyModuleExtension::init( PyOutputWriter * pow )
{
  pPyModule_ = this->createPyModule();

  if (!pPyModule_)
    return NULL;
  
  pow_ = pow;
  
  pyModuleCommandHandler_ = new PyModuleExtendedCommandHandler( this );
  ServerDataProcessor::instance()->addCommandHandler( pyModuleCommandHandler_ );
  this->clientID( ServerDataProcessor::instance()->clientID() );
  
  PyObject * arg = NULL;
  if (ServerDataProcessor::instance()->teamColour() == BlueTeam) {
    arg = Py_BuildValue( "s", "blue" );
  }
  else {
    arg = Py_BuildValue( "s", "pink" );
  }
  PyObject_SetAttrString( pPyModule_, "TeamColour", arg );
  //PyModule_AddObject( pyModule, "TeamColour", arg );
  
  Py_DECREF( arg );
  
  arg = Py_BuildValue( "i", ServerDataProcessor::instance()->teamMemberID() );
  PyObject_SetAttrString( pPyModule_, "MemberID", arg );
  //PyModule_AddObject( pyModule, "MemberID", arg );
  Py_DECREF( arg );
  
  return pPyModule_;
}

void PyModuleExtension::fini()
{
  if (pPyModule_) {
    //Py_DECREF( pPyModule_ );
    for (size_t i = 0; i < g_PyModuleTimerList.size(); i++) {
      ServerDataProcessor::instance()->delTimer( g_PyModuleTimerList[i] );
    }
    g_PyModuleTimerList.clear();
    
    pPyModule_ = NULL;
    ServerDataProcessor::instance()->removeCommandHandler( pyModuleCommandHandler_ );
    delete pyModuleCommandHandler_;
    pyModuleCommandHandler_ = NULL;
    pow_ = NULL;
  }
}

void PyModuleExtension::write( const char * str )
{
  if (!pow_)
    return;
  
  pow_->write( str );
}

void PyModuleExtension::sendTeamMessage( const char * mesg )
{
  if (!pow_)
    return;

  if (!mesg || strlen( mesg ) == 0)
    return;
  
  pow_->broadcastMessage( mesg );
}

void PyModuleExtension::clientID( char cID )
{
  if (!pPyModule_)
    return;

  clientID_ = cID;
  
  TeamColour teamColour = (TeamColour)(cID & 0xf);
  int teamID = (cID >> 4) & 0xf;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = NULL;
  if (teamColour == BlueTeam) {
    arg = Py_BuildValue( "s", "blue" );
  }
  else {
    arg = Py_BuildValue( "s", "pink" );
  }
  PyObject_SetAttrString( pPyModule_, "TeamColour", arg );
  
  Py_DECREF( arg );
  
  arg = Py_BuildValue( "i", teamID );
  PyObject_SetAttrString( pPyModule_, "MemberID", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

void PyModuleExtension::setTeamColour( TeamColour teamColour )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = NULL;
  
  if (teamColour == BlueTeam) {
    arg = Py_BuildValue( "s", "blue" );
  }
  else {
    arg = Py_BuildValue( "s", "pink" );
  }
  PyObject_SetAttrString( pPyModule_, "TeamColour", arg );
  
  Py_DECREF( arg );
  PyGILState_Release( gstate );
  ServerDataProcessor::instance()->setTeamColour( teamColour );
  clientID_ = (clientID_ & 0xf0) | (teamColour & 0xf);
}

void PyModuleExtension::invokeCallback( const char * fnName, PyObject * arg )
{
  if (!pPyModule_)
    return;
  
  //DEBUG_MSG( "Attempt get callback function %s\n", fnName );
  
  PyObject * callbackFn = PyObject_GetAttrString( pPyModule_, const_cast<char *>(fnName) );
  if (!callbackFn) {
    PyErr_Clear();
    return;
  }
  else if (!PyCallable_Check( callbackFn )) {
    PyErr_Format( PyExc_TypeError, "%s is not callable object", fnName );
  }
  else {
    PyObject * pResult = PyObject_CallObject( callbackFn, arg );
    if (PyErr_Occurred()) {
      PyErr_Print();
    }
    Py_XDECREF( pResult );
  }
  Py_DECREF( callbackFn );
}

// internal helper functions
void PyModuleExtension::swapCallbackHandler( PyObject * & master, PyObject * newObj )
{
  if (newObj) {
    if (master) {
      Py_DECREF( master );
    }
    master = newObj;
    Py_INCREF( master );
  }
  else if (master) {
    Py_DECREF( master );
    master = NULL;
  }
}

void PyModuleExtension::invokeCallbackHandler( PyObject * & cbObj, PyObject * arg )
{
  if (cbObj) {
    PyObject * pResult = PyObject_CallObject( cbObj, arg );
    if (PyErr_Occurred()) {
      PyErr_Print();
    }
    Py_XDECREF( pResult );
  }
}

//#pragma mark PyModuleExtendedCommandHandler implmentation
PyModuleExtendedCommandHandler::PyModuleExtendedCommandHandler( PyModuleExtension * pyExtModule )
{
  pyExtModule_ = pyExtModule;
}

/** @name Remote Client Access Functions
 *
 */
/*! \typedef onRemoteCommand(cmd_id, cmd_text)
 *  \memberof PyPR2.
 *  \brief Callback function when a custom command is received from a remote client.
 *  \param int cmd_id. Custom command ID.
 *  \param str cmd_text. Custom command text string.
 *  \return None.
 *  \note The remote client must take the exclusive control of the robot before
 *   its commands can trigger this callback function.
 */
bool PyModuleExtendedCommandHandler::executeRemoteCommand( PyRideExtendedCommand command,
                                                       const unsigned char * optionalData,
                                                       const int optionalDataLength )
{
  if (!pyExtModule_)
    return false;
  
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  if (optionalData) {
    std::string data( (char *)optionalData, optionalDataLength );
    arg = Py_BuildValue( "(is)", (int) command, data.c_str() );
  }
  else {
    arg = Py_BuildValue( "(is)", (int) command, "" );
  }
  pyExtModule_->invokeCallback( "onRemoteCommand", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  return true;
}

void PyModuleExtendedCommandHandler::cancelCurrentOperation()
{
  if (!pyExtModule_)
    return;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  pyExtModule_->invokeCallback( "onCurrentOperationCanceled", NULL );
  
  PyGILState_Release( gstate );
}

/*! \typedef onUserLogOn(user_name)
 *  \memberof PyPR2.
 *  \brief Callback function when a user logs in through a remote client.
 *  \param str user_name. The name of the user.
 *  \return None.
 */
bool PyModuleExtendedCommandHandler::onUserLogOn( const std::string & username )
{
  if (!pyExtModule_)
    return false;
  
  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(s)", username.c_str() );

  pyExtModule_->invokeCallback( "onUserLogOn", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );
  //TODO: return return value from Python callback script call
  return true;
}

/*! \typedef onUserLogOff(user_name)
 *  \memberof PyPR2.
 *  \brief Callback function when a remote user logs off.
 *  \param str user_name. The name of the user.
 *  \return None.
 */
/**@}*/
void PyModuleExtendedCommandHandler::onUserLogOff( const std::string & username )
{
  if (!pyExtModule_)
    return;
  
  PyObject * arg = NULL;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  arg = Py_BuildValue( "(s)", username.c_str() );

  pyExtModule_->invokeCallback( "onUserLogOff", arg );
  Py_DECREF( arg );

  PyGILState_Release( gstate );

}

/** @name Timer Management Functions
 *
 */
/**@{*/
/*! \typedef onTimer(timer_id)
 *  \memberof PyPR2.
 *  \brief Callback function when a timer object is fired.
 *  \param int timer_id. ID of the timer object.
 *  \return None.
 */
void PyModuleExtendedCommandHandler::onTimer( const long timerID )
{
  if (!pyExtModule_)
    return;
  
  std::string username;
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  arg = Py_BuildValue( "(i)", timerID );
  
  pyExtModule_->invokeCallback( "onTimer", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

/*! \typedef onTimerLapsed(timer_id)
 *  \memberof PyPR2.
 *  \brief Callback function when a timer object is fired for the last time.
 *  \param int timer_id. ID of the timer object.
 *  \return None.
 *  \note This callback function works only on timers with limited life span.
 */
/**@}*/
void PyModuleExtendedCommandHandler::onTimerLapsed( const long timerID )
{
  if (!pyExtModule_)
    return;
  
  std::string username;
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  arg = Py_BuildValue( "(i)", timerID );
  
  pyExtModule_->invokeCallback( "onTimerLapsed", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

/** @name Miscellaneous Functions
 *
 */
/**@{*/
/*! \typedef onSnapshotImage(image_name)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.takeCameraSnapshot is called.
 *  \param str image_name. Path for the saved image.
 *  \return None.
 */
/**@}*/
void PyModuleExtendedCommandHandler::onSnapshotImage( const string & name )
{
  if (!pyExtModule_)
    return;
  
  std::string username;
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  arg = Py_BuildValue( "(s)", name.c_str() );
  
  pyExtModule_->invokeCallback( "onSnapshotImage", arg );
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}
} // namespace pyride
