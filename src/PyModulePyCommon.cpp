/*
 *  PyModulePyCommon.cpp
 *  PyRIDE
 *
 *  Created by Xun Wang on 16/12/13.
 *  Copyright 2013 Galaxy Network. All rights reserved.
 *
 */
#include <arpa/inet.h>

#ifdef DEFINE_COMMON_PYMODULE_METHODS
  { "declare", (PyCFunction)PyModule_declare, METH_NOARGS,
    "Declare the robot to all available PyRIDE remote clients." },
  { "disconnect", (PyCFunction)PyModule_disconnect, METH_VARARGS,
    "Disconnect PyRIDE one or all consoles." },
  { "takeCameraSnapshot", (PyCFunction)PyModule_TakeCameraSnapshot, METH_VARARGS,
    "Use robot camera to take a snapshot." },
  { "getMyIPAddress", (PyCFunction)PyModule_GetMyIPAddress, METH_NOARGS,
    "Get the current robot IP address." },
  { "updateOperationalStatus", (PyCFunction)PyModule_UpdateOperationalStatus, METH_VARARGS,
    "Dispatch robot operational data to PyRIDE consoles." },
  { "updateRobotTelemetry", (PyCFunction)PyModule_UpdateRobotTelemetry, METH_VARARGS,
    "Dispatch robot telemetry data." },
  { "listCurrentUsers", (PyCFunction)PyModule_ListCurrentUsers, METH_NOARGS,
    "List all users who are currently log on the robot." },
  { "listAllUsers", (PyCFunction)PyModule_ListAllUsers, METH_NOARGS,
    "List all users on the robot." },
  { "blockRemoteExclusiveControl", (PyCFunction)PyModule_BlockRemoteExclusiveControl, METH_VARARGS,
    "Block or unblock remote client exclusive control of the robot." },
  { "saveConfiguration", (PyCFunction)PyModule_SaveConfiguration, METH_NOARGS,
    "Save the current PyRIDE configuration." },
  { "addUser", (PyCFunction)PyModule_AddUser, METH_VARARGS,
    "Add a new user on the robot." },
  { "removeUser", (PyCFunction)PyModule_RemoveUser, METH_VARARGS,
    "Remove an existing user on the robot." },
  { "changeUserPassword", (PyCFunction)PyModule_ChangeUserPassword, METH_VARARGS,
    "Change the password of an existing user on the robot." },
  { "addTimer", (PyCFunction)PyModule_AddTimer, METH_VARARGS,
    "Add a new timer object." },
  { "isTimerRunning", (PyCFunction)PyModule_IsTimerRunning, METH_VARARGS,
    "Check a timer with ID is still running." },
  { "removeTimer", (PyCFunction)PyModule_RemoveTimer, METH_VARARGS,
    "Remove a timer object with ID." },
  { "removeAllTimers", (PyCFunction)PyModule_RemoveAllTimers, METH_NOARGS,
    "Remove all timer objects." },
  { "listActiveVideoObjects", (PyCFunction)PyModule_ActiveVideoObjects, METH_NOARGS,
    "List the ID of all active video objects." },
  { "dispatchVideoTo", (PyCFunction)PyModule_DispatchVideoData, METH_VARARGS|METH_KEYWORDS,
    "Send image data from a video object to a specific remote host." },
#undef DEFINE_COMMON_PYMODULE_METHODS
#endif

#ifdef INCLUDE_COMMON_PYMODULE_MEHTODS

#ifndef PYRIDE_ROBOT_MODEL
#error You must define PYRIDE_ROBOT_MODEL macro first!
#endif

/*
 *  Common static methods in PyRIDE Python robot extension module
 *
 */
/** @name Remote Client Access Functions
 *
 */
/**@{*/
static PyObject * PyModule_declare( PyObject * self )
{
  ServerDataProcessor::instance()->discoverConsoles();
  Py_RETURN_NONE;
}

/*! \fn disconnect( username )
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Disconnect all remote clients.
 *  \param str username. Optional name of an online user to be disconnected. If no username is provided. All connected clients will be disconnected.
 *  \return None
 */
static PyObject * PyModule_disconnect( PyObject * self, PyObject * args )
{
  char * username = NULL;

  if (!PyArg_ParseTuple( args, "|s", &username )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  SOCKET_T fd = INVALID_SOCKET;
  if (username) {
    if (AppConfigManager::instance()->getOnlineUserClientFD( username, fd )) {
      ServerDataProcessor::instance()->disconnectConsole( fd );
    }
    else {
      PyErr_Format( PyExc_ValueError, "Py%s.disconnect: user '%s' is not online!",
            PYRIDE_ROBOT_MODEL, username );
      return NULL;
    }
  }
  else {
    ServerDataProcessor::instance()->disconnectConsoles();
  }
  Py_RETURN_NONE;
}

/*! \fn blockRemoteExclusiveControl( toblock )
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Block or unblock exclusive control of connected remote clients.
 *
 *  Allow or block the exclusive control right of remote clients. If a client
 *  has already taken the exclusive control, the control right will be revoked
 *  when the input is set to True.
 *  \param bool toblock. True for block; False for unblock.
 *  \return None
 */
static PyObject * PyModule_BlockRemoteExclusiveControl( PyObject * self, PyObject * args )
{
  PyObject * isYesObj = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &isYesObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (!PyBool_Check( isYesObj )) {
    PyErr_Format( PyExc_ValueError, "Py%s.blockRemoteExclusiveControl: the parameter must be a boolean!", PYRIDE_ROBOT_MODEL );
    return NULL;
  }
  ServerDataProcessor::instance()->blockRemoteExclusiveControl( PyObject_IsTrue( isYesObj ) );
  Py_RETURN_NONE;
}

/*! \fn updateOperationalStatus(op_state, op_text)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Send robot operational status notification to all connected remote clients.
 *  \param int op_state. Must be a positive integer representing the operational status.
 *  \param str op_text. Optional text for the operational status.
 *  \return None.
 *  \note For pre allocated operational status ID, check constants.py.
 */
/**@}*/
static PyObject * PyModule_UpdateOperationalStatus( PyObject * self, PyObject * args )
{
  int state;
  char * dataStr = NULL;
  
  if (!PyArg_ParseTuple( args, "i|s", &state, &dataStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (state < 0 || state > 4) {
    PyErr_Format( PyExc_ValueError, "Py%s.updateOperationalStatus: invalid status %d!", PYRIDE_ROBOT_MODEL,
                 state );
    return NULL;
  }
  if (dataStr) {
    ServerDataProcessor::instance()->updateOperationalStatus( (RobotOperationalState)state,
                                                             dataStr,
                                                             (int)strlen( dataStr ) );
  }
  else {
    ServerDataProcessor::instance()->updateOperationalStatus( (RobotOperationalState)state );
  }
  
  Py_RETURN_NONE;
}

/** @name Miscellaneous Functions
 *
 */
/**@{*/
/*! \fn takeCameraSnapshot( all_camera )
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Take a snapshot for the robot camera(s)
 *  \param bool all_camera. Take a snapshot from all available cameras.
 *  \return None
 *  \note Image(s) taken from the robot camera(s) will be saved in <b>/removeable/recordings/cameras</b>
 *  directory on the PR2.
 */
static PyObject * PyModule_TakeCameraSnapshot( PyObject * self, PyObject * args )
{
  PyObject * takeAllObj = NULL;
  
  if (!PyArg_ParseTuple( args, "|O", &takeAllObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (takeAllObj && !PyBool_Check( takeAllObj )) {
    PyErr_Format( PyExc_ValueError, "Py%s.takeCameraSnapshot: the parameter must be a boolean!", PYRIDE_ROBOT_MODEL );
    return NULL;
  }
  
  ServerDataProcessor::instance()->takeCameraSnapshot( (takeAllObj && PyObject_IsTrue( takeAllObj )) );
  Py_RETURN_NONE;
}

/*! \fn getMyIPAddress()
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Get the IP address of the robot.
 *  \return The robot IP address in text.
 */
/**@}*/
static PyObject * PyModule_GetMyIPAddress( PyObject * self )
{
  struct in_addr myaddr;
  myaddr.s_addr = ntohl( ServerDataProcessor::instance()->getMyIPAddress() );
  if (myaddr.s_addr) {
    return Py_BuildValue( "s", inet_ntoa( myaddr ) );
  }
  else {
    Py_RETURN_NONE;
  }
}

static PyObject * PyModule_UpdateRobotTelemetry( PyObject * self, PyObject * args )
{
  float pos_x, pos_y, pos_theta;
  
  if (!PyArg_ParseTuple( args, "fff", &pos_x, &pos_y, &pos_theta )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  ServerDataProcessor::instance()->updateRobotTelemetry( pos_x, pos_y, pos_theta );
  
  Py_RETURN_NONE;
}

/** @name Remote Client User Management Functions
 *
 */
/**@{*/
/*! \fn listCurrentUsers()
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Return a list of users who are currently logged in through remote clients.
 *
 *  \return A list of user names.
 */
static PyObject * PyModule_ListCurrentUsers( PyObject * self )
{
  std::vector<std::string> userList;
  
  int nofusers = AppConfigManager::instance()->listCurrentUsers( userList );
  
  PyObject * retList = PyList_New( nofusers );
  PyObject * item = NULL;
  for (int i = 0; i < nofusers; i++) {
    item = Py_BuildValue( "s", userList[i].c_str() );
    PyList_SetItem( retList, i, item );
  }
  return retList;
}

/*! \fn listAllUsers()
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Return a list of users who can log in remotely.
 *
 *  \return A list of user names.
 */
static PyObject * PyModule_ListAllUsers( PyObject * self )
{
  std::vector<std::string> userList;
  
  int nofusers = AppConfigManager::instance()->listAllUsers( userList );
  
  PyObject * retList = PyList_New( nofusers );
  PyObject * item = NULL;
  for (int i = 0; i < nofusers; i++) {
    item = Py_BuildValue( "s", userList[i].c_str() );
    PyList_SetItem( retList, i, item );
  }
  return retList;
}

/*! \fn saveConfiguration()
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Save the current user access configuration.
 *
 *  \return None.
 */
static PyObject * PyModule_SaveConfiguration( PyObject * self )
{
  AppConfigManager::instance()->saveConfig();
  Py_RETURN_NONE;
}

/*! \fn addUser(user_name, user_password)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Add a new remote user into the system.
 *  \param str user_name.
 *  \param str user_password.
 *  \return None.
 */
static PyObject * PyModule_AddUser( PyObject * self, PyObject * args )
{
  char * nameStr = NULL;
  char * passwordStr = NULL;
  
  if (!PyArg_ParseTuple( args, "ss", &nameStr, &passwordStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (!AppConfigManager::instance()->addUser( nameStr, passwordStr )) {
    PyErr_Format( PyExc_ValueError, "Py%s.addUser: invalid user name and password combination."
                 "Choose a different name or password, make sure password is longer than four characters.", PYRIDE_ROBOT_MODEL );
    return NULL;
  }
  Py_RETURN_NONE;
}

/*! \fn removeUser(user_name)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Remove an existing remote user from the system.
 *  \param str user_name name.
 *  \return None.
 */
static PyObject * PyModule_RemoveUser( PyObject * self, PyObject * args )
{
  char * nameStr = NULL;
  
  if (!PyArg_ParseTuple( args, "s", &nameStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (!AppConfigManager::instance()->delUser( nameStr )) {
    PyErr_Format( PyExc_ValueError, "Py%s.delUser: user '%s' does not exist.", PYRIDE_ROBOT_MODEL, nameStr );
    return NULL;
  }
  Py_RETURN_NONE;
}

/*! \fn changeUserPassword(user_name, old_password, new_password)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Change an existing remote user's password.
 *  \param str user_name name.
 *  \param str old_password.
 *  \param str new_password.
 *  \return None.
 */
/**@}*/
static PyObject * PyModule_ChangeUserPassword( PyObject * self, PyObject * args )
{
  char * nameStr = NULL;
  char * newPasswordPtr = NULL;
  char * oldPasswordPtr = NULL;
  
  if (!PyArg_ParseTuple( args, "sss", &nameStr, &oldPasswordPtr, &newPasswordPtr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (!AppConfigManager::instance()->changeUserPassword( nameStr, oldPasswordPtr, newPasswordPtr )) {
    PyErr_Format( PyExc_ValueError, "Py%s.changeUserPassword: invalid user name and password combination."
                 "Make sure you have correct user name and old password, new password is longer than four characters.", PYRIDE_ROBOT_MODEL );
    return NULL;
  }
  Py_RETURN_NONE;
}

/** @name Timer Management Functions
 *
 */
/**@{*/
/*! \fn addTimer(init_start, repeats, interval)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Add a timer object
 *  \param int init_start. Initial start time (in seconds) after the method is called.
 *  \param long repeats. A number of times the timer shall be called. -1 means infinite.
 *  \param int interval. Time interval (in seconds) between timer callbacks.
 *  \return long The ID of the timer object.
 */
static PyObject * PyModule_AddTimer( PyObject * self, PyObject * args )
{
  int initTime;
  long repeats = 0;
  long interval = 1;
  
  if (!PyArg_ParseTuple( args, "i|ll", &initTime, &repeats, &interval )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (initTime <= 0) {
    PyErr_Format( PyExc_ValueError, "Py%s.addTimer: invalid initial invoking time."
                 "The first argument must be greater than zero.", PYRIDE_ROBOT_MODEL );
    return NULL;
  }
  long timerID = ServerDataProcessor::instance()->addTimer( initTime, repeats, interval );
  
  g_PyModuleTimerList.push_back( timerID );
  
  return Py_BuildValue( "l", timerID );
}

/*! \fn removeTimer(timer_id)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Remove a timer object.
 *  \param long timer_id. ID of the timer object pending for removal.
 *  \return None.
 */
static PyObject * PyModule_RemoveTimer( PyObject * self, PyObject * args )
{
  long timerID;
  
  if (!PyArg_ParseTuple( args, "l", &timerID )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (timerID <= 0) {
    PyErr_Format( PyExc_ValueError, "Py%s.removeTimer: invalid timer ID."
                 "The ID must be greater than zero.", PYRIDE_ROBOT_MODEL );
    return NULL;
  }
  bool found = false;
  std::vector<long>::iterator iter;
  for (iter = g_PyModuleTimerList.begin(); iter != g_PyModuleTimerList.end(); iter++) {
    if (*iter == timerID) {
      found = true;
      break;
    }
  }
  if (!found) {
    PyErr_Format( PyExc_ValueError, "Py%s.removeTimer: unknown timer ID %d.", PYRIDE_ROBOT_MODEL, (int)timerID );
    return NULL;
  }
  g_PyModuleTimerList.erase( iter );
  ServerDataProcessor::instance()->delTimer( timerID );
  
  Py_RETURN_NONE;
}

/*! \fn isTimerRunning(timer_id)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Check whether a timer object is still alive.
 *  \param long timer_id. ID of the timer object.
 *  \return True = timer is alive; False = timer is dead.
 */
static PyObject * PyModule_IsTimerRunning( PyObject * self, PyObject * args )
{
  long timerID;
  
  if (!PyArg_ParseTuple( args, "l", &timerID )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (timerID <= 0) {
    PyErr_Format( PyExc_ValueError, "Py%s.isTimerRunning: invalid timer ID."
                 "The ID must be greater than zero.", PYRIDE_ROBOT_MODEL );
    return NULL;
  }
  if (ServerDataProcessor::instance()->isTimerRunning( timerID )) {
    Py_RETURN_TRUE;
  }
  else {
    Py_RETURN_FALSE;
  }
}

/*! \fn removeAllTimers()
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Remove all timer objects in the system.
 *  \return None.
 */
/**@}*/
static PyObject * PyModule_RemoveAllTimers( PyObject * self )
{
  for (size_t i = 0; i < g_PyModuleTimerList.size(); i++) {
    ServerDataProcessor::instance()->delTimer( g_PyModuleTimerList[i] );
  }
  g_PyModuleTimerList.clear();
  Py_RETURN_NONE;
}

static PyObject * PyModule_ActiveVideoObjects( PyObject * self )
{
  std::vector<std::string> deviceList;
  
  int nofdevice = ServerDataProcessor::instance()->activeVideoObjectList( deviceList );
  
  PyObject * retList = PyList_New( nofdevice );
  PyObject * item = NULL;
  for (int i = 0; i < nofdevice; i++) {
    item = Py_BuildValue( "s", deviceList[i].c_str() );
    PyList_SetItem( retList, i, item );
  }
  return retList;
}

static const char *kDispatchVideoKWlist[] = { "devidx", "host", "port", "todispatch", NULL };

static PyObject * PyModule_DispatchVideoData( PyObject * self, PyObject * args, PyObject * keywds )
{
  int devid = -1;
  char * hostname = NULL;
  int port = 0;
  PyObject * isYesObj = NULL;
  
  if (!PyArg_ParseTupleAndKeywords( args, keywds, "isiO", (char**)kDispatchVideoKWlist, &devid, &hostname, &port, &isYesObj ) ||
      !PyBool_Check( isYesObj )) {  // PyArg_ParseTuple will set the error status.
    PyErr_Format( PyExc_ValueError, "Py%s.dispatchVideoTo: invalid keyworded input parameters.", PYRIDE_ROBOT_MODEL );
    return NULL;
  }

  if (devid < 0) {
    PyErr_Format( PyExc_ValueError, "Py%s.dispatchVideoTo: selected device index must be a non-negative number.", PYRIDE_ROBOT_MODEL );
    return NULL;
  }
  
  unsigned long saddr = 0;
  struct hostent * hostInfo = gethostbyname( hostname ); // try resolve name first
  if (!hostInfo) {
#ifdef WIN32
    saddr = inet_addr( host );
    if (saddr == INADDR_NONE) {
#else
    if (inet_pton( AF_INET, hostname, &saddr ) != 1) {
#endif
      PyErr_Format( PyExc_ValueError, "Py%s.dispatchVideoTo: unable to resove host %s.", PYRIDE_ROBOT_MODEL, hostname );
      return NULL;
    }
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

  if (port < 20000 || port > 55000) {
    PyErr_Format( PyExc_ValueError, "Py%s.dispatchVideoTo: remote host port must be greater than 20000.", PYRIDE_ROBOT_MODEL );
    return NULL;
  }

  if (ServerDataProcessor::instance()->dispatchVideoDataTo( devid, cAddr, (short)port, PyObject_IsTrue( isYesObj ) )) {
    Py_RETURN_NONE;
  }
  else {
    PyErr_Format( PyExc_SystemError, "Py%s.dispatchVideoTo: %s dispatching video data to %s failed.",
                 PYRIDE_ROBOT_MODEL, PyObject_IsTrue( isYesObj ) ? "start" : "stop", hostname );
    return NULL;
  }
}


#undef INCLUDE_COMMON_PYMODULE_MEHTODS
#endif