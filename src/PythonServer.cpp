#include <string.h>
#include "PythonServer.h"

namespace pyride {

#define max( a, b ) (a > b) ? a : b

static const char goodByeMsg[] = "\r\nGoodbye from UTS PyRIDE Python Console.\r\n";

PythonServer *  PythonServer::s_pPythonServer = NULL;

void * pytelnet_thread( void * processor )
{
  ((PythonServer *)processor)->continuousProcessing();
  return NULL;
}

PythonServer * PythonServer::instance()
{
  if (!s_pPythonServer)
    s_pPythonServer = new PythonServer();
    
  return s_pPythonServer;
}

PythonServer::PythonServer() :
  runThread_( (pthread_t)NULL ),
  isActive_( false ),
  hasInterpreter_( false ),
  prevStderr_( NULL ),
  prevStdout_( NULL ),
  pSysModule_( NULL ),
  pMainModule_( NULL ),
  pMainScript_( NULL ),
  pPyMod_( NULL ),
  udpSocket_( INVALID_SOCKET ),
  tcpSocket_( INVALID_SOCKET ),
  dgramBuffer_( NULL ),
  clientDataBuffer_( NULL ),
  clientList_( NULL ),
  maxFD_( INVALID_SOCKET ),
  runningTelnetConsole_( false ),
  keepRunning_( false ),
  activeSession_( NULL ),
  pyModuleExtension_( NULL )
{
  customPythonHome[0] = '\0';
  customScriptBase[0] = '\0';

  initIPAddresses();
  pthread_mutexattr_init( &t_mta );
  pthread_mutexattr_settype( &t_mta, PTHREAD_MUTEX_RECURSIVE );
  pthread_mutex_init( &t_mutex_, &t_mta );  
}

PythonServer::~PythonServer()
{
  pthread_mutex_destroy( &t_mutex_ );
  pthread_mutexattr_destroy( &t_mta );
}

void PythonServer::init( bool enableTelnetConsole, PyModuleExtension * pyModule, const char * scriptDir, const char * pythonHome )
{
  pyModuleExtension_ = pyModule;

  if (scriptDir) {
    strncpy( customScriptBase, scriptDir,  256 );
  }
  if (pythonHome) {
    strncpy( customPythonHome, pythonHome, 256 );
  }

  if (!initPyInterpreter()) {
    ERROR_MSG( "Unable to initialise Python interpreter!\n" );
    return;
  }

  FD_ZERO( &masterFDSet_ );

  initUDPListener();

  if (enableTelnetConsole)
    initTelnetConsole();
  
  keepRunning_ = true;

  if (pthread_create( &runThread_, NULL, pytelnet_thread, this ) ) {
    ERROR_MSG( "Unable to create thread to pull telnet inputs.\n" );
    return;
  }

  runMainScript();
  isActive_ = true;
}

void PythonServer::fini()
{
  finiTelnetConsole();
 
  keepRunning_ = false;

  pthread_join( runThread_, NULL ); // allow thread to exit

  if (dgramBuffer_) {
    close( udpSocket_ );
    udpSocket_ = INVALID_SOCKET;
    delete [] dgramBuffer_;
    dgramBuffer_ = NULL;
  }
  maxFD_ = 0;
  FD_ZERO( &masterFDSet_ );

  finiPyInterpreter();
}

void PythonServer::initIPAddresses()
{
  sAddr_.sin_family = AF_INET;
  sAddr_.sin_addr.s_addr = INADDR_ANY;
  sAddr_.sin_port = htons( PYTHON_SERVER_PORT );

  bcAddr_.sin_family = AF_INET;
  inet_pton( AF_INET, PYRIDE_BROADCAST_IP, &bcAddr_.sin_addr.s_addr );
  bcAddr_.sin_port = htons( PYTHON_SERVER_PORT );
  
#ifdef USE_MULTICAST
  multiCastReq_.imr_multiaddr.s_addr = bcAddr_.sin_addr.s_addr;
  multiCastReq_.imr_interface.s_addr = INADDR_ANY;
#endif  
}

bool PythonServer::initUDPListener()
{
  if ((udpSocket_ = socket( AF_INET, SOCK_DGRAM, 0 ) ) == INVALID_SOCKET) {
    ERROR_MSG( "PythonServer::initUDPListener: unable to create UDP socket.\n" );
    return false;
  }
  // setup broadcast option
  int turnon = 1;
#ifdef USE_MULTICAST
  char ttl = 30;
  if (setsockopt( udpSocket_, IPPROTO_IP, IP_MULTICAST_TTL, (char *)&ttl, sizeof( ttl ) ) < 0) {
    ERROR_MSG( "PythonServer::initUDPListener: failed to set multicast TTL on UDP socket.\n" );
    close( udpSocket_ );
    return false;
  }
  if (setsockopt( udpSocket_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&multiCastReq_, sizeof( multiCastReq_ ) ) < 0) {
    ERROR_MSG( "PythonServer::initUDPListener: failed to join multicast group on UDP socket.\n" );
    close( udpSocket_ );
    return false;
  }
#else // !USE_MULTICAST
  if (setsockopt( udpSocket_, SOL_SOCKET, SO_BROADCAST, (char *)&turnon, sizeof( turnon ) ) < 0) {
    ERROR_MSG( "PythonServer::initUDPListener: failed to enable broadcast on UDP socket.\n" );
    close( udpSocket_ );
    return false;
  }
#endif

  if (setsockopt( udpSocket_, SOL_SOCKET, SO_REUSEADDR, (char *)&turnon, sizeof( turnon ) ) < 0) {
    ERROR_MSG( "PythonServer::initUDPListener: failed to enable reuse address on UDP socket.\n" );
    close( udpSocket_ );
    return false;
  }

#ifdef SO_REUSEPORT
  setsockopt( udpSocket_, SOL_SOCKET, SO_REUSEPORT, (char *)&turnon, sizeof( turnon ) );
#endif

  if (bind( udpSocket_, (struct sockaddr *)&sAddr_, sizeof( sAddr_ ) ) < 0) {
    ERROR_MSG( "PythonServer::initUDPListener: unable to bind to network interface.\n" );
    close( udpSocket_ );
    return false;
  }

  maxFD_ = max( maxFD_, udpSocket_ );
  FD_SET( udpSocket_, &masterFDSet_ );

  if (dgramBuffer_ == NULL)
    dgramBuffer_ = new unsigned char[PYTHONSERVER_BUFFER_SIZE];

  return true;
}

bool PythonServer::initPyInterpreter()
{
  struct stat dirInfo;
  char scriptPath[256];
  char * evnset = getenv( "SCRIPT_HOME" );

  if (strlen(customScriptBase) > 0 && stat( customScriptBase, &dirInfo ) == 0 && S_ISDIR(dirInfo.st_mode)) {
    strcpy( scriptPath, customScriptBase );
  }
  else if (evnset && stat( evnset, &dirInfo ) == 0 && S_ISDIR(dirInfo.st_mode)) {
    strcpy( scriptPath, evnset );
  }
  else {
    if (strlen(customScriptBase) > 0) {
      ERROR_MSG( "Invalid custom script path %s, use default.\n", customScriptBase );
    }
    else if (evnset) {
      ERROR_MSG( "Invalid custom script path %s, use default.\n", evnset );
    }
#ifdef ROS_BUILD
    int retval = readlink( "/proc/self/exe", scriptPath, 256 );
    if (retval > 0) {
      strcpy( strrchr( scriptPath, '/' ), "/scripts/" );
    }
#else
    strcpy( scriptPath, DEFAULT_PYTHON_SCRIPT_PATH );
#endif
  }
  INFO_MSG( "use script path %s.\n", scriptPath );

  // initialise Python interpreter
  if (Py_IsInitialized()) {
    INFO_MSG( "Python interpreter is already in use, restartpython() is not allowed.\n" );
    hasInterpreter_ = true;
  }
  else {
    if (strlen(customPythonHome)) {
      Py_SetPythonHome( (char *)customPythonHome );
    }
    else {
      Py_SetPythonHome( (char *)"/usr" );
    }

    Py_InitializeEx( 0 );
    PyEval_InitThreads();
    PyEval_ReleaseLock(); // release the GIL lock so that other threads can acquire it.
  }

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  // modify existing system path
  std::string versionStr = strtok( (char*)Py_GetVersion(), " " );
  std::string packagePath = ":";
  packagePath += Py_GetPrefix(); packagePath += "/lib/python";
  packagePath += versionStr.substr( 0, versionStr.find_last_of( '.' ) );
  std::string sysPathStr( Py_GetPath() );
  size_t delpos = sysPathStr.find( ':' );
  sysPathStr.replace( 0, delpos, scriptPath );
  sysPathStr += packagePath + "/site-packages";
  PySys_SetPath( (char*)sysPathStr.c_str() );

  pSysModule_ = PyImport_ImportModule( "sys" );

  if (!pSysModule_) {
    ERROR_MSG( "PythonServer: Failed to import sys module\n" );
    return false;
  }
  
  welcomeStr_ = "Welcome to UTS PyRIDE Python Console [Python version ";
  welcomeStr_ += versionStr + "]";
  
  pMainModule_ = PyImport_AddModule( "__main__" );
  Py_INCREF( pMainModule_ );
  if (!pMainModule_) {
    // we are in deep trouble should abort
    ERROR_MSG( "%s", "PythonServer failed to import __main__ module" );
    return false;
  }

  initModuleExtension();
  PyGILState_Release( gstate );
  
  return true;
}

void PythonServer::finiPyInterpreter()
{
  isActive_ = false;
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyErr_Clear();
  if (pMainScript_) {
    Py_DECREF( pMainScript_ );
    pMainScript_ = NULL;
  }
  finiModuleExtension();
  if (pSysModule_) {
    Py_DECREF( pSysModule_ );
    pSysModule_ = NULL;
  }
  if (pMainModule_) {
    Py_DECREF( pMainModule_ );
    pMainModule_ = NULL;
  }
  PyErr_Clear();
  if (!hasInterpreter_) {
    Py_Finalize();
  }
}

void PythonServer::runMainScript()
{  
  // run main script
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * mainDict = PyModule_GetDict( pMainModule_ );

  pMainScript_ = PyImport_ImportModuleEx( (char *)PYRIDE_MAIN_SCRIPT_NAME, mainDict, mainDict, NULL );

  if (pMainScript_) {
    PyObject_SetAttrString( pMainModule_, PYRIDE_MAIN_SCRIPT_NAME, pMainScript_ );
    PyObject * mainFn = PyObject_GetAttrString( pMainScript_, "main" );
    if (mainFn && PyCallable_Check( mainFn )) {
      PyObject * pResult = PyObject_CallObject( mainFn, NULL );
      if (pResult == NULL) {
        ERROR_MSG( "PythonServer: Failed to run main function in script"
                   " %s\n", PYRIDE_MAIN_SCRIPT_NAME );
        PyErr_Print();
        PyErr_Clear();
      }
      else {
        Py_DECREF( pResult );
      }
    }
    else {
      ERROR_MSG( "PythonServer: missing main function in script %s\n",
                 PYRIDE_MAIN_SCRIPT_NAME );
    }
  }
  else {
    if (PyErr_Occurred() == PyExc_ImportError) {
      ERROR_MSG( "PythonServer: Unable to import main script"
                 " %s\n", PYRIDE_MAIN_SCRIPT_NAME );
      PyErr_Print();
    }
    PyErr_Clear();
  }
  PyGILState_Release( gstate );
}

bool PythonServer::initTelnetConsole()
{
  if (runningTelnetConsole_)
    return true;

  if ((tcpSocket_ = socket( AF_INET, SOCK_STREAM, 0 )) == INVALID_SOCKET) {
    ERROR_MSG( "PythonServer::initTCPListener: unable to create TCP socket.\n" );
    return false;
  }

  int turnon = 1;
  if (setsockopt( tcpSocket_, SOL_SOCKET, SO_REUSEADDR, (char *)&turnon, sizeof( turnon ) ) < 0) {
    ERROR_MSG( "PythonServer::initTCPListener: failed to enable reuse addr option on TCP socket.\n" );
    close( tcpSocket_ );
    return false;
  }

  if (bind( tcpSocket_, (struct sockaddr *)&sAddr_, sizeof( sAddr_ ) ) < 0) {
    ERROR_MSG( "PythonServer::initTCPListener: unable to bind to network interface.\n" );
    close( tcpSocket_ );
    return false;
  }
  
  if (listen( tcpSocket_, 5 ) < 0) {
    ERROR_MSG( "PythonServer::initTCPListener: unable to listen for incoming data.\n" );
    close( tcpSocket_ );
    return false;
  }

  INFO_MSG( "Python server is listening on TCP port %d.\n", PYTHON_SERVER_PORT );
  maxFD_ = max( maxFD_, tcpSocket_ );
  FD_SET( tcpSocket_, &masterFDSet_ );

  if (clientDataBuffer_ == NULL)
    clientDataBuffer_ = new unsigned char[PS_RECEIVE_BUFFER_SIZE];

  prevStderr_ = PyObject_GetAttrString( pSysModule_, "stderr" );
  prevStdout_ = PyObject_GetAttrString( pSysModule_, "stdout" );

  PyObject_SetAttrString( pSysModule_, "stderr", pPyMod_ );
  PyObject_SetAttrString( pSysModule_, "stdout", pPyMod_ );

  runningTelnetConsole_ = true;

  return true;
}

void PythonServer::finiTelnetConsole()
{
  if (!runningTelnetConsole_)
    return;

  runningTelnetConsole_ = false;
  
  disconnectClient( NULL, true );
  close( tcpSocket_ );
  tcpSocket_ = INVALID_SOCKET;

  if (prevStderr_) {
    PyObject_SetAttrString( pSysModule_, "stderr", prevStderr_ );
    Py_DECREF( prevStderr_ );
    prevStderr_ = NULL;
  }

  if (prevStdout_) {
    PyObject_SetAttrString( pSysModule_, "stdout", prevStdout_ );
    Py_DECREF( prevStdout_ );
    prevStdout_ = NULL;
  }

  if (clientDataBuffer_) {
    delete [] clientDataBuffer_;
    clientDataBuffer_ = NULL;
  }
}

void PythonServer::restartPythonServer()
{
  if (hasInterpreter_) {
    ERROR_MSG( "Python interpreter was started by other module. We cannot force it to restart.\n" );
    return;
  }
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  if (prevStderr_) {
    PyObject_SetAttrString( pSysModule_, "stderr", prevStderr_ );
    Py_DECREF( prevStderr_ );
    prevStderr_ = NULL;
  }

  if (prevStdout_) {
    PyObject_SetAttrString( pSysModule_, "stdout", prevStdout_ );
    Py_DECREF( prevStdout_ );
    prevStdout_ = NULL;
  }

  PyGILState_Release( gstate ); // may not be necessary

  this->finiPyInterpreter();
  this->initPyInterpreter();
  this->broadcastServerMessage( "\r\nPython Interpreter has been restarted. Attempt to rerun main script.\r\n" );
  if (runningTelnetConsole_) {
    prevStderr_ = PyObject_GetAttrString( pSysModule_, "stderr" );
    prevStdout_ = PyObject_GetAttrString( pSysModule_, "stdout" );

    PyObject_SetAttrString( pSysModule_, "stderr", pPyMod_ );
    PyObject_SetAttrString( pSysModule_, "stdout", pPyMod_ );
  }
  this->runMainScript();
  //PyEval_ReleaseLock(); // release the GIL lock so that other threads can acquire it.
  isActive_ = true;
}

void PythonServer::processIncomingData( fd_set * readyFDSet )
{
  struct sockaddr_in cAddr;
  int cLen = sizeof( cAddr );
  SOCKET_T fd = INVALID_SOCKET;

  if (FD_ISSET( udpSocket_, readyFDSet )) {
    int readLen = recvfrom( udpSocket_, dgramBuffer_, PS_RECEIVE_BUFFER_SIZE,
      0, (sockaddr *)&cAddr, (socklen_t *)&cLen );
    if (readLen <= 0) {
      ERROR_MSG( "PythonServer::continuousProcessing: error accepting "
        "incoming UDP packet. error %d\n", errno );
    }
    else {
      processUDPInput( dgramBuffer_, readLen, cAddr );
    }
  }

  if (!runningTelnetConsole_)
    return;

  if (FD_ISSET( tcpSocket_, readyFDSet )) {
    // accept incoming TCP connection and read the stream
    fd = accept( tcpSocket_, (sockaddr *)&cAddr, (socklen_t *)&cLen );
    if (fd != INVALID_SOCKET) {
      addFdToClientList( fd, cAddr );
    }
    else if (errno != ECONNABORTED) {
      ERROR_MSG( "PythonServer::continuousProcessing: error accepting "
             "incoming TCP connection error = %d\n", errno );
    }
  }

  pthread_mutex_lock( &t_mutex_ );
  ClientItem * clientPtr = clientList_;
  ClientItem * prevClientPtr = clientPtr;
  while (clientPtr) {
    fd = clientPtr->fd;
    if (FD_ISSET( fd, readyFDSet )) {
      int readLen = read( fd, clientDataBuffer_, PS_RECEIVE_BUFFER_SIZE );
      if (readLen <= 0) {
        if (readLen == 0) {
          INFO_MSG( "Socket connection %d closed.\n", fd );
        }
        else {
          ERROR_MSG( "PythonServer::continuousProcessing: "
                    "error reading data stream on %d error = %d.\n", fd, errno );
        }
        disconnectClient( clientPtr );
      }
      else {
        clientPtr->pSession->processInput( clientPtr, clientDataBuffer_, readLen );
      }
    }
    if (clientPtr->fd == INVALID_SOCKET) { // client has been disconnected
      delete clientPtr->pSession;
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
  pthread_mutex_unlock( &t_mutex_ );
}

void PythonServer::continuousProcessing()
{
  int maxFD = 0;
  fd_set readyFDSet;

  while (keepRunning_) {
    FD_ZERO( &readyFDSet );
    memcpy( &readyFDSet, &masterFDSet_, sizeof( masterFDSet_ ) );
    maxFD = maxFD_;

    select( maxFD + 1, &readyFDSet, NULL, NULL, NULL );

    this->processIncomingData( &readyFDSet );
  }
}

void PythonServer::processUDPInput( const unsigned char * recBuffer, int recBytes, struct sockaddr_in & cAddr )
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

  char command, subcommand, cID;
  if (!messageValidation( message, messageSize, cID, command, subcommand ))
    return;

  switch (command) {
    case ROBOT_TEAM_MSG:
    {
      if (!pyModuleExtension_)
        break;
      char ownID = pyModuleExtension_->clientID();
      if ((ownID & 0xf) != (cID & 0xf)) {// ignore opposing team message
        //WARNING_MSG( "Team message from the opposing team? ignore.\n" );
        break;
      }
      if ((ownID >> 4) == (cID >> 4)) {// ignore own message
        break;
      }      
      char * commandData = (char *)message + PYRIDE_MSG_HEADER_SIZE;
      int commandDataLen = messageSize - PYRIDE_MSG_MIN_LENGTH;

      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();

      PyObject * arg = Py_BuildValue( "(is)", (int)(cID >> 4), std::string( commandData, commandDataLen ).c_str() );
      pyModuleExtension_->invokeCallback( "onPeerMessage", arg );
      Py_DECREF( arg );

      PyGILState_Release( gstate );
    }
      break;
    default:
      ERROR_MSG( "PythonServer::processUDPInput()"
        "Unknown team message.\n" );
  }
}

bool PythonServer::RunMyString( const char * command )
{
  // grab thread lock
  //DEBUG_MSG( "Run command string: %s\n", command );

  if (!pMainModule_) {
    // we are in deep trouble should abort
    ERROR_MSG( "PythonServer:: RunMyString no  __main__ module.\n" );
    return false;
  }
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * mainDict = PyModule_GetDict( pMainModule_ );

  PyObject * ret = PyRun_String( command, Py_single_input, mainDict, mainDict );
  
  if (ret == NULL) { // python command returns error
    PyErr_Print();
    PyGILState_Release( gstate );
    return false;
  }
  
  Py_DECREF( ret );

  if (Py_FlushLine())
    PyErr_Clear();
    
  PyGILState_Release( gstate );

  return true;
}

PythonServer::ClientItem * PythonServer::addFdToClientList( const SOCKET_T & fd, struct sockaddr_in & cAddr )
{
  ClientItem * newClient = new ClientItem;
  newClient->fd = fd;
  newClient->addr = cAddr;
  newClient->pSession = new PythonSession( this, fd );
  newClient->pNext = NULL;

  pthread_mutex_lock( &t_mutex_ );

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

  pthread_mutex_unlock( &t_mutex_ );
  return newClient;
}

void PythonServer::disconnectClient( ClientItem * client, bool sendNotification )
{
  if (sendNotification) { // let the other end to close down the connection first
    pthread_mutex_lock( &t_mutex_ );
    if (client) {
      if (client->fd != INVALID_SOCKET)
        client->pSession->sayGoodBye();
    }
    else {
      ClientItem * fdPtr = clientList_;
      while (fdPtr) {
        if (fdPtr->fd != INVALID_SOCKET)
          fdPtr->pSession->sayGoodBye();
        fdPtr = fdPtr->pNext;
      }
    }
    pthread_mutex_unlock( &t_mutex_ );    
  }
  pthread_mutex_lock( &t_mutex_ );
  if (client) {
    if (client->fd != INVALID_SOCKET) {
      close( client->fd );
      FD_CLR( client->fd, &masterFDSet_ );
      client->fd = INVALID_SOCKET; // reset fd. this is also a flag for client item removal    
    }
  }
  else { // disconnect all clients
    ClientItem * fdPtr = clientList_;
    while (fdPtr) {
      if (fdPtr->fd != INVALID_SOCKET) {
        close( fdPtr->fd );
        FD_CLR( fdPtr->fd, &masterFDSet_ );
        fdPtr->fd = INVALID_SOCKET;
        delete fdPtr->pSession;
        fdPtr->pSession = NULL;
      }
      ClientItem * tmpPtr = fdPtr;
      fdPtr = fdPtr->pNext;
      delete tmpPtr;
    }
    clientList_ = NULL;
  }    
  pthread_mutex_unlock( &t_mutex_ );    
}

void PythonServer::broadcastServerMessage( const char * mesg )
{
  ClientItem * fdPtr = clientList_;
  while (fdPtr) {
    if (fdPtr->pSession) {
      fdPtr->pSession->write( mesg );
      fdPtr->pSession->writePrompt();      
    }
    fdPtr = fdPtr->pNext;
  }
}

bool PythonServer::messageValidation( const unsigned char * receivedMesg, const int receivedBytes, char & cID,
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

void PythonServer::initModuleExtension()
{
  if (!pyModuleExtension_) {
    ERROR_MSG( "No Python extension is known!\n" );
    return;
  }

  pPyMod_ = pyModuleExtension_->init( this );
  if (pPyMod_) {
    char runstring[200];
    sprintf( runstring, "import %s", pyModuleExtension_->name().c_str() );
    RunMyString( runstring );
  }
}

void PythonServer::finiModuleExtension()
{
  if (!pyModuleExtension_) {
    ERROR_MSG( "No Python extension is known!\n" );
    return;
  }

  pyModuleExtension_->fini();
  pPyMod_ = NULL;
}

void PythonServer::write( const char * msg )
{
  if (activeSession_) {
    activeSession_->write( msg );
  }
  else {
    INFO_MSG( "Script: %s\n", msg );
  }
}

void PythonServer::broadcastMessage( const char * msg ) // expect NULL terminated string
{
  if (!pyModuleExtension_)
    return;

  int msgSize = strlen( msg );
  int dataLength = PYRIDE_MSG_MIN_LENGTH + msgSize;  
  unsigned char * bcMesg = new unsigned char[dataLength];
  unsigned char * msgPtr = bcMesg;
  
  *msgPtr++ = PYRIDE_MSG_INIT;
  *msgPtr++ = PYRIDE_PROTOCOL_VERSION;
  *msgPtr++ = pyModuleExtension_->clientID();
  
  *msgPtr++ = (ROBOT_TEAM_MSG << 4);

  memcpy( (void *)msgPtr, msg, msgSize );
  msgPtr += msgSize;
  *msgPtr = PYRIDE_MSG_END;

#ifdef USE_ENCRYPTION
  unsigned char * encryptedMesg = NULL;
  int encryptedLength = 0;
  pthread_mutex_lock( &t_mutex_ );
  if (encryptMessage( bcMesg, dataLength, &encryptedMesg, &encryptedLength ) == 1) {
    sendto( udpSocket_, encryptedMesg, encryptedLength, 0, (struct sockaddr *)&bcAddr_, sizeof( bcAddr_ ) );
  }
  pthread_mutex_unlock( &t_mutex_ );
#else
  pthread_mutex_lock( &t_mutex_ );
  sendto( udpSocket_, bcMesg, dataLength, 0, (struct sockaddr *)&bcAddr_, sizeof( bcAddr_ ) );
  pthread_mutex_unlock( &t_mutex_ );
#endif
  delete [] bcMesg;
}

/**
 *  PythonSession class
 */
PythonSession::PythonSession( PythonServer * server, SOCKET_T fd ) :
  server_( server ),
  fd_( fd ),
  telnetSubnegotiation_( false ),
  promptStr_( ">>> " ),
  historyPos_( -1 ),
  charPos_( 0 ),
  multiline_( "" )
{
  this->connectReady();
}

PythonSession::~PythonSession()
{    
  readBuffer_.clear();
  historyBuffer_.clear();
  currentLine_ = "";
  multiline_ = "";
  charPos_ = 0;
}

void PythonSession::connectReady()
{
  unsigned char options[] =
  {
    TELNET_IAC, TELNET_DO, TELNET_LINEMODE,
    TELNET_IAC, TELNET_SB, TELNET_LINEMODE, 1, 4, TELNET_IAC, TELNET_SE,
    TELNET_IAC, TELNET_WILL, TELNET_ECHO, 0
  };

  this->write( (char*)options );
  this->write( server_->welcomeStr().c_str() );
  this->write( "\r\n" );
  this->writePrompt();
}

void PythonSession::processInput( PythonServer::ClientItem * client, unsigned char * recvData, int bytesReceived )
{
  if (!bytesReceived) {
    return;
  }

  for(int i = 0; i < bytesReceived; i++) {
    readBuffer_.push_back( recvData[i] );
  }

  while (!readBuffer_.empty()) {
    int c = (unsigned char)readBuffer_[0];

    // Handle (and ignore) telnet protocol commands.

    if (c == TELNET_IAC) {
      if (!this->handleTelnetCommand()) // wait for more input
        return;
      continue;
    }

    if (c == KEY_ESC) {
      if (!this->handleVTCommand())
        return;
      continue;
    }

    // If we're in telnet subnegotiation mode, ignore normal chars.

    if (telnetSubnegotiation_) {
      readBuffer_.pop_front();
      continue;
    }

    // If we got something printable, echo it and append it to
    // the current line.

    if (isprint( c )) {
      this->handleChar();
      continue;
    }

    switch (c) {
      case KEY_ENTER:
        this->handleLine( client );
        break;

      case KEY_BACKSPACE:
      case KEY_DEL:
        this->handleDel();
        break;

      case KEY_HTAB:
        this->handleTab();
        break;

      case KEY_CTRL_A:
        this->handleHome();
        break;
      case KEY_CTRL_E:
        this->handleEnd();
        break;

      case KEY_CTRL_C:
      case KEY_CTRL_D:
        server_->disconnectClient( client, true );
        return;

      default:
        readBuffer_.pop_front();
        break;
    }
  }
}

bool PythonSession::handleTelnetCommand()
{
  unsigned int bytesNeeded = 2;
  
  if (readBuffer_.size() < bytesNeeded)
    return false;

  unsigned int cmd = (unsigned char)readBuffer_[1];
  char str[256];

  switch (cmd) {
    case TELNET_WILL:
    case TELNET_WONT:
    case TELNET_DO:
    case TELNET_DONT:
      bytesNeeded = 3;
      break;

    case TELNET_SE:
      telnetSubnegotiation_ = false;
      break;

    case TELNET_SB:
      telnetSubnegotiation_ = true;
      break;

    case TELNET_IAC:
      // A literal 0xff. We don't care!
      break;

    default:
      sprintf( str, "Telnet command %d is unsupported.\r\n",
        cmd );
      this->write( str );
      break;
  }

  if (readBuffer_.size() < bytesNeeded)
    return false;

  while (bytesNeeded) {
    bytesNeeded--;
    readBuffer_.pop_front();
  }

  return true;
}

bool PythonSession::handleVTCommand()
{
  // Need 3 chars before we are ready.
  if (readBuffer_.size() < 3)
    return false;

  // Eat the ESC.
  readBuffer_.pop_front();

  if (readBuffer_.front() != '[' && readBuffer_.front() != 'O')
    return true;

  // Eat the [
  readBuffer_.pop_front();

  switch (readBuffer_.front()) {
    case 'A':
      this->handleUp();
      break;

    case 'B':
      this->handleDown();
      break;

    case 'C':
      this->handleRight();
      break;

    case 'D':
      this->handleLeft();
      break;
    default:
      return true;
  }

  readBuffer_.pop_front();
  return true;
}


/**
 *   This method handles a single character. It appends or inserts it
 *   into the buffer at the current position.
 */
void PythonSession::handleChar()
{
  currentLine_.insert( charPos_, 1, (char)readBuffer_.front() );
  int len = currentLine_.length() - charPos_;
  this->write( currentLine_.substr(charPos_, len).c_str() );

  char * bstr = new char[len];
  for(int i = 0; i < len - 1; i++)
    bstr[i] = '\b';
  bstr[len-1] = '\0';

  this->write( bstr );
  delete [] bstr;

  charPos_++;
  readBuffer_.pop_front();
}

/**
 *   This method handles an end of line. It executes the current command,
 *  and adds it to the history buffer.
 */
void PythonSession::handleLine( PythonServer::ClientItem * client )
{
  readBuffer_.pop_front();
  this->write( "\r\n" );

  if (currentLine_.empty()) {
    currentLine_ = multiline_;
    multiline_ = "";
  }
  else {
    historyBuffer_.push_back( currentLine_ );

    if (historyBuffer_.size() > MAX_HISTORY_COMMAND) {
      historyBuffer_.pop_front();
    }

    if (!multiline_.empty()) {
      multiline_ += "\n" + currentLine_;
      currentLine_ = "";
    }
  }

  if (!currentLine_.empty()) {
    currentLine_ += "\n";

    if (currentLine_[ currentLine_.length() - 2 ] == ':') {
      multiline_ += currentLine_;
    }
    else if (currentLine_.compare( "restartpython()\n" ) == 0) {
      server_->restartPythonServer();
      currentLine_ = "";
      historyPos_ = -1;
      charPos_ = 0;
      return;
    }
    else if (currentLine_.compare( "exit()\n" ) == 0) {
      server_->disconnectClient( client, true );
      return;
    }
    else {
      server_->activeSession( this );
      server_->RunMyString(( char *)currentLine_.c_str() );
      server_->activeSession( NULL ); // reset output
    }
  }

  currentLine_ = "";
  historyPos_ = -1;
  charPos_ = 0;

  this->writePrompt();
}


/**
 *  This method handles a del character.
 */
void PythonSession::handleDel()
{
  if (charPos_ > 0) {
    currentLine_.erase( charPos_ - 1, 1 );
    this->write( "\b" ERASE_EOL );
    charPos_--;
    int len = currentLine_.length() - charPos_;
    this->write( currentLine_.substr(charPos_, len).c_str() );

    char * bstr = new char[len+1];
    for(int i = 0; i < len; i++)
      bstr[i] = '\b';

    bstr[len] = '\0';
    this->write( bstr );
    delete [] bstr;
  }

  readBuffer_.pop_front();
}

/**
 *  This method handles a TAB character.
 */
void PythonSession::handleTab()
{
  // insert tab char
  readBuffer_[0] = (unsigned char) '\t';
  this->handleChar();
}

/**
 *   This method handles a key up event.
 */
void PythonSession::handleUp()
{
  if (historyPos_ < (int)historyBuffer_.size() - 1) {
    historyPos_++;
    currentLine_ = historyBuffer_[historyBuffer_.size() -
      historyPos_ - 1];

    this->write( "\r" ERASE_EOL );
    this->writePrompt();
    this->write( currentLine_.c_str() );
    charPos_ = currentLine_.length();
  }
}


/**
 *   This method handles a key down event.
 */
void PythonSession::handleDown()
{
  if (historyPos_ >= 0 ) {
    historyPos_--;

    if (historyPos_ == -1) {
      currentLine_ = "";
    }
    else {
      currentLine_ = historyBuffer_[historyBuffer_.size() -
        historyPos_ - 1];
    }

    this->write( "\r" ERASE_EOL );
    this->writePrompt();
    this->write( currentLine_.c_str() );
    charPos_ = currentLine_.length();
  }
}

/**
 *   This method handles a key left event.
 */
void PythonSession::handleLeft()
{
  if (charPos_ > 0) {
    charPos_--;
    this->write( "\033[D" );
  }
}

/**
 *   This method handles a key left event.
 */
void PythonSession::handleRight()
{
  if (charPos_ < currentLine_.length()) {
    charPos_++;
    this->write( "\033[C" );
  }
}

/**
 *   This method handles a key home/control a event.
 */
void PythonSession::handleHome()
{
  if (charPos_ > 0) {
    char ctrl[20];
    snprintf( ctrl, 20, "\033[%03dD", charPos_ );
    this->write( ctrl );
    charPos_ = 0;
  }
  readBuffer_.pop_front();
}

/**
 *   This method handles a key end/control e event.
 */
void PythonSession::handleEnd()
{
  if (charPos_ < currentLine_.length()) {
    int diff = currentLine_.length() - charPos_;
    char ctrl[20];
    snprintf( ctrl, 20, "\033[%03dC", diff );
    this->write( ctrl );
    charPos_ = currentLine_.length();
  }
  readBuffer_.pop_front();
}

void PythonSession::write( const char * str )
{
  if (strlen( str ) == 0) return;
  
  //DEBUG_MSG(("PythonSession::Send() send over %s chars\n", str));
  if (fd_ != INVALID_SOCKET) {
    ::write( fd_, str, strlen( str ) );
  }
}

/**
 *   This method prints a prompt to the socket.
 */
inline void PythonSession::writePrompt()
{
  this->write( multiline_.empty() ? ">>> " : "... " );
}

inline void PythonSession::sayGoodBye()
{
  this->write( goodByeMsg );
}
} // namespace pyride
