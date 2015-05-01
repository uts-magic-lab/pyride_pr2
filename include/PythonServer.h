#ifndef PythonServer_h_DEFINED
#define PythonServer_h_DEFINED

#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <net/if.h>
#include <sys/socket.h>
#include <errno.h>
#include <string>
#include <deque>

#include "PyRideCommon.h"
#include "PyModuleStub.h"

// define python config
#define PYTHONSERVER_BUFFER_SIZE  2048
#define PS_SEND_BUFFER_SIZE       PYTHONSERVER_BUFFER_SIZE*2
#define PS_RECEIVE_BUFFER_SIZE    PYTHONSERVER_BUFFER_SIZE
#define PYTHON_SERVER_PORT        27005

#ifdef ROS_BUILD
#define DEFAULT_PYTHON_SCRIPT_PATH "scripts"
#elif IOS_BUILD
#define DEFAULT_PYTHON_SCRIPT_PATH "scripts"
#else
#define DEFAULT_PYTHON_SCRIPT_PATH "/home/nao/naoqi/lib/python"
#endif

// define telnet protocol 
#define TELNET_ECHO     1
#define TELNET_LINEMODE 34
#define TELNET_SE       240
#define TELNET_SB       250
#define TELNET_WILL     251
#define TELNET_WONT     252
#define TELNET_DO       253
#define TELNET_DONT     254
#define TELNET_IAC      255

#define ERASE_EOL       "\033[K"

#define KEY_CTRL_A      1
#define KEY_CTRL_C      3
#define KEY_CTRL_D      4
#define KEY_CTRL_E      5
#define KEY_BACKSPACE   8
#define KEY_HTAB        9
#define KEY_DEL         127
#define KEY_ENTER       13
#define KEY_ESC         27

#define MAX_HISTORY_COMMAND      20
#define PYRIDE_MAIN_SCRIPT_NAME  "py_main"

namespace pyride {

class PythonSession;

class PythonServer : public PyOutputWriter
{
public:
  ~PythonServer();

  void init( bool enableTelnetConsole, PyModuleExtension * pyModule, const char * scriptDir = NULL, const char * pythonHome = NULL );
  void fini();
  void continuousProcessing();
  void restartPythonServer();

  void write( const char * msg );
  void broadcastMessage( const char * mesg );
  bool isActive() { return isActive_; }

  PyObject * mainScript() { return pMainScript_; }

  bool RunMyString( const char * command );

  std::string & welcomeStr() { return welcomeStr_; }
  void activeSession( PythonSession * session ) { activeSession_ = session; }

  static PythonServer * instance();

private:
  typedef struct sClientItem {
    SOCKET_T fd;
    struct sockaddr_in addr;
    struct sClientItem * pNext;
    PythonSession * pSession;
  } ClientItem;

  char customScriptBase[256];
  char customPythonHome[256];

  pthread_t runThread_;
  pthread_mutex_t t_mutex_;
  pthread_mutexattr_t t_mta;

  static PythonServer *  s_pPythonServer;

  bool isActive_;
  bool hasInterpreter_;

  PyObject *    prevStderr_;
  PyObject *    prevStdout_;
  PyObject *    pSysModule_;
  PyObject *    pMainModule_;
  PyObject *    pMainScript_;
  PyObject *    pPyMod_;

  struct sockaddr_in  sAddr_;
  struct sockaddr_in  bcAddr_;

  SOCKET_T  udpSocket_;
  SOCKET_T  tcpSocket_;

  unsigned char * dgramBuffer_;
  unsigned char * clientDataBuffer_;
  ClientItem * clientList_;

  int     maxFD_;
  fd_set  masterFDSet_;

  bool runningTelnetConsole_;
  bool keepRunning_;
  
  PythonSession * activeSession_;
  PyModuleExtension * pyModuleExtension_;

  std::string welcomeStr_;

  PythonServer();
  bool initUDPListener();
  bool initTelnetConsole();
  void finiTelnetConsole();

  bool initPyInterpreter();
  void runMainScript();
  void finiPyInterpreter();
  void initModuleExtension();
  void finiModuleExtension();

  void initIPAddresses();

  void processIncomingData( fd_set * readyFDSet );
  void processUDPInput( const unsigned char * recBuffer, int recBytes, struct sockaddr_in & cAddr );

  bool messageValidation( const unsigned char * receivedMesg, const int receivedBytes, char & cID,
                                         char & command, char & subcommand );
  ClientItem * addFdToClientList( const SOCKET_T & fd, struct sockaddr_in & cAddr );

  void disconnectClient( ClientItem * client, bool sendNotification = false );

  void broadcastServerMessage( const char * mesg );

  friend class PythonSession;
};

class PythonSession
{
public:
  PythonSession( PythonServer * server, SOCKET_T fd ); 
  ~PythonSession();

  void processInput( PythonServer::ClientItem * client, unsigned char * recvData, int bytesReceived );
  void sayGoodBye();

  void write( const char * str );
  void writePrompt();

private:
  PythonServer * server_;
  SOCKET_T fd_;

  bool telnetSubnegotiation_;

  std::string    promptStr_;

  std::deque<unsigned char> readBuffer_;
  std::deque<std::string> historyBuffer_;

  std::string currentLine_;
  int historyPos_;
  unsigned int charPos_;
  std::string multiline_;

  void connectReady();
  
  bool  handleTelnetCommand();
  bool  handleVTCommand();
  void  handleLine( PythonServer::ClientItem * client );
  void  handleDel();
  void  handleChar();
  void  handleTab();
  void  handleUp();
  void  handleDown();
  void  handleLeft();
  void  handleRight();
  void  handleHome();
  void  handleEnd();
};
} //namespace pyride
#endif // PythonServer_h_DEFINED
