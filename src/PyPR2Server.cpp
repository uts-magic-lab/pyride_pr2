/*
 *  PyPR2Server.cpp
 *  PyPR2Server
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <sys/types.h>
#include <sys/stat.h>

#include "PyPR2Server.h"
#include "PythonServer.h"
#include "PR2ProxyManager.h"
#include "AppConfigManager.h"
#include "PyPR2Module.h"
#include "VideoObject.h"

PYRIDE_LOGGING_DECLARE( "pyride_pr2.log" );

namespace pyride {
static const float kHFOV_Wide = 90.0;
static const float kHFOV_Narrow = 55.0;

PyPR2Server::PyPR2Server() :
  isRunning_( true )
{
  hcNodeHandle_ = new NodeHandle();
}

PyPR2Server::~PyPR2Server()
{
  delete hcNodeHandle_;
}

bool PyPR2Server::init()
{
  PYRIDE_LOGGING_INIT;

  // setup private parameter image_transport to compressed
  NodeHandle priNh( "~" );
  std::string filename;
  std::string scriptdir;
  bool useOptionNodes = false;
  bool useMoveIt = false;
  
  priNh.param<bool>( "use_optional_nodes", useOptionNodes, false );
  priNh.param<bool>( "use_moveit", useMoveIt, false );
  priNh.param<std::string>( "config_file", filename, "pyrideconfig.xml" );
  priNh.param<std::string>( "script_dir", scriptdir, "scripts" );
  AppConfigManager::instance()->loadConfigFromFile( filename.c_str() );

  if (!initVideoDevices()) {
    ERROR_MSG( "Unable to find any active robot camera.\n" );
  }

  nodeStatusSub_ = hcNodeHandle_->subscribe( "pyride_pr2/node_status", 1, &PyPR2Server::nodeStatusCB, this );

  PR2ProxyManager::instance()->initWithNodeHandle( hcNodeHandle_, useOptionNodes, useMoveIt );
  ServerDataProcessor::instance()->init( activeVideoDevices_, activeAudioDevices_ );
  ServerDataProcessor::instance()->addCommandHandler( this );
  ServerDataProcessor::instance()->setClientID( AppConfigManager::instance()->clientID() );
  ServerDataProcessor::instance()->setDefaultRobotInfo( PR2, AppConfigManager::instance()->startPosition() );
  
  PythonServer::instance()->init( AppConfigManager::instance()->enablePythonConsole(),
      PyPR2Module::instance(), scriptdir.c_str() );
  ServerDataProcessor::instance()->discoverConsoles();
  return true;
}

void PyPR2Server::stopProcess()
{
  isRunning_ = false;
}

void PyPR2Server::continueProcessing()
{
  ros::Rate publish_rate( kPublishFreq );

  while (isRunning_) {
    ros::spinOnce();

    PR2ProxyManager::instance()->publishCommands();

    publish_rate.sleep();
  }
}

void PyPR2Server::fini()
{
  INFO_MSG( "Disconnect to the controllers gracefully...\n" );

  this->notifySystemShutdown();

  // finalise controllers
  finiVideoDevices();

  nodeStatusSub_.shutdown();

  PR2ProxyManager::instance()->fini();
  AppConfigManager::instance()->fini();
  ServerDataProcessor::instance()->fini();
}

bool PyPR2Server::executeRemoteCommand( PyRideExtendedCommand command,
                                           const unsigned char * optionalData,
                                           const int optionalDataLength )
{
  // implement the routine to handle commands defined in PyRideExtendedCommand
  // in PyRideCommon.h
  // for example:
  bool status = true;
  switch (command) {
    case SPEAK:
    {
      float volume = *((float *)optionalData);
      //DEBUG_MSG( "received volume %f\n", volume );
      char * text = (char *)optionalData + sizeof( float );
      PR2ProxyManager::instance()->sayWithVolume( std::string( text, optionalDataLength - sizeof( float ) ), volume );
    }
      break;
    case HEAD_MOVE_TO: // position
    {
      float yaw = *((float *)optionalData);
      float pitch = *((float *)optionalData+1);
      
      yaw = kHFOV_Wide * yaw * kDegreeToRAD;
      pitch = kHFOV_Wide * pitch * kDegreeToRAD;

      status = PR2ProxyManager::instance()->moveHeadTo( yaw, pitch, true );
    }
      break;
    case UPDATE_HEAD_POSE: // velocity vector
    {
      float newHeadYaw = *((float *)optionalData);
      float newHeadPitch = *((float *)optionalData+1);
      PR2ProxyManager::instance()->updateHeadPos( newHeadYaw, newHeadPitch );
    }
      break;
    case BODY_MOVE_TO: // position
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      RobotPose newPose;
      memcpy( &newPose, dataPtr, sizeof( RobotPose ) );
      dataPtr += sizeof( RobotPose );
      float bestTime = *((float *)dataPtr);
      status = PR2ProxyManager::instance()->moveBodyTo( newPose, bestTime );
    }
      break;
    case UPDATE_BODY_POSE: // velocity vector
    {
      unsigned char * dataPtr = (unsigned char *)optionalData;
      RobotPose newPose;
      memcpy( &newPose, dataPtr, sizeof( RobotPose ) );
      PR2ProxyManager::instance()->updateBodyPose( newPose );
    }
      break;
    default:
      status = false;
      break;
  }
  return status;
}

void PyPR2Server::cancelCurrentOperation()
{
  PR2ProxyManager::instance()->sayWithVolume( "Emergency Stop!" );
}

// helper function
bool PyPR2Server::initVideoDevices()
{
  const DeviceInfoList * devlist = AppConfigManager::instance()->deviceInfoList();
  for (size_t i = 0; i < devlist->size(); i++) {
    DeviceInfo * deviceInfo = devlist->at( i );
    VideoObject * activeVideoObj = new VideoObject( *deviceInfo );
    if (activeVideoObj->initDevice()) {
      activeVideoDevices_.push_back( activeVideoObj );
    }
    else {
      ERROR_MSG( "Failed to activate a video device.\n" );
      delete activeVideoObj;
    }
  }
  return (activeVideoDevices_.size() != 0);
}

void PyPR2Server::finiVideoDevices()
{
  VideoDevice * videoObj = NULL;
  
  for (size_t i = 0; i < activeVideoDevices_.size(); i++) {
    videoObj = activeVideoDevices_.at( i );
    videoObj->finiDevice();
    delete videoObj;
  }
  activeVideoDevices_.clear();
}

void PyPR2Server::notifySystemShutdown()
{
  INFO_MSG( "PyPR2Server is shutting down..\n" );
  
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyPR2Module::instance()->invokeCallback( "onSystemShutdown", arg );
  
  PyGILState_Release( gstate );
}

/** @name Miscellaneous Callback Functions
 *
 */
/*! \typedef onNodeStatusUpdate( data )
 *  \memberof PyPR2.
 *  \brief Callback function invoked when an external ROS node dispatch status update message
 *   to PyRIDE on pyride_pr2/node_status topic.
 *  \param dictionary data. node status message in the format of {'node', 'timestamp', 'priority', 'message' }.
 *  \return None.
 */
/**@}*/

void PyPR2Server::nodeStatusCB( const pyride_pr2::NodeStatusConstPtr & msg )
{
  if (msg->for_console) { // reformat the string using colon separated format and pass directly to console
    stringstream ss;
    ss << msg->header.stamp << msg->node_id << ":" << msg->priority << ":" << msg->status_text;
    ServerDataProcessor::instance()->updateOperationalStatus( CUSTOM_STATE,
                                                                ss.str().c_str(),
                                                                (int)ss.str().length() );
    return;
  }
  
  PyObject * arg = NULL;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * retObj = PyDict_New();

  PyObject * elemObj = PyString_FromString( msg->node_id.c_str() );
  PyDict_SetItemString( retObj, "node", elemObj );
  Py_DECREF( elemObj );

  elemObj = PyFloat_FromDouble( (double)msg->header.stamp.sec + (double)msg->header.stamp.nsec / 1E9 );
  PyDict_SetItemString( retObj, "timestamp", elemObj );
  Py_DECREF( elemObj );

  elemObj = PyInt_FromLong( msg->priority );
  PyDict_SetItemString( retObj, "priority", elemObj );
  Py_DECREF( elemObj );

  elemObj = PyString_FromString( msg->status_text.c_str() );
  PyDict_SetItemString( retObj, "message", elemObj );
  Py_DECREF( elemObj );

  arg = Py_BuildValue( "(O)", retObj );

  PyPR2Module::instance()->invokeCallback( "onNodeStatusUpdate", arg );
  
  Py_DECREF( retObj );
  Py_DECREF( arg );

  PyGILState_Release( gstate );    
}

} // namespace pyride
