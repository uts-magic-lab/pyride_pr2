/*
 *  AppConfigManager.h
 *  PyRideServer
 *
 *  Created by Xun Wang on 17/08/12.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef APP_CONFIG_MANAGER_H
#define APP_CONFIG_MANAGER_H

#include <openssl/sha.h>
#ifdef ROS_BUILD
#include <tinyxml.h>
#else
#include <tinyxml/tinyxml.h>
#endif
#include <sys/types.h>
#include <sys/un.h>
#include <netinet/in.h>
#include <string>
#include <vector>
#include <map>

#include <PyRideCommon.h>
#include <DeviceController.h>

#ifdef ROS_BUILD
#define DEFAULT_CONFIGURATION_FILE "pyrideconfig.xml"
#else
#define DEFAULT_CONFIGURATION_FILE "/home/nao/naoqi/preferences/pyrideconfig.xml"
#endif

namespace pyride {

typedef struct {
  std::string name;
  unsigned char password[SHA256_DIGEST_LENGTH];
  SOCKET_T clientFD;
  struct sockaddr_in clientAddr;
} UserData;

class AppConfigManager
{
public:
  static AppConfigManager * instance();

  void loadConfigFromFile( const char * fileName );
  void saveConfig();

  bool signInUserWithPassword( const unsigned char * code, SOCKET_T fd, struct sockaddr_in & addr, std::string & username );
  bool signOutUser( SOCKET_T fd, std::string & username );
  
  bool addUser( const char * name, const char * password );
  bool delUser( const char * name );
  bool changeUserPassword( const char * name, const char * oldpassword, const char * newpassword );
  
  bool getOnlineUserClientFD( const char * name, SOCKET_T & fd );

  int listCurrentUsers( std::vector<std::string> & userNameList );
  int listAllUsers( std::vector<std::string> & userNameList );
  
  const DeviceInfoList * deviceInfoList() const { return &deviceInfoList_; }

  void fini();

  char clientID() { return clientID_; }
  bool enablePythonConsole() { return allowPythonTelnet_; }
  const RobotPose & startPosition() { return defaultPose_; }

private:
  typedef std::vector< UserData *> UserDataList;
  typedef std::map< SOCKET_T, UserData *> SignedInMap;

  SignedInMap signedInMap_;
  UserDataList userDataList_;
  DeviceInfoList deviceInfoList_;
  
  char clientID_;
  bool allowPythonTelnet_;
  RobotPose defaultPose_;

  std::string configFileName_;

  static AppConfigManager * s_instance;
  
  AppConfigManager();

  void loadUserInfo( TiXmlNode * userInfoNode );
  void loadRobotInfo( TiXmlNode * robotInfoNode );
  void loadDeviceInfo( TiXmlNode * devInfoNode );
  UserData * parseUserRecord( TiXmlNode * userNode );
  DeviceInfo * parseDeviceRecord( TiXmlNode * deviceNode );
};
} // namespace pyride

#endif // APP_CONFIG_MANAGER_H
