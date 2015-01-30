/*
 *  AppConfigManager.cpp
 *  PyRideServer
 *
 *  Created by Xun Wang on 17/08/12.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <sys/types.h>
#include <sys/stat.h>
#include <openssl/rand.h>
#include <openssl/bio.h>
#include <openssl/evp.h>
#include <stdio.h>

#include "AppConfigManager.h"

namespace pyride {

static const char rnd_seed[] = "This is my randomn seed generator";

AppConfigManager * AppConfigManager::s_instance = NULL;

AppConfigManager * AppConfigManager::instance()
{
  if (!s_instance)
    s_instance = new AppConfigManager();
  
  return s_instance;
}

AppConfigManager::AppConfigManager() :
  clientID_( (1 << 4) | BlueTeam ), // blue team 1
  allowPythonTelnet_( true )
{
  RAND_seed( rnd_seed, sizeof( rnd_seed ) );
  defaultPose_.x = defaultPose_.y = defaultPose_.theta = 0.0;
}

void AppConfigManager::loadConfigFromFile( const char * fileName )
{
  struct stat sb;
  
  if (stat( fileName, &sb ) == -1 || !S_ISREG( sb.st_mode )) {
    ERROR_MSG( "LoadConfig: unable to find configuration file %s\n",
              fileName );
    return;
  }
  
  TiXmlDocument doc( fileName );
  if (!doc.LoadFile()) {
    ERROR_MSG( "LoadConfig: "
              "Unable to load user inforamtion file %s: "
              "Error %s.\n", fileName, doc.ErrorDesc() );
    return;
  }
  configFileName_ = fileName;

  TiXmlNode * rootNode = NULL;
  rootNode = doc.FirstChild( "PyRIDE" );
  
  if (!rootNode) {
    ERROR_MSG( "LoadConfig: "
              "Invalid XML file %s.", fileName );
    return;
  }

  loadRobotInfo( rootNode );
  loadUserInfo( rootNode );
  loadDeviceInfo( rootNode );
}

void AppConfigManager::loadRobotInfo( TiXmlNode * robotInfoNode )
{
  TiXmlElement * teamColourElem = NULL;
  TiXmlElement * memberIDElem = NULL;
  TiXmlElement * defaultPoseElem = NULL;
  TiXmlElement * enablePySvrElem = NULL;
  
  char * colourText = NULL;
  char * idText = NULL;
  char * posText = NULL;
  char * pySvrText = NULL;
  int teamColour = -1;
  int memberID = -1;
  float posx = 0.0, posy = 0.0;
  
  teamColourElem = robotInfoNode->FirstChildElement( "TeamColour" );
  memberIDElem = robotInfoNode->FirstChildElement( "MemberID" );
  
  if (!teamColourElem || !memberIDElem) {
    ERROR_MSG( "LoadConfig: "
              "missing team member info." );
    goto getpos;
  }
  colourText = (char *)teamColourElem->GetText();
  idText = (char *)memberIDElem->GetText();
  if (colourText == NULL || idText == NULL) {
    ERROR_MSG( "LoadConfig: "
              "invalid team member info." );
    goto getpos;
  }
  if (strcasecmp( colourText, "blue" ) == 0) {
    teamColour = BlueTeam;
  }
  else if (strcasecmp( colourText, "pink" ) == 0){
    teamColour = PinkTeam;
  }
  memberID = atoi( idText );
  if (teamColour == -1 || memberID > 5 || memberID < 0) {
    ERROR_MSG( "LoadConfig: "
              "invalid team member info." );
    goto getpos;
  }
  clientID_ = (memberID & 0xf) << 4 | teamColour;
  
getpos:
  defaultPoseElem = robotInfoNode->FirstChildElement( "DefaultPosition" );
  if (!defaultPoseElem) {
    WARNING_MSG( "LoadConfig: "
                "missing default position." );
    goto getpysvr;
  }
  posText = (char*)defaultPoseElem->GetText();
  if (!posText || sscanf( posText, "%f %f", &posx, &posy ) == 0) {
    ERROR_MSG( "LoadConfig: "
              "invalid default position." );
    goto getpysvr;
  }
  defaultPose_.x = posx; defaultPose_.y = posy;
  
getpysvr:
  enablePySvrElem = robotInfoNode->FirstChildElement( "RemotePythonAccess" );
  pySvrText = (char*)enablePySvrElem->GetText();
  allowPythonTelnet_ = (pySvrText && (strcasecmp( pySvrText, "enable" ) == 0));
}

void AppConfigManager::loadUserInfo( TiXmlNode * userInfoNode )
{
  if (!userInfoNode)
    return;

  TiXmlNode * rootNode = NULL;
  TiXmlNode * userNode = NULL;

  rootNode = userInfoNode->FirstChild( "UserInfo" );
  if (!rootNode) {
    WARNING_MSG( "LoadConfig: "
              "Configuration contains no user access info.\n" );
    return;
  }  
  userNode = rootNode->FirstChild( "User" );
  int loadErrors = 0;
  int currentRecord = 1;

  while (userNode) {
    UserData * userRecord = parseUserRecord( userNode );
    if (userRecord) {
      userDataList_.push_back( userRecord );
    }
    else {
      ERROR_MSG( "LoadConfig: Corrupted user record %d.\n", currentRecord );
      loadErrors ++;
    }
    userNode = userNode->NextSibling( "User" );
    currentRecord ++;
  }
  if (loadErrors > 0) {
    WARNING_MSG( "LoadConfig: configuration file contains %d bad user record.\n",
                loadErrors );
  }
}

UserData * AppConfigManager::parseUserRecord( TiXmlNode * userNode )
{
  UserData * record = NULL;

  TiXmlElement * nameElm = userNode->FirstChildElement( "Name" );
  TiXmlElement * codeElm = userNode->FirstChildElement( "Password" );
  
  if (!nameElm || !codeElm) {
    ERROR_MSG( "LoadConfig: missing element "
              "UserName or Password.\n" );
    return record;
  }
  const char * name = nameElm->GetText();
  const char * code = codeElm->GetText();
  
  if (!name) {
    ERROR_MSG( "LoadConfig: invalid user name.\n" );
    return record;
  }
  
  if (code) {
    size_t outLen = 0;
    unsigned char * hashCode = ::decodeBase64( code, &outLen );
    if (hashCode && outLen == SHA256_DIGEST_LENGTH) {
      record = new UserData;
      record->name = std::string( name );
      memcpy( &(record->password), hashCode, SHA256_DIGEST_LENGTH );
      record->clientFD = INVALID_SOCKET;
      memset( &(record->clientAddr), 0, sizeof( struct sockaddr_in ) );
      free( hashCode );
      return record;
    }
    if (hashCode)
      free( hashCode );
  }
  ERROR_MSG( "LoadConfig: invalid user password.\n" );
  return record;
}

void AppConfigManager::loadDeviceInfo( TiXmlNode * deviceInfoNode )
{
  if (!deviceInfoNode)
    return;
  
  TiXmlNode * rootNode = NULL;
  TiXmlNode * videoNode = NULL;
  
  rootNode = deviceInfoNode->FirstChild( "DeviceInfo" );
  if (!rootNode) {
    WARNING_MSG( "LoadConfig: "
                "Configuration contains no device info. Use default devices.\n" );
    return;
  }

  videoNode = rootNode->FirstChild( "Video" );
  int loadErrors = 0;
  int currentRecord = 1;

  while (videoNode) {
    DeviceInfo * deviceRecord = parseDeviceRecord( videoNode );
    if (deviceRecord) {
      deviceRecord->index = deviceInfoList_.size();
      deviceInfoList_.push_back( deviceRecord );
    }
    else {
      ERROR_MSG( "LoadConfig: Corrupted device record %d.", currentRecord );
      loadErrors ++;
    }
    videoNode = videoNode->NextSibling( "Video" );
    currentRecord ++;
  }
  if (loadErrors > 0) {
    WARNING_MSG( "LoadConfig: configration file contains %d bad video device record.",
                loadErrors );
  }
}

DeviceInfo * AppConfigManager::parseDeviceRecord( TiXmlNode * deviceNode )
{
  TiXmlElement * idElem = deviceNode->FirstChildElement( "ID" );
  TiXmlElement * nameElem = deviceNode->FirstChildElement( "Name" );
  TiXmlElement * labelElem = deviceNode->FirstChildElement( "Label" );
  TiXmlElement * activeElem = deviceNode->FirstChildElement( "IsActive" );
  
  if (!idElem) {
    ERROR_MSG( "LoadConfig: invalid device record: missing tags." );
    return NULL;
  }
  
  const char * idText = idElem->GetText();
  
  if (!idText) {
    ERROR_MSG( "LoadConfig: invalid device record: mssing ID." );
    return NULL;
  }
  
  const char * nameText = nameElem->GetText();
  const char * labelText = labelElem->GetText();
  const char * activeText = activeElem->GetText();
  
  DeviceInfo * newDevice = new DeviceInfo;
  newDevice->deviceID = std::string( idText );
  
  if (nameText) {
    newDevice->deviceName = std::string( nameText );
  }
  if (labelText) {
    newDevice->deviceLabel = std::string( labelText );
  }
  if (activeText) {
    newDevice->shouldBeActive = (strcasecmp( activeText, "Yes") == 0);
  }
  else {
    newDevice->shouldBeActive = false;
  }
  
  return newDevice;
}

bool AppConfigManager::signInUserWithPassword( const unsigned char * code, SOCKET_T fd, struct sockaddr_in & addr, std::string & username )
{
  if (fd == INVALID_SOCKET)
    return false;

  UserData * info = NULL;

  bool found = false;
  // iterate through all records to find corresponding user/code
  UserDataList::iterator iter;
  for (iter = userDataList_.begin(); iter != userDataList_.end(); iter++) {
    found = (memcmp( &((*iter)->password), code, SHA256_DIGEST_LENGTH ) == 0);
    if (found)
      break;
  }
  if (found) {
    info = *iter;
    if (info->clientFD != INVALID_SOCKET) {
      ERROR_MSG( "Sign in user: user %s has already signed in.\n", info->name.c_str() );
      return false;
    }
    else {
      INFO_MSG( "User %s has just signed in.\n", info->name.c_str() );
      username = info->name;
      info->clientFD = fd;
      memcpy( &(info->clientAddr), &addr, sizeof( struct sockaddr_in ) );
      signedInMap_[fd] = info;
    }
  }
  else {
    ERROR_MSG( "Sign in user: unable to find user with the corresponding "
              "authentication code.\n" );
  }
  return found;
}

bool AppConfigManager::signOutUser( SOCKET_T fd, std::string & username )
{
  if (fd == INVALID_SOCKET)
    return false;

  SignedInMap::iterator iter = signedInMap_.find( fd );
  if (iter != signedInMap_.end()) {
    UserData * info = iter->second;
    // remove from map
    INFO_MSG( "User %s has just signed out.\n", info->name.c_str() );
    username = info->name;
    signedInMap_.erase( iter );
    info->clientFD = INVALID_SOCKET;
    memset( &(info->clientAddr), 0, sizeof( struct sockaddr_in ) );
    return true;
  }
  return false;
}

bool AppConfigManager::addUser( const char * name, const char * password )
{
  if (!name || !password || strlen( password ) < 4)
    return false;

  unsigned char code[SHA256_DIGEST_LENGTH];
  memset( code, 0, SHA256_DIGEST_LENGTH );
  
  secureSHA256Hash( (unsigned char*)password, strlen( password ), code );

  UserDataList::iterator iter;
  for (iter = userDataList_.begin(); iter != userDataList_.end(); iter++) {
    if ((*iter)->name.compare( name ) == 0)
      return false;
    if (memcmp( code, (*iter)->password, SHA256_DIGEST_LENGTH ) == 0) // every password must be unique
      return false;
  }

  UserData * newUser = new UserData;
  newUser->name = name;
  newUser->clientFD = INVALID_SOCKET;
  memcpy( newUser->password, code, SHA256_DIGEST_LENGTH );
  memset( &(newUser->clientAddr), 0, sizeof( struct sockaddr_in ) );
  userDataList_.push_back( newUser );
  return true;
}

bool AppConfigManager::delUser( const char * name )
{
  if (!name)
    return false;
  
  UserDataList::iterator iter;
  for (iter = userDataList_.begin(); iter != userDataList_.end(); iter++) {
    if ((*iter)->name.compare( name ) == 0) {
      userDataList_.erase( iter );
      return true;
    }
  }
  return false;
}

bool AppConfigManager::changeUserPassword( const char * name, const char * oldpassword, const char * newpassword )
{
  if (!name || !oldpassword || !newpassword || strlen( newpassword ) < 4)
    return false;
  
  bool found = false;
  UserDataList::iterator iter;
  for (iter = userDataList_.begin(); iter != userDataList_.end(); iter++) {
    found = ((*iter)->name.compare( name ) == 0);
    if (found)
      break;
  }
  if (!found)
    return false;

  unsigned char code[SHA256_DIGEST_LENGTH];
  memset( code, 0, SHA256_DIGEST_LENGTH );
  secureSHA256Hash( (unsigned char*)oldpassword, strlen( oldpassword ), code );
  if (memcmp( code, (*iter)->password, SHA256_DIGEST_LENGTH ))
    return false;

  memset( code, 0, SHA256_DIGEST_LENGTH );
  secureSHA256Hash( (unsigned char*)newpassword, strlen( newpassword ), code );
  memcpy( (*iter)->password, code, SHA256_DIGEST_LENGTH );
  return true;
}

void AppConfigManager::fini()
{
  this->saveConfig();

  for (UserDataList::iterator iter = userDataList_.begin();
       iter != userDataList_.end(); iter++)
  {
    delete *iter;
  }
  
  userDataList_.clear();
  signedInMap_.clear();
}

int AppConfigManager::listCurrentUsers( std::vector<std::string> & userNameList )
{
  userNameList.clear();
  SignedInMap::iterator iter = signedInMap_.begin();
  while (iter != signedInMap_.end()) {
    userNameList.push_back( iter->second->name );
    iter++;
  }
  return userNameList.size();
}

bool AppConfigManager::getOnlineUserClientFD( const char * name, SOCKET_T & fd )
{
  if (!name)
    return false;
  
  SignedInMap::iterator iter = signedInMap_.begin();
  UserData * found = NULL;

  while (iter != signedInMap_.end()) {
    if (iter->second->name.compare( name ) == 0) {
      found = iter->second;
      break;
    }
    iter++;
  }
  if (found) {
    fd = found->clientFD;
    return true;
  }
  else {
    return false;
  }
}

int AppConfigManager::listAllUsers( std::vector<std::string> & userNameList )
{
  userNameList.clear();
  UserDataList::iterator iter = userDataList_.begin();
  while (iter != userDataList_.end()) {
    userNameList.push_back( (*iter)->name );
    iter++;
  }
  return userNameList.size();
}

void AppConfigManager::saveConfig()
{
  if (configFileName_ == "") { // save to default file
    configFileName_ = DEFAULT_CONFIGURATION_FILE;
  }
  
  FILE * outFile = fopen( configFileName_.c_str(), "w" );
  
  if (!outFile) {
    ERROR_MSG( "Unable to save configuration to %s.\n", configFileName_.c_str() );
    return;
  }
  fprintf( outFile, "<PyRIDE>\n" );
  fprintf( outFile, "  <TeamColour> %s </TeamColour>\n", (clientID_ & BlueTeam) ? "blue" : "red" );
  fprintf( outFile, "  <MemberID> %d </MemberID>\n", (clientID_ >> 4) & 0xf );
  fprintf( outFile, "  <DefaultPosition> %.1f %.1f </DefaultPosition>\n",
          defaultPose_.x, defaultPose_.y );
  fprintf( outFile, "  <RemotePythonAccess> %s </RemotePythonAccess>\n", allowPythonTelnet_ ? "enable" : "disable" );
  
  char * encodeBuf = NULL;
  if (deviceInfoList_.size()) {
    fprintf( outFile, "  <DeviceInfo>\n" );
    for (size_t i = 0; i < deviceInfoList_.size(); i++) {
      fprintf( outFile, "    <Video>\n" );
      fprintf( outFile, "      <ID> %s </ID>\n", deviceInfoList_[i]->deviceID.c_str() );
      fprintf( outFile, "      <Name> %s </Name>\n", deviceInfoList_[i]->deviceName.c_str() );
      fprintf( outFile, "      <Label> %s </Label>\n", deviceInfoList_[i]->deviceLabel.c_str() );
      fprintf( outFile, "      <IsActive> %s </IsActive>\n", deviceInfoList_[i]->shouldBeActive ? "Yes" : "No" );
      fprintf( outFile, "    </Video>\n" );
    }
    fprintf( outFile, "  </DeviceInfo>\n" );
  }
  if (userDataList_.size()) {
    fprintf( outFile, "  <UserInfo>\n" );
    for (size_t i = 0; i < userDataList_.size(); i++) {
      fprintf( outFile, "    <User>\n" );
      fprintf( outFile, "      <Name> %s </Name>\n", userDataList_[i]->name.c_str() );
      encodeBuf = ::encodeBase64( userDataList_[i]->password, SHA256_DIGEST_LENGTH );
      if (encodeBuf) {
        fprintf( outFile, "      <Password> %s </Password>\n", encodeBuf );
        free( encodeBuf );
      }
      else {
        fprintf( outFile, "      <Password> </Password>\n" );
      }
      fprintf( outFile, "    </User>\n" );
    }
    fprintf( outFile, "  </UserInfo>\n" );
  }
  fprintf( outFile, "</PyRIDE>\n" );

  fclose( outFile );
}
} // namespace pyride
