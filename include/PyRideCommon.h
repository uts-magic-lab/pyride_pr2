/*
 *  PyRideCommon.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 20/04/2010.
 *  Copyright 2009 Galaxy Network. All rights reserved.
 *
 */

#ifndef PyRideCommon_h_DEFINED
#define PyRideCommon_h_DEFINED

#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>

#include "PyRideCustom.h"

#define PYRIDE_PROTOCOL_VERSION  3
#define PYRIDE_MSG_INIT          0xAC
#define PYRIDE_MSG_END           '#'
#define PYRIDE_MSG_MIN_LENGTH    5
#define PYRIDE_MSG_HEADER_SIZE   4

#define PYRIDE_BROADCAST_IP                "255.255.255.255"
#define PYRIDE_CONTROL_PORT                29430
#define PYRIDE_VIDEO_STREAM_BASE_PORT      35210 //34529
#define PYRIDE_DEFAULT_BUFFER_SIZE         4096
#define PYRIDE_MSG_BUFFER_SIZE             10240
#define PYRIDE_AUDIO_SAMPLE_RATE           48000
#define PYRIDE_AUDIO_FRAME_SIZE            256
#define PYRIDE_AUDIO_BYTES_PER_PACKET      46
#define PYRIDE_AUDIO_BITS_PER_SAMPLE       16
#define PYRIDE_AUDIO_BYTES_PER_SAMPLE      (PYRIDE_AUDIO_BITS_PER_SAMPLE/8)
#define PYRIDE_AUDIO_PLAY_BUFFERS          2


#if defined( IOS_BUILD )
#define PYRIDE_LOGGING_INIT
#define DEBUG_MSG( MSG... ) \
  printf( "DEBUG: " ); \
  printf( MSG );

#define ERROR_MSG( MSG... )  \
  printf( "ERROR: " ); \
  printf( MSG );

#define WARNING_MSG( MSG... )  \
  printf( "WARNING: " ); \
  printf( MSG );

#define INFO_MSG( MSG... )  \
  printf( "INFO: " ); \
  printf( MSG );

#else // !IOS_BUILD
#ifdef WIN32

#define WM_FGNOTIFY WM_USER+1

#define PYRIDE_LOGGING_DECLARE( LOGNAME )

#define PYRIDE_LOGGING_INIT

#ifdef PRODUCT_RELEASE
#define DEBUG_MSG( ... )
#else
#define DEBUG_MSG(...) \
  { char outputStr[200]; \
    sprintf_s( outputStr, 200, __VA_ARGS__ ); \
    OutputDebugStringA( outputStr ); \
  }
#endif

#define INFO_MSG(...) \
  { char outputStr[200]; \
    sprintf_s( outputStr, 200, __VA_ARGS__ ); \
    OutputDebugStringA( outputStr ); \
  }

#define WARNING_MSG(...)

#define ERROR_MSG(...) \
  { char outputStr[200]; \
    sprintf_s( outputStr, 200, __VA_ARGS__ ); \
    OutputDebugStringA( outputStr ); \
  }
#else
#define PYRIDE_NO_LOGGING \
FILE * s_pyridelog = NULL;

#define PYRIDE_LOGGING_DECLARE( LOGNAME ) \
FILE * s_pyridelog = NULL; \
const char * logFileName = LOGNAME

extern FILE * s_pyridelog;
#define PYRIDE_LOGGING_INIT \
{ \
  char * sep = NULL; \
  if ((sep = strrchr( (char*)logFileName, '/' )) != NULL ) { \
    struct stat sb; \
    int dirlen = sep - logFileName; \
    char * dirname = (char *) malloc( dirlen + 1 ); \
    memcpy( dirname, logFileName, dirlen ); \
    dirname[dirlen] = '\0'; \
    if (stat( dirname, &sb ) == -1) \
      mkdir( dirname, 0755 ); \
    free( dirname ); \
  } \
} \
s_pyridelog = fopen( logFileName, "a" )

//#define s_pyridelog stdout

#ifdef PRODUCT_RELEASE
#define DEBUG_MSG( MSG... )
#else
#define DEBUG_MSG( MSG... ) \
if (s_pyridelog) { \
  fprintf( s_pyridelog, "DEBUG: " ); \
  fprintf( s_pyridelog, MSG ); \
  fflush( s_pyridelog ); \
}
#endif

#define INFO_MSG( MSG... ) \
if (s_pyridelog) { \
  fprintf( s_pyridelog, "INFO: " ); \
  fprintf( s_pyridelog, MSG ); \
  fflush( s_pyridelog ); \
}

#define WARNING_MSG( MSG... ) \
if (s_pyridelog) { \
  fprintf( s_pyridelog, "WARNING: " ); \
  fprintf( s_pyridelog, MSG ); \
  fflush( s_pyridelog ); \
}
#define ERROR_MSG( MSG... ) \
if (s_pyridelog) { \
  fprintf( s_pyridelog, "ERROR: " ); \
  fprintf( s_pyridelog, MSG ); \
  fflush( s_pyridelog ); \
}

#define PYRIDE_LOGGING_FINI \
if (s_pyridelog) { \
  fclose( s_pyridelog ); \
}
#endif // !WIN32
#endif // IOS_BUILD

#ifdef WIN32
#include <winsock2.h>
#define SOCKET_T  SOCKET
#else
#define SOCKET_T  int
#define INVALID_SOCKET -1
#endif

#ifndef DEFINED_ENCRYPTION_KEY
#pragma message ( "Make sure you change the encryption key for PyRIDE client server communication. \
To disable this warning, set DEFINED_ENCRYPTION_KEY macro to true." )
#endif

enum CommandStatus {
  NONE = 0,
  OK,
  FAIL,
  DUPLICATE
};

enum TeamColour {
  BlueTeam = 1,
  PinkTeam = 2
};

typedef enum {
  RAW = 0,
  RGB,
  PROCESSED,
  RGBA,
  BGRA
}ImageFormat;

typedef enum {
  ROBOT_TEAM_MSG      = 0x0,
  ROBOT_DISCOVERY     = 0x1,
  ROBOT_DECLARE       = 0x2,
  ROBOT_TELEMETRY     = 0x3,
  ROBOT_STATUS        = 0x4,
  CLIENT_COMMAND    = 0x5,
  CLIENT_RESPONSE   = 0x6,
  CLIENT_SHUTDOWN   = 0x7
} PyRideControl;

typedef enum {
  USER_AUTH       = 0x1,
  VIDEO_SWITCH    = 0x0,
  VIDEO_START     = 0x1,
  VIDEO_STOP      = 0x2,
  VIDEO_FORMAT    = 0x3,
  TELEMETRY_START = 0x4,
  TELEMETRY_STOP  = 0x5,
  CUSTOM_COMMAND  = 0x6,
  CANCEL_CUR_OP   = 0x7
} PyRideCommand;

typedef enum {
  YELLOW_GOAL = 0x1,
  BLUE_GOAL   = 0x2,
  BALL        = 0x3,
  ROBOT       = 0x4
} ObjectType;

typedef enum {
  UNKNOWN    = 0x0,
  NAO        = 0x1,
  PR2        = 0x2,
  TURTLE_BOT = 0x3,
  ROMO       = 0x4
} RobotType;

typedef struct {
  float x;
  float y;
  float theta;
} RobotPose;

typedef struct {
  RobotType type;
  RobotPose pose;
  RobotOperationalState status;
  int nofcams;
  int nofaudios;
} RobotInfo;

typedef struct  {
  ObjectType objType;
  float x;
  float y;
} FieldObject;

typedef struct {
  bool isBlueTeam;
  bool isLogging;
  bool isAutoSampling;
  int  samplingRate;
} PyRideSettings;

typedef struct {
  int width;
  int height;
} CameraQuality;

typedef struct {
  char fps;
  char format;
  char resolution;
  char reserved;
  short dataport;
  short ctrlport;
} VideoSettings;

typedef struct {
  char channels;
  char reserved;
  short sampling;
  short dataport;
  short ctrlport;
} AudioSettings;

static const int kSupportFrameRate[] = { 1, 2, 5, 10, 15, 20, 25, 30 };
static const int kErrorFrameRate = 255;
static const int kMaxSamplingRate = 20;
static const int kMinSamplingRate = 1;
static const CameraQuality kSupportedCameraQuality[] = {{160, 120},{320,240},{640,480}};
static const int kCompressionRate[] = { 70, 60, 50 };
static const int kMotionCommandFreq = 5;
static const int kPublishFreq = 20;
static const int kUDPHeartBeatWindow = 60;

static const double kDegreeToRAD = 0.01745329252;

#ifdef USE_ENCRYPTION
#ifdef __cplusplus
extern "C" {
#endif
unsigned char * decodeBase64( const char * input, size_t * outLen );
char * encodeBase64( const unsigned char * input, size_t length );
void endecryptInit();
void endecryptFini();
int decryptMessage( const unsigned char * origMesg, int origMesgLength, unsigned char ** decryptedMesg, int * decryptedMesgLength );
int encryptMessage( const unsigned char * origMesg, int origMesgLength, unsigned char ** encryptedMesg, int * encryptedMesgLength );
int secureSHA256Hash( const unsigned char * password, const int pwlen, unsigned char * code );

#ifdef __cplusplus
}
#endif

#endif // USE_ENCRYPTION

#ifdef WIN32
int win_gettimeofday( struct timeval * tp,void * tz );
#endif
#endif // PyRideCommon_h_DEFINED
