/*
 *  PyPR2Module.cpp
 *  
 */

#include <pthread.h>
#include <string>
#include "PyPR2Module.h"
#include "PR2ProxyManager.h"

namespace pyride {

#define PYRIDE_ROBOT_MODEL  "PR2"

PyDoc_STRVAR( PyPR2_doc, \
             "PyPR2 is the main Python extension module of PyRIDE on PR2/ROS system." );

/*! \class PyPR2
 *  \brief PyPR2 is the main Python extension module of PyRIDE on PR2/ROS system.
 *
 *  PyPR2 module consists of a set of callable Python methods specifically related to
 *  PR2 low level functionalities and a set of callback functions that should be implemented
 *  by PR2 programmers.
 */
PyPR2Module * PyPR2Module::s_pyPR2Module = NULL;

static const char *kLeftArmKWlist[] = { "l_shoulder_pan_joint", "l_shoulder_lift_joint", "l_upper_arm_roll_joint", "l_elbow_flex_joint", "l_forearm_roll_joint", "l_wrist_flex_joint", "l_wrist_roll_joint", "time_to_reach", NULL };
static const char *kRightArmKWlist[] = { "r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_upper_arm_roll_joint", "r_elbow_flex_joint", "r_forearm_roll_joint", "r_wrist_flex_joint", "r_wrist_roll_joint", "time_to_reach", NULL };
static const char *kPoseKWlist[] = { "position", "orientation", NULL };
static const char *kArmPoseKWlist[] = { "position", "orientation", "use_left_arm", "time_to_reach", NULL };
  

static PyObject * PyModule_write( PyObject *self, PyObject * args )
{
  char * msg;
  std::string outputMsg;

  if (!PyArg_ParseTuple( args, "s", &msg )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  // Next send it to all (active) clients.
  while (*msg) {
    if (*msg == '\n')
      outputMsg += "\r\n";
    else
      outputMsg += *msg;
    msg++;
  }
  
  PyPR2Module::instance()->write( outputMsg.c_str() );
  
  Py_RETURN_NONE;
}

/** @name Miscellaneous Functions
 *
 */
/**@{*/
/*! \fn setToMannequinMode( to_set )
 *  \brief Set PR2 to the mannequin mode
 *  \memberof PyPR2
 *  \param bool to_set. True = mannequin mode; False = normal mode.
 *  \return None.
 */
/*! \fn startDataRecording( data_type, output_file )
 *  \brief Start to record various sensor data into a ROS bag file.
 *  \memberof PyPR2
 *  \param int data_type. Types of sensor data to be recorded. Different sensor datatype can be concate together with '|'.
 *  \param str output_file. Optional output file name path.
 *  \return None.
 *  \note Check constants.py for sensor data types.
 *  \note Default output file name path is in the format of /removable/recordings/%Y%m%d_%H%M_%sensordatatype_data.bag.
 */

/*! \fn stopDataRecording()
 *  \brief Stop recording PR2 sensor data.
 *  \memberof PyPR2
 *  \return None.
 */
/*! \fn startJoystickControl()
 *  \brief Start controlling PR2 with a PS3 joystick.
 *  \memberof PyPR2
 *  \return None.
 */
/*! \fn stopJoystickControl()
 *  \brief Stop controlling PR2 with a PS3 joystick.
 *  \memberof PyPR2
 *  \return None.
 */
/*! \fn setProjectorOff( to_set )
 *  \brief Set PR2's texture projector to off or on.
 *  \memberof PyPR2
 *  \param bool to_set. True = set projector off; False = set projector on.
 *  \return None.
 */

/*! \fn setTeamMemberID(member_id, team_id)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \memberof PyROMO
 *  \brief Sets the ID number of the robot.
 *
 *  The robot can be assigned as a member
 *  of blue team (number 1) or pink team (number 2). This facilitates robot to
 *  robot communication.
 *  \note This function is inherited from robot soccer competition
 *  code for NAO.
 *  \param int member_id. Must be a non-negative integer.
 *  \param int team_id. Must be either 1 (blue team) or 2 (pink team).
 *  \return None.
 */
static PyObject * PyModule_SetTeamMemberID( PyObject *self, PyObject * args )
{
  int teamID, teamColour;
  if (!PyArg_ParseTuple( args, "ii", &teamID, &teamColour )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (teamID < 0) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setTeamMemberID: invalid "
                 "team member ID %d!", teamID );
    
    return NULL;
  }
  if (teamColour < BlueTeam || teamColour > PinkTeam) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setTeamMemberID: invalid "
                 "team colour %d! Valid format: %d = Pink; %d = Blue", teamColour, PinkTeam, BlueTeam );

    return NULL;
  }

  ServerDataProcessor::instance()->setTeamMemberID( teamID, (TeamColour)teamColour );
  PyPR2Module::instance()->clientID( ServerDataProcessor::instance()->clientID() );

  Py_RETURN_NONE;
}

/*! \fn sendTeamMessage( message )
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \memberof PyROMO
 *  \brief Broadcast a message to other robots in the same team.
 *
 *  \param str message. A text based message.
 *  \return None
 */
static PyObject * PyModule_sendTeamMessage( PyObject *self, PyObject * args )
{
  char * dataStr = NULL;
  
  if (!PyArg_ParseTuple( args, "s", &dataStr )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  PyPR2Module::instance()->sendTeamMessage( dataStr );
  Py_RETURN_NONE;
}

/*! \fn say(text)
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \memberof PyROMO
 *  \brief Use Text-to-Speech system to say the input text.
 *  \param str text. Text to be spoken by the robot.
 *  \return None.
 */
static PyObject * PyModule_PR2SayWithVolume( PyObject * self, PyObject * args )
{
  float volume = 0.0;
  char * dataStr = NULL;
  PyObject * toBlockObj = NULL;
  
  if (!PyArg_ParseTuple( args, "s|fO", &dataStr, &volume, &toBlockObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (toBlockObj && !PyBool_Check( toBlockObj )) {
    PyErr_Format( PyExc_ValueError, "PyPR2.say: third parameter must be a boolean!" );
    return NULL;
  }
  if (volume < 0.0 || volume > 1.0) {
    PyErr_Format( PyExc_ValueError, "PyPR2.say: invalid voice volume!" );
    return NULL;
  }
  if (dataStr) {
    PR2ProxyManager::instance()->sayWithVolume( string( dataStr ), volume,
                                               (toBlockObj && PyObject_IsTrue( toBlockObj )) );
  }
  Py_RETURN_NONE;
}

/*! \fn getBatteryStatus()
 *  \memberof PyPR2
 *  \brief Return the current robot battery status.
 *  \return tuple(battery percentage, is_plugged_in, estimated remaining battery time).
 */
static PyObject * PyModule_PR2GetBatteryStatus( PyObject * self )
{
  int batpercent = 0;
  bool isplugged = false;
  float time_remain = 0.0;
  
  PR2ProxyManager::instance()->getBatteryStatus( batpercent, isplugged, time_remain );
  
  return Py_BuildValue( "(isf)", batpercent, isplugged ? "plugged in" :
                       "unplugged", time_remain );
}

/*! \fn getLowPowerThreshold()
 *  \memberof PyPR2
 *  \brief Return the current low power threshold
 *  \return Battery percentage integer.
 */
static PyObject * PyModule_PR2GetLowPowerThreshold( PyObject * self )
{
  return Py_BuildValue( "i", PR2ProxyManager::instance()->getLowPowerThreshold() );
}

/*! \fn setLowPowerThreshold(battery_percentage)
 *  \memberof PyPR2
 *  \brief Set the low power threshold.
 *
 *  When the threshold is greater than zero
 *  this level, callback PyPR2.onBatteryChargeChange or PyPR2.onPowerPluggedChange
 *  will be invoked when battery status changes.
 *  \param int battery_percentage. Must be non-negative.
 *  \return None.
 */
/**@}*/
static PyObject * PyModule_PR2SetLowPowerThreshold( PyObject * self, PyObject * args )
{
  int low_threshold = 0;
  
  if (!PyArg_ParseTuple( args, "i", &low_threshold )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (low_threshold < 0 || low_threshold >= 100) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setLowPowerThreshold: input must be an integer within [0..100)!" );
    return NULL;
  }
  PR2ProxyManager::instance()->setLowPowerThreshold( low_threshold );
  Py_RETURN_NONE;
}

/*! \fn listTFFrames()
 *  \memberof PyPR2
 *  \brief Return a list of supported PR2 TF frame names.
 *  \return list(frame names).
 */
static PyObject * PyModule_PR2ListTFFrames( PyObject * self )
{
  std::vector<std::string> framelist;
  
  PR2ProxyManager::instance()->getTFFrameList( framelist );
  int fsize = (int)framelist.size();
  PyObject * retObj = PyList_New( fsize );
  for (int i = 0; i < fsize; ++i) {
    PyList_SetItem( retObj, i, PyString_FromString( framelist[i].c_str() ) );
  }
  return retObj;
}

/*! \fn isSupportedTFFrame()
 *  \memberof PyPR2
 *  \brief Check whether the input TF frame is supported in the current system.
 *  \return bool. True == supported; False == not supported.
 */
static PyObject * PyModule_PR2CheckTFFrame( PyObject * self, PyObject * args )
{
  char * framename = NULL;

  if (!PyArg_ParseTuple( args, "s", &framename )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (PR2ProxyManager::instance()->isTFFrameSupported( framename ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveTorsoBy(length, best_time)
 *  \memberof PyPR2
 *  \brief Move the PR2 torso height by a certain length.
 *  \param float length. The relative change in torso height.
 *  \param float best_time. Optional, ask the robot try its best to reach the input pose in this timeframe.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PR2MoveTorsoBy( PyObject * self, PyObject * args )
{
  float length = 0.0;
  float bestTime = 5.0; //seconds
  
  if (!PyArg_ParseTuple( args, "f|f", &length, &bestTime )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (PR2ProxyManager::instance()->moveBodyTorsoBy( length, bestTime ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveBodyTo(x,y,theta,best_time)
 *  \memberof PyPR2
 *  \brief Move the PR2 body to a pose at (x,y,theta).
 *  \param float x. X coordinate w.r.t. the current pose.
 *  \param float y. Y coordinate w.r.t. the current pose.
 *  \param float theta. Angular position w.r.t. the current pose.
 *  \param float best_time. Optional, ask the robot try its best to reach the input pose in this timeframe.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PR2MoveBodyTo( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;
  float theta = 0.0;
  float bestTime = 5.0; //seconds
  
  if (!PyArg_ParseTuple( args, "fff|f", &xcoord, &ycoord, &theta, &bestTime )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  RobotPose pose;
  pose.x = xcoord;
  pose.y = ycoord;
  pose.theta = theta;

  if (PR2ProxyManager::instance()->moveBodyTo( pose, bestTime ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn moveBodyWithSpeed(x,y,theta)
 *  \memberof PyPR2
 *  \brief Set the PR2 body moving speed in the form of (x,y,theta).
 *  \param float x. Speed(m/s) in X axis direction.
 *  \param float y. Speed(m/s) in Y axis direction.
 *  \param float theta. Angular turning rate (rad/s) w.r.t Z axis.
 *  \return None.
 *  \note Speed is restricted to maximum of 1 m/s in X/Y direction and 0.7rad/s in turning rate.
 *  \warning This is a low level control function and no safty check is provided.
 *  Programmers must ensure PR2 robot movement is controlled safely.
 */
static PyObject * PyModule_PR2MoveBodyWithSpeed( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;
  float theta = 0.0;
  
  if (!PyArg_ParseTuple( args, "fff", &xcoord, &ycoord, &theta )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  RobotPose pose;
  pose.x = xcoord;
  pose.y = ycoord;
  pose.theta = theta;
  
  PR2ProxyManager::instance()->updateBodyPose( pose, true );
  Py_RETURN_NONE;
}

/*! \fn moveHeadTo(head_yaw, head_pitch)
 *  \memberof PyPR2
 *  \brief Move the PR2 head to a specific yaw and pitch position.
 *  \param float head_yaw. Must be in radian.
 *  \param float head_pitch. Must be in radian.
 *  \return bool. True == valid command; False == invalid command.
 *  \todo This function has not been fully tested on PR2 and is still buggy.
 */
static PyObject * PyModule_PR2MoveHeadTo( PyObject * self, PyObject * args )
{
  double yaw = 0.0;
  double pitch = 0.0;
  
  if (!PyArg_ParseTuple( args, "dd", &yaw, &pitch )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (PR2ProxyManager::instance()->moveHeadTo( yaw * kDegreeToRAD, pitch * kDegreeToRAD ))
    Py_RETURN_TRUE;

  else
    Py_RETURN_FALSE;
}

/*! \fn pointHeadTo(reference_frame, x, y, z)
 *  \memberof PyPR2
 *  \brief Point the PR2 head towards a specific coordinates in a reference frame.
 *  \param str reference_frame. Text label for the requested reference frame (TF frame name).
 *  \param float x. X coordinate.
 *  \param float y. Y coordinate.
 *  \param float z. Z coordinate.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PR2PointHeadTo( PyObject * self, PyObject * args )
{
  float xcoord = 0.0;
  float ycoord = 0.0;
  float zcoord = 0.0;
  char * reqframe = NULL;

  if (!PyArg_ParseTuple( args, "sfff", &reqframe, &xcoord, &ycoord, &zcoord )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  if (strlen(reqframe) == 0) {
    PyErr_Format( PyExc_ValueError, "PyPR2.pointHeadTo: request frame must not be empty string." );
    return NULL;
  }
  if (PR2ProxyManager::instance()->pointHeadTo( reqframe, xcoord, ycoord, zcoord ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn getHeadPos()
 *  \memberof PyPR2
 *  \memberof PyNAO
 *  \brief Get the current robot head yaw and pitch in radian.
 *  \return tuple(head_yaw, head_pitch)
 */
static PyObject * PyModule_PR2GetHeadPos( PyObject * self )
{
  double yaw = 0.0;
  double pitch = 0.0;

  if (PR2ProxyManager::instance()->getHeadPos( yaw, pitch )) {
    return Py_BuildValue( "(dd)", yaw, pitch );
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyPR2.getHeadPos: unable to get head position!" );
    return NULL;
  }
}

/*! \fn getJointPos(joint_name)
 *  \memberof PyPR2
 *  \brief Get the current (angle) position of a joint.
 *  \param str joint_name. Name of a joint.
 *  \return float position. Must be in radian.
 */
static PyObject * PyModule_PR2GetJointPos( PyObject * self, PyObject * args )
{
  double value = 0.0;
  char * joint_name = NULL;

  if (!PyArg_ParseTuple( args, "s", &joint_name )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (PR2ProxyManager::instance()->getJointPos( joint_name, value )) {
    return Py_BuildValue( "d", value );
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyPR2.getJointPos: unable to get current position for joint %s!", joint_name );
    return NULL;
  }
}

/*! \fn getPositionForJoints(joint_names)
 *  \memberof PyPR2
 *  \brief Get the current (angle) position of a joint.
 *  \param list joint_names. A list of joint names.
 *  \return list(joint_positions).
 */
static PyObject * PyModule_PR2GetPositionForJoints( PyObject * self, PyObject * args )
{
  PyObject * jnames = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &jnames )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyList_Check( jnames ) || PyList_Size( jnames ) <= 0) {
    PyErr_Format( PyExc_ValueError, "PyPR2.getPositionForJoints: must provide a list of joints." );
    return NULL;
  }

  int listSize = PyList_Size( jnames );
  
  std::vector<std::string> joints;
  std::vector<double> positions;
  
  PyObject * ckObj = NULL;
  
  for (int i = 0; i < listSize; i++) {
    ckObj = PyList_GetItem( jnames, i );
    if (PyString_Check( ckObj )) {
      joints.push_back( string( PyString_AsString( ckObj ) ));
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPR2.getPositionForJoints: joint name at index %d is not a string.", i );
      return NULL;
    }
  }

  if (PR2ProxyManager::instance()->getPositionForJoints( joints, positions )) {
    PyObject * retObj = PyList_New( listSize );
    for (int i = 0; i < listSize; i++) {
      PyList_SetItem( retObj, i, PyFloat_FromDouble( positions.at( i ) ) );
    }
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyPR2.getPositionForJoints: unable to get joint position." );
    return NULL;
  }
}

/*! \fn getArmJointPositions(left_arm)
 *  \memberof PyPR2
 *  \brief Get the current joint positions of one of the PR2 arm.
 *  \param bool left_arm. True for left arm; False for right arm.
 *  \return dictionary(arm_joint_positions).
 *  \note Returned dictionary use joint names as keys.
 */
static PyObject * PyModule_PR2GetArmJointPositions( PyObject * self, PyObject * args )
{
  PyObject * armsel = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &armsel )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyBool_Check( armsel )) {
    PyErr_Format( PyExc_ValueError, "PyPR2.getArmJointPositions: input parameter must be a boolean!" );
    return NULL;
  }


  std::vector<std::string> joints( 7 );
  std::vector<double> positions;

  if (PyObject_IsTrue( armsel )) {
    joints[0] = "l_shoulder_pan_joint";
    joints[1] = "l_shoulder_lift_joint";
    joints[2] = "l_upper_arm_roll_joint";
    joints[3] = "l_elbow_flex_joint";
    joints[4] = "l_forearm_roll_joint";
    joints[5] = "l_wrist_flex_joint";
    joints[6] = "l_wrist_roll_joint";
  }
  else {
    joints[0] = "r_shoulder_pan_joint";
    joints[1] = "r_shoulder_lift_joint";
    joints[2] = "r_upper_arm_roll_joint";
    joints[3] = "r_elbow_flex_joint";
    joints[4] = "r_forearm_roll_joint";
    joints[5] = "r_wrist_flex_joint";
    joints[6] = "r_wrist_roll_joint";
  }

  if (PR2ProxyManager::instance()->getPositionForJoints( joints, positions )) {
    PyObject * retObj = PyDict_New();
    for (int i = 0; i < 7; i++) {
      PyObject * numObj = PyFloat_FromDouble( positions.at( i ) );
      PyDict_SetItemString( retObj, joints.at( i ).c_str(), numObj );
      Py_DECREF( numObj );
    }
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyPR2.getArmJointPositions: unable to arm joint positions." );
    return NULL;
  }
}

/*! \fn getRobotPose()
 *  \memberof PyPR2
 *  \brief Get the current PR2 body pose.
 *  \return tuple(position,orientation).
 *  \note Orientation is in quaternion form (w,x,y,z).
 */
static PyObject * PyModule_PR2GetRobotPose( PyObject * self )
{
  std::vector<double> positions(3, 0.0);
  std::vector<double> orientation(4, 0.0);
  
  if (PR2ProxyManager::instance()->getRobotPose( positions, orientation )) {
    PyObject * retObj = PyDict_New();
    PyObject * posObj = PyTuple_New( 3 );
    PyObject * orientObj = PyTuple_New( 4 );
    PyTuple_SetItem( posObj, 0, PyFloat_FromDouble( positions[0] ) );
    PyTuple_SetItem( posObj, 1, PyFloat_FromDouble( positions[1] ) );
    PyTuple_SetItem( posObj, 2, PyFloat_FromDouble( positions[2] ) );
    PyTuple_SetItem( orientObj, 0, PyFloat_FromDouble( orientation[0] ) );
    PyTuple_SetItem( orientObj, 1, PyFloat_FromDouble( orientation[1] ) );
    PyTuple_SetItem( orientObj, 2, PyFloat_FromDouble( orientation[2] ) );
    PyTuple_SetItem( orientObj, 3, PyFloat_FromDouble( orientation[3] ) );

    PyDict_SetItemString( retObj, "position", posObj );
    PyDict_SetItemString( retObj, "orientation", orientObj );
    Py_DECREF( posObj );
    Py_DECREF( orientObj );
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyPR2.getRobotPose: unable to get robot position!" );
    return NULL;
  }
}

/*! \fn getRelativeTF(reference_frame, frame)
 *  \memberof PyPR2
 *  \brief Get the relative position and orientation of an input frame w.r.t to a reference frame.
 *  \param str reference_frame. Label of reference TF frame.
 *  \param str frame. Label of the target frame.
 *  \return tuple(position,orientation).
 *  \note Orientation is in quaternion form (w,x,y,z).
 */
static PyObject * PyModule_PR2GetRelativeTF( PyObject * self, PyObject * args )
{
  char * frame1 = NULL;
  char * frame2 = NULL;
  
  if (!PyArg_ParseTuple( args, "ss", &frame1, &frame2 )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  std::vector<double> positions(3, 0.0);
  std::vector<double> orientation(4, 0.0);
  
  if (PR2ProxyManager::instance()->getRelativeTF( frame1, frame2, positions, orientation )) {
    PyObject * retObj = PyDict_New();
    PyObject * posObj = PyTuple_New( 3 );
    PyObject * orientObj = PyTuple_New( 4 );
    PyTuple_SetItem( posObj, 0, PyFloat_FromDouble( positions[0] ) );
    PyTuple_SetItem( posObj, 1, PyFloat_FromDouble( positions[1] ) );
    PyTuple_SetItem( posObj, 2, PyFloat_FromDouble( positions[2] ) );
    PyTuple_SetItem( orientObj, 0, PyFloat_FromDouble( orientation[0] ) );
    PyTuple_SetItem( orientObj, 1, PyFloat_FromDouble( orientation[1] ) );
    PyTuple_SetItem( orientObj, 2, PyFloat_FromDouble( orientation[2] ) );
    PyTuple_SetItem( orientObj, 3, PyFloat_FromDouble( orientation[3] ) );
    
    PyDict_SetItemString( retObj, "position", posObj );
    PyDict_SetItemString( retObj, "orientation", orientObj );
    Py_DECREF( posObj );
    Py_DECREF( orientObj );
    return retObj;
  }
  else {
    PyErr_Format( PyExc_SystemError, "PyPR2.getRelativeTF: unable to get relative "
                 "transformation of frame '%s' w.r.t '%s'!",
                 frame2, frame1 );
    return NULL;
  }
}

/*! \fn tuckBothArms()
 *  \memberof PyPR2
 *  \brief Tuck both PR2 arms to their safe positions.
 *  \return None.
 */
static PyObject * PyModule_PR2TuckBothArms( PyObject * self )
{
  if (PR2ProxyManager::instance()->tuckArms( true, true )) {
    Py_RETURN_TRUE;
  }
  else {
    Py_RETURN_FALSE;
  }
}

/*! \fn navigateBodyTo(target_position, target_orientation)
 *  \memberof PyPR2
 *  \brief Navigate PR2 body to a specified pose.
 *  \param tuple target_position. Position in the form of (x,y,z).
 *  \param tuple target_orientation. Orientation in quarternion form (w,x,y,z).
 *  \return None.
 *  \note Must have PR2 navigation stack running prior the start of PyRIDE.
 */
static PyObject * PyModule_PR2NavigateBodyTo( PyObject * self, PyObject * args, PyObject * keywds )
{
  PyObject * posObj = NULL;
  PyObject * orientObj = NULL;
  
  if (!PyArg_ParseTupleAndKeywords( args, keywds, "OO", (char**)kPoseKWlist, &posObj, &orientObj ) ||
      !PyTuple_Check( posObj ) || !PyTuple_Check( orientObj ))
  {
    PyErr_Format( PyExc_ValueError, "PyPR2.navigateBodyTo: input parameter must be a dictionary with position and orientation tuples." );
    return NULL;
  }
  
  if (PyTuple_Size( posObj ) != (Py_ssize_t)3 ||
      PyTuple_Size( orientObj ) != (Py_ssize_t)4)
  {
    PyErr_Format( PyExc_ValueError,
                 "PyPR2.navigateBodyTo: position must be a tuple of 3 and orientation must be a tuple of 4." );
    return NULL;
  }
  
  std::vector<double> position(3, 0.0);
  std::vector<double> orientation(4, 0.0);

  PyObject * tmpObj = NULL;

  for (int i = 0; i < 3; i++) {
    tmpObj = PyTuple_GetItem( posObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyPR2.navigateBodyTo: position tuple must have float numbers." );
      return NULL;
    }
    position[i] = PyFloat_AsDouble( tmpObj );
  }

  for (int i = 0; i < 4; i++) {
    tmpObj = PyTuple_GetItem( orientObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyPR2.navigateBodyTo: orientation tuple must have float numbers." );
      return NULL;
    }
    orientation[i] = PyFloat_AsDouble( tmpObj );
  }

  PR2ProxyManager::instance()->navigateBodyTo( position, orientation );
  Py_RETURN_NONE;
}

/*! \fn moveArmPoseTo(target_position,target_orientation, use_left_arm, time_to_reach)
 *  \memberof PyPR2
 *  \brief Move a PR2 arm to a specified pose within a time frame.
 *  \param tuple target_position. Position in (x,y,z).
 *  \param tuple target_orientation. Orientation in quarternion form (w,x,y,z).
 *  \param bool use_left_arm. True to move the left arm; False to use the right arm.
 *  \param float time_to_reach. Requested timeframe for reaching the pose.
 *  \return None.
 *  \note Must have PR2 arm navigation stack running prior the start of PyRIDE.
 */
static PyObject * PyModule_PR2MoveArmPoseTo( PyObject * self, PyObject * args, PyObject * keywds )
{
  PyObject * posObj = NULL;
  PyObject * orientObj = NULL;
  PyObject * armselObj = NULL;
  double time_to_reach = 5.0;
  
  if (!PyArg_ParseTupleAndKeywords( args, keywds, "OOO|d", (char**)kArmPoseKWlist, &posObj, &orientObj, &armselObj, &time_to_reach ) ||
      !PyTuple_Check( posObj ) || !PyTuple_Check( orientObj ) || !PyBool_Check( armselObj ))
  {
    PyErr_Format( PyExc_ValueError, "PyPR2.moveArmPoseTo: input parameter must be a dictionary with position, orientation tuples and use_left_arm boolean flag." );
    return NULL;
  }
  
  if (PyTuple_Size( posObj ) != (Py_ssize_t)3 ||
      PyTuple_Size( orientObj ) != (Py_ssize_t)4)
  {
    PyErr_Format( PyExc_ValueError,
                 "PyPR2.moveArmPoseTo: position must be a tuple of 3 and orientation must be a tuple of 4." );
    return NULL;
  }
  
  std::vector<double> position(3, 0.0);
  std::vector<double> orientation(4, 0.0);
  
  PyObject * tmpObj = NULL;
  
  for (int i = 0; i < 3; i++) {
    tmpObj = PyTuple_GetItem( posObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyPR2.moveArmPoseTo: position tuple must have float numbers." );
      return NULL;
    }
    position[i] = PyFloat_AsDouble( tmpObj );
  }
  
  for (int i = 0; i < 4; i++) {
    tmpObj = PyTuple_GetItem( orientObj, i );
    if (!PyFloat_Check( tmpObj )) {
      PyErr_Format( PyExc_ValueError,
                   "PyPR2.moveArmPoseTo: orientation tuple must have float numbers." );
      return NULL;
    }
    orientation[i] = PyFloat_AsDouble( tmpObj );
  }
  
  if (PR2ProxyManager::instance()->moveArmWithGoalPose( PyObject_IsTrue( armselObj ), position, orientation, time_to_reach )) {
    Py_RETURN_TRUE;
  }
  else {
    Py_RETURN_FALSE;
  }
}

/*! \fn moveArmWithJointPos(joint_position, time_to_reach)
 *  \memberof PyPR2
 *  \brief Move a PR2 arm to the specified joint position within a time frame.
 *  \param dict joint_position. A dictionary of arm joint positions in radian.
 *  The dictionary must the same structure as the return of PyPR2.getArmJointPositions.
 *  \param float time_to_reach. Timeframe for reaching the pose.
 *  \return None.
 */
static PyObject * PyModule_PR2MoveArmWithJointPos( PyObject * self, PyObject * args, PyObject * keywds )
{
  double s_p_j, s_l_j, u_a_r_j, e_f_j, f_r_j, w_f_j, w_r_j;
  double time_to_reach = 2.0;
  
  bool isLeftArm = false;
  
  if (PyArg_ParseTupleAndKeywords( args, keywds, "ddddddd|d", (char**)kLeftArmKWlist,
                       &s_p_j, &s_l_j, &u_a_r_j, &e_f_j, &f_r_j,
                       &w_f_j, &w_r_j, &time_to_reach ))
  {
    isLeftArm = true;
  }
  else {
    PyErr_Clear();
    if (!PyArg_ParseTupleAndKeywords( args, keywds, "ddddddd|d", (char**)kRightArmKWlist,
                                    &s_p_j, &s_l_j, &u_a_r_j, &e_f_j, &f_r_j,
                                    &w_f_j, &w_r_j, &time_to_reach ))
    {
      // PyArg_ParseTuple will set the error status.
      return NULL;
    }
  }

  std::vector<double> positions( 7, 0.0 );
  positions[0] = s_p_j;
  positions[1] = s_l_j;
  positions[2] = u_a_r_j;
  positions[3] = e_f_j;
  positions[4] = f_r_j;
  positions[5] = w_f_j;
  positions[6] = w_r_j;

  PR2ProxyManager::instance()->moveArmWithJointPos( isLeftArm, positions, time_to_reach );
  Py_RETURN_NONE;
}

/*! \fn moveArmWithJointTrajectory(joint_trajectory)
 *  \memberof PyPR2
 *  \brief Move a PR2 arm to a sequence of waypoints, i.e. joint trajectory.
 *  \param list joint_trajectory. A list of waypoints that contain joint position dictionaries with the same structure
 *  of the PyPR2.moveArmWithJointPos.
 *  \return None.
 */
static PyObject * PyModule_PR2MoveArmWithJointTraj( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &trajObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  int listSize = 0;
  
  if (!PyList_Check( trajObj ) || (listSize = PyList_Size( trajObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectory: input parameter must be a non empty list of dictionary!" );
    return NULL;
  }

  PyObject * jointPos = NULL;
  PyObject * jval = NULL;
  int armsel = 0; // 1 for left and 2 for right
  
  std::vector< std::vector<double> > trajectory;
  std::vector<float> times_to_reach( listSize, 2.0 ); // default to 2 seconds;

  for (int i = 0; i < listSize; ++i) {
    jointPos = PyList_GetItem( trajObj, i );
    if (!PyDict_Check( jointPos ) || PyDict_Size( jointPos ) < 7) {
      PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectory: input list item %d "
                   "must be a dictionary containing all 7 joint entries for a PR2 arm!", i );
      return NULL;
    }
    if (!armsel) { // check first object to determine whether we have either left or right arm joint data
      PyObject * key = PyString_FromString( kLeftArmKWlist[0] );
      if (PyDict_Contains( jointPos, key )) {
        armsel = 1;
      }
      else {
        Py_DECREF( key );
        key = PyString_FromString( kRightArmKWlist[0] );
        if (PyDict_Contains( jointPos, key )) {
          armsel = 2;
        }
      }
      Py_DECREF( key );
      if (!armsel) {
        PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectory: input list item %d contains"
                     " values not related to PR2 arms!", i );
        return NULL;        
      }
    }

    std::vector<double> arm_joint_pos( 7, 0.0 );

    for (int k = 0; k < 7; k++) {
      jval = PyDict_GetItemString( jointPos, (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
      if (!jval) {
        PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectory: input list item %d has"
                     " missing %s joint value!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      if (!PyFloat_Check( jval )) {
        PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectory: input list item %d has"
                     " invalid %s joint values!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      arm_joint_pos[k] = PyFloat_AsDouble( jval );
    }
    trajectory.push_back( arm_joint_pos );
    jval = PyDict_GetItemString( jointPos, "time_to_reach" );
    if (jval && PyFloat_Check( jval )) {
      times_to_reach[i] = (float)PyFloat_AsDouble( jval );
    }
  }

  PR2ProxyManager::instance()->moveArmWithJointTrajectory( (armsel == 1), trajectory, times_to_reach );
  Py_RETURN_NONE;
}

/*! \fn moveArmWithJointTrajectoryAndSpeed(joint_trajectory)
 *  \memberof PyPR2
 *  \brief Move a PR2 arm to a sequence of waypoints, i.e. joint trajectory, coupled with associated joint velocities.
 *  \param list joint_trajectory. A list of waypoints with the joint data dictionaries structure
 *  of { "joint_name" : { "position" : value, "velocity" : value }, ... }. Each list item, i.e. a waypoint must also
 *  have a time to reach value that is consistent with the joint velocities at the adjacent waypoints.
 *  \return None.
 */
static PyObject * PyModule_PR2MoveArmWithJointTrajAndSpeed( PyObject * self, PyObject * args )
{
  PyObject * trajObj = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &trajObj )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  int listSize = 0;
  
  if (!PyList_Check( trajObj ) || (listSize = PyList_Size( trajObj )) == 0) {
    PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectoryAndSpeed: input parameter must be a non empty list of dictionary!" );
    return NULL;
  }
  
  PyObject * jointPos = NULL;
  PyObject * jval = NULL;
  int armsel = 0; // 1 for left and 2 for right
  
  std::vector< std::vector<double> > trajectory;
  std::vector< std::vector<double> > joint_velocities;
  std::vector<float> times_to_reach( listSize, 2.0 ); // default to 2 seconds;

  for (int i = 0; i < listSize; ++i) {
    jointPos = PyList_GetItem( trajObj, i );
    if (!PyDict_Check( jointPos ) || PyDict_Size( jointPos ) < 7) {
      PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectoryAndSpeed: input list item %d "
                   "must be a dictionary containing all 7 joint entries for a PR2 arm!", i );
      return NULL;
    }
    if (!armsel) { // check first object to determine whether we have either left or right arm joint data
      PyObject * key = PyString_FromString( kLeftArmKWlist[0] );
      if (PyDict_Contains( jointPos, key )) {
        armsel = 1;
      }
      else {
        Py_DECREF( key );
        key = PyString_FromString( kRightArmKWlist[0] );
        if (PyDict_Contains( jointPos, key )) {
          armsel = 2;
        }
      }
      Py_DECREF( key );
      if (!armsel) {
        PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectoryAndSpeed: input list item %d contains"
                     " values not related to PR2 arms!", i );
        return NULL;
      }
    }
    
    std::vector<double> arm_joint_pos( 7, 0.0 );
    std::vector<double> arm_joint_vel( 7, 0.0 );
    
    for (int k = 0; k < 7; k++) {
      jval = PyDict_GetItemString( jointPos, (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
      if (!jval) {
        PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectoryAndSpeed: input list item %d has"
                     " missing %s joint value!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      if (!PyDict_Check( jval )){
        PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectoryAndSpeed: input list item %d contains "
                     " non-dictionary data for joint %s!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      PyObject * pos_val = PyDict_GetItemString( jval, "position" );
      PyObject * vel_val = PyDict_GetItemString( jval, "velocity" );
      if (!(pos_val && vel_val && PyFloat_Check( pos_val ) && PyFloat_Check( vel_val ))) {
        PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectoryAndSpeed: input list item %d contains invalid"
                     " dictionary data for joint %s!", i,
                     (armsel == 1 ? kLeftArmKWlist[k] : kRightArmKWlist[k]) );
        return NULL;
      }
      arm_joint_pos[k] = PyFloat_AsDouble( pos_val );
      arm_joint_vel[k] = PyFloat_AsDouble( vel_val );
    }
    trajectory.push_back( arm_joint_pos );
    joint_velocities.push_back( arm_joint_vel );
    
    jval = PyDict_GetItemString( jointPos, "time_to_reach" );
    if (jval && PyFloat_Check( jval )) {
      times_to_reach[i] = (float)PyFloat_AsDouble( jval );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPR2.moveArmWithJointTrajectoryAndSpeed: input list item %d contains invalid"
                   " time to reach data!", i );
    }
  }
  
  PR2ProxyManager::instance()->moveArmWithJointTrajectoryAndSpeed( (armsel == 1), trajectory, joint_velocities, times_to_reach );
  Py_RETURN_NONE;
}

/*! \fn cancelMoveArmAction(is_left_arm)
 *  \memberof PyPR2
 *  \brief Cancel current arm movement action invoked by PyPR2.moveArmWithJointPos or
 *  PyPR2.moveArmWithJointTrajectory method call.
 *  \param bool is_left_arm. True cancels the left arm movement; False cancels the right arm movement.
 *  \return None.
 *  \todo This function has not been fully tested on PR2 and may be buggy.
 */
static PyObject * PyModule_PR2CancelMoveArmAction( PyObject * self, PyObject * args )
{
  PyObject * armsel = NULL;
  
  if (!PyArg_ParseTuple( args, "O", &armsel )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyBool_Check( armsel )) {
    PyErr_Format( PyExc_ValueError, "PyPR2.cancelMoveArmAction: input parameter must be a boolean!" );
    return NULL;
  }

  PR2ProxyManager::instance()->cancelArmMovement( PyObject_IsTrue( armsel ) );
  Py_RETURN_NONE;
}

/*! \fn cancelMoveBodyAction()
 *  \memberof PyPR2
 *  \brief Cancel current body movement action invoked by PyPR2.moveBodyTo or PyPR2.moveBodyWithSpeed method call.
 *  \return None.
 *  \todo This function has not been fully tested on PR2 and may be buggy.
 */
static PyObject * PyModule_PR2CancelMoveBodyAction( PyObject * self )
{
  PR2ProxyManager::instance()->cancelBodyMovement();
  Py_RETURN_NONE;
}

/*! \fn openGripper(which_gripper)
 *  \memberof PyPR2
 *  \brief Opens one or both PR2 grippers.
 *  \param int which_gripper. 1 = left gripper, 2 = right gripper and 3 = both grippers.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PR2OpenGripper( PyObject * self, PyObject * args )
{
  int mode = 0;
  if (!PyArg_ParseTuple( args, "i", &mode )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }

  if (mode < 1 || mode > 3) {
    PyErr_Format( PyExc_ValueError, "PyPR2.openGripper: invalid gripper number! 1 = left gripper, 2 = right gripper and 3 = both grippers." );
    return NULL;
  }
    
  if (PR2ProxyManager::instance()->setGripperPosition( mode, 0.08 ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn closeGripper(which_gripper)
 *  \memberof PyPR2
 *  \brief Closes one or both PR2 grippers.
 *  \param int which_gripper. 1 = left gripper, 2 = right gripper and 3 = both grippers.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PR2CloseGripper( PyObject * self, PyObject * args )
{
  int mode = 0;
  if (!PyArg_ParseTuple( args, "i", &mode )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (mode < 1 || mode > 3) {
    PyErr_Format( PyExc_ValueError, "PyPR2.closeGripper: invalid gripper number! 1 = left gripper, 2 = right gripper and 3 = both grippers." );
    return NULL;
  }
  
  if (PR2ProxyManager::instance()->setGripperPosition( mode, 0.0 ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn setGripperPosition(which_gripper, position)
 *  \memberof PyPR2
 *  \brief Opens one or both PR2 grippers.
 *  \param int which_gripper. 1 = left gripper, 2 = right gripper and 3 = both grippers.
 *  \param float position. Must be in range of [0.0,0.08].
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PR2SetGripperPosition( PyObject * self, PyObject * args )
{
  int mode = 0;
  double value = 0.0;
  
  if (!PyArg_ParseTuple( args, "id", &mode, &value )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (mode < 1 || mode > 3) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setGripperPosition: invalid gripper number! 1 = left gripper, 2 = right gripper and 3 = both grippers." );
    return NULL;
  }

  if (value < 0.0 || value > 0.08) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setGripperPosition: invalid gripper position. Must be between 0.0 and 0.08." );
    return NULL;
  }

  if (PR2ProxyManager::instance()->setGripperPosition( mode, value ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn setTiltLaserPeriodic(amplitude, period)
 *  \memberof PyPR2
 *  \brief Set the PR2 tilt laser in a pure periodic movement.
 *  \param float amplitude. Must be non-negative. Amplitude of the periodic movement.
 *  \param float period. Must be non-negative. Time period of the movement.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PR2SetTiltLaserPeriodic( PyObject * self, PyObject * args )
{
  double amp = 0;
  double period = 0.0;
  
  if (!PyArg_ParseTuple( args, "dd", &amp, &period )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (amp < 0.0 || period < 0.0) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setTiltLaserPeriodic: invalid amplitude or period." );
    return NULL;
  }

  if (PR2ProxyManager::instance()->setTiltLaserPeriodicCmd( amp, period ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn setTiltLaserTraj(positions, durations)
 *  \memberof PyPR2
 *  \brief Set a specific sequence of repeated PR2 tilt laser position.
 *  \param list positions. A list of (minimum two) tilt laser positions.
 *  \param list durations. A list of corresponding durations at the defined tilt laser position.
 *  \return bool. True == valid command; False == invalid command.
 */
static PyObject * PyModule_PR2SetTiltLaserTraj( PyObject * self, PyObject * args )
{
  PyObject * pos = NULL;
  PyObject * dur = NULL;
  
  if (!PyArg_ParseTuple( args, "OO", &pos, &dur )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (!PyList_Check( pos ) || PyList_Size( pos ) <= 1) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setTiltLaserTraj: must provide positions as a list with (at least) two elements as the first argument." );
    return NULL;
  }
  if (!PyList_Check( dur ) || PyList_Size( dur ) <= 1) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setTiltLaserTraj: must provide durations as a list with (at least) two elements as the second argument." );
    return NULL;
  }
  
  int listSize = PyList_Size( pos );
  if (listSize != PyList_Size( dur )) {
    PyErr_Format( PyExc_ValueError, "PyPR2.setTiltLaserTraj: position and durations must be equal size lists." );
    return NULL;
  }
  
  std::vector<double> positions;
  std::vector<Duration> durations;

  PyObject * ckObj = NULL;

  for (int i = 0; i < listSize; i++) {
    ckObj = PyList_GetItem( pos, i );
    if (PyFloat_Check( ckObj )) {
      positions.push_back( PyFloat_AsDouble( ckObj ) );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPR2.setTiltLaserTraj: position at index %d is not a float/double number.", i );
      return NULL;
    }
    ckObj = PyList_GetItem( dur, i );
    if (PyFloat_Check( ckObj )) {
      durations.push_back( Duration(PyFloat_AsDouble( ckObj )) );
    }
    else {
      PyErr_Format( PyExc_ValueError, "PyPR2.setTiltLaserTraj: duration at index %d is not a float/double number.", i );
      return NULL;
    }
  }

  if (PR2ProxyManager::instance()->setTiltLaserTrajCmd( positions, durations ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}

/*! \fn registerBaseScanCallback( callback_function, target_frame )
 *  \memberof PyPR2
 *  \brief Register a callback function for receiving base laser scan data.
 *  None object can be used to stop receiving the scan data.
 *  If target frame is provided, the 3D position (x,y,z) w.r.t the target 
 *  frame will be returned. Otherwise, raw laser scan data is returned.
 *  \param callback function that takes a list of raw laser range data or 3D position data as the input.
 *  \param string target_frame. Optional, the name of the target frame
 *  \return None
 */
static PyObject * PyModule_PR2RegisterBaseScanData( PyObject * self, PyObject * args )
{
  PyObject * callbackFn = NULL;
  char * target_frame = NULL;
  
  if (!PyArg_ParseTuple( args, "O|s", &callbackFn, &target_frame )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (callbackFn == Py_None) {
    PyPR2Module::instance()->setBaseScanCallback( NULL );
    PR2ProxyManager::instance()->deregisterForBaseScanData();
    Py_RETURN_NONE;
  }
  
  if (!PyCallable_Check( callbackFn )) {
    PyErr_Format( PyExc_ValueError, "First input parameter is not callable object" );
    return NULL;
  }

  PyPR2Module::instance()->setBaseScanCallback( callbackFn );
  
  if (target_frame) {
    if (!PR2ProxyManager::instance()->isTFFrameSupported( target_frame )) {
      PyErr_Format( PyExc_ValueError, "Input target frame is not supported!" );
      return NULL;
    }
    PR2ProxyManager::instance()->registerForBaseScanData( target_frame );
  }
  else {
    PR2ProxyManager::instance()->registerForBaseScanData();
  }
  Py_RETURN_NONE;
}

/*! \fn registerTiltScanCallback( callback_function, target_frame )
 *  \memberof PyPR2
 *  \brief Register a callback function for receiving tilt laser scan data.
 *  None object can be used to stop receiving the scan data.
 *  If target frame is provided, the 3D position (x,y,z) w.r.t the target
 *  frame will be returned. Otherwise, raw laser scan data is returned.
 *  \param callback function that takes a list of raw laser range data or 3D position data as the input.
 *  \param string target_frame. Optional, the name of the target frame
 *  \return None
 */
static PyObject * PyModule_PR2RegisterTiltScanData( PyObject * self, PyObject * args )
{
  PyObject * callbackFn = NULL;
  char * target_frame = NULL;
  
  if (!PyArg_ParseTuple( args, "O|s", &callbackFn, &target_frame )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (callbackFn == Py_None) {
    PyPR2Module::instance()->setTiltScanCallback( NULL );
    PR2ProxyManager::instance()->deregisterForTiltScanData();
    Py_RETURN_NONE;
  }
  
  if (!PyCallable_Check( callbackFn )) {
    PyErr_Format( PyExc_ValueError, "First input parameter is not callable object" );
    return NULL;
  }
  
  PyPR2Module::instance()->setTiltScanCallback( callbackFn );
  
  if (target_frame) {
    if (!PR2ProxyManager::instance()->isTFFrameSupported( target_frame )) {
      PyErr_Format( PyExc_ValueError, "Input target frame is not supported!" );
      return NULL;
    }
    PR2ProxyManager::instance()->registerForTiltScanData( target_frame );
  }
  else {
    PR2ProxyManager::instance()->registerForTiltScanData();
  }
  Py_RETURN_NONE;
}

#ifdef WITH_PR2HT
/*! \fn registerHumanDetectTracking( detection_callback, tracking_callback )
 *  \memberof PyPR2
 *  \brief Register callback functions to receive human detection and tracking information.
 *  Currently support only detection and tracking human (object).
 *  None object can be used to stop receiving human detection and tracking notifications.
 *  \param detection_callback function that takes inputs of (object_type, detection_id, identification_number, status)
 *  \param tracking_callback (optional) function that takes a list of dictionaries of { 'object_type', 'track_id',
 *  'bound' (in topleft x, y, width, height), 'est_pos' (in x, y z)}.
 *  \return None
 */
static PyObject * PyModule_PR2RegisterObjectDetectTracking( PyObject * self, PyObject * args )
{
  PyObject * detectcb = NULL;
  PyObject * trackcb = NULL;
  
  if (!PyArg_ParseTuple( args, "O|O", &detectcb, &detectcb )) {
    // PyArg_ParseTuple will set the error status.
    return NULL;
  }
  
  if (detectcb == Py_None) {
    PyPR2Module::instance()->setObjectDTCallback( NULL, NULL );
    PR2ProxyManager::instance()->enableHumanDetection( false );
    Py_RETURN_NONE;
  }
  
  if (!PyCallable_Check( detectcb )) {
    PyErr_Format( PyExc_ValueError, "First input parameter is not callable object" );
    return NULL;
  }

  if (trackcb && !PyCallable_Check( trackcb )) {
    PyErr_Format( PyExc_ValueError, "Secode input parameter is not callable object" );
    return NULL;
  }

  PyPR2Module::instance()->setObjectDTCallback( detectcb, trackcb );
  
  if (PR2ProxyManager::instance()->enableHumanDetection( true, (trackcb != NULL) ))
    Py_RETURN_TRUE;
  else
    Py_RETURN_FALSE;
}
#endif

#define INCLUDE_COMMON_PYMODULE_MEHTODS
#include "PyModulePyCommon.cpp"

static PyMethodDef PyModule_methods[] = {
  { "write", (PyCFunction)PyModule_write, METH_VARARGS,
    "standard output for UTS PR2 Python console." },
  { "setTeamMemberID", (PyCFunction)PyModule_SetTeamMemberID, METH_VARARGS,
    "Set PR2 team member ID and team colour." },
  { "sendTeamMessage", (PyCFunction)PyModule_sendTeamMessage, METH_VARARGS,
    "Send a message to the rest team members." },
  { "say", (PyCFunction)PyModule_PR2SayWithVolume, METH_VARARGS,
    "Let PR2 speak with an optional volume." },
  { "pointHeadTo", (PyCFunction)PyModule_PR2PointHeadTo, METH_VARARGS,
    "Point PR2 head to a new 3D position in wide stereo camera frame." },
  { "getHeadPos", (PyCFunction)PyModule_PR2GetHeadPos, METH_NOARGS,
    "Get PR2's head position." },
  { "getJointPos", (PyCFunction)PyModule_PR2GetJointPos, METH_VARARGS,
    "Get a joint's current position." },
  { "getPositionForJoints", (PyCFunction)PyModule_PR2GetPositionForJoints, METH_VARARGS,
    "Get positions for a list of joints." },
  { "getArmJointPositions", (PyCFunction)PyModule_PR2GetArmJointPositions, METH_VARARGS,
    "Get joint positions of PR2 arms." },
  { "getRobotPose", (PyCFunction)PyModule_PR2GetRobotPose, METH_NOARGS,
    "Get the current PR2 pose." },
  { "getRelativeTF", (PyCFunction)PyModule_PR2GetRelativeTF, METH_VARARGS,
    "Get the relative TF between two frames with the first frame as the reference frame." },
  { "moveHeadTo", (PyCFunction)PyModule_PR2MoveHeadTo, METH_VARARGS,
    "Move PR2 head to a new position (in degree)." },
  { "moveBodyTo", (PyCFunction)PyModule_PR2MoveBodyTo, METH_VARARGS,
    "Move PR2 base to a new pose." },
  { "moveBodyWithSpeed", (PyCFunction)PyModule_PR2MoveBodyWithSpeed, METH_VARARGS,
    "Set PR2 base moving speed." },
  { "navigateBodyTo", (PyCFunction)PyModule_PR2NavigateBodyTo, METH_VARARGS|METH_KEYWORDS,
    "Navigate PR2 base to a new pose." },
  { "cancelMoveBodyAction", (PyCFunction)PyModule_PR2CancelMoveBodyAction, METH_NOARGS,
    "Cancel the active move body actions." },
  { "tuckBothArms", (PyCFunction)PyModule_PR2TuckBothArms, METH_NOARGS,
    "Tuck both PR2 arms." },
  { "moveTorsoBy", (PyCFunction)PyModule_PR2MoveTorsoBy, METH_VARARGS,
    "Move PR2 torso up or down." },
  { "moveArmPoseTo", (PyCFunction)PyModule_PR2MoveArmPoseTo, METH_VARARGS|METH_KEYWORDS,
    "Move one of PR2 arms end point pose to a coordinate wrt torso." },
  { "moveArmWithJointPos", (PyCFunction)PyModule_PR2MoveArmWithJointPos, METH_VARARGS|METH_KEYWORDS,
    "Move one of PR2 arms with specific joint positions." },
  { "moveArmWithJointTrajectory", (PyCFunction)PyModule_PR2MoveArmWithJointTraj, METH_VARARGS,
    "Move one of PR2 arms in a specific joint trajectory (a list of joint positions)." },
  { "moveArmWithJointTrajectoryAndSpeed", (PyCFunction)PyModule_PR2MoveArmWithJointTrajAndSpeed, METH_VARARGS,
    "Move one of PR2 arms in a specific joint trajectory with joint velocity (a list of joint positions with associated velocity)." },
  { "cancelMoveArmAction", (PyCFunction)PyModule_PR2CancelMoveArmAction, METH_VARARGS,
    "Cancel the active move arm actions." },
  { "openGripper", (PyCFunction)PyModule_PR2OpenGripper, METH_VARARGS,
    "Open one or both PR2 grippers." },
  { "closeGripper", (PyCFunction)PyModule_PR2CloseGripper, METH_VARARGS,
    "Close one or both PR2 grippers." },
  { "setGripperPosition", (PyCFunction)PyModule_PR2SetGripperPosition, METH_VARARGS,
    "Set specific position on one or both PR2 grippers." },
  { "setTiltLaserPeriodic", (PyCFunction)PyModule_PR2SetTiltLaserPeriodic, METH_VARARGS,
    "Set periodic movement on PR2 tilt laser." },
  { "setTiltLaserTraj", (PyCFunction)PyModule_PR2SetTiltLaserTraj, METH_VARARGS,
    "Set specific PR2 tilt laser trajectory." },
  { "getBatteryStatus", (PyCFunction)PyModule_PR2GetBatteryStatus, METH_NOARGS,
    "Get the current battery status." },
  { "getLowPowerThreshold", (PyCFunction)PyModule_PR2GetLowPowerThreshold, METH_NOARGS,
    "Get the low power warning threshold." },
  { "setLowPowerThreshold", (PyCFunction)PyModule_PR2SetLowPowerThreshold, METH_VARARGS,
    "Set the low power warning threshold." },
  { "listTFFrames", (PyCFunction)PyModule_PR2ListTFFrames, METH_NOARGS,
    "List supported PR2 TF frames." },
  { "isSupportedTFFrame", (PyCFunction)PyModule_PR2CheckTFFrame, METH_VARARGS,
    "Check whether the input TF frames is supported." },
  { "registerBaseScanCallback", (PyCFunction)PyModule_PR2RegisterBaseScanData, METH_VARARGS,
    "Register (or deregister) a callback function to get base laser scan data. If target frame is not given, raw data is returned." },
  { "registerTiltScanCallback", (PyCFunction)PyModule_PR2RegisterTiltScanData, METH_VARARGS,
    "Register (or deregister) a callback function to get tilt laser scan data. If target frame is not given, raw data is returned." },
#ifdef WITH_PR2HT
  { "registerHumanDetectTracking", (PyCFunction)PyModule_PR2RegisterObjectDetectTracking, METH_VARARGS,
    "Register (or deregister) callback functions to get human detection and tracking information." },
#endif
#define DEFINE_COMMON_PYMODULE_METHODS
#include "PyModulePyCommon.cpp"
  { NULL, NULL, 0, NULL }           /* sentinel */
};

PyPR2Module::PyPR2Module() : PyModuleExtension( "PyPR2" )
{
  baseScanCB_ = tiltScanCB_ = NULL;
  objectDetectCB_ = objectTrackCB_ = NULL;
}

PyPR2Module::~PyPR2Module()
{
  if (baseScanCB_) {
    Py_DECREF( baseScanCB_ );
    baseScanCB_ = NULL;
  }
  if (tiltScanCB_) {
    Py_DECREF( tiltScanCB_ );
    tiltScanCB_ = NULL;
  }
  if (objectDetectCB_) {
    Py_DECREF( objectDetectCB_ );
    objectDetectCB_ = NULL;
  }
  if (objectTrackCB_) {
    Py_DECREF( objectTrackCB_ );
    objectTrackCB_ = NULL;
  }
}

PyObject * PyPR2Module::createPyModule()
{
  return Py_InitModule3( "PyPR2", PyModule_methods, PyPR2_doc );
}

PyPR2Module * PyPR2Module::instance()
{
  if (!s_pyPR2Module)
    s_pyPR2Module = new PyPR2Module();
    
  return s_pyPR2Module;
}
  
void PyPR2Module::invokeBaseScanCallback( PyObject * arg )
{
  if (baseScanCB_) {
    PyObject * pResult = PyObject_CallObject( baseScanCB_, arg );
    if (PyErr_Occurred()) {
      PyErr_Print();
    }
    Py_XDECREF( pResult );
  }
}

void PyPR2Module::invokeTiltScanCallback( PyObject * arg )
{
  if (tiltScanCB_) {
    PyObject * pResult = PyObject_CallObject( tiltScanCB_, arg );
    if (PyErr_Occurred()) {
      PyErr_Print();
    }
    Py_XDECREF( pResult );
  }
}
  
void PyPR2Module::setBaseScanCallback( PyObject * obj )
{
  if (obj) {
    if (baseScanCB_) {
      Py_DECREF( baseScanCB_ );
    }
    baseScanCB_ = obj;
    Py_INCREF( baseScanCB_ );
  }
  else if (baseScanCB_) {
    Py_DECREF( baseScanCB_ );
    baseScanCB_ = NULL;
  }
}

void PyPR2Module::setTiltScanCallback( PyObject * obj )
{
  if (obj) {
    if (tiltScanCB_) {
      Py_DECREF( tiltScanCB_ );
    }
    tiltScanCB_ = obj;
    Py_INCREF( tiltScanCB_ );
  }
  else if (tiltScanCB_) {
    Py_DECREF( tiltScanCB_ );
    tiltScanCB_ = NULL;
  }
}

#ifdef WITH_PR2HT
void PyPR2Module::invokeObjectDetectionCallback( PyObject * arg )
{
  if (objectDetectCB_) {
    PyObject * pResult = PyObject_CallObject( objectDetectCB_, arg );
    if (PyErr_Occurred()) {
      PyErr_Print();
    }
    Py_XDECREF( pResult );
  }
}

void PyPR2Module::invokeObjectTrackingCallback( PyObject * arg )
{
  if (objectTrackCB_) {
    PyObject * pResult = PyObject_CallObject( objectTrackCB_, arg );
    if (PyErr_Occurred()) {
      PyErr_Print();
    }
    Py_XDECREF( pResult );
  }
}

void PyPR2Module::setObjectDTCallback( PyObject * detectcb, PyObject * trackcb )
{
  if (detectcb) {
    if (objectDetectCB_) {
      Py_DECREF( objectDetectCB_ );
    }
    objectDetectCB_ = detectcb;
    Py_INCREF( objectDetectCB_ );
  }
  else if (objectDetectCB_) {
    Py_DECREF( objectDetectCB_ );
    objectDetectCB_ = NULL;
  }

  if (trackcb) {
    if (objectTrackCB_) {
      Py_DECREF( objectTrackCB_ );
    }
    objectTrackCB_ = trackcb;
    Py_INCREF( objectTrackCB_ );
  }
  else if (objectTrackCB_) {
    Py_DECREF( objectTrackCB_ );
    objectTrackCB_ = NULL;
  }
}
#endif
} // namespace pyride
