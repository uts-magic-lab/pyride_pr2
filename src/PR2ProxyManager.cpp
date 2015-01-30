/*
 *  PR2ProxyManager.cpp
 *  PyPR2Server
 *
 *  Created by Xun Wang on 9/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */
#include <pr2_msgs/SetPeriodicCmd.h>
#include <pr2_msgs/SetLaserTrajCmd.h>
#include "PR2ProxyManager.h"
#include "PyPR2Module.h"

#ifdef WITH_PR2HT
#include "pr2ht/DetectTrackControl.h"
#endif

namespace pyride {

static const float kMaxWalkSpeed = 1.0;
static const float kYawRate = 0.7;  // ~ 45 degree
static const float kTorsoMoveRate = 0.05; // 5cm/s
static const float kHeadYawRate = 0.7;
static const float kHeadPitchRate = 0.7;
static const float kHeadPosTolerance = 0.01;
static const long   kMotionCommandGapTolerance = 2 * 1000000 / kMotionCommandFreq; // 0.4 sec
static const double kDT = 1.0/double( kPublishFreq );
static const double kHorizon = 5.0 * kDT;
static const double kMaxHeadTilt = 1.4;
static const double kMinHeadTilt = -0.4;
static const double kMaxHeadPan = 2.7;
static const double kMaxTorsoHeight = 1.06; // w.r.t. base_link frame
static const double kMinTorsoHeight = 0.75;

static const char *kPR2TFFrameList[] = { "map", "odom_combined", "base_footprint", "base_link",
  "torso_lift_link", "head_pan_link", "head_tilt_link", "double_stereo_link",
  "r_forearm_cam", "l_forearm_cam", "wide_stereo_r_stereo_camera_frame",
  "wide_stereo_l_stereo_camera_frame", "narrow_stereo_r_stereo_camera_frame",
  "narrow_stereo_l_stereo_camera_frame","wide_stereo_link", "narrow_stereo_link",
  "wide_stereo_optical_frame", "narrow_stereo_optical_frame", "imu_link", "sensor_mount_link",
  "high_def_frame", "high_def_optical_frame", "laser_tilt_link", "base_laser_link",
  NULL };

static const int kPR2TFFrameListSize = sizeof( kPR2TFFrameList ) / sizeof( kPR2TFFrameList[0] );

// helper function
inline double PR2ProxyManager::clamp( double val, double max )
{
  if (val > max) {
    return max;
  }
  else if (val < -max) {
    return -max;
  }
  else {
    return val;
  }
}

inline double PR2ProxyManager::max( double val1, double val2 )
{
  return (val1 >= val2 ? val1 : val2);
}
          
PR2ProxyManager * PR2ProxyManager::s_pPR2ProxyManager = NULL;

PR2ProxyManager::PR2ProxyManager() :
  rawBaseScanSub_( NULL ),
  rawTiltScanSub_( NULL ),
#ifdef WITH_PR2HT
  htObjStatusSub_( NULL ),
  htObjUpdateSub_( NULL ),
#endif
  baseScanSub_( NULL ),
  tiltScanSub_( NULL ),
  baseScanNotifier_( NULL ),
  tiltScanNotifier_( NULL ),
  bodyCtrlWithOdmetry_( false ),
  bodyCtrlWithNavigation_( false ),
  torsoCtrl_( false ),
  headCtrlWithOdmetry_( false ),
  headCtrlWithActionClient_( false ),
  tuckArmCtrl_( false ),
  lGripperCtrl_( false ),
  rGripperCtrl_( false ),
  lArmCtrl_( false ),
  rArmCtrl_( false ),
  pickupOrPlaceCtrl_( false ),
  lArmActionTimeout_( 20 ),
  rArmActionTimeout_( 20 ),
  bodyActionTimeout_( 100 ),
  torsoClient_( NULL ),
  phClient_( NULL ),
  tacClient_( NULL ),
  lgripperClient_( NULL ),
  rgripperClient_( NULL ),
  mlapClient_( NULL ),
  mrapClient_( NULL ),
  mlacClient_( NULL ),
  mracClient_( NULL ),
  pickupClient_( NULL ),
  placeClient_( NULL ),
  moveBaseClient_( NULL ),
  isCharging_( true ),
  batCapacity_( 100 ),
  lowPowerThreshold_( 0 ), // # no active power monitoring
  batTimeRemain_( Duration( 1.0 ) )
{
}

PR2ProxyManager::~PR2ProxyManager()
{
}
  
PR2ProxyManager * PR2ProxyManager::instance()
{
  if (!s_pPR2ProxyManager) {
    s_pPR2ProxyManager = new PR2ProxyManager();
  }
  return s_pPR2ProxyManager;
}

void PR2ProxyManager::initWithNodeHandle( NodeHandle * nodeHandle, bool useOptionNodes )
{
  mCtrlNode_ = nodeHandle;

  mPub_ = mCtrlNode_->advertise<geometry_msgs::Twist>( "cmd_vel", 1 );
  hPub_ = mCtrlNode_->advertise<trajectory_msgs::JointTrajectory>( "head_vel", 1 );
  torsoPub_ = mCtrlNode_->advertise<trajectory_msgs::JointTrajectory>( "torso_vel", 1 );

  jointSub_ = mCtrlNode_->subscribe( "joint_states", 1, &PR2ProxyManager::jointStateDataCB, this );
  powerSub_ = mCtrlNode_->subscribe( "power_state", 1, &PR2ProxyManager::powerStateDataCB, this );

#ifdef WITH_PR2HT
  htClient_ = mCtrlNode_->serviceClient<pr2ht::DetectTrackControl>( "enable_hdt" );

  if (!htClient_.exists()) {
    ROS_INFO( "No human detection service is available." );
  }
#endif

  mCmd_.linear.x = mCmd_.linear.y = mCmd_.angular.z = 0;
  headPitchRate_ = headYawRate_ = 0.0;
  targetYaw_ = targetPitch_ = 0.0;
  
  int trials = 0;
  phClient_ = new PointHeadClient( "/head_traj_controller/point_head_action", true );
  while (!phClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the point head action server to come up." );
    trials++;
  }
  if (!phClient_->isServerConnected()) {
    ROS_INFO( "Point head action server is down." );
    delete phClient_;
    phClient_ = NULL;
  }

  trials = 0;
  torsoClient_ = new TorsoClient( "torso_controller/position_joint_action", true );
  
  while (!torsoClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the torso action server to come up." );
    trials++;
  }
  if (!torsoClient_->isServerConnected()) {
    ROS_INFO( "Torso action server is down." );
    delete tacClient_;
    tacClient_ = NULL;
  }

  trials = 0;
  lgripperClient_ = new GripperClient( "l_gripper_controller/gripper_action", true );
  while (!lgripperClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for left gripper action server to come up." );
    trials++;
  }
  if (!lgripperClient_->isServerConnected()) {
    ROS_INFO( "Left gripper action server is down." );
    delete lgripperClient_;
    lgripperClient_ = NULL;
  }
  
  trials = 0;
  rgripperClient_ = new GripperClient( "r_gripper_controller/gripper_action", true );
  while (!rgripperClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for right gripper action server to come up." );
    trials++;
  }
  if (!rgripperClient_->isServerConnected()) {
    ROS_INFO( "Right gripper action server is down." );
    delete rgripperClient_;
    rgripperClient_ = NULL;
  }

  trials = 0;
  mlacClient_ = new TrajectoryClient( "l_arm_controller/joint_trajectory_action", true );
  while (!mlacClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for move left arm by joint action server to come up." );
    trials++;
  }
  if (!mlacClient_->isServerConnected()) {
    ROS_INFO( "Move left arm by joint action server is down." );
    delete mlacClient_;
    mlacClient_ = NULL;
  }

  trials = 0;
  mracClient_ = new TrajectoryClient( "r_arm_controller/joint_trajectory_action", true );
  while (!mracClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for move right arm by joint action server to come up." );
    trials++;
  }
  if (!mracClient_->isServerConnected()) {
    ROS_INFO( "Move right arm by joint action server is down." );
    delete mracClient_;
    mracClient_ = NULL;
  }

  trials = 0;
  tacClient_ = new TuckArmsActionClient( "tuck_arms", true );
  
  while (!tacClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
    ROS_INFO( "Waiting for the tuck arm action server to come up." );
    trials++;
  }
  if (!tacClient_->isServerConnected()) {
    ROS_INFO( "Tuck arm action server is down." );
    delete tacClient_;
    tacClient_ = NULL;
  }

  if (useOptionNodes) {
    trials = 0;
    mlapClient_ = new MoveArmActionClient( "move_left_arm", true );
    while (!mlapClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
      ROS_INFO( "Waiting for move left arm by pose action server to come up." );
      trials++;
    }
    if (!mlapClient_->isServerConnected()) {
      ROS_INFO( "Move left arm by pose action server is down." );
      delete mlapClient_;
      mlapClient_ = NULL;
    }
    
    trials = 0;
    mrapClient_ = new MoveArmActionClient( "move_right_arm", true );
    while (!mrapClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
      ROS_INFO( "Waiting for move right arm by pose action server to come up." );
      trials++;
    }
    if (!mrapClient_->isServerConnected()) {
      ROS_INFO( "Move right arm by pose action server is down." );
      delete mrapClient_;
      mrapClient_ = NULL;
    }
    
    trials = 0;
    moveBaseClient_ = new MoveBaseClient( "move_base", true );
    while (!moveBaseClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
      ROS_INFO( "Waiting for move base server to come up." );
      trials++;
    }
    if (!moveBaseClient_->isServerConnected()) {
      ROS_INFO( "Move base action server is down." );
      delete moveBaseClient_;
      moveBaseClient_ = NULL;
    }
    
    trials = 0;
    while (!service::waitForService( "/object_detection", ros::Duration(2.0)) &&
           trials < 2 )
    {
      ROS_INFO( "Waiting for object detection service to come up." );
      trials++;
    }
    
    if (!service::exists( "/object_detection", true )) {
      ROS_INFO( "No object detection service available." );
      goto doneInit;  // I know
    }
    
    //wait for collision map processing client
    trials = 0;
    while (!service::waitForService( "/tabletop_collision_map_processing/tabletop_collision_map_processing",
                                    ros::Duration(2.0)) && trials < 2)
    {
      ROS_INFO( "Waiting for collision processing service to come up." );
      trials++;
    }
    
    if (!service::exists( "/tabletop_collision_map_processing/tabletop_collision_map_processing", true )) {
      ROS_INFO( "No collision processing service available." );
      goto doneInit;  // I know
    }
    
    objDetectService_ = mCtrlNode_->serviceClient<tabletop_object_detector::TabletopDetection>( "/object_detection", true );
    
    collideProcService_ = mCtrlNode_->serviceClient<tabletop_collision_map_processing::TabletopCollisionMapProcessing>
    ( "/tabletop_collision_map_processing/tabletop_collision_map_processing", true );
    
    //wait for pickup client
    trials = 0;
    pickupClient_ = new PickupActionClient( "/object_manipulator/object_manipulator_pickup", true );
    while (!pickupClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
      ROS_INFO( "Waiting for pickup action server to come up." );
      trials++;
    }
    if (!pickupClient_->isServerConnected()) {
      ROS_INFO( "Pickup action server is down." );
      delete pickupClient_;
      pickupClient_ = NULL;
    }
    
    trials = 0;
    placeClient_ = new PlaceActionClient( "/object_manipulator/object_manipulator_place", true );
    while (!placeClient_->waitForServer( ros::Duration( 2.0 ) ) && trials < 2) {
      ROS_INFO( "Waiting for place action server to come up." );
      trials++;
    }
    if (!placeClient_->isServerConnected()) {
      ROS_INFO( "Place action server is down." );
      delete placeClient_;
      placeClient_ = NULL;
    }
  }

doneInit:
  this->getHeadPos( reqHeadYaw_, reqHeadPitch_ );
  
  ROS_INFO( "PR2 PyRIDE is fully initialised." );
}

void PR2ProxyManager::fini()
{
  if (phClient_) {
    delete phClient_;
    phClient_ = NULL;
  }
  if (tacClient_) {
    delete tacClient_;
    tacClient_ = NULL;
  }
  if (lgripperClient_) {
    delete lgripperClient_;
    lgripperClient_ = NULL;
  }
  if (rgripperClient_) {
    delete rgripperClient_;
    rgripperClient_ = NULL;
  }
  if (mlapClient_) {
    delete mlapClient_;
    mlapClient_ = NULL;
  }
  if (mrapClient_) {
    delete mrapClient_;
    mrapClient_ = NULL;
  }
  if (mlacClient_) {
    delete mlacClient_;
    mlacClient_ = NULL;
  }
  if (mracClient_) {
    delete mracClient_;
    mracClient_ = NULL;
  }
  if (pickupClient_) {
    delete pickupClient_;
    pickupClient_ = NULL;
  }
  if (placeClient_) {
    delete placeClient_;
    placeClient_ = NULL;
  }
  if (moveBaseClient_) {
    delete moveBaseClient_;
    moveBaseClient_ = NULL;
  }
  jointSub_.shutdown();
}

/** @name Action Callback Functions
 *
 */
/**@{*/
/*! \typedef onHeadActionSuccess()
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.pointHeadTo method call is successful.
 *  \return None.
 */
/*! \typedef onHeadActionFailed()
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.pointHeadTo method call is failed.
 *  \return None.
 */
void PR2ProxyManager::doneHeadAction( const actionlib::SimpleClientGoalState & state,
            const PointHeadResultConstPtr & result )
{
  headCtrlWithActionClient_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onHeadActionSuccess", NULL );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onHeadActionFailed", NULL );
  }
  PyGILState_Release( gstate );
  
  ROS_INFO("Head action finished in state [%s]", state.toString().c_str());
}

void PR2ProxyManager::doneTuckArmAction( const actionlib::SimpleClientGoalState & state,
                                        const TuckArmsResultConstPtr & result )
{
  tuckArmCtrl_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onTuckArmActionSuccess", NULL );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onTuckArmActionFailed", NULL );
  }
  
  PyGILState_Release( gstate );
  
  ROS_INFO( "Tuck arm action finished in state [%s]", state.toString().c_str());
}
  
/*! \typedef onMoveArmPoseActionSuccess(is_left_arm)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.moveArmPoseTo method call is successful.
 *  \param bool is_left_arm. True = left arm; False = right arm.
 *  \return None.
 */
/*! \typedef onMoveArmPoseActionFailed(is_left_arm)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.moveArmPoseTo method call is failed.
 *  \param bool is_left_arm. True = left arm; False = right arm.
 *  \return None.
 */
void PR2ProxyManager::doneMoveLArmPoseAction( const actionlib::SimpleClientGoalState & state,
                                         const MoveArmResultConstPtr & result)
{
  lArmCtrl_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_True );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onMoveArmPoseActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onMoveArmPoseActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO("move arm action finished in state [%s]", state.toString().c_str());
}

void PR2ProxyManager::doneMoveRArmPoseAction( const actionlib::SimpleClientGoalState & state,
                                         const MoveArmResultConstPtr & result)
{
  rArmCtrl_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_False );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onMoveArmPoseActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onMoveArmPoseActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO("move arm action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onMoveArmActionSuccess(is_left_arm)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.moveArmWithJointPos or PyPR2.moveArmWithJointTrajectory method call is successful.
 *  \param bool is_left_arm. True = left arm; False = right arm.
 *  \return None.
 */
/*! \typedef onMoveArmActionFailed(is_left_arm)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.moveArmWithJointPos or PyPR2.moveArmWithJointTrajectory method call is failed.
 *  \param bool is_left_arm. True = left arm; False = right arm.
 *  \return None.
 */
void PR2ProxyManager::doneMoveLArmAction( const actionlib::SimpleClientGoalState & state,
                                        const JointTrajectoryResultConstPtr & result)
{
  lArmCtrl_ = false;

  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(O)", Py_True );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onMoveArmActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onMoveArmActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );

  ROS_INFO("move arm action finished in state [%s]", state.toString().c_str());
}

void PR2ProxyManager::doneMoveRArmAction( const actionlib::SimpleClientGoalState & state,
                                         const JointTrajectoryResultConstPtr & result)
{
  rArmCtrl_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_False );

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onMoveArmActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onMoveArmActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO("move arm action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onMoveTorsoSuccess()
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.moveTorsoBy method call is successful.
 *  \return None.
 */
/*! \typedef onMoveTorsoFailed()
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.moveTorsoBy method call is failed.
 *  \return None.
 */
void PR2ProxyManager::doneTorsoAction( const actionlib::SimpleClientGoalState & state,
                                      const SingleJointPositionResultConstPtr & result )
{
  torsoCtrl_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onMoveTorsoSuccess", NULL );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onMoveTorsoFailed", NULL );
  }
  
  PyGILState_Release( gstate );
  
  ROS_INFO( "Torso action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onNavigateBodySuccess()
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.navigateBodyTo method call is successful.
 *  \return None.
 */
/*! \typedef onNavigateBodyFailed()
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.navigateBodyTo method call is failed.
 *  \return None.
 */
void PR2ProxyManager::doneNavgiateBodyAction( const actionlib::SimpleClientGoalState & state,
                                             const MoveBaseResultConstPtr & result )
{
  bodyCtrlWithNavigation_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onNavigateBodySuccess", NULL );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onNavigateBodyFailed", NULL );
  }
  
  PyGILState_Release( gstate );
  
  ROS_INFO("nagivate body finished in state [%s]", state.toString().c_str());
}

void PR2ProxyManager::moveLArmActionFeedback( const JointTrajectoryFeedbackConstPtr & feedback )
{
  ROS_INFO( "Left arm trajectory move action feedback." );

  /*
  if (feedback->time_to_completion.toSec() > lArmActionTimeout_) {
    ROS_INFO( "Left arm trajectory move action will exceed %f seconds, force cancellation.", lArmActionTimeout_);
    mlacClient_->cancelGoal();
    lArmCtrl_ = false;

    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();

    PyObject * arg = Py_BuildValue( "(O)", Py_True );

    PyPR2Module::instance()->invokeCallback( "onMoveArmActionFailed", arg );

    Py_DECREF( arg );

    PyGILState_Release( gstate );
  }
   */
}

void PR2ProxyManager::moveRArmActionFeedback( const JointTrajectoryFeedbackConstPtr & feedback )
{
  ROS_INFO( "Right arm trajectory move action feedback." );
  /*
  if (feedback->time_to_completion.toSec() > rArmActionTimeout_) {
    ROS_INFO( "Right arm trajectory move action will exceed %f seconds, force cancellation.", rArmActionTimeout_);
    mracClient_->cancelGoal();
    rArmCtrl_ = false;
    
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    PyObject * arg = Py_BuildValue( "(O)", Py_False );
    
    PyPR2Module::instance()->invokeCallback( "onMoveArmActionFailed", arg );
    
    Py_DECREF( arg );
    
    PyGILState_Release( gstate );
  }
   */
}

void PR2ProxyManager::moveLArmPoseActionFeedback( const MoveArmFeedbackConstPtr & feedback )
{
  if (feedback->time_to_completion.toSec() > lArmActionTimeout_) {
    ROS_INFO( "Left arm movement by pose action will exceed %f seconds, force cancellation.", lArmActionTimeout_);
    mlapClient_->cancelGoal();
    lArmCtrl_ = false;
    
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
   
    PyObject * arg = Py_BuildValue( "(O)", Py_True );
   
    PyPR2Module::instance()->invokeCallback( "onMoveArmPoseActionFailed", arg );
    
    Py_DECREF( arg );
   
    PyGILState_Release( gstate );
  }
}

void PR2ProxyManager::moveRArmPoseActionFeedback( const MoveArmFeedbackConstPtr & feedback )
{
  if (feedback->time_to_completion.toSec() > rArmActionTimeout_) {
    ROS_INFO( "Right arm movement by pose action will exceed %f seconds, force cancellation.", rArmActionTimeout_);
    mrapClient_->cancelGoal();
    rArmCtrl_ = false;
    
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    PyObject * arg = Py_BuildValue( "(O)", Py_False );
    
    PyPR2Module::instance()->invokeCallback( "onMoveArmPoseActionFailed", arg );
    
    Py_DECREF( arg );
    
    PyGILState_Release( gstate );
  }
}

/*! \typedef onGripperActionSuccess(is_left_gripper)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.openGripper, PyPR2.closeGripper and PyPR2.setGripperPosition method call is successful.
 *  \param bool is_left_gripper. True means left gripper; False means right gripper.
 *  \return None.
 */
/*! \typedef onGripperActionFailed(is_left_gripper)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.openGripper, PyPR2.closeGripper and PyPR2.setGripperPosition method call is failed.
 *  \param bool is_left_gripper. True means left gripper; False means right gripper.
 *  \return None.
 */
void PR2ProxyManager::doneLGripperAction( const actionlib::SimpleClientGoalState & state,
                                         const Pr2GripperCommandResultConstPtr & result )
{
  lGripperCtrl_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();

  PyObject * arg = Py_BuildValue( "(O)", Py_True );

  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onGripperActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onGripperActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO( "Left gripper action finished in state [%s]", state.toString().c_str());
}

void PR2ProxyManager::doneRGripperAction( const actionlib::SimpleClientGoalState & state,
                                         const Pr2GripperCommandResultConstPtr & result )
{
  rGripperCtrl_ = false;
  
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_False );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onGripperActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onGripperActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO( "Right gripper action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onPickupActionSuccess(is_left_arm)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.detectAndPickupObject method call is successful.
 *  \param bool is_left_arm. True means left arm; False means right arm.
 *  \return None.
 */
/*! \typedef onPickupActionFailed(is_left_arm)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.detectAndPickupObject method call is failed.
 *  \param bool is_left_arm. True means left arm; False means right arm.
 *  \return None.
 */
void PR2ProxyManager::doneLPickupAction( const actionlib::SimpleClientGoalState & state,
                                       const PickupResultConstPtr & result )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_True );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onPickupActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onPickupActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO("Left arm pick up action finished in state [%s]", state.toString().c_str());
}

void PR2ProxyManager::doneRPickupAction( const actionlib::SimpleClientGoalState & state,
                                        const PickupResultConstPtr & result )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_False );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onPickupActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onPickupActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO("Right arm pick up action finished in state [%s]", state.toString().c_str());
}

/*! \typedef onPlaceActionSuccess(is_left_arm)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.detectAndPickupObject method call is successful.
 *  \param bool is_left_arm. True means left arm; False means right arm.
 *  \return None.
 */
/*! \typedef onPlaceActionFailed(is_left_arm)
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.detectAndPickupObject method call is failed.
 *  \param bool is_left_arm. True means left arm; False means right arm.
 *  \return None.
 */
void PR2ProxyManager::doneRPlaceAction( const actionlib::SimpleClientGoalState & state,
                                      const PlaceResultConstPtr & result )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_False );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onPlaceActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onPlaceActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO("Right arm place action finished in state [%s]", state.toString().c_str());
}
  
void PR2ProxyManager::doneLPlaceAction( const actionlib::SimpleClientGoalState & state,
                                       const PlaceResultConstPtr & result )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(O)", Py_True );
  
  if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
    PyPR2Module::instance()->invokeCallback( "onPlaceActionSuccess", arg );
  }
  else {
    PyPR2Module::instance()->invokeCallback( "onPlaceActionFailed", arg );
  }
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
  
  ROS_INFO("Left arm place action finished in state [%s]", state.toString().c_str());
}

void PR2ProxyManager::sayWithVolume( const std::string & text, float volume, bool toBlock )
{
  soundClient_.say( text.c_str() );
}

void PR2ProxyManager::setAudioVolume( const float vol )
{
  
}

void PR2ProxyManager::baseScanDataCB( const sensor_msgs::LaserScanConstPtr & msg )
{
  if (baseScanTransformFrame_.length() > 0) { // we transform to point cloud w.r.t to the frame
    sensor_msgs::PointCloud cloud;
    try  {
      tflistener_.waitForTransform( baseScanTransformFrame_, msg->header.frame_id,
                                   msg->header.stamp,
                                   ros::Duration().fromSec( (msg->ranges.size()-1) * msg->time_increment ) );

      lprojector_.transformLaserScanToPointCloud( baseScanTransformFrame_, *msg, cloud, tflistener_ );
    }
    catch (tf::TransformException& e)
    {
      return;
    }
    
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    size_t psize = cloud.points.size();
    
    PyObject * retList = PyList_New( psize );

    for (size_t i = 0; i < psize; ++i) {
      PyList_SetItem( retList, i, Py_BuildValue( "(ddd)", cloud.points[i].x, cloud.points[i].y, cloud.points[i].z ) );
    }
    PyObject * arg = Py_BuildValue( "(O)", retList );

    PyPR2Module::instance()->invokeBaseScanCallback( arg );
    
    Py_DECREF( arg );
    Py_DECREF( retList );
    
    PyGILState_Release( gstate );
  }
  else {
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    size_t rsize = msg->ranges.size();
    size_t isize = msg->intensities.size();
    
    PyObject * rangeData = PyList_New( rsize );
    PyObject * intensityData = PyList_New( isize );
    
    float data = -0.1;
    for (size_t i = 0; i < rsize; ++i) {
      data = msg->ranges[i];
      if (data > msg->range_max || data < msg->range_min) {
        data = -0.1; // invalid data
      }
      PyList_SetItem( rangeData, i, PyFloat_FromDouble( data ) );
    }

    data = 0.0;
    for (size_t i = 0; i < isize; ++i) {
      data = msg->intensities[i];
      PyList_SetItem( intensityData, i, PyFloat_FromDouble( data ) );
    }

    PyObject * arg = Py_BuildValue( "(OO)", rangeData, intensityData );
    
    PyPR2Module::instance()->invokeBaseScanCallback( arg );

    Py_DECREF( arg );
    Py_DECREF( rangeData );
    Py_DECREF( intensityData );
    
    PyGILState_Release( gstate );
  }
}

void PR2ProxyManager::tiltScanDataCB( const sensor_msgs::LaserScanConstPtr & msg )
{
  if (tiltScanTransformFrame_.length() > 0) { // we transform to point cloud w.r.t to the frame
    sensor_msgs::PointCloud cloud;
    try  {
      tflistener_.waitForTransform( tiltScanTransformFrame_, msg->header.frame_id,
                                   msg->header.stamp,
                                   ros::Duration().fromSec( (msg->ranges.size()-1) * msg->time_increment ) );
      
      lprojector_.transformLaserScanToPointCloud( tiltScanTransformFrame_, *msg, cloud, tflistener_ );
    }
    catch (tf::TransformException& e)
    {
      return;
    }
    
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    size_t psize = cloud.points.size();
    
    PyObject * retList = PyList_New( psize );
    
    for (size_t i = 0; i < psize; ++i) {
      PyList_SetItem( retList, i, Py_BuildValue( "(ddd)", cloud.points[i].x, cloud.points[i].y, cloud.points[i].z ) );
    }
    PyObject * arg = Py_BuildValue( "(O)", retList );
    
    PyPR2Module::instance()->invokeTiltScanCallback( arg );
    
    Py_DECREF( arg );
    Py_DECREF( retList );
    
    PyGILState_Release( gstate );
  }
  else {
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    size_t rsize = msg->ranges.size();
    size_t isize = msg->intensities.size();
    
    PyObject * rangeData = PyList_New( rsize );
    PyObject * intensityData = PyList_New( isize );
    
    float data = -0.1;
    for (size_t i = 0; i < rsize; ++i) {
      data = msg->ranges[i];
      if (data > msg->range_max || data < msg->range_min) {
        data = -0.1; // invalid data
      }
      PyList_SetItem( rangeData, i, PyFloat_FromDouble( data ) );
    }
    
    data = 0.0;
    for (size_t i = 0; i < isize; ++i) {
      data = msg->intensities[i];
      PyList_SetItem( intensityData, i, PyFloat_FromDouble( data ) );
    }
    
    PyObject * arg = Py_BuildValue( "(OO)", rangeData, intensityData );
    
    PyPR2Module::instance()->invokeTiltScanCallback( arg );
    
    Py_DECREF( arg );
    Py_DECREF( rangeData );
    Py_DECREF( intensityData );
    
    PyGILState_Release( gstate );
  }
}

#ifdef WITH_PR2HT
void PR2ProxyManager::htObjStatusCB( const pr2ht::TrackedObjectStatusChangeConstPtr & msg )
{
  PyGILState_STATE gstate;
  gstate = PyGILState_Ensure();
  
  PyObject * arg = Py_BuildValue( "(iiii)", msg->objtype, msg->trackid,
                                 msg->nameid, msg->status );
  
  PyPR2Module::instance()->invokeObjectDetectionCallback( arg );
  
  Py_DECREF( arg );
  
  PyGILState_Release( gstate );
}

void PR2ProxyManager::htObjUpdateCB( const pr2ht::TrackedObjectUpdateConstPtr & msg )
{
    PyGILState_STATE gstate;
    gstate = PyGILState_Ensure();
    
    size_t rsize = msg->objects.size();
  
    PyObject * retList = PyList_New( rsize );
  
    for (size_t i = 0; i < rsize; ++i) {
      pr2ht::TrackedObjectInfo obj = msg->objects[i];
      
      PyObject * retObj = PyDict_New();
      PyObject * elemObj = PyInt_FromLong( obj.objtype );
      PyDict_SetItemString( retObj, "object_type", elemObj );
      Py_DECREF( elemObj );

      elemObj = PyInt_FromLong( obj.id );
      PyDict_SetItemString( retObj, "track_id", elemObj );
      Py_DECREF( elemObj );

      elemObj = PyTuple_New( 4 );
      PyTuple_SetItem( elemObj, 0, PyFloat_FromDouble( obj.bound.tl_x ) );
      PyTuple_SetItem( elemObj, 1, PyFloat_FromDouble( obj.bound.tl_y ) );
      PyTuple_SetItem( elemObj, 2, PyFloat_FromDouble( obj.bound.width ) );
      PyTuple_SetItem( elemObj, 3, PyFloat_FromDouble( obj.bound.height ) );
      PyDict_SetItemString( retObj, "bound", elemObj );
      Py_DECREF( elemObj );

      elemObj = PyTuple_New( 3 );
      PyTuple_SetItem( elemObj, 0, PyFloat_FromDouble( obj.est_pos.x ) );
      PyTuple_SetItem( elemObj, 1, PyFloat_FromDouble( obj.est_pos.y ) );
      PyTuple_SetItem( elemObj, 2, PyFloat_FromDouble( obj.est_pos.z ) );
      PyDict_SetItemString( retObj, "est_pos", elemObj );
      Py_DECREF( elemObj );

      PyList_SetItem( retList, i, retObj );
    }
    
    PyObject * arg = Py_BuildValue( "(O)", retList );
    
    PyPR2Module::instance()->invokeObjectTrackingCallback( arg );
    
    Py_DECREF( arg );
    Py_DECREF( retList );
  
    PyGILState_Release( gstate );
}
#endif

bool PR2ProxyManager::getRobotPose( std::vector<double> & positions, std::vector<double> & orientation )
{
  tf::StampedTransform curTransform;
  
  try {
    tflistener_.waitForTransform( "odom_combined", "base_footprint",
                                 ros::Time(0), ros::Duration( 1.0 ) );
    
    
    tflistener_.lookupTransform( "odom_combined", "base_footprint",
                                ros::Time(0), curTransform );
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    return false;
  }
  
  tf::Vector3 pos = curTransform.getOrigin();
  tf::Quaternion orient = curTransform.getRotation();

  positions[0] = pos.getX();
  positions[1] = pos.getY();
  positions[2] = pos.getZ();
  
  orientation[0] = orient.w();
  orientation[1] = orient.x();
  orientation[2] = orient.y();
  orientation[3] = orient.z();
  
  return true;
}
  
bool PR2ProxyManager::getRelativeTF( const char * ref_frame,
                                        const char * child_frame,
                                        std::vector<double> & positions,
                                        std::vector<double> & orientation )
{
  if (!ref_frame || !child_frame) {
    return false;
  }
  tf::StampedTransform curTransform;
  
  try {
    tflistener_.waitForTransform( ref_frame, child_frame,
                                 ros::Time(0), ros::Duration( 1.0 ) );
    
    
    tflistener_.lookupTransform( ref_frame, child_frame,
                                ros::Time(0), curTransform );
  }
  catch (tf::TransformException ex) {
    ROS_ERROR( "%s",ex.what() );
    return false;
  }
  
  tf::Vector3 pos = curTransform.getOrigin();
  tf::Quaternion orient = curTransform.getRotation();
  
  positions[0] = pos.getX();
  positions[1] = pos.getY();
  positions[2] = pos.getZ();
  
  orientation[0] = orient.w();
  orientation[1] = orient.x();
  orientation[2] = orient.y();
  orientation[3] = orient.z();
  
  return true;
}

bool PR2ProxyManager::navigateBodyTo( const std::vector<double> & positions, const std::vector<double> & orientation )
{
  if (bodyCtrlWithNavigation_ || bodyCtrlWithOdmetry_ || !moveBaseClient_)
    return false;
  
  if (positions.size() != (size_t)3 || orientation.size() != (size_t)4) {
    return false;
  }

  move_base_msgs::MoveBaseGoal goal;
  
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose.position.x = positions[0];
  goal.target_pose.pose.position.y = positions[1];
  goal.target_pose.pose.position.z = positions[2];
  goal.target_pose.pose.orientation.w = orientation[0];
  goal.target_pose.pose.orientation.x = orientation[1];
  goal.target_pose.pose.orientation.y = orientation[2];
  goal.target_pose.pose.orientation.z = orientation[3];
  
  moveBaseClient_->sendGoal( goal,
                        boost::bind( &PR2ProxyManager::doneNavgiateBodyAction, this, _1, _2 ),
                        MoveBaseClient::SimpleActiveCallback(),
                        MoveBaseClient::SimpleFeedbackCallback() );

  bodyCtrlWithNavigation_ = true;
  return true;
}

bool PR2ProxyManager::getHeadPos( double & yaw, double & pitch )
{
  yaw = pitch = 0.0;
  int resCnt = 0;

  boost::mutex::scoped_lock lock( joint_mutex_ );
  for (size_t i = 0; i < curJointNames_.size(); i++) {
    if (curJointNames_.at( i ).compare( "head_pan_joint" ) == 0) {
      yaw = curJointPositions_.at( i );
      resCnt++;
    }
    else if (curJointNames_.at( i ).compare( "head_tilt_joint" ) == 0) {
      pitch = curJointPositions_.at( i );
      resCnt++;
    }
    if (resCnt == 2) {
      return true;
    }
  }
  return false;
}
  
bool PR2ProxyManager::getPositionForJoints( std::vector<std::string> & joint_names, std::vector<double> & positions )
{
  positions.clear();
  
  boost::mutex::scoped_lock lock( joint_mutex_ );
  for (size_t j = 0; j < joint_names.size(); j++) {
    for (size_t i = 0; i < curJointNames_.size(); i++) {
      if (curJointNames_.at( i ).compare( joint_names.at( j ) ) == 0) {
        positions.push_back( curJointPositions_.at( i ) );
        break;
      }
    }
  }
  return (joint_names.size() == positions.size());
}

bool PR2ProxyManager::getJointPos( const char * joint_name, double & value )
{
  if (joint_name == NULL)
    return false;

  boost::mutex::scoped_lock lock( joint_mutex_ );
  for (size_t i = 0; i < curJointNames_.size(); i++) {
    if (curJointNames_.at( i ).compare( joint_name ) == 0) {
      value = curJointPositions_.at( i );
      return true;
    }
  }
  return false;
}

void PR2ProxyManager::registerForBaseScanData()
{
  if (rawBaseScanSub_ || baseScanSub_) {
    ROS_WARN( "Already registered for base laser scan." );
  }
  else {
    rawBaseScanSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "base_scan", 1, &PR2ProxyManager::baseScanDataCB, this ) );
  }
}

void PR2ProxyManager::registerForBaseScanData( const std::string & target_frame )
{
  if (rawBaseScanSub_ || baseScanSub_) {
    ROS_WARN( "Already registered for base laser scan." );
  }
  else {
    baseScanTransformFrame_ = target_frame;
    baseScanSub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>( *mCtrlNode_, "base_scan", 10 );
    baseScanNotifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>( *baseScanSub_, tflistener_, baseScanTransformFrame_, 10 );

    baseScanNotifier_->registerCallback( boost::bind( &PR2ProxyManager::baseScanDataCB, this, _1 ) );
    baseScanNotifier_->setTolerance( ros::Duration( 0.01 ) );
  }
}

void PR2ProxyManager::deregisterForBaseScanData()
{
  if (rawBaseScanSub_) {
    rawBaseScanSub_->shutdown();
    delete rawBaseScanSub_;
    rawBaseScanSub_ = NULL;
  }

  if (baseScanSub_) {
    baseScanTransformFrame_ = "";
    baseScanSub_->unsubscribe();
    baseScanNotifier_->clear();
    delete baseScanNotifier_;
    delete baseScanSub_;
    baseScanNotifier_ = NULL;
    baseScanSub_ = NULL;
  }
}
  
void PR2ProxyManager::registerForTiltScanData()
{
  if (rawTiltScanSub_ || tiltScanSub_) {
    ROS_WARN( "Already registered for tilt laser scan." );
  }
  else {
    rawTiltScanSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "tilt_scan", 1, &PR2ProxyManager::tiltScanDataCB, this ) );
  }
}

void PR2ProxyManager::registerForTiltScanData( const std::string & target_frame )
{
  if (rawTiltScanSub_ || tiltScanSub_) {
    ROS_WARN( "Already registered for tilt laser scan." );
  }
  else {
    tiltScanTransformFrame_ = target_frame;
    tiltScanSub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>( *mCtrlNode_, "tilt_scan", 10 );
    tiltScanNotifier_ = new tf::MessageFilter<sensor_msgs::LaserScan>( *tiltScanSub_, tflistener_, tiltScanTransformFrame_, 10 );
    
    tiltScanNotifier_->registerCallback( boost::bind( &PR2ProxyManager::tiltScanDataCB, this, _1 ) );
    tiltScanNotifier_->setTolerance( ros::Duration( 0.01 ) );
  }
}
  
void PR2ProxyManager::deregisterForTiltScanData()
{
  if (rawTiltScanSub_) {
    rawTiltScanSub_->shutdown();
    delete rawTiltScanSub_;
    rawTiltScanSub_ = NULL;
  }
  
  if (tiltScanSub_) {
    tiltScanTransformFrame_ = "";
    tiltScanSub_->unsubscribe();
    tiltScanNotifier_->clear();
    delete tiltScanNotifier_;
    delete tiltScanSub_;
    tiltScanNotifier_ = NULL;
    tiltScanSub_ = NULL;
  }
}

bool PR2ProxyManager::moveHeadTo( double yaw, double pitch )
{
  if (headCtrlWithActionClient_ || headCtrlWithOdmetry_)
    return false;
  
  this->getHeadPos( reqHeadYaw_, reqHeadPitch_ );
  
  double newYaw, newPitch;
  newYaw = clamp( reqHeadYaw_ + yaw, kMaxHeadPan );
  newPitch = reqHeadPitch_ + pitch;
  
  if (newPitch < kMinHeadTilt) {
    newPitch = kMinHeadTilt;
  }
  else if (newPitch > kMaxHeadTilt) {
    newPitch = kMaxHeadTilt;
  }

  headYawRate_ = clamp( newYaw - reqHeadYaw_, kHeadYawRate );
  headPitchRate_ = clamp( newPitch - reqHeadPitch_, kHeadPitchRate );

  targetYaw_ = newYaw;
  targetPitch_ = newPitch;
  //ROS_INFO( "tyaw, tpitch = %f, %f", targetYaw_, targetPitch_ );
  headCtrlWithOdmetry_ = (headYawRate_ != 0.0 || headPitchRate_ != 0.0);
  return true;
}
  
bool PR2ProxyManager::pointHeadTo( const std::string & frame, float x, float y, float z )
{
  if (headCtrlWithActionClient_ || headCtrlWithOdmetry_)
    return false;

  tf::StampedTransform transform;

  pr2_controllers_msgs::PointHeadGoal goal;
  
  //the target point, expressed in the requested frame
  geometry_msgs::PointStamped point;

  point.header.frame_id = frame;
  point.point.x = x;
  point.point.y = y;
  point.point.z = z;
  
  goal.target = point;
  
  //we are pointing the high-def camera frame
  //(pointing_axis defaults to X-axis)
  goal.pointing_frame = "wide_stereo_link";
 
  goal.pointing_axis.x = 1;
  goal.pointing_axis.y = 0;
  goal.pointing_axis.z = 0;

  //take at least 0.5 seconds to get there
  goal.min_duration = ros::Duration(0.5);
  
  //and go no faster than 1 rad/s
  goal.max_velocity = 1.0;

  // Need boost::bind to pass in the 'this' pointer
  phClient_->sendGoal( goal,
                      boost::bind( &PR2ProxyManager::doneHeadAction, this, _1, _2 ),
                      PointHeadClient::SimpleActiveCallback(),
                      PointHeadClient::SimpleFeedbackCallback() );

  headCtrlWithActionClient_ = true;
  return true;
}

bool PR2ProxyManager::tuckArms( bool tuckleft, bool tuckright )
{
  if (!tacClient_ || tuckArmCtrl_ || lArmCtrl_ || rArmCtrl_ || pickupOrPlaceCtrl_)
    return false;
  
  pr2_common_action_msgs::TuckArmsGoal goal;

  goal.tuck_left = tuckleft;
  goal.tuck_right = tuckright;

  tuckArmCtrl_ = true;

  tacClient_->sendGoal( goal,
                       boost::bind( &PR2ProxyManager::doneTuckArmAction, this, _1, _2 ),
                       TuckArmsActionClient::SimpleActiveCallback(),
                       TuckArmsActionClient::SimpleFeedbackCallback() );
  return true;
}
  
bool PR2ProxyManager::detectAndPickupObject( bool isLeftArm, int objNo )
{
  if (tuckArmCtrl_ || pickupOrPlaceCtrl_ || lArmCtrl_ || rArmCtrl_) {
    ROS_ERROR( "Existing motion is in progress." );
    return false;
  }
  
  if (!pickupClient_) {
    ROS_ERROR( "No active pickup client." );
    return false;
  }
  
  if (objNo < 1) {
    return false;
  }
  int objid = objNo - 1;

  //call the tabletop detection
  ROS_INFO( "Calling tabletop detector." );
  
  tabletop_object_detector::TabletopDetection tbdetect;
  //we want recognized database objects returned
  //set this to false if you are using the pipeline without the database
  tbdetect.request.return_clusters = true;
  //we want the individual object point clouds returned as well
  tbdetect.request.return_models = false;
  tbdetect.request.num_models = 1;
  
  if (!objDetectService_.call( tbdetect )) {
    ROS_ERROR( "Tabletop detection service failed" );
    return false;
  }
  if (tbdetect.response.detection.result !=
      tbdetect.response.detection.SUCCESS)
  {
    ROS_ERROR("Tabletop detection returned error code %d",
              tbdetect.response.detection.result);
    return false;
  }
  if (tbdetect.response.detection.clusters.empty() &&
      tbdetect.response.detection.models.empty() )
  {
    ROS_ERROR( "The tabletop detector detected the table, "
              "but found no objects" );
    return false;
  }
  
  //call collision map processing
  ROS_INFO( "Calling collision map processing" );
  tabletop_collision_map_processing::TabletopCollisionMapProcessing cmproc;
  
  //pass the result of the tabletop detection
  
  cmproc.request.detection_result = tbdetect.response.detection;
  //ask for the existing map and collision models to be reset
  cmproc.request.reset_collision_models = true;
  cmproc.request.reset_attached_models = true;
  
  //ask for the results to be returned in base link frame
  cmproc.request.desired_frame = "base_link";
  
  if (!collideProcService_.call( cmproc )) {
    ROS_ERROR( "Collision map processing service failed" );
    return false;
  }
  //the collision map processor returns instances of graspable objects
  if (cmproc.response.graspable_objects.size() < (size_t)objid) {
    ROS_ERROR( "Collision map processing returned less or no graspable objects w.r.t. objID." );
    return false;
  }

  //call object pickup
  ROS_INFO( "Calling the pickup action" );
  object_manipulation_msgs::PickupGoal goal;
  
  //pass one of the graspable objects returned
  //by the collision map processor
  goal.target = cmproc.response.graspable_objects.at( objid );
  
  //pass the name that the object has in the collision environment
  //this name was also returned by the collision map processor
  goal.collision_object_name = cmproc.response.collision_object_names.at( objid );
  //pass the collision name of the table, also returned by the collision
  //map processor
  goal.collision_support_surface_name = cmproc.response.collision_support_surface_name;
  //pick up the object with the right arm
  //we will be lifting the object along the "vertical" direction
  //which is along the z axis in the base_link frame
  geometry_msgs::Vector3Stamped direction;
  direction.header.stamp = ros::Time::now();
  direction.header.frame_id = "base_link";
  direction.vector.x = 0;
  direction.vector.y = 0;
  direction.vector.z = 1;
  goal.lift.direction = direction;
  
  //request a vertical lift of 10cm after grasping the object
  goal.lift.desired_distance = 0.1;
  goal.lift.min_distance = 0.05;
  //do not use tactile-based grasping or tactile-based lift
  goal.use_reactive_lift = false;
  goal.use_reactive_execution = false;
  
  if (isLeftArm) {
    goal.arm_name = "left_arm";
    pickupClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneLPickupAction, this, _1, _2 ),
                          PickupActionClient::SimpleActiveCallback(),
                          PickupActionClient::SimpleFeedbackCallback() );
  }
  else {
    goal.arm_name = "right_arm";
    pickupClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneRPickupAction, this, _1, _2 ),
                          PickupActionClient::SimpleActiveCallback(),
                          PickupActionClient::SimpleFeedbackCallback() );
  }
  return true;
}

void PR2ProxyManager::moveArmWithJointPos( bool isLeftArm, std::vector<double> & positions, float time_to_reach )
{
  if (positions.size() != 7 || tuckArmCtrl_ || pickupOrPlaceCtrl_) {
    return;
  }
  
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  
  // First, the joint names, which apply to all waypoints
  if (isLeftArm) {
    if (!mlacClient_) {
      return;
    }
    if (lArmCtrl_) {
      ROS_WARN( "Left arm is in motion." );
      return;
    }
    goal.trajectory.joint_names.push_back( "l_shoulder_pan_joint" );
    goal.trajectory.joint_names.push_back( "l_shoulder_lift_joint" );
    goal.trajectory.joint_names.push_back( "l_upper_arm_roll_joint" );
    goal.trajectory.joint_names.push_back( "l_elbow_flex_joint" );
    goal.trajectory.joint_names.push_back( "l_forearm_roll_joint" );
    goal.trajectory.joint_names.push_back( "l_wrist_flex_joint" );
    goal.trajectory.joint_names.push_back( "l_wrist_roll_joint" );
    lArmCtrl_ = true;
  }
  else {
    if (!mracClient_) {
      return;
    }
    if (rArmCtrl_) {
      ROS_WARN( "Right arm is in motion." );
      return;
    }
    goal.trajectory.joint_names.push_back( "r_shoulder_pan_joint" );
    goal.trajectory.joint_names.push_back( "r_shoulder_lift_joint" );
    goal.trajectory.joint_names.push_back( "r_upper_arm_roll_joint" );
    goal.trajectory.joint_names.push_back( "r_elbow_flex_joint" );
    goal.trajectory.joint_names.push_back( "r_forearm_roll_joint" );
    goal.trajectory.joint_names.push_back( "r_wrist_flex_joint" );
    goal.trajectory.joint_names.push_back( "r_wrist_roll_joint" );
    rArmCtrl_ = true;
  }

  goal.trajectory.points.resize( 1 );
  
  // First trajectory point
  // Positions
  
  goal.trajectory.points[0].positions.resize( 7 );
    // Velocities
  goal.trajectory.points[0].velocities.resize( 7 );

  for (size_t j = 0; j < 7; ++j) {
    goal.trajectory.points[0].positions[j] = positions[j];
    goal.trajectory.points[0].velocities[j] = 0.0;
  }
  // To be reached 2 seconds after starting along the trajectory
  goal.trajectory.points[0].time_from_start = ros::Duration( time_to_reach );

  if (isLeftArm) {
    lArmActionTimeout_ = time_to_reach * 1.05; // give additional 5% allowance
    mlacClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneMoveLArmAction, this, _1, _2 ),
                          TrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &PR2ProxyManager::moveLArmActionFeedback, this, _1 ) );
  }
  else {
    rArmActionTimeout_ = time_to_reach * 1.05; // give additional 5% allowance
    mracClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneMoveRArmAction, this, _1, _2 ),
                          TrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &PR2ProxyManager::moveRArmActionFeedback, this, _1 ) );
  }
}

void PR2ProxyManager::moveArmWithJointTrajectory( bool isLeftArm, std::vector< std::vector<double> > & trajectory,
                                                  std::vector<float> & times_to_reach )
{
  if (tuckArmCtrl_ || pickupOrPlaceCtrl_) {
    return;
  }
  
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  
  // First, the joint names, which apply to all waypoints
  if (isLeftArm) {
    if (!mlacClient_) {
      return;
    }
    if (lArmCtrl_) {
      ROS_WARN( "Left arm is in motion." );
      return;
    }
    goal.trajectory.joint_names.push_back( "l_shoulder_pan_joint" );
    goal.trajectory.joint_names.push_back( "l_shoulder_lift_joint" );
    goal.trajectory.joint_names.push_back( "l_upper_arm_roll_joint" );
    goal.trajectory.joint_names.push_back( "l_elbow_flex_joint" );
    goal.trajectory.joint_names.push_back( "l_forearm_roll_joint" );
    goal.trajectory.joint_names.push_back( "l_wrist_flex_joint" );
    goal.trajectory.joint_names.push_back( "l_wrist_roll_joint" );
    lArmCtrl_ = true;
  }
  else {
    if (!mracClient_) {
      return;
    }
    if (rArmCtrl_) {
      ROS_WARN( "Right arm is in motion." );
      return;
    }
    goal.trajectory.joint_names.push_back( "r_shoulder_pan_joint" );
    goal.trajectory.joint_names.push_back( "r_shoulder_lift_joint" );
    goal.trajectory.joint_names.push_back( "r_upper_arm_roll_joint" );
    goal.trajectory.joint_names.push_back( "r_elbow_flex_joint" );
    goal.trajectory.joint_names.push_back( "r_forearm_roll_joint" );
    goal.trajectory.joint_names.push_back( "r_wrist_flex_joint" );
    goal.trajectory.joint_names.push_back( "r_wrist_roll_joint" );
    rArmCtrl_ = true;
  }
  
  goal.trajectory.points.resize( trajectory.size() );
  
  float time_to_reach_for_pt = 0.0;
  
  for (size_t jp = 0; jp < trajectory.size(); ++jp) {
    goal.trajectory.points[jp].positions.resize( 7 );
    // Velocities
    goal.trajectory.points[jp].velocities.resize( 7 );
    
    for (size_t j = 0; j < 7; ++j) {
      goal.trajectory.points[jp].positions[j] = trajectory[jp][j];
      goal.trajectory.points[jp].velocities[j] = 0.0;
    }
    time_to_reach_for_pt += times_to_reach[jp];
    goal.trajectory.points[jp].time_from_start = ros::Duration( time_to_reach_for_pt );
  }
  
  if (isLeftArm) {
    lArmActionTimeout_ = time_to_reach_for_pt * 1.05; // give additional 5% allowance
    mlacClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneMoveLArmAction, this, _1, _2 ),
                          TrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &PR2ProxyManager::moveLArmActionFeedback, this, _1 ) );
  }
  else {
    rArmActionTimeout_ = time_to_reach_for_pt * 1.05; // give additional 5% allowance
    mracClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneMoveRArmAction, this, _1, _2 ),
                          TrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &PR2ProxyManager::moveRArmActionFeedback, this, _1 ) );
  }
}

void PR2ProxyManager::moveArmWithJointTrajectoryAndSpeed( bool isLeftArm,
                                        std::vector< std::vector<double> > & trajectory,
                                        std::vector< std::vector<double> > & joint_velocities,
                                        std::vector<float> & times_to_reach )
{
  if (tuckArmCtrl_ || pickupOrPlaceCtrl_) {
    return;
  }
  
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  
  // First, the joint names, which apply to all waypoints
  if (isLeftArm) {
    if (!mlacClient_) {
      return;
    }
    if (lArmCtrl_) {
      ROS_WARN( "Left arm is in motion." );
      return;
    }
    goal.trajectory.joint_names.push_back( "l_shoulder_pan_joint" );
    goal.trajectory.joint_names.push_back( "l_shoulder_lift_joint" );
    goal.trajectory.joint_names.push_back( "l_upper_arm_roll_joint" );
    goal.trajectory.joint_names.push_back( "l_elbow_flex_joint" );
    goal.trajectory.joint_names.push_back( "l_forearm_roll_joint" );
    goal.trajectory.joint_names.push_back( "l_wrist_flex_joint" );
    goal.trajectory.joint_names.push_back( "l_wrist_roll_joint" );
    lArmCtrl_ = true;
  }
  else {
    if (!mracClient_) {
      return;
    }
    if (rArmCtrl_) {
      ROS_WARN( "Right arm is in motion." );
      return;
    }
    goal.trajectory.joint_names.push_back( "r_shoulder_pan_joint" );
    goal.trajectory.joint_names.push_back( "r_shoulder_lift_joint" );
    goal.trajectory.joint_names.push_back( "r_upper_arm_roll_joint" );
    goal.trajectory.joint_names.push_back( "r_elbow_flex_joint" );
    goal.trajectory.joint_names.push_back( "r_forearm_roll_joint" );
    goal.trajectory.joint_names.push_back( "r_wrist_flex_joint" );
    goal.trajectory.joint_names.push_back( "r_wrist_roll_joint" );
    rArmCtrl_ = true;
  }
  
  goal.trajectory.points.resize( trajectory.size() );

  float time_to_reach_for_pt = 0.0;

  for (size_t jp = 0; jp < trajectory.size(); ++jp) {
    goal.trajectory.points[jp].positions.resize( 7 );
    // Velocities
    goal.trajectory.points[jp].velocities.resize( 7 );
    
    for (size_t j = 0; j < 7; ++j) {
      goal.trajectory.points[jp].positions[j] = trajectory[jp][j];
      goal.trajectory.points[jp].velocities[j] = joint_velocities[jp][j];
    }
    time_to_reach_for_pt += times_to_reach[jp];
    goal.trajectory.points[jp].time_from_start = ros::Duration( time_to_reach_for_pt );
  }
  
  if (isLeftArm) {
    lArmActionTimeout_ = time_to_reach_for_pt * 1.05; // give additional 5% allowance
    mlacClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneMoveLArmAction, this, _1, _2 ),
                          TrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &PR2ProxyManager::moveLArmActionFeedback, this, _1 ) );
  }
  else {
    rArmActionTimeout_ = time_to_reach_for_pt * 1.05; // give additional 5% allowance
    mracClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneMoveRArmAction, this, _1, _2 ),
                          TrajectoryClient::SimpleActiveCallback(),
                          boost::bind( &PR2ProxyManager::moveRArmActionFeedback, this, _1 ) );
  }
}

void PR2ProxyManager::moveArmWithGoalPose( bool isLeftArm, std::vector<double> & position,
                                          std::vector<double> & orientation,
                                          float time_to_reach )
{
  if (position.size() != 3 || orientation.size() != 4 || tuckArmCtrl_ || pickupOrPlaceCtrl_) {
    return;
  }

  arm_navigation_msgs::MoveArmGoal goal;
  
  goal.motion_plan_request.num_planning_attempts = 1;
  goal.motion_plan_request.planner_id = std::string("");
  goal.planner_service_name = std::string( "ompl_planning/plan_kinematic_path" );
  goal.motion_plan_request.allowed_planning_time = ros::Duration(5.0);

  goal.motion_plan_request.goal_constraints.position_constraints.resize( 1 );
  goal.motion_plan_request.goal_constraints.orientation_constraints.resize( 1 );

  goal.motion_plan_request.goal_constraints.position_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.position_constraints[0].header.frame_id = "torso_lift_link";
  
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.x = position[0];
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.y = position[1];
  goal.motion_plan_request.goal_constraints.position_constraints[0].position.z = position[2];
  
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.type = arm_navigation_msgs::Shape::BOX;
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back( 0.02 ) ;
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back( 0.02 );
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_shape.dimensions.push_back( 0.02 );
  
  goal.motion_plan_request.goal_constraints.position_constraints[0].constraint_region_orientation.w = orientation[0];
  goal.motion_plan_request.goal_constraints.position_constraints[0].weight = 1.0;
  
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.stamp = ros::Time::now();
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].header.frame_id = "torso_lift_link";
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.w = orientation[0];
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.x = orientation[1];
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.y = orientation[2];
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].orientation.z = orientation[3];
  
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_roll_tolerance = 0.04;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_pitch_tolerance = 0.04;
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].absolute_yaw_tolerance = 0.04;
  
  goal.motion_plan_request.goal_constraints.orientation_constraints[0].weight = 1.0;
  
  // First, the joint names, which apply to all waypoints
  if (isLeftArm) {
    if (!mlapClient_) {
      return;
    }
    if (lArmCtrl_) {
      ROS_WARN( "Left arm is in motion." );
      return;
    }
    goal.motion_plan_request.group_name = "left_arm";
    goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = "l_wrist_roll_link";
    goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "l_wrist_roll_link";
    lArmCtrl_ = true;
  }
  else {
    if (!mrapClient_) {
      return;
    }
    if (rArmCtrl_) {
      ROS_WARN( "Right arm is in motion." );
      return;
    }
    goal.motion_plan_request.group_name = "right_arm";
    goal.motion_plan_request.goal_constraints.position_constraints[0].link_name = "r_wrist_roll_link";
    goal.motion_plan_request.goal_constraints.orientation_constraints[0].link_name = "r_wrist_roll_link";
    rArmCtrl_ = true;
  }
  
  
  if (isLeftArm) {
    lArmActionTimeout_ = time_to_reach;
    mlapClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneMoveLArmPoseAction, this, _1, _2 ),
                          MoveArmActionClient::SimpleActiveCallback(),
                          boost::bind( &PR2ProxyManager::moveLArmPoseActionFeedback, this, _1 ) );
  }
  else {
    rArmActionTimeout_ = time_to_reach;
    mrapClient_->sendGoal( goal,
                          boost::bind( &PR2ProxyManager::doneMoveRArmPoseAction, this, _1, _2 ),
                          MoveArmActionClient::SimpleActiveCallback(),
                          boost::bind( &PR2ProxyManager::moveRArmPoseActionFeedback, this, _1 ) );
  }
}

void PR2ProxyManager::cancelArmMovement( bool isLeftArm )
{
  if (isLeftArm) {
    if (!lArmCtrl_) {
      return;
    }
    if (mlacClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE ||
        mlacClient_->getState() == actionlib::SimpleClientGoalState::PENDING)
    {
      mlacClient_->cancelGoal();
    }
    lArmCtrl_ = false;
  }
  else {
    if (!rArmCtrl_) {
      return;
    }
    if (mracClient_->getState() == actionlib::SimpleClientGoalState::ACTIVE ||
        mracClient_->getState() == actionlib::SimpleClientGoalState::PENDING)
    {
      mracClient_->cancelGoal();
    }
    rArmCtrl_ = false;
  }
}

void PR2ProxyManager::cancelBodyMovement()
{
  if (!bodyCtrlWithOdmetry_)
    return;

  mCmd_.linear.x = mCmd_.linear.y = mCmd_.angular.z = 0.0;
  bodyCtrlWithOdmetry_ = false;
}
  
bool PR2ProxyManager::setGripperPosition( int whichgripper, double position )
{
  if (position > 0.08 || position < 0.0 )
    return false;

  pr2_controllers_msgs::Pr2GripperCommandGoal gaction;
  gaction.command.position = position;
  gaction.command.max_effort = 60.0;  // not too harsh. TODO: use sensor gripper action later on.
  
  switch (whichgripper) {
    case 1:
      if (!lgripperClient_ || lGripperCtrl_) {
        return false;
      }
      else {
        lGripperCtrl_ = true;
        lgripperClient_->sendGoal( gaction,
                                  boost::bind( &PR2ProxyManager::doneLGripperAction, this, _1, _2 ),
                                  GripperClient::SimpleActiveCallback(),
                                  GripperClient::SimpleFeedbackCallback() );
      }
      break;
    case 2:
      if (!rgripperClient_ || rGripperCtrl_) {
        return false;
      }
      else {
        rGripperCtrl_ = true;
        rgripperClient_->sendGoal( gaction,
                                  boost::bind( &PR2ProxyManager::doneRGripperAction, this, _1, _2 ),
                                  GripperClient::SimpleActiveCallback(),
                                  GripperClient::SimpleFeedbackCallback() );
      }
      break;
    case 3:
      if (!lgripperClient_ || lGripperCtrl_ ||
          !rgripperClient_ || rGripperCtrl_)
      {
        return false;
      }
      else {
        lGripperCtrl_ = true;
        rGripperCtrl_ = true;
        lgripperClient_->sendGoal( gaction,
                                  boost::bind( &PR2ProxyManager::doneLGripperAction, this, _1, _2 ),
                                  GripperClient::SimpleActiveCallback(),
                                  GripperClient::SimpleFeedbackCallback() );
        rgripperClient_->sendGoal( gaction,
                                  boost::bind( &PR2ProxyManager::doneRGripperAction, this, _1, _2 ),
                                  GripperClient::SimpleActiveCallback(),
                                  GripperClient::SimpleFeedbackCallback() );
      }
      break;
    default:
      ROS_ERROR( "SetGripperPosition: unknown gripper" );
      return false;
      break;
  }
  return true;
}

#ifdef WITH_PR2HT
bool PR2ProxyManager::enableHumanDetection( bool toEnable, bool enableTrackingNotif )
{
  if (!htClient_.exists()) {
    return false;
  }
  pr2ht::DetectTrackControl srvMsg;
  
  if (toEnable) {
    if (htObjStatusSub_) {
      ROS_WARN( "Human detection service is already enabled." );
      return true;
    }
  }
  else if (!htObjStatusSub_) {
    ROS_WARN( "Human detection service is already disabled." );
    return true;
  }

  srvMsg.request.tostart = toEnable;
  
  if (htClient_.call( srvMsg ) && srvMsg.response.ret) {
    if (toEnable) {
      htObjStatusSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "/pr2_ht/object_status", 1, &PR2ProxyManager::htObjStatusCB, this ) );
      if (enableTrackingNotif) {
        htObjUpdateSub_ = new ros::Subscriber( mCtrlNode_->subscribe( "/pr2_ht/object_update", 1, &PR2ProxyManager::htObjUpdateCB, this ) );
      }
    }
    else {
      if (htObjStatusSub_) {
        htObjStatusSub_->shutdown();
        delete htObjStatusSub_;
        htObjStatusSub_ = NULL;
      }
      if (htObjUpdateSub_) {
        htObjUpdateSub_->shutdown();
        delete htObjUpdateSub_;
        htObjUpdateSub_ = NULL;
      }
    }
    return true;
  }
  return false;
}
#endif

bool PR2ProxyManager::setTiltLaserPeriodicCmd( double amp, double period, double offset )
{
  if (period < 0.1 || amp < 0.0) {
    return false;
  }
  ServiceClient client = mCtrlNode_->serviceClient<pr2_msgs::SetPeriodicCmd>("laser_tilt_controller/set_periodic_cmd");
  pr2_msgs::SetPeriodicCmd cmd;
  cmd.request.command.profile = "linear";
  cmd.request.command.period = period;
  cmd.request.command.amplitude = amp;
  cmd.request.command.offset = offset;
  if (client.call( cmd )) {
    ROS_INFO( "Start tilt laser periodic move at %f.", cmd.response.start_time.toSec() );
    return true;
  }
  return false;
}

bool PR2ProxyManager::setTiltLaserTrajCmd( std::vector<double> & positions, std::vector<Duration> & durations )
{
  if (positions.size() == 0 || durations.size() == 0 || positions.size() != durations.size()) {
    return false;
  }
  ServiceClient client = mCtrlNode_->serviceClient<pr2_msgs::SetLaserTrajCmd>("laser_tilt_controller/set_traj_cmd");
  pr2_msgs::SetLaserTrajCmd cmd;
  cmd.request.command.profile = "linear";
  cmd.request.command.position = positions;
  cmd.request.command.time_from_start = durations;
  cmd.request.command.max_velocity = -1.0;
  cmd.request.command.max_acceleration = -1.0;

  if (client.call( cmd )) {
    return true;
  }
  return false;
}

void PR2ProxyManager::updateHeadPose( float yaw, float pitch )
{
  if (headCtrlWithActionClient_ || headCtrlWithOdmetry_)
    return;
  
  headYawRate_ = clamp( yaw, kHeadYawRate );
  headPitchRate_ = clamp( pitch, kHeadPitchRate );
  
  cmdTimeStamp_ = ros::Time::now();
}

bool PR2ProxyManager::moveBodyTo( const RobotPose & pose, const float bestTime )
{
  if (bodyCtrlWithOdmetry_ || bodyCtrlWithNavigation_)
    return false;
  
  tflistener_.waitForTransform( "base_footprint", "odom_combined",
                               ros::Time(0), ros::Duration( 1.0 ) );
  
  //we will record transforms here
  tflistener_.lookupTransform( "base_footprint", "odom_combined",
                              ros::Time(0), startTransform_ );
  
  poseTrans_ = pose;
  while (poseTrans_.theta <= -M_PI) poseTrans_.theta += 2 * M_PI;
  while (poseTrans_.theta > M_PI) poseTrans_.theta -= 2 * M_PI;
  
  
  mCmd_.linear.x = clamp( poseTrans_.x / bestTime, kMaxWalkSpeed );
  mCmd_.linear.y = clamp( poseTrans_.y / bestTime, kMaxWalkSpeed );
  mCmd_.angular.z = clamp( poseTrans_.theta / bestTime, kYawRate );
  
  // calculate estimated finished time.
  double reqTime = 0.0;
  if (mCmd_.linear.x != 0.0)
    reqTime = poseTrans_.x / mCmd_.linear.x;
  if (mCmd_.linear.y != 0.0)
    reqTime = max( reqTime, poseTrans_.y / mCmd_.linear.y );
  if (mCmd_.angular.z != 0.0)
    reqTime = max( reqTime, poseTrans_.theta / mCmd_.angular.z );
  
  bcwoTimeToComplete_ = ros::Time::now() + ros::Duration( reqTime * 1.1 ); // give additional 10% allowance
  
   // for slow running simulator that we increase the speed
  mCmd_.linear.x *= 2.5;
  mCmd_.linear.y *= 2.5;
  mCmd_.angular.z *= 2.5;
  
  bodyCtrlWithOdmetry_ = true;
  return true;
}

void PR2ProxyManager::updateBodyPose( const RobotPose & speed, bool localupdate )
{
  if (localupdate) {
    geometry_msgs::Twist mCmd;
    
    mCmd.linear.x = clamp( speed.x, kMaxWalkSpeed );
    mCmd.linear.y = clamp( speed.y, kMaxWalkSpeed );
    mCmd.angular.z = clamp( speed.theta, kYawRate );
    
    ROS_INFO( "PR2 Body moving speed update." );
    
    mPub_.publish( mCmd );  // publish once
  }
  else {
    if (bodyCtrlWithOdmetry_)
      return;

    mCmd_.linear.x = clamp( speed.x, kMaxWalkSpeed );
    mCmd_.linear.y = clamp( speed.y, kMaxWalkSpeed );
    mCmd_.angular.z = clamp( speed.theta, kYawRate );
    
    cmdTimeStamp_ = ros::Time::now();
  }
}

bool PR2ProxyManager::moveBodyTorsoBy( const float rel_pos, const float bestTime )
{
  if (torsoCtrl_ || !torsoClient_)
    return false;

  tf::StampedTransform curTransform;

  try {
    tflistener_.lookupTransform( "base_link", "torso_lift_link",
                                ros::Time(0), curTransform );
  }
  catch (tf::TransformException ex) {
    ROS_ERROR( "%s",ex.what() );
    return false;
  }

  double torsoPos = curTransform.getOrigin().getZ() + rel_pos;
  if (torsoPos > kMaxTorsoHeight || torsoPos < kMinTorsoHeight) {
    return false;
  }
  double torso_speed = fabs(rel_pos / bestTime);
  if (torso_speed > kTorsoMoveRate) {
    return false;
  }
  pr2_controllers_msgs::SingleJointPositionGoal torsoGoal;
  torsoGoal.position = torsoPos - kMinTorsoHeight;
  //up.min_duration = ros::Duration(2.0);
  
  torsoGoal.max_velocity = torso_speed;
  torsoClient_->sendGoal( torsoGoal,
                         boost::bind( &PR2ProxyManager::doneTorsoAction, this, _1, _2 ),
                         TorsoClient::SimpleActiveCallback(),
                         TorsoClient::SimpleFeedbackCallback() );
  torsoCtrl_ = true;

  return true;
}

void PR2ProxyManager::jointStateDataCB( const sensor_msgs::JointStateConstPtr & msg )
{
  boost::mutex::scoped_lock lock( joint_mutex_ );
  curJointNames_ = msg->name;
  curJointPositions_ = msg->position;
}

/*! \typedef onPowerPluggedChange(is_plugged_in)
 *  \memberof PyPR2.
 *  \brief Callback function when PR2 power status changes.
 *  \param bool is_plugged_in. True if the robot is plugged in main power.
 *  \return None.
 *  \note Require low power threshold to be greater than zero.
 */
/*! \typedef onBatteryChargeChange(battery_status)
 *  \memberof PyPR2.
 *  \brief Callback function when PR2 battery status changes.
 *  \param tuple battery_status. A tuple of (battery percentage,is_charging,is_battery_below_threshold).
 *  \return None.
 *  \note Require low power threshold to be greater than zero.
 */
void PR2ProxyManager::powerStateDataCB(const pr2_msgs::PowerStateConstPtr & msg )
{
  boost::mutex::scoped_lock lock( bat_mutex_ );

  bool charging = (msg->AC_present > 0);
  int batpercent = msg->relative_capacity;
  batTimeRemain_ = msg->time_remaining;

  if (lowPowerThreshold_ > 0) {
    PyObject * arg = NULL;
    if (charging != isCharging_) {
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();
      
      arg = Py_BuildValue( "(O)", charging ? Py_True : Py_False );
      
      PyPR2Module::instance()->invokeCallback( "onPowerPluggedChange", arg );
      Py_DECREF( arg );
      
      PyGILState_Release( gstate );
    }
    if (batpercent != batCapacity_) {
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();
      
      arg = Py_BuildValue( "(iOO)", batpercent, charging ? Py_True : Py_False,
                          (batpercent < lowPowerThreshold_ ? Py_True : Py_False) );

      PyPR2Module::instance()->invokeCallback( "onBatteryChargeChange", arg );
      Py_DECREF( arg );
      
      PyGILState_Release( gstate );
    }
  }
  isCharging_ = charging;
  batCapacity_ = batpercent;
}

void PR2ProxyManager::setLowPowerThreshold( int percent )
{
  if (percent >= 0 && percent < 100) {
    lowPowerThreshold_ = percent;
  }
}

void PR2ProxyManager::getBatteryStatus( int & percentage, bool & isplugged, float & timeremain )
{
  boost::mutex::scoped_lock lock( bat_mutex_ );
  isplugged = isCharging_;
  percentage = batCapacity_;
  timeremain = (float)batTimeRemain_.toSec();
}

/*! \typedef onMoveBodySuccess()
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.moveBodyTo method call is successful.
 *  \return None.
 */
/*! \typedef onMoveBodyFailed()
 *  \memberof PyPR2.
 *  \brief Callback function when PyPR2.moveBodyTo method call is failed.
 *  \return None.
 */
void PR2ProxyManager::publishCommands()
{
  if (bodyCtrlWithOdmetry_) {
    mPub_.publish( mCmd_ );
    
    tf::StampedTransform curTransform;
    
    try {
      tflistener_.lookupTransform( "base_footprint", "odom_combined",
                                ros::Time(0), curTransform );
    }
    catch (tf::TransformException ex) {
      ROS_ERROR( "%s",ex.what() );
      return;
    }
    // check translation
    tf::Transform relTransform = startTransform_.inverse() * curTransform;
    
    bool distReached = (relTransform.getOrigin().length2() >= (poseTrans_.x * poseTrans_.x + poseTrans_.y * poseTrans_.y));
    
    if (distReached) {
      mCmd_.linear.x = 0.0; mCmd_.linear.y = 0.0;
    }
    
    tf::Vector3 desired_turn_axis( 0, 0, 1 );

    // -ve theta is clockwise
    if (poseTrans_.theta > 0)
      desired_turn_axis = -desired_turn_axis;

    tf::Vector3 actual_turn_axis = relTransform.getRotation().getAxis();
    
    double angle_turned = relTransform.getRotation().getAngle();
    bool rotationReached = (poseTrans_.theta == 0.0);

    if (fabs(angle_turned) > 1.0e-2) {
      if (actual_turn_axis.dot( desired_turn_axis ) < 0)
        angle_turned = 2 * M_PI - angle_turned;

      rotationReached = (fabs(angle_turned) >= fabs(poseTrans_.theta));

    }
    
    if (rotationReached) {
      mCmd_.angular.z = 0.0;
    }

    bodyCtrlWithOdmetry_ = !(distReached && rotationReached); // set to false when we reach target. shouldn't have race conditions.
    if (!bodyCtrlWithOdmetry_) {
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();
      
      PyPR2Module::instance()->invokeCallback( "onMoveBodySuccess", NULL );
      
      PyGILState_Release( gstate );
      
      ROS_INFO( "Move body action finished." );
    }
    else if (this->isBodyControlWithOdometryTimeExpired()) {
      bodyCtrlWithOdmetry_ = false;
      PyGILState_STATE gstate;
      gstate = PyGILState_Ensure();
      
      PyPR2Module::instance()->invokeCallback( "onMoveBodyFailed", NULL );
      
      PyGILState_Release( gstate );
      
      ROS_INFO( "Move body action failed." );
    }
  }
  else if (mCmd_.linear.x || mCmd_.linear.y || mCmd_.angular.z) {
    // check if we have recent command update from the client
    // if not, assume the worse and do not publish cached commmand
    if ((ros::Time::now() - cmdTimeStamp_) < ros::Duration( kMotionCommandGapTolerance * 1E6 )) {
      mPub_.publish( mCmd_ );
    }
    else {
      bool fire = false;
      if (fabs(mCmd_.linear.x) > 0.05) {
        mCmd_.linear.x *= 0.5;
        fire = true;
      }
      else {
        mCmd_.linear.x = 0.0;
      }
      if (fabs(mCmd_.linear.y) > 0.05) {
        mCmd_.linear.y *= 0.5;
        fire = true;
      }
      else {
        mCmd_.linear.y = 0.0;
      }      
      if (fabs(mCmd_.angular.z) > 0.05) {
        mCmd_.angular.z *= 0.5;
        fire = true;
      }
      else {
        mCmd_.angular.z = 0.0;
      }
      if (fire) {
        mPub_.publish( mCmd_ );
      }
    }
  }
  
  if (headCtrlWithActionClient_) {
    return;
  }
  else if (headCtrlWithOdmetry_) {

    //this->getHeadPos( reqHeadYaw_, reqHeadPitch_ ); // get the latest data

    if (headYawRate_ > 0.0) {
      headYawRate_ = (reqHeadYaw_ >= targetYaw_ ? 0.0 : headYawRate_);
    }
    else if (headYawRate_ < 0.0) {
      headYawRate_ = (reqHeadYaw_ <= targetYaw_ ? 0.0 : headYawRate_);
    }
    if (headPitchRate_ > 0.0) {
      headPitchRate_ = (reqHeadPitch_ >= targetPitch_ ? 0.0 : headPitchRate_);
    }
    else if (headPitchRate_ < 0.0) {
      headPitchRate_ = (reqHeadPitch_ <= targetPitch_ ? 0.0 : headPitchRate_);
    }

    headCtrlWithOdmetry_ = (headYawRate_ != 0.0 || headPitchRate_ != 0.0);
    
    if (headCtrlWithOdmetry_) {
      trajectory_msgs::JointTrajectory traj;
      traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
      traj.joint_names.push_back( "head_pan_joint" );
      traj.joint_names.push_back( "head_tilt_joint" );
      traj.points.resize(1);

      traj.points[0].positions.push_back( reqHeadYaw_ + headYawRate_ * kHorizon );
      traj.points[0].velocities.push_back( headYawRate_ );
      traj.points[0].positions.push_back( reqHeadPitch_ + headPitchRate_ * kHorizon );
      traj.points[0].velocities.push_back( headPitchRate_ );
      traj.points[0].time_from_start = ros::Duration( kHorizon );
      
      // Updates the current positions
      reqHeadYaw_ += headYawRate_ * kDT;
      reqHeadPitch_ += headPitchRate_ * kDT;

      hPub_.publish( traj );
    }
  }
  else if (headYawRate_ != 0.0 || headPitchRate_ != 0.0) {
    if ((ros::Time::now() - cmdTimeStamp_) < ros::Duration( kMotionCommandGapTolerance * 1E6 )) {
      trajectory_msgs::JointTrajectory traj;
      traj.header.stamp = ros::Time::now() + ros::Duration(0.01);
      traj.joint_names.push_back( "head_pan_joint" );
      traj.joint_names.push_back( "head_tilt_joint" );
      traj.points.resize(1);
      traj.points[0].positions.push_back( reqHeadYaw_ + headYawRate_ * kHorizon );
      traj.points[0].velocities.push_back( headYawRate_ );
      traj.points[0].positions.push_back( reqHeadPitch_ + headPitchRate_ * kHorizon );
      traj.points[0].velocities.push_back( headPitchRate_ );
      traj.points[0].time_from_start = ros::Duration( kHorizon );

      hPub_.publish( traj );

      // Updates the current positions
      reqHeadYaw_ += headYawRate_ * kDT;
      reqHeadYaw_ = max(min(reqHeadYaw_, kMaxHeadPan), -kMaxHeadPan);
      reqHeadPitch_ += headPitchRate_ * kDT;
      reqHeadPitch_ = max(min(reqHeadPitch_, kMaxHeadTilt), kMinHeadTilt);      
    }
  }
}
  
bool PR2ProxyManager::isBodyControlWithOdometryTimeExpired()
{
  return (ros::Time::now() >= bcwoTimeToComplete_);
}
  
void PR2ProxyManager::getTFFrameList( std::vector<std::string> & list )
{
  list.clear();
  for (int i = 0; i < kPR2TFFrameListSize - 1 ; ++i) {
    list.push_back( kPR2TFFrameList[i] );
  }
}
  
bool PR2ProxyManager::isTFFrameSupported( const char * frame_name )
{
  if (!frame_name || strlen( frame_name ) > 40) { //rudimentary check
    return false;
  }
  int i = 0;
  while (kPR2TFFrameList[i]) {
    if (strncmp( frame_name, kPR2TFFrameList[i], strlen( kPR2TFFrameList[i]) ) == 0) {
      return true;
    }
    i++;
  }
  return false;
}
  
/**@}*/
} // namespace pyride
