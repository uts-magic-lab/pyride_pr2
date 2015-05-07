/*
 *  PR2ProxyManager.h
 *  PyPR2Server
 *
 *  Created by Xun Wang on 9/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef PR2_PROXY_MANAGER_H
#define PR2_PROXY_MANAGER_H

#include <string>
#include <ros/ros.h>

#include <sound_play/sound_play.h>

#include <actionlib/client/simple_action_client.h>

#include <pr2_msgs/PowerState.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <pr2_common_action_msgs/TuckArmsGoal.h>
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_controllers_msgs/PointHeadAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>

#include <move_base_msgs/MoveBaseAction.h>

#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud.h>

#include <std_msgs/String.h>
#include <std_msgs/Float64.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/Twist.h>

#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/message_filter.h>
#include <tf/transform_listener.h>


// moveit interface
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <shape_tools/solid_primitive_dims.h>

#ifdef WITH_PR2HT
#include <pr2ht/TrackedObjectStatusChange.h>
#include <pr2ht/TrackedObjectUpdate.h>
#endif

#ifdef WITH_RHYTH_DMP
#include <rhyth_dmp/OutputTrajData.h>
#endif

#include "PyRideCommon.h"

using namespace std;
using namespace ros;
using namespace pr2_controllers_msgs;
using namespace pr2_common_action_msgs;
using namespace move_base_msgs;

namespace pyride {

typedef actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> TorsoClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::Pr2GripperCommandAction> GripperClient;
typedef actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> TuckArmsActionClient;
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::JointTrajectoryAction> TrajectoryClient;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  
class PR2ProxyManager
{
public:
  static PR2ProxyManager * instance();
  ~PR2ProxyManager();

  void initWithNodeHandle( NodeHandle * nodeHandle, bool useOptionNodes = false, bool useMoveIt = false );
    
  void sayWithVolume( const std::string & text, float volume  = 0.0, bool toBlock = false );
  void setAudioVolume( const float vol );

  bool getPositionForJoints( std::vector<std::string> & joint_names,
                            std::vector<double> & positions );
  bool getJointPos( const char * joint_name, double & value );

  bool getHeadPos( double & yaw, double & pitch );
  bool getRobotPose( std::vector<double> & positions,
                    std::vector<double> & orientation );
  bool getRelativeTF( const char * frame1,
                         const char * frame2,
                         std::vector<double> & positions,
                         std::vector<double> & orientation );

  bool moveHeadTo( double yaw, double pitch, bool relative = false );
  bool pointHeadTo( const std::string & frame, float x, float y, float z );
  void updateHeadPose( float yaw, float pitch );
  
  bool tuckArms( bool tuckleft, bool tuckright );
  bool moveArmWithGoalPose( bool isLeftArm, std::vector<double> & position,
                           std::vector<double> & orientation, float time_to_reach = 10.0 );
  void moveArmWithJointPos( bool isLeftArm, std::vector<double> & positions,
                           float time_to_reach = 5.0 );
  void moveArmWithJointTrajectory( bool isLeftArm,
                                   std::vector< std::vector<double> > & trajectory,
                                   std::vector<float> & times_to_reach );
  void moveArmWithJointTrajectoryAndSpeed( bool isLeftArm,
                                  std::vector< std::vector<double> > & trajectory,
                                  std::vector< std::vector<double> > & joint_velocities,
                                  std::vector<float> & times_to_reach );

  void cancelArmMovement( bool isLeftArm );
  
  bool addSolidObject( const std::string & name, std::vector<double> & volume,
      std::vector<double> & position, std::vector<double> & orientation );

  void removeSolidObject( const std::string & name );

  void listSolidObjects( std::vector<std::string> & list );

  bool pickupObject( const std::string & name, const std::string & place, std::vector<double> & grasp_pose,
      bool isLeftArm = false, double approach_dist = 0.4 );

  bool placeObject( const std::string & name, const std::string & place, std::vector<double> & place_pose,
      bool isLeftArm = false, double approach_dist = 0.4 );

  bool setGripperPosition( int whichgripper, double position );
  bool setTiltLaserPeriodicCmd( double amp, double period, double offset = 0.0 );
  bool setTiltLaserTrajCmd( std::vector<double> & positions,
                           std::vector<Duration> & durations );

  bool moveBodyTo( const RobotPose & pose, const float bestTime );
  bool moveBodyTorsoBy( const float rel_pos, const float bestTime );

  void updateBodyPose( const RobotPose & pose, bool localupdate = false );
  
  bool navigateBodyTo( const std::vector<double> & positions,
                      const std::vector<double> & orientation );
  
#ifdef WITH_PR2HT
  bool enableHumanDetection( bool enable, bool enableTrackingNotif = false );
#endif
  
#ifdef WITH_RHYTH_DMP
  void subscribeRawTrajInput( bool enable );
#endif

  void cancelBodyMovement();

  void publishCommands();
  
  void getBatteryStatus( int & percentage, bool & isplugged, float & timeremain );

  int getLowPowerThreshold() { return lowPowerThreshold_; }
  void setLowPowerThreshold( int percent );
  
  void registerForBaseScanData();
  void registerForTiltScanData();
  void registerForBaseScanData( const std::string & target_frame );
  void registerForTiltScanData( const std::string & target_frame );
  void deregisterForBaseScanData();
  void deregisterForTiltScanData();
  
  void getTFFrameList( std::vector<std::string> & list );
  bool isTFFrameSupported( const char * frame_name );

  bool useMoveIt() const { return (rarmGroup_ != NULL && larmGroup_ != NULL); }
  void fini();

private:
  NodeHandle * mCtrlNode_;
  Publisher mPub_;
  Publisher torsoPub_;
  Publisher hPub_;
  Publisher colObjPub_;
  Subscriber jointSub_;
  Subscriber powerSub_;

  Subscriber * rawBaseScanSub_;
  Subscriber * rawTiltScanSub_;

#ifdef WITH_PR2HT
  Subscriber * htObjStatusSub_;
  Subscriber * htObjUpdateSub_;

  ServiceClient htClient_;
#endif

#ifdef WITH_RHYTH_DMP
  Subscriber * dmpTrajDataSub_;
  // use separate thread and message queue to cope with possible high resolution inputs
  AsyncSpinner * dmpTrajThread_;
  CallbackQueue dmpTrajQueue_;

  ServiceClient dmpClient_;
#endif

  message_filters::Subscriber<sensor_msgs::LaserScan> * baseScanSub_;
  message_filters::Subscriber<sensor_msgs::LaserScan> * tiltScanSub_;

  tf::MessageFilter<sensor_msgs::LaserScan> * baseScanNotifier_;
  tf::MessageFilter<sensor_msgs::LaserScan> * tiltScanNotifier_;
  
  sound_play::SoundClient soundClient_;
  
  laser_geometry::LaserProjection lprojector_;
  
  boost::mutex bat_mutex_;
  boost::mutex joint_mutex_;
  
  bool bodyCtrlWithOdmetry_;
  bool bodyCtrlWithNavigation_;
  bool torsoCtrl_;
  bool headCtrlWithOdmetry_;
  bool headCtrlWithActionClient_;
  bool tuckArmCtrl_;
  bool lGripperCtrl_;
  bool rGripperCtrl_;
  bool lArmCtrl_;
  bool rArmCtrl_;
  
  float lArmActionTimeout_;
  float rArmActionTimeout_;
  float bodyActionTimeout_;

  std::string baseScanTransformFrame_;
  std::string tiltScanTransformFrame_;
  
  std::vector<std::string> curJointNames_;
  std::vector<double> curJointPositions_;
  
  tf::TransformListener tflistener_;
  tf::StampedTransform startTransform_;
  
  TorsoClient * torsoClient_;
  PointHeadClient * phClient_;
  TuckArmsActionClient * tacClient_;
  GripperClient * lgripperClient_;
  GripperClient * rgripperClient_;
  
  moveit::planning_interface::MoveGroup * rarmGroup_;
  moveit::planning_interface::MoveGroup * larmGroup_;

  std::vector<std::string> solidObjectsInScene_;

  //moveit::planning_interface::PlanningSceneInterface planningSceneInf_;

  TrajectoryClient * mlacClient_;
  TrajectoryClient * mracClient_;
  
  MoveBaseClient * moveBaseClient_;
  
  RobotPose poseTrans_;

  ros::Time cmdTimeStamp_;
  ros::Time bcwoTimeToComplete_; // time to complete bodyCtrlWithOdmetry_
  ros::Time hcwoTimeToComplete_; // time to complete headCtrlWithOdmetry_

  geometry_msgs::Twist mCmd_;
  double headYawRate_, headPitchRate_;
  double reqHeadYaw_, reqHeadPitch_;
  double targetYaw_, targetPitch_;
  
  // power state
  bool isCharging_;
  int batCapacity_;
  int lowPowerThreshold_;
  Duration batTimeRemain_;

  static PR2ProxyManager * s_pPR2ProxyManager;

  PR2ProxyManager();
  double clamp( double val, double max );
  double max( double val1, double val2 );
  bool isBodyControlWithOdometryTimeExpired();
  bool isHeadControlWithOdometryTimeExpired();
  
  void doneHeadAction( const actionlib::SimpleClientGoalState & state,
                      const PointHeadResultConstPtr & result );
  void doneTorsoAction( const actionlib::SimpleClientGoalState & state,
                       const SingleJointPositionResultConstPtr & result );
  
  void doneTuckArmAction( const actionlib::SimpleClientGoalState & state,
                         const TuckArmsResultConstPtr & result );

  void doneMoveLArmAction( const actionlib::SimpleClientGoalState & state,
                          const JointTrajectoryResultConstPtr & result );
  void doneMoveRArmAction( const actionlib::SimpleClientGoalState & state,
                          const JointTrajectoryResultConstPtr & result );
  
  void doneLGripperAction( const actionlib::SimpleClientGoalState & state,
                          const Pr2GripperCommandResultConstPtr & result );
  void doneRGripperAction( const actionlib::SimpleClientGoalState & state,
                          const Pr2GripperCommandResultConstPtr & result );

  void doneNavgiateBodyAction( const actionlib::SimpleClientGoalState & state,
                              const MoveBaseResultConstPtr & result );
  
  void moveLArmActionFeedback( const JointTrajectoryFeedbackConstPtr & feedback );
  void moveRArmActionFeedback( const JointTrajectoryFeedbackConstPtr & feedback );

  void jointStateDataCB( const sensor_msgs::JointStateConstPtr & msg );
  void powerStateDataCB( const pr2_msgs::PowerStateConstPtr & msg );
  void baseScanDataCB( const sensor_msgs::LaserScanConstPtr & msg );
  void tiltScanDataCB( const sensor_msgs::LaserScanConstPtr & msg );

  bool findSolidObjectInScene( const std::string & name );

#ifdef WITH_PR2HT
  void htObjStatusCB( const pr2ht::TrackedObjectStatusChangeConstPtr & msg );
  void htObjUpdateCB( const pr2ht::TrackedObjectUpdateConstPtr & msg );
#endif

#ifdef WITH_RHYTH_DMP
  void trajectoryDataInputCB( const rhyth_dmp::OutputTrajDataConstPtr & msg );
#endif

};
}; // namespace pyride

#endif // PR2_PROXY_MANAGER_H
