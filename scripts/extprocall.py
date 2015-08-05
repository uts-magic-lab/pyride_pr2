import os
import stat
import signal
import subprocess
import constants
import time
import PyPR2

RECORDED_DATA_DIR = '/removable/recordings'

class ProcConduit:
  def __init__( self ):
    self.mannequin = None
    self.recording = None
    self.joyControl = None

    if not os.path.exists( RECORDED_DATA_DIR ) or not os.path.isdir( RECORDED_DATA_DIR ):
      print 'Create data recording directory', RECORDED_DATA_DIR
      try:
        os.makedirs( RECORDED_DATA_DIR )
        os.chmod( RECORDED_DATA_DIR, stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IWGRP )
        os.chown( RECORDED_DATA_DIR, -1, 100 )
      except:
        print 'Unable to create data recording directory', RECORDED_DATA_DIR
    self.setCallbacks()

  def spawnProc( self, cmd ):
    # The os.setsid() is passed in the argument preexec_fn so
    # it's run after the fork() and before  exec() to run the shell.
    pro = subprocess.Popen( cmd, stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid )
    return pro

  def killProc( self, proc ):
    if not proc or not isinstance( proc, subprocess.Popen ):
      print 'Input is not a process object'
      return

    os.killpg( proc.pid, signal.SIGINT )  # Send the signal to all the process groups

  def setToMannequinMode( self, isYes ):
    if not isinstance( isYes, bool ):
      print 'Expect a boolean input'
      return
    elif isYes:
      if self.mannequin:
        print 'Already in mannequin mode'
      else:
        self.mannequin = self.spawnProc( 'roslaunch pr2_mannequin_mode pr2_mannequin_mode.launch > /dev/null 2>&1' )
        PyPR2.say( "Start mannequin mode." )
    else:
      if self.mannequin:
        self.killProc( self.mannequin )
        self.mannequin = None
        subprocess.call( 'rosservice call /pr2_controller_manager/switch_controller \
           "{start_controllers: [\'r_arm_controller\',\'l_arm_controller\'], \
           stop_controllers: [],  strictness: 2}"', shell=True )

        PyPR2.say( "Stop mannequin mode." )

  def setProjectorOff( self, isYes ):
    if not isinstance( isYes, bool ):
      print 'Expect a boolean input'
      return
    elif isYes:
      subprocess.call( 'rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode 1', shell=True )
    else:
      subprocess.call( 'rosrun dynamic_reconfigure dynparam set camera_synchronizer_node projector_mode 2', shell=True )

  def setBaseScanIntensityOn( self ):
    #PyPR2.say( "Turning on base scan intensity, please wait" )
    subprocess.call( 'rosrun dynamic_reconfigure dynparam set /base_hokuyo_node skip 2', shell=True )
    time.sleep( 0.5 )
    subprocess.call( 'rosrun dynamic_reconfigure dynparam set /base_hokuyo_node intensity True', shell=True )
    time.sleep( 0.5 )
    subprocess.call( 'rosrun dynamic_reconfigure dynparam set /base_hokuyo_node allow_unsafe_settings True', shell=True )
    time.sleep( 0.5 )
    subprocess.call( 'rosrun dynamic_reconfigure dynparam set /base_hokuyo_node max_ang 2.2689', shell=True )
    #PyPR2.say( "base scan intensity should be on" )
    
  def startDataRecording( self, mode, filename = "" ):
    cmd = 'rosbag record -b 1024 '
    str = ''
    if mode & constants.REC_CAM:
      #cmd = cmd + '-e "/(.*)_stereo/(left|right)/image_rect_color" '
      cmd = cmd + '-e "/wide_stereo/left/image_rect_color" ' # record only one camera data
      str = '_cam'
    if mode & constants.REC_KINECT:
      cmd = cmd + '"/camera/rgb/image_rect_color" "/camera/depth_registered/image_rect" '
      str = '_kinect'
    if mode & constants.REC_SCAN:
      cmd = cmd + '-e "/(.*)_scan$" '
      str = str + '_laser'
    if mode & constants.REC_IMU:
      cmd = cmd + '"/torso_lift_imu/data" '
      str = str + '_imu'
    if mode & constants.REC_JOINTS:
      cmd = cmd + '"/joint_states" '
      str = str + '_joint'
    if mode & constants.REC_TF:
      cmd = cmd + '"/tf" '
      str = str + '_tf'

    if filename == "":
      cmd = cmd + '--duration=1m --split -O %s/%s%s_data.bag' % \
        (RECORDED_DATA_DIR, time.strftime( "%Y%m%d_%H%M", time.localtime()), str)
    else:
      cmd = cmd + '--duration=1m --split -O %s/%s.bag' % \
        (RECORDED_DATA_DIR, filename)

    if self.recording:
      self.killProc( self.recording )

    self.recording = self.spawnProc( cmd )
    PyPR2.say( "Start data recording!" )

  def stopDataRecording( self ):
    if self.recording:
      self.killProc( self.recording )
      self.recording = None
      PyPR2.say( "Stopped data recording!" )

  def startJoystickControl( self ):
    if self.joyControl:
      print 'already in joystick control mode'
      return
  
    self.joyControl = self.spawnProc( 'roslaunch pr2_teleop teleop_joystick.launch > /dev/null 2>&1' )
    PyPR2.say( "Start joystick control." )

  def stopJoystickControl( self ):
    if self.joyControl:
      self.killProc( self.joyControl )
      self.joyControl = None
      PyPR2.say( "Stopped joystick control." )

  def setCallbacks( self ):
    PyPR2.setProjectorOff = self.setProjectorOff
    PyPR2.setToMannequinMode = self.setToMannequinMode
    PyPR2.startDataRecording = self.startDataRecording
    PyPR2.stopDataRecording = self.stopDataRecording
    PyPR2.startJoystickControl = self.startJoystickControl
    PyPR2.stopJoystickControl = self.stopJoystickControl
    PyPR2.turnOnBaseScanIntensity = self.setBaseScanIntensityOn
    
  def fini( self ):
    self.stopJoystickControl()
    self.stopDataRecording()
    self.setToMannequinMode( False )

