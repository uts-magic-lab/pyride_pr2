import PyPR2
import math
import constants
import tinstate
import messenger
import tininfo
import tinmind
import extprocall
import iksresolver

from timers import timermanager

myMessenger = None
extProcCall = None
iksResolver = None
msgTryTimer = -1

def userLogon( name ):
  PyPR2.say( '%s has logged on.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, False )

def userLogoff( name ):
  PyPR2.say( '%s has logged off.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, len(PyPR2.listCurrentUsers()) == 0)

def remoteCommandActions( cmd, arg ):
  pass

def nodeStatusUpdate( node_id, priority, text ):
  #process messages from external ROS nodes here.
  pass

def timerLapsedActions( id ):
  global myMessenger, msgTryTimer

  if msgTryTimer == id and myMessenger.checkin():
    PyPR2.removeTimer( msgTryTimer )
    msgTryTimer = -1
  else:
    timermanager.onTimerCall( id )

def timerActions( id ):
  timermanager.onTimerCall( id )

def powerPlugChangeActions( isplugged ):
  global myMessenger
  
  text = ""
  if isplugged:
    text = "I'm on main power."
  else:
    text = "I'm on battery power."
  
  PyPR2.say( text )
  
  if myMessenger:
    myMessenger.updatestatus( text )

def batteryChargeChangeActions( batpc, isplugged, time_remain ):
  global myMessenger
  
  if batpc < 20 and not isplugged:
    PyNAO.say( "I'm low on battery, please put me back on main power." )
    
    if myMessenger:
      myMessenger.updatestatus( "I have only %d percent battery power left!" % batpc )

def systemShutdownActions():
  global myMessenger
  global extProcCall

  PyPR2.say( 'I am going off line. Goodbye.' )

  myMessenger.checkout()
  extProcCall.fini()

def main():
  global myMessenger, msgTryTimer
  global extProcCall, iksResolver
  
  extProcCall = extprocall.ProcConduit()
  iksResolver = iksresolver.IKSResolver()
  
  PyPR2.onUserLogOn = userLogon
  PyPR2.onUserLogOff = userLogoff
  PyPR2.onTimer = timerActions
  PyPR2.onTimerLapsed = timerLapsedActions
  PyPR2.onRemoteCommand = remoteCommandActions
  PyPR2.onSystemShutdown = systemShutdownActions
  PyPR2.onPowerPluggedChange = powerPlugChangeActions
  PyPR2.onBatteryChargeChange = batteryChargeChangeActions
  PyPR2.onNodeStatusUpdate = nodeStatusUpdate
  
  PyPR2.setProjectorOff = extProcCall.setProjectorOff
  PyPR2.setToMannequinMode = extProcCall.setToMannequinMode
  PyPR2.startDataRecording = extProcCall.startDataRecording
  PyPR2.stopDataRecording = extProcCall.stopDataRecording
  PyPR2.startJoystickControl = extProcCall.startJoystickControl
  PyPR2.stopJoystickControl = extProcCall.stopJoystickControl
  PyPR2.turnOnBaseScanIntensity = extProcCall.setBaseScanIntensityOn
   
  myMessenger = messenger.Messenger()
  if not myMessenger.checkin():
    msgTryTimer = PyPR2.addTimer( 10*60, -1, 10*60 )

  PyPR2.say( constants.INTRO_TEXT )
  PyPR2.setLowPowerThreshold( 20 )

