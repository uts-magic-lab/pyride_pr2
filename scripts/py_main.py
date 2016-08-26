import PyPR2
import math
import constants
import tinstate
import messenger
import tininfo
import tinmind
import extprocall
import iksresolver
import ipkspawner

from timers import timermanager

myMessenger = None
extProcCall = None
iksResolver = None
ipkSpawner = None
msgTryTimer = -1

def userLogon( name ):
  PyPR2.say( '%s has logged on.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, False )

def userLogoff( name ):
  PyPR2.say( '%s has logged off.' % name )
  tinstate.updateStatus( constants.USER_PRESENT, len(PyPR2.listCurrentUsers()) == 0)

def remoteCommandActions( cmd, arg ):
  if cmd == constants.LEARN_POSE:
    if arg == "Tuck Arms":
      PyPR2.tuckBothArms()

def nodeStatusUpdate( data ):
  global ipkSpawner
  #print data
  if data['node'] == 'jupyter':
    ipkSpawner.spawnkernel( data['message'] )
  #process messages from external ROS nodes here.
  #data['node'] data['priority'] data['message'] data['timestamp']

def timerLapsedActions( id ):
  timermanager.onTimerLapsed( id )

def timerActions( id ):
  global myMessenger, msgTryTimer

  if msgTryTimer == id and myMessenger.checkin():
    PyPR2.removeTimer( msgTryTimer )
    msgTryTimer = -1
  else:
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
    PyPR2.say( "I'm low on battery, please put me back on main power." )

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
  global ipkSpawner

  extProcCall = extprocall.ProcConduit()
  iksResolver = iksresolver.IKSResolver()
  ipkSpawner = ipkspawner.IPKSpawner()

  PyPR2.onUserLogOn = userLogon
  PyPR2.onUserLogOff = userLogoff
  PyPR2.onTimer = timerActions
  PyPR2.onTimerLapsed = timerLapsedActions
  PyPR2.onRemoteCommand = remoteCommandActions
  PyPR2.onSystemShutdown = systemShutdownActions
  PyPR2.onPowerPluggedChange = powerPlugChangeActions
  PyPR2.onBatteryChargeChange = batteryChargeChangeActions
  PyPR2.onNodeStatusUpdate = nodeStatusUpdate

  myMessenger = messenger.Messenger()
  if not myMessenger.checkin():
    msgTryTimer = PyPR2.addTimer( 10*60, -1, 10*60 )

  PyPR2.say( constants.INTRO_TEXT )
  PyPR2.setLowPowerThreshold( 20 )
