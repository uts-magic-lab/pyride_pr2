import PyPR2
import time
import constants
import tinstate
import tininfo
import tinmind
import os
import stat

from twitter import *
from timers import *

TWITTER_STATE_FILE = '/removable/recordings/laststate.txt'

class TextMessage:
  pass

class Messenger( timerclient.TimerClient ):
  def __init__( self ):
    super(Messenger, self ).__init__()
    self.lastoken = 0
    self.token = 0
    self.twtaccess = None
    self.messages = []
    self.archive = []
    self.timedmessages = []
    self.useraliases = {}
    self.startupTime = None
    self.appdisabled = (tininfo.TiNTwOAToken == None)

    if self.appdisabled:
      print '''PyRIDE PR2 twitter app is diabled. Create a Twitter account and application for
               your PR2 and enter the access tokens and secrets in the tininfo.py. Make sure
               your Twitter account application allow read and write access.'''
      return
    #load last twitter token
    f = None
    try:
      f = open( TWITTER_STATE_FILE, 'r')
      txtlines = f.readlines()
      for line in txtlines:
        token = line.split('=')
        if len( token ) == 2 and token[0].strip() == 'token':
          self.lastoken = int( token[1].strip() )
          f.close()
    except:
      pass

    f = None
    #load last twitter token
    try:
      f = open( 'useralias.txt', 'r')
      txtlines = f.readlines()
      for line in txtlines:
        token = line.split('=')
        if len( token ) == 2:
          self.useraliases[ token[0].strip() ] = token[1].strip()
    except:
      pass

  def checkin( self ):
    if self.appdisabled:
      return False

    try:
      self.twtaccess = Twitter(auth=OAuth(tininfo.TiNTwOAToken, tininfo.TiNTwOASecret, tininfo.TiNTwConKey, tininfo.TiNTwConSecret))
    except:
      print 'Unable to check into PyRIDE PR2 twitter app. Try again in 10mins.'
      return False

    self.startupTime = time.localtime()
    self.updatestatus( time.strftime( "I'm back online at %H:%M to serve ", time.localtime() ) + tininfo.TiNLocation + "."  )
  
    self.token = self.lastoken
    self.getmessages()
    tinstate.updateStatus( constants.NEW_MESSAGES, len( self.messages ) == 0 )
    tid = PyPR2.addTimer( 60, -1, 60 )
    self.timercontext[tid] = 'getmsg'
    timermanager.addTimer( tid, self )
    #add purge archive time
    purgetime = timermanager.calcTimePeriodFromNow( "4:00" )
    if purgetime > 0:
      tid = PyPR2.addTimer( purgetime, -1, 24*60*3600 )
      self.timercontext[tid] = 'purgemsg'
      timermanager.addTimer( tid, self )

    return True

  def checkout( self ):
    if self.twtaccess == None:
      return

    self.updatestatus( time.strftime( "I'm going offline at %H:%M for maintenance. See you later.", time.localtime() ) )

    self.savestate()
    self.stoptimers()

    self.twtaccess = None

  def onTimer( self, tid ):
    if tid not in self.timercontext:
      return
      
    if self.timercontext[tid] == 'getmsg':
      self.getmessages()
      tinstate.updateStatus( constants.NEW_MESSAGES, len( self.messages ) == 0 )
    elif self.timercontext[tid] == 'purgemsg':
      self.archive = []
      tinstate.updateStatus( constants.ARCHIVE_MESSAGES, True )
                            
  def getmessages( self ):
    if self.twtaccess == None:
      return

    try:
      if self.token == 0:
        newmessages = self.twtaccess.direct_messages()
      else:
        newmessages = self.twtaccess.direct_messages(since_id=self.token)
    except:
      print "Unable to retrieve messages"
      return

    if len( newmessages ) == 0:
      return

    self.lastoken = newmessages[-1]['id']
    self.token = newmessages[-1]['id']

    #store only the basic info
    modmsg = []
    for newmsg in newmessages:
      msg = TextMessage()
      msg.sender = newmsg['sender']['name']
      msg.senderid = newmsg['sender']['id']
      msg.created_at = time.localtime(time.mktime(time.strptime( newmsg['created_at'], "%a %b %d %H:%M:%S +0000 %Y")) - time.altzone)
      msg.text = newmsg['text'].strip()
      msg.id = newmsg['id']
      #if msg.text.startswith( '#now#' ):
        #msg.text = msg.text[5:]
        #PyPR2.say( "Urgent message announcement")
        #self.announcemsg( [msg] )
        #tinstate.updateStatus( constants.ARCHIVE_MESSAGES, len(self.archive) == 0 )
      if msg.text.startswith( '#qa#' ):
        if time.mktime(msg.created_at) >= time.mktime(self.startupTime): #ignore all direct requests before startup
          response = tinmind.respond( msg.text[4:] )
          if response:
            self.replymsg( msg.senderid, response )
        continue
      #modmsg.append( msg )
    
    #self.messages = modmsg + self.messages

  def replymsg( self, replyid, mesg ):
    if self.twtaccess == None:
      return

    try:
      tmo = self.twtaccess.direct_messages.new( user=replyid, text=mesg )
    except:
      print 'Unable to send message to', replyid

  def updatestatus( self, text ):
    if self.twtaccess == None:
      return

    try:
      self.twtaccess.statuses.update( status=text )
    except:
      print 'Unable to update my status'

  def announcemsg( self, mesgs ):
    if len( mesgs ) == 0:
      PyPR2.say( "There is no message to be announced" )
      return

    curTime = time.localtime()
    for i in mesgs:
      if i.created_at.tm_yday == curTime.tm_yday:
        sayTime = time.strftime( "Message received at %H hour %M minute", i.created_at )
      else:
        sayTime = time.strftime( "Message received at %H hour %M minute on %A %B %d", i.created_at )
      if i.sender in self.useraliases:
        frmWho = "From %s," % self.useraliases[ i.sender ]
      else:
        frmWho = "From %s," % i.sender
      PyPR2.say( "%s %s %s" % (sayTime, frmWho, i.text) )

    PyPR2.say( "Message announcement is complete." )

  def announce( self ):
    self.archive = self.messages + self.archive
    self.messages = []
    #self.announcemsg( self.archive )
    if len(self.archive) > 0:
      self.lastoken = self.archive[0].id
      self.savestate()

    tinstate.updateStatus( constants.NEW_MESSAGES, True )
    tinstate.updateStatus( constants.ARCHIVE_MESSAGES, len(self.archive) == 0 )

  def purgearchive( self ):
    if len(self.archive) == 0:
      PyPR2.say( "No archived message to be deleted." )
      return

    self.archive = []
    tinstate.updateStatus( constants.ARCHIVE_MESSAGES, True )
    PyPR2.say( "All archived message have been deleted." )

  def savestate( self ):
    f = None
    try:
      f = open( TWITTER_STATE_FILE, 'w')
      f.write( "token = %d\n" % self.lastoken)
      f.close()
      os.chmod( TWITTER_STATE_FILE, stat.S_IRUSR | stat.S_IWUSR | stat.S_IRGRP | stat.S_IWGRP )
      os.chown( TWITTER_STATE_FILE, -1, 100 )
    except:
      print "Unable to save last states."
      pass

  def stoptimers( self ):
    for tid in self.timercontext:
      PyPR2.removeTimer( tid )
      timermanager.delTimer( tid )

    self.timercontext = {}

