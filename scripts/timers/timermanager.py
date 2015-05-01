import time

timerpool = {}

def addTimer( tid, client ):
  #global timerpool
  timerpool[tid] = client

def delTimer( tid ):
  if tid in timerpool:
    del timerpool[tid]

def onTimerLapsed( tid ):
  #global timerpool
  if tid in timerpool:
    timerpool[tid].onTimerLapsed( tid )
    del timerpool[tid]

def onTimerCall( tid ):
  #global timerpool
  if tid in timerpool:
    timerpool[tid].onTimer( tid )

def calcTimePeriodFromNow( target ):
  diff = -1
  t = None
  
  if type( target ) != time.struct_time:
    #target must be in "HH:MM" format
    try:
      t = time.strptime( target, '%H:%M' )
    except:
      print "target time is not in %H:%M format"
      return diff
  else:
    t = target

  now = time.localtime()

  if t.tm_year == 1900: #assume target time has only hour/minute component
    daydiff = 1
  else:
    daydiff = t.tm_yday - now.tm_yday

  diff = t.tm_hour - now.tm_hour

  if diff < 0 and daydiff > 0:
    diff = 24 * daydiff + diff

  diff = diff * 60 + t.tm_min - now.tm_min

  return diff * 60
