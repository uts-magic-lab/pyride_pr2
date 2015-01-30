import PyPR2
import constants

status = constants.CLEAN_SLATE

def updateStatus( state, negate ):
  global status
  #TOD should check state first
  if negate:
    status = status &  ~state
  else:
    status = status | state
