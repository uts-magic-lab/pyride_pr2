import os
import sys
import PyPR2

MagiksPR2Path = 'Magiks/magiks/projects/s_pr2'

class IKSError( Exception ):
  pass

class IKSResolver( object ):
  def __init__( self ):
    self.spr2_obj = None
    self.iks_in_use = 0
    self.np = None
    self.geometry = None
    self.resolveIKS()
    self.useSPR2()

  def moveArmWithSPR2( self, **kwargs ):
    if not self.spr2_obj:
      return False

    if not kwargs.has_key( 'position' ) or not kwargs.has_key( 'orientation' ) or not kwargs.has_key( 'use_left_arm'):
      print 'Invalid input argument'
      return False

    self.spr2_obj.arm_speed = 0.1

    if kwargs['use_left_arm']:
      self.spr2_obj.larm_reference = True
    else:
      self.spr2_obj.larm_reference = False
 
    arm_orient = self.geometry.Orientation_3D( kwargs['orientation'], representation = 'quaternion' )
    self.spr2_obj.set_target( self.np.array(kwargs['position']), arm_orient.matrix() )
    return self.spr2_obj.arm_target()

  def dummyMoveArmTo( self, **kwargs ):
    raise IKSError( 'NO IKS solver is available to PyRIDE' )

  def resolveIKS( self ):
    PyPR2.moveArmTo = self.dummyMoveArmTo
    iksPath = os.path.join( sys.path[0], MagiksPR2Path )
    if os.path.exists( iksPath ):
      sys.path.append('/usr/local/lib/python2.7/dist-packages/')
      sys.path.append('/usr/lib/python2.7/dist-packages/')
      sys.path.append('/usr/lib/pymodules/python2.7')
      sys.path.append(iksPath)
      try:
        import initialize
        from magiks.specific_geometries.pr2 import pyride_synchronizer as pys
        import numpy as np
        from math_tools.geometry import geometry
        self.np = np
        self.geometry = geometry
        self.spr2_obj = pys.PyRide_PR2()
      except:
        print 'unable to load S-PR2/Magiks engine'
        self.spr2_obj = None

  def useSPR2( self ):
    if self.spr2_obj:
      PyPR2.moveArmTo = self.moveArmWithSPR2
      self.iks_in_use = 2
      print 'PyRIDE is using S-PR2 for PR2'
      
  def useMoveIt( self ):
    if PyPR2.useMoveIt():
      PyPR2.moveArmTo = PyPR2.moveArmPoseTo
      self.iks_in_use = 1
      print 'PyRIDE is using MoveIt! for PR2'

  def iksInUse( self ):
    if self.iks_in_use == 1:
      return 'MoveIt!'
    elif self.iks_in_use == 2:
      return 'S-PR2'
    else:
      return 'None'

