import os
import sys
import PyPR2
import time

MagiksPR2Path = 'Magiks/packages/nima/robotics/kinematics/special_geometries/pr2'

class IKSError( Exception ):
  pass

class IKSResolver( object ):
  def __init__( self ):
    self.spr2_obj = None
    self.use_move_it = False

  def moveArmWithSPR2( self, **kwargs ):
    if not self.spr2_obj:
      return False

    if not kwargs.has_key( 'position' ) or not kwargs.has_key( 'orientation' ) or not kwargs.has_key( 'use_left_arm'):
      print 'Invalid input argument'
      return False

    self.spr2_obj.arm_speed = 0.05

    if kwargs['use_left_arm']:
      self.spr2_obj.larm_reference = True
    else:
      self.spr2_obj.larm_reference = False
 
    arm_orient = self.geometry.Orientation_3D( kwargs['orientation'], representation = 'quaternion' )
    self.spr2_obj.set_target( self.np.array(kwargs['position']), arm_orient.matrix() )
    return self.spr2_obj.arm_target() #on the selected arm only

  def dummyMoveArmTo( self, **kwargs ):
    raise IKSError( 'NO IKS solver is available to PyRIDE' )

  def resolveIKS( self ):
    if PyPR2.useMoveIt():
      self.use_move_it = True
      PyPR2.moveArmTo = PyPR2.moveArmPoseTo
      print 'PyRIDE is using MoveIt! for PR2'
      return True
    else: 
      iksPath = os.path.join( sys.path[0], MagiksPR2Path )
      if os.path.exists( iksPath ):
        sys.path.append('/usr/local/lib/python2.7/dist-packages/')
        sys.path.append('/usr/lib/python2.7/dist-packages/')
        sys.path.append('/usr/lib/pymodules/python2.7')
        sys.path.append(iksPath)
        try:
          import __init__
          __init__.set_file_path( False )

          import pyride_synchronizer as pys
          import numpy as np
          from packages.nima.mathematics.geometry import geometry
          self.np = np
          self.geometry = geometry
          self.spr2_obj = pys.PyRide_PR2()
        except:
          print 'unable to load S-PR2/Magiks engine'
          PyPR2.moveArmTo = self.dummyMoveArmTo
          return False
      

        PyPR2.moveArmTo = self.moveArmWithSPR2
        return True 
    return False

  def iksInUse( self ):
    if self.use_move_it:
      return 'MoveIt'
    elif self.spr2_obj:
      return 'S-PR2'
    else:
      return 'None'

