#!/usr/bin/env python
from __future__ import print_function
import rospy
from pyride_common_msgs.msg import NodeStatus, NodeMessage
import sys
import json

keep_running = True
kernel_started = False

def notif( data ):
  global keep_running, kernel_started

  if data.node_id == 'jupyter':
    if data.command == 'stop':
      print( "stop ipytrigger" )
      keep_running = False
    elif data.command == 'start':
      kernel_started = True

if __name__ == '__main__':
  rospy.init_node( 'ipytrigger', anonymous = True )
  print( 'input arg {}'.format( sys.argv ) )
  pub = rospy.Publisher( '/pyride/node_status', NodeStatus, queue_size = 1 )
  rospy.Subscriber( '/pyride/node_message', NodeMessage, notif )
  msg = NodeStatus()
  msg.header.stamp = rospy.get_rostime()
  msg.node_id = 'jupyter'
  msg.for_console = False
  msg.priority = 100
  msg.status_text = sys.argv[2]
  print( "status_text = {}".format( msg.status_text ))
  rate = rospy.Rate( 1 )
  firemsg = True
  count = 2
  try:
    while not rospy.is_shutdown():
      rate.sleep()
      if firemsg: #send only once
        pub.publish( msg )
        print( 'send' )
        firemsg = False

      if not kernel_started:
        if count <= 0:
          rospy.signal_shutdown( "shutting down ipytrigger" )
          break
        else:
          count -= 1

      if not keep_running:
        rospy.signal_shutdown( "shutting down ipytrigger" )
        break
  except rospy.ROSInterruptException:
    pass
