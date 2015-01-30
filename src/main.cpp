/*
 *  main.cpp
 *  PyPR2Server
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <ros/ros.h>
#include "PyPR2Server.h"

using namespace pyride;

int main( int argc, char * argv[] )
{
  ros::init( argc, argv, "pyride_pr2" );

  PyPR2Server hcServer;
  
  hcServer.init();

  hcServer.continueProcessing();
  
  hcServer.fini();
  
  return 0;
}
