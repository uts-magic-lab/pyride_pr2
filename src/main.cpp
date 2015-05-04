/*
 *  main.cpp
 *  PyPR2Server
 *
 *  Created by Xun Wang on 09/03/2012.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#include <ros/ros.h>
#include <signal.h>

#include "PyPR2Server.h"

using namespace pyride;
static PyPR2Server * s_server = NULL;

void stopProcess( int sig )
{
  if (s_server)
    s_server->stopProcess();
}

int main( int argc, char * argv[] )
{
  ros::init( argc, argv, "pyride_pr2" );

  s_server = new PyPR2Server();

  signal( SIGINT, ::stopProcess );
  
  s_server->init();

  s_server->continueProcessing();
  
  s_server->fini();
  
  delete s_server;

  ros::shutdown();
  return 0;
}
