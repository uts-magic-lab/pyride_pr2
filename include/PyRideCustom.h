/*
 *  PyRideCustom.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 10/06/10.
 *  Copyright 2010 Galaxy Network. All rights reserved.
 *
 */

// This file contains list of custom command and status definitions
// that extends the current communication between NAOs and iPads.
#ifndef PyRideCustom_h_DEFINED
#define PyRideCustom_h_DEFINED

typedef enum {
  LEARN_BACKGROUND = 0x0,
  LEARN_MOVEMENT,
  LEARN_POSE,
  LEARN_OBJECT,
  LEARN_STRATEGY,
  LEARN_COMPLETE,
  REPLAY,
  CANCEL_LEARNED,
  FOLLOW_OBJECT,
  LOOK_AT,
  SPEAK,
  HEAD_MOVE_TO,
  BODY_MOVE_TO,
  SAVE_PATH,
  MOVE_IN_PATH,
  UPDATE_HEAD_POSE,
  UPDATE_BODY_POSE,
  EXCLUSIVE_CTRL_REQUEST,
  EXCLUSIVE_CTRL_RELEASE,
  EXCLUSIVE_CTRL_REJECT
} PyRideExtendedCommand;

static const int NonExclusiveExtendedCommands[] = { HEAD_MOVE_TO, SPEAK,
  LEARN_OBJECT };
static const int NonExcmdSize = sizeof( NonExclusiveExtendedCommands ) / sizeof( NonExclusiveExtendedCommands[0] );

typedef enum {
  IDLE               = 0x0,
  LEARNING           = 0x1,
  LEARNING_COMPLETE  = 0x2,
  FAULTY_HARDWARE    = 0x3,
  CUSTOM_STATE       = 0x4,
  EXCLUSIVE_CONTROL  = 0x5,
  NORMAL_CONTROL     = 0x6,
  EXCLUSIVE_CONTROL_OVERRIDE = 0x7
} RobotOperationalState;

#endif // PyRideCustom_h_DEFINED
