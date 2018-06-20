/*
 * Sketch     Modify wireless communication channel for robot
 * Platform   Freenove Quadruped Robot (Arduino/Genuino Mega 2560)
 * Brief      This sketch is used to show how to modify wireless communication channel between robot 
 *            and remote when using default function.
 *            The remote control should set the same channel to control robot.
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/03/30
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

// Include FNQR (Freenove Quadruped Robot) library
#include <FNQR.h>

FNQR robot;

void setup() {
  // Set wireless communication channel
  // Call this function before robot.Start()
  // Set 5 byte type data
  robot.SetRC(1,2,3,4,5);
  // Start Freenove Quadruped Robot with default function
  robot.Start(true);
}

void loop() {
  // Update Freenove Quadruped Robot
  robot.Update();
}

