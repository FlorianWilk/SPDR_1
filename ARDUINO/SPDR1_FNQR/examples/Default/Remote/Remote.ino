/*
 * Sketch     Default function sketch for remote
 * Platform   Freenove Smart Car Remote (Arduino/Genuino Uno)
 * Brief      This sketch is used to control Freenove Quadruped Robot by Remote.
 *            Turn on only one of S1, S2 or S3 to experience different modes of the robot.
 *            Changing the code will make the remote function not working properly.
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/03/30
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

// Include FNQR (Freenove Quadruped Robot) library
#include <FNQR.h>

FNQRRemote remote;

void setup() {
  // Start remote
  remote.Start();
}

void loop() {
  // Update remote
  remote.Update();
}

