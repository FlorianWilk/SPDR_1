/*
 * Sketch     Move body function example
 * Platform   Freenove Quadruped Robot (Arduino/Genuino Mega 2560)
 * Brief      This sketch is used to show how to control Freenove Quadruped Robot.
 *            You can easily achieve custom function by using FNQR library we provide.
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
  // Custom setup code start

  // Custom setup code end
  // Start Freenove Quadruped Robot
  robot.Start();
}

void loop() {
  // Custom loop code start
  robot.MoveBody(30, 0, 0);
  robot.MoveBody(0, 30, 0);
  robot.MoveBody(-30, 0, 0);
  robot.MoveBody(0, -30, 0);
  robot.MoveBody(0, 0, 0);
  robot.MoveBody(0, 0, -15);
  robot.MoveBody(0, 0, 45);
  robot.MoveBody(0, 0, 0);
  robot.SleepMode();
  
  while (true);
  // Custom loop code end
}

