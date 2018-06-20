/*
 * Sketch     Default function sketch for robot
 * Platform   Freenove Quadruped Robot (Arduino/Genuino Mega 2560)
 * Brief      This sketch is used to show default function of Freenove Quadruped Robot.
 *            You can control the robot by remote control or Processing applictin.
 *            Changing the code will make the default function not working properly.
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
  // Start Freenove Quadruped Robot with default function
  robot.Start(true);
}

void loop() {
  // Update Freenove Quadruped Robot
  robot.Update();
}

