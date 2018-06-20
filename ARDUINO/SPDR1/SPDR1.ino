/*
   Sketch     Default function sketch for robot
   Platform   Freenove Quadruped Robot (Arduino/Genuino Mega 2560)
   Brief      This sketch is used to show default function of Freenove Quadruped Robot.
              You can control the robot by remote control or Processing applictin.
              Changing the code will make the default function not working properly.
   Author     Ethan Pan @ Freenove (support@freenove.com)
   Date       2018/03/30
  Copyright  Copyright © Freenove (http://www.freenove.com)
  License    Creative Commons Attribution ShareAlike 3.0
             (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
  -----------------------------------------------------------------------------------------------*/

#include <SPDR1_FNQRComm.h>

Communication robot;

void setup() {
  robot.Start();

  Serial3.begin(115200);
}

void loop() {
  if (Serial3.available()) {
    char c = Serial3.read();
    switch (c) {
      case 'a': robot.ActiveMode();
        break;
      case 's': robot.SleepMode(); Serial3.println("SleepMode"); break;
      case 'f': robot.CrawlBackward(); break;
      case 'b': robot.CrawlForward(); break;
      case 'l': robot.TurnRight(); break;
      case 'r': robot.TurnLeft(); break;
      case 'R': robot.RotateBody(Serial3.parseFloat()
                                   , Serial3.parseFloat(),
                                   Serial3.parseFloat(), Serial3.parseFloat()); break;
      case 'M': robot.MoveBody(Serial3.parseFloat(), Serial3.parseFloat(), Serial3.parseFloat()); break;

    }
  }
}

