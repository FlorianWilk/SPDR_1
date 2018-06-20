
/*
   Sketch     based on the Default function sketch for robot
   Platform   Freenove Quadruped Robot (Arduino/Genuino Mega 2560)
   Author     Ethan Pan @ Freenove (support@freenove.com) - Thanks to Ethan for all the code!
   Date       2017/05/18
   Copyright  Copyright © Freenove (http://www.freenove.com)
   License    Creative Commons Attribution ShareAlike 3.0
              (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
   Modified by Florian Wilk
   -----------------------------------------------------------------------------------------------*/

#include <FNQR.h>

FNQR robot;
Robot r;
void setup() {

  // Serial. is our connected computer - the optional devloper console
  // Serial3. is our connected raspberry pi. we will send status commands/replys using
  //
  // * as prefix for global status messages like init-start/end
  // + for messages regarding servos and movement / positions
  // ! for exceptional warnings like low voltage
  
  Serial.begin(115200);
  
  // Fallback, since 115200 does not seem to work. 
  // TODO: try 38400
  Serial3.begin(9600);
  Serial.println("initializing controller");
  Serial3.println("*INIT_S");
  robot.Start();

  Serial3.println("*INIT_E");
  Serial.println("controller initialized");
  Serial3.print("*V:");
  Serial3.println( robot.communication.GetSupplyVoltage());
  
  // Do it for the Show
  delay(3000);
  robot.RotateBody(1, 0, 0, 45);
  delay(1000);
  robot.RotateBody(-1, 0, 1, -20);
  delay(1000);
  robot.RotateBody(1, 0, 1, 20);
  delay(1000);
  robot.MoveBody(0, 0, 45);
  delay(1000);

  // Stop throwing things from tables 
  //robot.CrawlBackward();
  //robot.CrawlBackward();
  
  robot.communication.robotAction.InitialState();
  robot.SleepMode();
  Serial3.println("*UP");
  robot.communication.robotAction.speedMoveBody = 3.45f;
  robot.communication.robotAction.speedRotateBody = 3.45f;
}

long lastm =  millis();

void loop() {
  
  // print out voltage every 10 secs
  if (millis() - lastm > 10000) {
    Serial3.print("*V:");
    float v=robot.communication.GetSupplyVoltage();
    Serial3.println(v);
    Serial.print("Controller voltage: ");
    Serial.println(v);
    lastm = millis();
  }
  
  // process serial input from our pi - simple commands for now
  if (Serial3.available()) {
    byte nr = Serial3.read();
    switch (nr) {
      case 'x':      robot.MoveBody(0, 0, 45);
        break;
      case 'X':      robot.MoveBody(0, 0, -45);
        break;
      case 'S': robot.SleepMode(); break;
      case 's': robot.ActiveMode(); break;
      case 'r': robot.RotateBody(1, 0, 0, 45);
        delay(200);
        robot.RotateBody(-1, 0, 1, -20);
        delay(200);
        robot.RotateBody(1, 0, 1, 20);
        delay(200);
        break;
      case 'l': robot.TurnLeft(); break;
      case 'L': robot.TurnRight(); break;
      case 'F': robot.CrawlBackward(); break;
      case 'f': robot.CrawlForward(); break;
      case '1':   robot.communication.robotAction.robot.InstallState(); break;
      case '2':   robot.communication.robotAction.robot.CalibrateState(); break;
      case '3':   robot.communication.robotAction.robot.BootState(); break;
      case '4':   robot.communication.robotAction.robot.CalibrateVerify(); break;


    }
  }
}
