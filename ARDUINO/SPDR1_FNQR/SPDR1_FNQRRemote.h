/*
 * File       Class for remote of Freenove Quadruped Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/03/30
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#pragma once

#include <Arduino.h>

#include "SPDR1_FNQROrders.h"

#include <SPI.h>
#include "RF24.h"

class FNQRRemote
{
public:
  FNQRRemote();

 /*
  * Brief     Start the remote
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void Start();

 /*
  * Brief     Update the wireless communication
  *           The loop() function should only call this function.
  * Param     None
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void Update();

 /*
  * Brief     Set wireless communication channel
  *           Call this function before Start()
  *           If don't call this function, will use default value.
  *           The robot should set the same channel to be able to controlled by this romote.
  * Param     bytex     bytes to define the channel
  * Retval    None
  * -----------------------------------------------------------------------------------------------*/
  void SetRC(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4);

private:
  const int pot1Pin = A0,
    pot2Pin = A1,
    joystickXPin = A2,
    joystickYPin = A3,
    joystickZPin = 7,
    s1Pin = 4,
    s2Pin = 3,
    s3Pin = 2,
    led1Pin = 6,
    led2Pin = 5,
    led3Pin = 8;

  const int joystickDeadZone = 100;

  void StartPins();

  const int rf24WriteInterval = 20;

  static const int outDataSize = 32;

  RF24 rf24 = RF24(9, 10);
  byte rf24Address[6] = { 'F', 'N', 'K', '2', '7' };
  byte rf24OutData[outDataSize];
  byte rf24OutDataCounter = 0;

  void StartRF24();
};
