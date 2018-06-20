/*
 * File       Communication orders for Freenove Quadruped Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/03/30
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#pragma once

class Orders
{
  // Format:  [transStart] [order] [data 0] [data 1] ... [data n] [transEnd]
  //          [x] is byte type and the range of [order] and [data x] is 0~127
  // Process: The requesting party send the order, then the responding party respond the order.
  //          The non blocking order will be responded immediately, and the blocking order will
  //          be responded orderStart immediately, then respond orderDone after completion.

public:
  // Transmission control bytes, range is 128 ~ 255
  static const byte transStart = 128;
  static const byte transEnd = 129;


  // Orders, range is 0 ~ 127
  // Some orders have proprietary response orders, others use orderStart and orderDone
  // The even orders is sent by the requesting party, and the odd orders is sent by the 
  // responding party.


  // Non blocking orders, range 0 ~ 63

  // Request echo, to confirm the device
  static const byte requestEcho = 0;            // [order]
  // Respond echo
  static const byte echo = 1;                   // [order]

  // Request supply voltage
  static const byte requestSupplyVoltage = 10;  // [order]
  // Respond supply voltage
  static const byte supplyVoltage = 11;         // [order] [voltage * 100 / 128] [voltage * 100 % 128]

  //
  static const byte requestMoveLeg = 20;        // [order] [leg] [64 + dx] [64 + dy] [64 + dz]
  static const byte requestCalibrate = 22;      // [order]

  //
  static const byte requestChangeIO = 30;       // [order] [IOindex] [1/0]

  // Dynamic orders, range 40 ~ 63
  // The dynamic order will be responded immediately, although the order has just begun.
  static const byte requestMoveBodyTo = 40;     // [order] [64 + x] [64 + y] [64 + z]
  static const byte requestRotateBodyTo = 42;   // [order] [64 + x] [64 + y]

  // Universal responded orders
  static const byte orderStart = 21;            // [order]
  static const byte orderDone = 23;             // [order]


  // Blocking orders, range 64 ~ 127

  //
  static const byte requestCrawlForward = 64;   // [order]
  static const byte requestCrawlBackward = 66;  // [order]
  static const byte requestTurnLeft = 68;       // [order]
  static const byte requestTurnRight = 70;      // [order]
  static const byte requestActiveMode = 72;     // [order]
  static const byte requestSleepMode = 74;      // [order]
  static const byte requestSwitchMode = 76;     // [order]

  //
  static const byte requestInstallState = 80;   // [order]
  static const byte requestCalibrateState = 82; // [order]
  static const byte requestBootState = 84;      // [order]

  //
  static const byte requestCalibrateVerify = 90;// [order]

  //
  static const byte requestMoveBody = 100;      // [order] [64 + x] [64 + y] [64 + z]
  static const byte requestRotateBody = 102;    // [order] [64 + x] [64 + y]
};
