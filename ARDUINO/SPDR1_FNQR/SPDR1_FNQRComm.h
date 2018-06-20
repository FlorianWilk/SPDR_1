/*
 * File       Communication class for Freenove Quadruped Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/03/30
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#pragma once
#if defined(__AVR_ATmega2560__)

#include "SPDR1_FNQRBasic.h"
#include "SPDR1_FNQROrders.h"

#include <SPI.h>

class Communication
{
public:
  Communication();
  void Start();


  void UpdateCommunication();
  void UpdateOrder();

  RobotAction robotAction;

    void ActiveMode();
    void SleepMode();
    void SwitchMode();
    void CrawlForward();
    void CrawlBackward();
    void TurnLeft();
    void TurnRight();
    void MoveBody(float x, float y, float z);
    void RotateBody(float x, float y, float z, float angle);

    float GetSupplyVoltage();
private:
  volatile byte blockedOrder = 0;
  volatile byte dynamicOrder = 0;

  unsigned long lastBlockedOrderTime = 0;
  const unsigned long autoSleepOvertime = 10000;

  byte moveBodyParameters[3];
  byte rotateBodyParameters[2];

  void UpdateBlockedOrder();
  void UpdateDynamicOrder();
  void UpdateAutoSleep();

  const int stateLedPin = 13;
  bool stateLedState = LOW;

  void StartStateLed();
  void SetStateLed(bool state);
  void ReverseStateLed();


  const int pins[8] = { 20,21,A0,A1,15,14,2,3 };

  void StartPins();

  static const int inDataSize = 32;
  static const int outDataSize = 32;

  byte serialInData[inDataSize];
  byte serialInDataCounter = 0;
  void StartSerial();
  void UpdateSerial();



  enum OrderSource { FromSerial, FromRF24, FromESP8266, FromNone };
  OrderSource orderSource = OrderSource::FromNone;

  enum OrderState { ExecuteStart, ExecuteDone, ExecuteNone };
  OrderState orderState = OrderState::ExecuteNone;

  void HandleOrder(byte data[], OrderSource orderSource);

  void CheckBlockedOrder();

  void SaveRobotBootState(Robot::State state);
  Robot::State GetRobotBootState();
  void SetRobotBootState(Robot::State state);

  unsigned long ledCounter = 0;
  const int ledBlinkCycle = 20;
  int ledState = 0;
  void UpdateStateLED();
};

void UpdateService();

#endif
