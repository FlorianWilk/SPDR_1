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
#include "RF24.h"

#include "ESP8266.h"

class Communication
{
public:
  Communication();
  void Start(bool commFunction = true);

  void SetWiFi(String name, String password);
  void SetRC(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4);

  void UpdateCommunication();
  void UpdateOrder();

  RobotAction robotAction;
  bool commFunction;

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

  float GetSupplyVoltage();

  const int pins[8] = { 20,21,A0,A1,15,14,2,3 };

  void StartPins();

  static const int inDataSize = 32;
  static const int outDataSize = 32;

  byte serialInData[inDataSize];
  byte serialInDataCounter = 0;
  void StartSerial();
  void UpdateSerial();

  RF24 rf24 = RF24(9, 53);
  byte rf24Address[6] = { 'F', 'N', 'K', '2', '7' };
  bool isRF24Available = false;
  byte rf24InData[inDataSize];
  byte rf24InDataCounter = 0;

  void StartRF24();
  void UpdateRF24();

  String esp8266SSID = "Freenove Quadruped Robot";
  String esp8266PWD = "Freenove";
  const unsigned long esp8266Port = 65535;
  ESP8266 esp8266 = ESP8266(Serial2, 115200);
  bool isESP8266Available = false;
  byte esp8266ClientID;
  byte esp8266InData[inDataSize];
  byte esp8266InDataCounter = 0;

  void StartESP8266();
  void UpdateESP8266();

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
