/*
 * File       Class for remote of Freenove Quadruped Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/03/30
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#include "SPDR1_FNQRRemote.h"

FNQRRemote::FNQRRemote() {}

void FNQRRemote::Start()
{
  StartPins();
  StartRF24();
}

void FNQRRemote::StartPins()
{
  pinMode(joystickZPin, INPUT);
  pinMode(s1Pin, INPUT);
  pinMode(s2Pin, INPUT);
  pinMode(s2Pin, INPUT);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(led3Pin, OUTPUT);
}

void FNQRRemote::StartRF24()
{
  rf24.begin();
  rf24.setPALevel(RF24_PA_LOW);
  rf24.setDataRate(RF24_1MBPS);
  rf24.setRetries(0, 15);
  rf24.openWritingPipe(rf24Address);
  rf24.openReadingPipe(1, rf24Address);
  rf24.stopListening();
}

void FNQRRemote::Update()
{
  int pot1Value = analogRead(pot1Pin);
  int pot2Value = analogRead(pot2Pin);
  int joystickXValue = analogRead(joystickXPin);
  int joystickYValue = analogRead(joystickYPin);
  bool joystickZValue = digitalRead(joystickZPin);
  bool s1Value = digitalRead(s1Pin);
  bool s2Value = digitalRead(s2Pin);
  bool s3Value = digitalRead(s3Pin);

  rf24OutDataCounter = 0;

  rf24OutData[rf24OutDataCounter++] = Orders::transStart;

  if (!s1Value)
  {
    if (joystickXValue < 512 - joystickDeadZone)
      rf24OutData[rf24OutDataCounter++] = Orders::requestTurnLeft;
    else if (joystickXValue > 512 + joystickDeadZone)
      rf24OutData[rf24OutDataCounter++] = Orders::requestTurnRight;
    else if (joystickYValue < 512 - joystickDeadZone)
      rf24OutData[rf24OutDataCounter++] = Orders::requestCrawlForward;
    else if (joystickYValue > 512 + joystickDeadZone)
      rf24OutData[rf24OutDataCounter++] = Orders::requestCrawlBackward;
    else if (!joystickZValue)
      rf24OutData[rf24OutDataCounter++] = Orders::requestSwitchMode;
    else
      rf24OutData[rf24OutDataCounter++] = Orders::requestEcho;
  }
  else if (!s2Value)
  {
    rf24OutData[rf24OutDataCounter++] = Orders::requestMoveBodyTo;
    rf24OutData[rf24OutDataCounter++] = map(joystickXValue, 0, 1024, 30, -30) + 64;
    rf24OutData[rf24OutDataCounter++] = map(joystickYValue, 0, 1024, 30, -30) + 64;
    rf24OutData[rf24OutDataCounter++] = map(pot2Value, 0, 1024, -15, 45) + 64;
  }
  else if (!s3Value)
  {
    rf24OutData[rf24OutDataCounter++] = Orders::requestRotateBodyTo;
    rf24OutData[rf24OutDataCounter++] = map(joystickYValue, 0, 1024, 10, -10) + 64;
    rf24OutData[rf24OutDataCounter++] = map(joystickXValue, 0, 1024, -10, 10) + 64;
  }
  else
    rf24OutData[rf24OutDataCounter++] = Orders::requestEcho;

  rf24OutData[rf24OutDataCounter++] = Orders::transEnd;

  if (rf24.write(rf24OutData, rf24OutDataCounter))
  {
    digitalWrite(led3Pin, HIGH);
    delay(rf24WriteInterval);
    digitalWrite(led3Pin, LOW);
  }
  else
  {
    delay(rf24WriteInterval);
  }

  analogWrite(led1Pin, map(pot1Value, 0, 1023, 0, 255));
  analogWrite(led2Pin, map(pot2Value, 0, 1023, 0, 255));
}

void FNQRRemote::SetRC(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4)
{
  rf24Address[0] = byte0;
  rf24Address[1] = byte1;
  rf24Address[2] = byte2;
  rf24Address[3] = byte3;
  rf24Address[4] = byte4;
}
