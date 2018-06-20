/*
 * File       Class for control Freenove Quadruped Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/03/30
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#if defined(__AVR_ATmega2560__)

#include "SPDR1_FNQR.h"

FNQR::FNQR() {}

void FNQR::Start(bool commFunction)
{
  communication.Start(commFunction);
}

void FNQR::Update()
{
  if (communication.commFunction)
    communication.UpdateOrder();
}

void FNQR::SetWiFi(String name, String password)
{
  communication.SetWiFi(name, password);
}

void FNQR::SetRC(byte byte0, byte byte1, byte byte2, byte byte3, byte byte4)
{
  communication.SetRC(byte0, byte1, byte2, byte3, byte4);
}

void FNQR::ActiveMode()
{
  if (!communication.commFunction)
    communication.robotAction.ActiveMode();
}

void FNQR::SleepMode()
{
  if (!communication.commFunction)
    communication.robotAction.SleepMode();
}

void FNQR::SwitchMode()
{
  if (!communication.commFunction)
    communication.robotAction.SwitchMode();
}

void FNQR::CrawlForward()
{
  if (!communication.commFunction)
    communication.robotAction.CrawlForward();
}

void FNQR::CrawlBackward()
{
  if (!communication.commFunction)
    communication.robotAction.CrawlBackward();
}

void FNQR::TurnLeft()
{
  if (!communication.commFunction)
    communication.robotAction.TurnLeft();
}

void FNQR::TurnRight()
{
  if (!communication.commFunction)
    communication.robotAction.TurnRight();
}

void FNQR::MoveBody(float x, float y, float z)
{
  if (!communication.commFunction)
    communication.robotAction.MoveBody(x, y, z);
}

void FNQR::RotateBody(float x, float y, float z, float angle)
{
  if (!communication.commFunction)
    communication.robotAction.RotateBody(x, y, z, angle);
}

#endif
