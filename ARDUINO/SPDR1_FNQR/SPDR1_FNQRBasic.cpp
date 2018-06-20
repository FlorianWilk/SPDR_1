/*
 * File       Basic classes for Freenove Quadruped Robot
 * Author     Ethan Pan @ Freenove (support@freenove.com)
 * Date       2018/03/30
 * Copyright  Copyright © Freenove (http://www.freenove.com)
 * License    Creative Commons Attribution ShareAlike 3.0
 *            (http://creativecommons.org/licenses/by-sa/3.0/legalcode)
 * -----------------------------------------------------------------------------------------------*/

#if defined(__AVR_ATmega2560__)

#include "SPDR1_FNQRBasic.h"

Power::Power() {}

void Power::Set(int adcReference, float samplingProportion, bool powerGroupAutoSwitch)
{
  this->adcReference = adcReference;
  analogReference(adcReference);

  this->samplingProportion = samplingProportion;
  this->powerGroupAutoSwitch = powerGroupAutoSwitch;

  for (int i = 0; i < samplingSize; i++)
    samplingData[i] = 0;

  pinMode(powerGroup1Pin, OUTPUT);
  pinMode(powerGroup2Pin, OUTPUT);
  pinMode(powerGroup3Pin, OUTPUT);
}

void Power::Update()
{
  Sampling();

  if (!powerGroupAutoSwitch)
    return;

  if (voltage > powerGroupOnVoltage)
    powerGroupState = true;
  else if (voltage < powerGroupOffVoltage)
    powerGroupState = false;

  if (++updateCounter < powerGroupBootInterval)
    return;
  updateCounter = 0;

  if (powerGroupState)
  {
    if (!powerGroup1State)
    {
      SetPowerGroupState(1, true);
      return;
    }
    else if (!powerGroup2State)
    {
      SetPowerGroupState(2, true);
      return;
    }
    else if (!powerGroup3State)
    {
      SetPowerGroupState(3, true);
      return;
    }
  }
  else
  {
    if (powerGroup1State)
    {
      SetPowerGroupState(1, false);
      return;
    }
    else if (powerGroup2State)
    {
      SetPowerGroupState(2, false);
      return;
    }
    else if (powerGroup3State)
    {
      SetPowerGroupState(3, false);
      return;
    }
  }
}

void Power::Sampling()
{
  float voltage;

  switch (adcReference)
  {
  case EXTERNAL:
    voltage = analogRead(samplingPin) * (2.5 * (32 / (32 + 6.2))) / 1023 / samplingProportion;
    break;
  default:
    voltage = analogRead(samplingPin) * 5.0 / 1023 / samplingProportion;
    break;
  }

  samplingData[samplingDataCounter++] = voltage;
  if (samplingDataCounter == samplingSize)
    samplingDataCounter = 0;

  float peakVoltage = 0;
  for (int i = 0; i < samplingSize; i++)
    if (peakVoltage < samplingData[i])
      peakVoltage = samplingData[i];

  samplingPeakData[samplingPeakDataCounter++] = peakVoltage;
  if (samplingPeakDataCounter == samplingPeakSize)
    samplingPeakDataCounter = 0;

  float averagePeakVoltage = 0;
  for (int i = 0; i < samplingPeakSize; i++)
    averagePeakVoltage += samplingPeakData[i];
  averagePeakVoltage /= samplingPeakSize;

  this->voltage = averagePeakVoltage;
}

void Power::SetPowerGroupState(int group, bool state)
{
  switch (group)
  {
  case 1:
    digitalWrite(powerGroup1Pin, state);
    powerGroup1State = state;
    break;
  case 2:
    digitalWrite(powerGroup2Pin, state);
    powerGroup2State = state;
    break;
  case 3:
    digitalWrite(powerGroup3Pin, state);
    powerGroup3State = state;
    break;
  }
}

Point::Point() {}

Point::Point(float x, float y, float z)
{
  this->x = x;
  this->y = y;
  this->z = z;
}

float Point::GetDistance(Point point1, Point point2)
{
  return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2) + pow(point1.z - point2.z, 2));
}

RobotLegsPoints::RobotLegsPoints() {}

RobotLegsPoints::RobotLegsPoints(Point leg1, Point leg2, Point leg3, Point leg4)
{
  this->leg1 = leg1;
  this->leg2 = leg2;
  this->leg3 = leg3;
  this->leg4 = leg4;
}

RobotJoint::RobotJoint() {}

int RobotJoint::firstRotateDelay = 0;

void RobotJoint::Set(int servoPin, float jointZero, bool jointDir, float jointMinAngle, float jointMaxAngle, int offsetAddress)
{
  this->servoPin = servoPin;
  this->jointZero = jointZero;
  this->jointDir = jointDir;
  this->offsetAddress = offsetAddress;
  this->jointMinAngle = jointMinAngle;
  this->jointMaxAngle = jointMaxAngle;

  int offsetInt = EEPROM.read(offsetAddress) * 256 + EEPROM.read(offsetAddress + 1);
  offsetInt = offsetInt / 2 * ((offsetInt % 2) ? 1 : -1);
  float offset = offsetInt * 0.01;
  this->offset = offset;
}

void RobotJoint::SetOffset(float offset)
{
  int offsetInt = offset * 100;
  offsetInt = abs(offsetInt) * 2 + ((offset > 0) ? 1 : 0);

  if (offsetInt < 0 || offsetInt > 65535)
    return;

  EEPROM.write(offsetAddress, offsetInt / 256);
  EEPROM.write(offsetAddress + 1, offsetInt % 256);
  this->offset = offset;
}

void RobotJoint::SetOffsetEnableState(bool state)
{
  isOffsetEnable = state;
}

void RobotJoint::RotateToDirectly(float jointAngle)
{
  float servoAngle;

  if (isOffsetEnable)
    servoAngle = jointZero + (jointDir ? 1 : -1) * (jointAngle + offset);
  else
    servoAngle = jointZero + (jointDir ? 1 : -1) * jointAngle;

  while (servoAngle > 360)
    servoAngle -= 360;
  while (servoAngle < 0)
    servoAngle += 360;
  if (servoAngle > 180)
    return;

  if (isFirstRotate)
  {
    isFirstRotate = false;
    servo.attach(servoPin);
    servo.write(servoAngle);
    delay(firstRotateDelay);
  }
  else
  {
    servo.write(servoAngle);
  }

  jointAngleNow = jointAngle;
  servoAngleNow = servoAngle;
}

float RobotJoint::GetJointAngle(float servoAngle)
{
  return (jointDir ? 1 : -1) * (servoAngle - jointZero);
}

bool RobotJoint::CheckJointAngle(float jointAngle)
{
  while (jointAngle > 360)
    jointAngle -= 360;
  while (jointAngle < 0)
    jointAngle += 360;

  if (jointAngle >= jointMinAngle && jointAngle <= jointMaxAngle)
    return true;
  else
    return false;
}

RobotLeg::RobotLeg() {}

void RobotLeg::Set(float xOrigin, float yOrigin)
{
  this->xOrigin = xOrigin;
  this->yOrigin = yOrigin;
}

void RobotLeg::SetOffsetEnableState(bool state)
{
  jointA.SetOffsetEnableState(state);
  jointB.SetOffsetEnableState(state);
  jointC.SetOffsetEnableState(state);
}

void RobotLeg::CalculatePoint(float alpha, float beta, float gamma, volatile float &x, volatile float &y, volatile float &z)
{
  // transform angle to radian
  alpha = alpha * PI / 180;
  beta = beta * PI / 180;
  gamma = gamma * PI / 180;
  // calculate u-v coordinate
  float u, v;
  u = RobotShape::d + RobotShape::e * sin(beta) + RobotShape::f * sin(gamma - beta);
  v = RobotShape::c + RobotShape::e * cos(beta) - RobotShape::f * cos(gamma - beta);
  // calculate x-y-z coordinate
  x = xOrigin + u * cos(alpha);
  y = yOrigin + u * sin(alpha);
  z = v;
}

void RobotLeg::CalculatePoint(float alpha, float beta, float gamma, Point &point)
{
  CalculatePoint(alpha, beta, gamma, point.x, point.y, point.z);
}

void RobotLeg::CalculateAngle(float x, float y, float z, float &alpha, float &beta, float &gamma)
{
  // calculate u-v angle
  float u, v;
  u = sqrt(pow(x - xOrigin, 2) + pow(y - yOrigin, 2));
  v = z;
  beta = PI / 2 - acos((pow(RobotShape::e, 2) + (pow(u - RobotShape::d, 2) + pow(v - RobotShape::c, 2)) - pow(RobotShape::f, 2)) / (2 * RobotShape::e * sqrt(pow(u - RobotShape::d, 2) + pow(v - RobotShape::c, 2)))) - atan2(v - RobotShape::c, u - RobotShape::d);
  gamma = acos((pow(RobotShape::e, 2) + pow(RobotShape::f, 2) - (pow(u - RobotShape::d, 2) + pow(v - RobotShape::c, 2))) / (2 * RobotShape::e * RobotShape::f));
  // calculate x-y-z angle
  alpha = atan2(y - yOrigin, x - xOrigin);
  if (xOrigin < 0 && yOrigin < 0)
    alpha = alpha + PI;
  if (xOrigin < 0 && yOrigin < 0)
    alpha = alpha + PI;
  // transform radian to angle
  alpha = alpha * 180 / PI;
  beta = beta * 180 / PI;
  gamma = gamma * 180 / PI;
}

void RobotLeg::CalculateAngle(Point point, float &alpha, float &beta, float &gamma)
{
  CalculateAngle(point.x, point.y, point.z, alpha, beta, gamma);
}

bool RobotLeg::CheckPoint(Point point)
{
  float alpha, beta, gamma;
  CalculateAngle(point, alpha, beta, gamma);
  if (CheckAngle(alpha, beta, gamma))
  {
    Point pointNew;
    CalculatePoint(alpha, beta, gamma, pointNew);
    if (Point::GetDistance(point, pointNew) < negligibleDistance)
      return true;
  }
  return false;
}

bool RobotLeg::CheckAngle(float alpha, float beta, float gamma)
{
  if (jointA.CheckJointAngle(alpha) && jointB.CheckJointAngle(beta) && jointC.CheckJointAngle(gamma))
    return true;
  else
    return false;
}

void RobotLeg::MoveTo(Point point)
{
  if (!CheckPoint(point))
    return;

  pointGoal = point;
  isBusy = true;
}

void RobotLeg::MoveToRelatively(Point point)
{
  point = Point(pointGoal.x + point.x, pointGoal.y + point.y, pointGoal.z + point.z);
  MoveTo(point);
}

void RobotLeg::WaitUntilFree()
{
  while (isBusy);
}

void RobotLeg::ServosRotateTo(float degreeA, float degreeB, float degreeC)
{
  float alpha = jointA.GetJointAngle(degreeA);
  float beta = jointB.GetJointAngle(degreeB);
  float gamma = jointC.GetJointAngle(degreeC);

  Point point;
  CalculatePoint(alpha, beta, gamma, point);

  MoveTo(point);
}

void RobotLeg::MoveToDirectly(Point point)
{
  if (!CheckPoint(point))
    return;

  float alpha, beta, gamma;
  CalculateAngle(point, alpha, beta, gamma);
  RotateToDirectly(alpha, beta, gamma);
}

void RobotLeg::MoveToDirectlyRelatively(Point point)
{
  point = Point(pointGoal.x + point.x, pointGoal.y + point.y, pointGoal.z + point.z);
  MoveToDirectly(point);
}

void RobotLeg::RotateToDirectly(float alpha, float beta, float gamma)
{
  jointC.RotateToDirectly(gamma);
  jointB.RotateToDirectly(beta);
  jointA.RotateToDirectly(alpha);

  Point point;
  CalculatePoint(alpha, beta, gamma, point);

  if (isFirstMove)
  {
    isFirstMove = false;
    pointGoal = point;
  }

  pointNow = point;
}

Robot::Robot() {}

void Robot::Start()
{
  controllerVersion = EEPROM.read(EepromAddresses::controllerVersion);

  switch (controllerVersion)
  {
  case 20:
    RobotJoint::firstRotateDelay = 0;
    power.Set(EXTERNAL, 2.0 / (2.0 + 6.2), true);
    break;
  default:
    RobotJoint::firstRotateDelay = 50;
    power.Set(DEFAULT, 10.0 / (10.0 + 10.0), false);
    break;
  }

  leg1.Set(-RobotShape::a, RobotShape::a);
  leg2.Set(-RobotShape::a, -RobotShape::a);
  leg3.Set(RobotShape::a, RobotShape::a);
  leg4.Set(RobotShape::a, -RobotShape::a);

  leg1.jointA.Set(22, -45, true, 90, 180, EepromAddresses::servo22);
  leg1.jointB.Set(23, 0, true, 0, 180, EepromAddresses::servo23);
  leg1.jointC.Set(24, 0, true, 0, 180, EepromAddresses::servo24);
  leg2.jointA.Set(25, -135, true, 180, 270, EepromAddresses::servo25);
  leg2.jointB.Set(26, 180, false, 0, 180, EepromAddresses::servo26);
  leg2.jointC.Set(27, 180, false, 0, 180, EepromAddresses::servo27);
  leg3.jointA.Set(39, 45, true, 0, 90, EepromAddresses::servo39);
  leg3.jointB.Set(38, 180, false, 0, 180, EepromAddresses::servo38);
  leg3.jointC.Set(37, 180, false, 0, 180, EepromAddresses::servo37);
  leg4.jointA.Set(36, 135, true, 270, 360, EepromAddresses::servo36);
  leg4.jointB.Set(35, 0, true, 0, 180, EepromAddresses::servo35);
  leg4.jointC.Set(34, 0, true, 0, 180, EepromAddresses::servo34);

  MoveToDirectly(bootPoints);
}

void Robot::InstallState()
{
  state = State::Install;
  SetOffsetEnableState(false);
  SetSpeed(RobotLeg::defaultStepDistance);
  leg1.ServosRotateTo(90, 90, 90);
  leg2.ServosRotateTo(90, 90, 90);
  leg3.ServosRotateTo(90, 90, 90);
  leg4.ServosRotateTo(90, 90, 90);
  WaitUntilFree();
}

void Robot::CalibrateState()
{
  state = State::Calibrate;
  SetOffsetEnableState(false);
  SetSpeed(RobotLeg::defaultStepDistance);
  MoveTo(calibrateStatePoints);
  WaitUntilFree();
}

void Robot::CalibrateServos()
{
  if (state != State::Calibrate)
    return;
  CalibrateLeg(leg1, calibratePoints.leg1);
  CalibrateLeg(leg2, calibratePoints.leg2);
  CalibrateLeg(leg3, calibratePoints.leg3);
  CalibrateLeg(leg4, calibratePoints.leg4);
  SetOffsetEnableState(true);
}

void Robot::CalibrateVerify()
{
  state = State::Calibrate;
  SetSpeed(RobotLeg::defaultStepDistance);
  MoveTo(calibrateStatePoints);
  WaitUntilFree();
  SetOffsetEnableState(true);
  MoveTo(calibratePoints);
  WaitUntilFree();
}

void Robot::BootState()
{
  SetOffsetEnableState(true);
  SetSpeed(RobotLeg::defaultStepDistance);
  MoveTo(bootPoints);
  WaitUntilFree();
  state = State::Boot;
}

void Robot::MoveTo(RobotLegsPoints points)
{
  leg1.MoveTo(points.leg1);
  leg2.MoveTo(points.leg2);
  leg3.MoveTo(points.leg3);
  leg4.MoveTo(points.leg4);
}

void Robot::MoveTo(RobotLegsPoints points, float speed)
{
  SetSpeed(speed);
  MoveTo(points);
}

void Robot::MoveToRelatively(Point point)
{
  leg1.MoveToRelatively(point);
  leg2.MoveToRelatively(point);
  leg3.MoveToRelatively(point);
  leg4.MoveToRelatively(point);
}

void Robot::MoveToRelatively(Point point, float speed)
{
  SetSpeed(speed);
  MoveToRelatively(point);
}

void Robot::WaitUntilFree()
{
  while (leg1.isBusy || leg2.isBusy || leg3.isBusy || leg4.isBusy);
}

void Robot::SetSpeed(float speed)
{
  leg1.stepDistance = speed;
  leg2.stepDistance = speed;
  leg3.stepDistance = speed;
  leg4.stepDistance = speed;
}

void Robot::SetSpeed(float speed1, float speed2, float speed3, float speed4)
{
  leg1.stepDistance = speed1;
  leg2.stepDistance = speed2;
  leg3.stepDistance = speed3;
  leg4.stepDistance = speed4;
}

void Robot::GetPointsNow(RobotLegsPoints & points)
{
  points.leg1 = leg1.pointNow;
  points.leg2 = leg2.pointNow;
  points.leg3 = leg3.pointNow;
  points.leg4 = leg4.pointNow;
}

void Robot::Update()
{
  UpdateAction();
  power.Update();
}

void Robot::CalibrateLeg(RobotLeg &leg, Point calibratePoint)
{
  float alpha, beta, gamma;
  leg.CalculateAngle(calibratePoint, alpha, beta, gamma);
  leg.jointA.SetOffset(leg.jointA.jointAngleNow - alpha);
  leg.jointB.SetOffset(leg.jointB.jointAngleNow - beta);
  leg.jointC.SetOffset(leg.jointC.jointAngleNow - gamma);
}

void Robot::UpdateAction()
{
  UpdateLegAction(leg1);
  UpdateLegAction(leg2);
  UpdateLegAction(leg3);
  UpdateLegAction(leg4);
}

void Robot::UpdateLegAction(RobotLeg &leg)
{
  float distance = Point::GetDistance(leg.pointNow, leg.pointGoal);
  float xDistance = leg.pointGoal.x - leg.pointNow.x;
  float yDistance = leg.pointGoal.y - leg.pointNow.y;
  float zDistance = leg.pointGoal.z - leg.pointNow.z;
  float xStep = xDistance / distance * leg.stepDistance;
  float yStep = yDistance / distance * leg.stepDistance;
  float zStep = zDistance / distance * leg.stepDistance;
  Point pointGoal = Point(leg.pointNow.x + xStep, leg.pointNow.y + yStep, leg.pointNow.z + zStep);

  if (distance >= leg.stepDistance)
  {
    leg.isBusy = true;
    leg.MoveToDirectly(pointGoal);
  }
  else if (distance >= RobotLeg::negligibleDistance)
  {
    leg.isBusy = true;
    leg.MoveToDirectly(leg.pointGoal);
  }
  else
  {
    leg.isBusy = false;
  }
}

void Robot::MoveToDirectly(RobotLegsPoints points)
{
  leg1.MoveToDirectly(points.leg1);
  leg2.MoveToDirectly(points.leg2);
  leg3.MoveToDirectly(points.leg3);
  leg4.MoveToDirectly(points.leg4);
}

void Robot::SetOffsetEnableState(bool state)
{
  leg1.SetOffsetEnableState(state);
  leg2.SetOffsetEnableState(state);
  leg3.SetOffsetEnableState(state);
  leg4.SetOffsetEnableState(state);
}

RobotAction::RobotAction() {}

void RobotAction::Start()
{
  robot.Start();
}

void RobotAction::ActiveMode()
{
  ActionState();
  if (legsState == LegsState::Move || legsState == LegsState::Rotate)
    InitialState();
  if (mode == Mode::Active)
    return;

  LegsMoveToRelatively(Point(0, 0, -bodyLift), crawlSpeedBody);
  mode = Mode::Active;
}

void RobotAction::SleepMode()
{
  ActionState();
  if (legsState == LegsState::Move || legsState == LegsState::Rotate)
    InitialState();
  if (mode == Mode::Sleep)
    return;

  LegsMoveToRelatively(Point(0, 0, bodyLift), crawlSpeedBody);
  mode = Mode::Sleep;
}

void RobotAction::SwitchMode()
{
  ActionState();
  if (mode == Mode::Active)
    SleepMode();
  else
    ActiveMode();
}

void RobotAction::CrawlForward()
{
  ActionState();
  if (legsState == LegsState::Move || legsState == LegsState::Rotate)
    InitialState();
  if (mode != Mode::Active)
    ActiveMode();

  if (legsState == LegsState::Initial)
  {
    FirstStepForward();
  }
  else if (legsState == LegsState::Feet34Long)
  {
    //
    LegsMoveToRelatively(Point(0, -0.5 * crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg1.stepDistance = crawlSpeedLeg;

    robot.leg1.MoveToRelatively(Point(0, 0, legLift));
    robot.leg2.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg4.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg1.WaitUntilFree();

    robot.leg1.MoveToRelatively(Point(0, 7 * crawlOffsetY, 0));
    robot.leg1.WaitUntilFree();

    robot.leg1.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, -crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg4.stepDistance = crawlSpeedLeg;

    robot.leg1.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg4.MoveToRelatively(Point(0, 0, legLift));
    robot.leg4.WaitUntilFree();

    robot.leg4.MoveToRelatively(Point(0, 7 * crawlOffsetY, 0));
    robot.leg4.WaitUntilFree();

    robot.leg4.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, -0.5 * crawlOffsetY, 0), crawlSpeedBody);
    ////
    legsState = LegsState::Feet12Long;
  }
  else
  {
    //
    LegsMoveToRelatively(Point(0, -0.5 * crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg3.stepDistance = crawlSpeedLeg;

    robot.leg1.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, 0, legLift));
    robot.leg4.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg3.WaitUntilFree();

    robot.leg3.MoveToRelatively(Point(0, 7 * crawlOffsetY, 0));
    robot.leg3.WaitUntilFree();

    robot.leg3.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, -crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg2.stepDistance = crawlSpeedLeg;

    robot.leg1.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, 0, legLift));
    robot.leg3.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg4.MoveToRelatively(Point(0, -crawlOffsetY, 0));
    robot.leg2.WaitUntilFree();

    robot.leg2.MoveToRelatively(Point(0, 7 * crawlOffsetY, 0));
    robot.leg2.WaitUntilFree();

    robot.leg2.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, -0.5 * crawlOffsetY, 0), crawlSpeedBody);
    ////
    legsState = LegsState::Feet34Long;
  }
}

void RobotAction::CrawlBackward()
{
  ActionState();
  if (legsState == LegsState::Move || legsState == LegsState::Rotate)
    InitialState();
  if (mode != Mode::Active)
    ActiveMode();

  if (legsState == LegsState::Initial)
  {
    FirstStepBackward();
  }
  else if (legsState == LegsState::Feet12Long)
  {
    //
    LegsMoveToRelatively(Point(0, 0.5 * crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg4.stepDistance = crawlSpeedLeg;

    robot.leg1.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg4.MoveToRelatively(Point(0, 0, legLift));
    robot.leg4.WaitUntilFree();

    robot.leg4.MoveToRelatively(Point(0, -7 * crawlOffsetY, 0));
    robot.leg4.WaitUntilFree();

    robot.leg4.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg1.stepDistance = crawlSpeedLeg;

    robot.leg1.MoveToRelatively(Point(0, 0, legLift));
    robot.leg2.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg4.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg1.WaitUntilFree();

    robot.leg1.MoveToRelatively(Point(0, -7 * crawlOffsetY, 0));
    robot.leg1.WaitUntilFree();

    robot.leg1.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, 0.5 * crawlOffsetY, 0), crawlSpeedBody);
    ////
    legsState = LegsState::Feet34Long;
  }
  else
  {
    //
    LegsMoveToRelatively(Point(0, 0.5 * crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg2.stepDistance = crawlSpeedLeg;

    robot.leg1.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, 0, legLift));
    robot.leg3.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg4.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg2.WaitUntilFree();

    robot.leg2.MoveToRelatively(Point(0, -7 * crawlOffsetY, 0));
    robot.leg2.WaitUntilFree();

    robot.leg2.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, crawlOffsetY, 0), crawlSpeedBody);
    //
    robot.leg3.stepDistance = crawlSpeedLeg;

    robot.leg1.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg2.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg3.MoveToRelatively(Point(0, 0, legLift));
    robot.leg4.MoveToRelatively(Point(0, crawlOffsetY, 0));
    robot.leg3.WaitUntilFree();

    robot.leg3.MoveToRelatively(Point(0, -7 * crawlOffsetY, 0));
    robot.leg3.WaitUntilFree();

    robot.leg3.MoveToRelatively(Point(0, 0, -legLift));
    robot.WaitUntilFree();
    //
    LegsMoveToRelatively(Point(0, 0.5 * crawlOffsetY, 0), crawlSpeedBody);
    ////
    legsState = LegsState::Feet12Long;
  }
}

void RobotAction::TurnLeft()
{
  Turn(-turnAngle);
}

void RobotAction::TurnRight()
{
  Turn(turnAngle);
}

void RobotAction::MoveBody(float x, float y, float z)
{
  ActionState();
  if (legsState != LegsState::Move)
    InitialState();

  x = constrain(x, -30, 30);
  y = constrain(y, -30, 30);
  z = constrain(z, -15, 45);

  RobotLegsPoints points = robot.bootPoints;
  GetMoveBodyPoints(points, Point(x, y, bodyLift + z));
  LegsMoveTo(points, speedMoveBody);

  legsState = LegsState::Move;
}

void RobotAction::RotateBody(float x, float y, float z, float angle)
{
  ActionState();
  if (legsState != LegsState::Rotate)
    InitialState();

  angle = constrain(angle, -15, 15);

  RobotLegsPoints points = robot.bootPoints;
  GetMoveBodyPoints(points, Point(0, 0, bodyLift));
  GetRotateBodyPoints(points, x, y, z, angle);
  LegsMoveTo(points, speedRotateBody);

  legsState = LegsState::Rotate;
}

void RobotAction::InitialState()
{
  ActionState();
  if (mode != Mode::Active)
    ActiveMode();
  if (legsState == LegsState::Initial)
    return;

  if (legsState == LegsState::Feet34Long)
  {
    LegsMoveToRelatively(Point(-crawlOffsetX, crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg1, Point(0, 2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(0, -2 * crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg2, Point(0, -2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(2 * crawlOffsetX, 0, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg4, Point(0, 2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(0, 2 * crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg3, Point(0, -2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(-crawlOffsetX, -crawlOffsetY, 0), turnSpeedBody);
  }
  else if (legsState == LegsState::Feet12Long)
  {
    LegsMoveToRelatively(Point(crawlOffsetX, crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg3, Point(0, 2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(0, -2 * crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg4, Point(0, -2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(-2 * crawlOffsetX, 0, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg2, Point(0, 2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(0, 2 * crawlOffsetY, 0), turnSpeedBody);
    LegStepToRelatively(robot.leg1, Point(0, -2 * crawlOffsetY, 0), crawlSpeedLeg);
    LegsMoveToRelatively(Point(crawlOffsetX, -crawlOffsetY, 0), turnSpeedBody);
  }
  else if (legsState == LegsState::Move)
  {
    MoveBody(0, 0, 0);
  }
  else if (legsState == LegsState::Rotate)
  {
    RotateBody(0, 0, 0, 0);
  }
  legsState = LegsState::Initial;
}

void RobotAction::LegMoveToRelativelyDirectly(int leg, Point point)
{
  switch (leg)
  {
  case 1:
    robot.leg1.MoveToRelatively(point);
    break;
  case 2:
    robot.leg2.MoveToRelatively(point);
    break;
  case 3:
    robot.leg3.MoveToRelatively(point);
    break;
  case 4:
    robot.leg4.MoveToRelatively(point);
    break;
  }
}

void RobotAction::ActionState()
{
  if (robot.state != Robot::State::Action)
  {
    robot.BootState();
    robot.state = Robot::State::Action;
    mode = Mode::Sleep;
    legsState = LegsState::Initial;
  }
}

void RobotAction::FirstStepForward()
{
  //
  LegsMoveToRelatively(Point(crawlOffsetX, 0, 0), turnSpeedBody);
  LegStepToRelatively(robot.leg3, Point(0, 4 * crawlOffsetY, 0), crawlSpeedLeg);
  LegsMoveToRelatively(Point(-crawlOffsetX, -2 * crawlOffsetY, 0), turnSpeedBody);
  LegStepToRelatively(robot.leg2, Point(0, 4 * crawlOffsetY, 0), crawlSpeedLeg);
  ////
  legsState = LegsState::Feet34Long;
}

void RobotAction::FirstStepBackward()
{
  //
  LegsMoveToRelatively(Point(-crawlOffsetX, 0, 0), turnSpeedBody);
  LegStepToRelatively(robot.leg2, Point(0, -4 * crawlOffsetY, 0), crawlSpeedLeg);
  LegsMoveToRelatively(Point(crawlOffsetX, 2 * crawlOffsetY, 0), turnSpeedBody);
  LegStepToRelatively(robot.leg3, Point(0, -4 * crawlOffsetY, 0), crawlSpeedLeg);
  ////
  legsState = LegsState::Feet12Long;
}

void RobotAction::LegStepTo(RobotLeg & leg, Point point, float speed)
{
  leg.stepDistance = speed;
  leg.MoveToRelatively(Point(0, 0, legLift));
  leg.WaitUntilFree();
  leg.MoveTo(Point(point.x, point.y, leg.pointNow.z));
  leg.WaitUntilFree();
  leg.MoveToRelatively(Point(0, 0, -legLift));
  leg.WaitUntilFree();
}

void RobotAction::LegStepToRelatively(RobotLeg &leg, Point point, float speed)
{
  leg.stepDistance = speed;
  leg.MoveToRelatively(Point(0, 0, legLift));
  leg.WaitUntilFree();
  leg.MoveToRelatively(Point(point.x, point.y, 0));
  leg.WaitUntilFree();
  leg.MoveToRelatively(Point(0, 0, -legLift));
  leg.WaitUntilFree();
}

void RobotAction::LegMoveToRelatively(RobotLeg & leg, Point point, float speed)
{
  leg.stepDistance = speed;
  leg.MoveToRelatively(point);
  leg.WaitUntilFree();
}

void RobotAction::LegsMoveTo(RobotLegsPoints points)
{
  robot.MoveTo(points);
  robot.WaitUntilFree();
}

void RobotAction::LegsMoveTo(RobotLegsPoints points, float speed)
{
  robot.SetSpeed(speed);
  robot.MoveTo(points);
  robot.WaitUntilFree();
}

void RobotAction::LegsMoveToRelatively(Point point, float speed)
{
  robot.SetSpeed(speed);
  robot.MoveToRelatively(point);
  robot.WaitUntilFree();
}

void RobotAction::GetCrawlPoints(RobotLegsPoints & points, Point point)
{
  GetCrawlPoint(points.leg1, point);
  GetCrawlPoint(points.leg2, point);
  GetCrawlPoint(points.leg3, point);
  GetCrawlPoint(points.leg4, point);
}

void RobotAction::GetCrawlPoint(Point & point, Point direction)
{
  point = Point(point.x + direction.x, point.y + direction.y, point.z + direction.z);
}

void RobotAction::Turn(float angle)
{
  ActionState();
  if (legsState == LegsState::Move || legsState == LegsState::Rotate)
    InitialState();
  if (mode != Mode::Active)
    ActiveMode();

  angle /= 4;

  RobotLegsPoints points;

  if (angle > 0)
  {
    if (legsState == LegsState::Feet34Long)
    {
      // body move away from leg2
      LegsMoveToRelatively(Point(-turnOffset, -turnOffset, 0), turnSpeedBody);
      // leg 2 up
      LegMoveToRelatively(robot.leg2, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      // set leg 2 point
      points.leg2 = robot.bootPoints.leg2;
      GetTurnPoint(points.leg2, angle * 3);
      GetCrawlPoint(points.leg2, Point(-turnOffset, -turnOffset, robot.leg2.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedLeg, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 2 down
      LegMoveToRelatively(robot.leg2, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg4
      LegsMoveToRelatively(Point(2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 4 up
      LegMoveToRelatively(robot.leg4, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      // set leg 4 point
      points.leg4 = robot.bootPoints.leg4;
      GetTurnPoint(points.leg4, angle * 2);
      GetCrawlPoint(points.leg4, Point(turnOffset, -turnOffset, robot.leg4.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedBody, turnSpeedLeg);
      LegsMoveTo(points);
      // leg 4 down
      LegMoveToRelatively(robot.leg4, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg3
      LegsMoveToRelatively(Point(0, 2 * turnOffset, 0), turnSpeedBody);
      // leg 3 up
      LegMoveToRelatively(robot.leg3, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      // set leg 3 point
      points.leg3 = robot.bootPoints.leg3;
      GetTurnPoint(points.leg3, angle * 1);
      GetCrawlPoint(points.leg3, Point(turnOffset, turnOffset, robot.leg3.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedLeg, turnSpeedBody);
      LegsMoveTo(points);
      // leg 3 down
      LegMoveToRelatively(robot.leg3, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg1
      LegsMoveToRelatively(Point(-2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 1 up
      LegMoveToRelatively(robot.leg1, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      // set leg 1 point
      points.leg1 = robot.bootPoints.leg1;
      GetTurnPoint(points.leg1, angle * 0);
      GetCrawlPoint(points.leg1, Point(-turnOffset, turnOffset, robot.leg1.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedLeg, turnSpeedBody, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 1 down
      LegMoveToRelatively(robot.leg1, Point(0, 0, -legLift), turnSpeedLeg);

      // body move to center
      LegsMoveToRelatively(Point(turnOffset, -turnOffset, 0), turnSpeedBody);
    }
    else
    {
      // body move away from leg3
      LegsMoveToRelatively(Point(turnOffset, turnOffset, 0), turnSpeedBody);
      // leg 3 up
      LegMoveToRelatively(robot.leg3, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      // set leg 3 point
      points.leg3 = robot.bootPoints.leg3;
      GetTurnPoint(points.leg3, angle * 3);
      GetCrawlPoint(points.leg3, Point(turnOffset, turnOffset, robot.leg3.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedLeg, turnSpeedBody);
      LegsMoveTo(points);
      // leg 3 down
      LegMoveToRelatively(robot.leg3, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg1
      LegsMoveToRelatively(Point(-2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 1 up
      LegMoveToRelatively(robot.leg1, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      // set leg 1 point
      points.leg1 = robot.bootPoints.leg1;
      GetTurnPoint(points.leg1, angle * 2);
      GetCrawlPoint(points.leg1, Point(-turnOffset, turnOffset, robot.leg1.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedLeg, turnSpeedBody, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 1 down
      LegMoveToRelatively(robot.leg1, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg2
      LegsMoveToRelatively(Point(0, -2 * turnOffset, 0), turnSpeedBody);
      // leg 2 up
      LegMoveToRelatively(robot.leg2, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      // set leg 2 point
      points.leg2 = robot.bootPoints.leg2;
      GetTurnPoint(points.leg2, angle * 1);
      GetCrawlPoint(points.leg2, Point(-turnOffset, -turnOffset, robot.leg2.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedLeg, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 2 down
      LegMoveToRelatively(robot.leg2, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg4
      LegsMoveToRelatively(Point(2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 4 up
      LegMoveToRelatively(robot.leg4, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      // set leg 4 point
      points.leg4 = robot.bootPoints.leg4;
      GetTurnPoint(points.leg4, angle * 0);
      GetCrawlPoint(points.leg4, Point(turnOffset, -turnOffset, robot.leg4.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedBody, turnSpeedLeg);
      LegsMoveTo(points);
      // leg 4 down
      LegMoveToRelatively(robot.leg4, Point(0, 0, -legLift), turnSpeedLeg);

      // body move to center
      LegsMoveToRelatively(Point(-turnOffset, turnOffset, 0), turnSpeedBody);
    }
  }
  else
  {
    if (legsState == LegsState::Feet12Long)
    {
      // body move away from leg4
      LegsMoveToRelatively(Point(turnOffset, -turnOffset, 0), turnSpeedBody);
      // leg 4 up
      LegMoveToRelatively(robot.leg4, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      // set leg 4 point
      points.leg4 = robot.bootPoints.leg4;
      GetTurnPoint(points.leg4, angle * 3);
      GetCrawlPoint(points.leg4, Point(turnOffset, -turnOffset, robot.leg4.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedBody, turnSpeedLeg);
      LegsMoveTo(points);
      // leg 4 down
      LegMoveToRelatively(robot.leg4, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg2
      LegsMoveToRelatively(Point(-2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 2 up
      LegMoveToRelatively(robot.leg2, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      // set leg 2 point
      points.leg2 = robot.bootPoints.leg2;
      GetTurnPoint(points.leg2, angle * 2);
      GetCrawlPoint(points.leg2, Point(-turnOffset, -turnOffset, robot.leg2.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedLeg, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 2 down
      LegMoveToRelatively(robot.leg2, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg1
      LegsMoveToRelatively(Point(0, 2 * turnOffset, 0), turnSpeedBody);
      // leg 1 up
      LegMoveToRelatively(robot.leg1, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      // set leg 1 point
      points.leg1 = robot.bootPoints.leg1;
      GetTurnPoint(points.leg1, angle * 1);
      GetCrawlPoint(points.leg1, Point(-turnOffset, turnOffset, robot.leg1.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedLeg, turnSpeedBody, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 1 down
      LegMoveToRelatively(robot.leg1, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg3
      LegsMoveToRelatively(Point(2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 3 up
      LegMoveToRelatively(robot.leg3, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      // set leg 3 point
      points.leg3 = robot.bootPoints.leg3;
      GetTurnPoint(points.leg3, angle * 0);
      GetCrawlPoint(points.leg3, Point(turnOffset, turnOffset, robot.leg3.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedLeg, turnSpeedBody);
      LegsMoveTo(points);
      // leg 3 down
      LegMoveToRelatively(robot.leg3, Point(0, 0, -legLift), turnSpeedLeg);

      // body move to center
      LegsMoveToRelatively(Point(-turnOffset, -turnOffset, 0), turnSpeedBody);
    }
    else
    {
      // body move away from leg1
      LegsMoveToRelatively(Point(-turnOffset, turnOffset, 0), turnSpeedBody);
      // leg 1 up
      LegMoveToRelatively(robot.leg1, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      // set leg 1 point
      points.leg1 = robot.bootPoints.leg1;
      GetTurnPoint(points.leg1, angle * 3);
      GetCrawlPoint(points.leg1, Point(-turnOffset, turnOffset, robot.leg1.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedLeg, turnSpeedBody, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 1 down
      LegMoveToRelatively(robot.leg1, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg3
      LegsMoveToRelatively(Point(2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 3 up
      LegMoveToRelatively(robot.leg3, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      // set leg 3 point
      points.leg3 = robot.bootPoints.leg3;
      GetTurnPoint(points.leg3, angle * 2);
      GetCrawlPoint(points.leg3, Point(turnOffset, turnOffset, robot.leg3.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedLeg, turnSpeedBody);
      LegsMoveTo(points);
      // leg 3 down
      LegMoveToRelatively(robot.leg3, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg4
      LegsMoveToRelatively(Point(0, -2 * turnOffset, 0), turnSpeedBody);
      // leg 4 up
      LegMoveToRelatively(robot.leg4, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(-turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(turnOffset, -turnOffset, 0));
      // set leg 4 point
      points.leg4 = robot.bootPoints.leg4;
      GetTurnPoint(points.leg4, angle * 1);
      GetCrawlPoint(points.leg4, Point(turnOffset, -turnOffset, robot.leg4.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedBody, turnSpeedBody, turnSpeedLeg);
      LegsMoveTo(points);
      // leg 4 down
      LegMoveToRelatively(robot.leg4, Point(0, 0, -legLift), turnSpeedLeg);

      // body move away from leg2
      LegsMoveToRelatively(Point(-2 * turnOffset, 0, 0), turnSpeedBody);
      // leg 2 up
      LegMoveToRelatively(robot.leg2, Point(0, 0, legLift), turnSpeedLeg);
      // set legs points
      robot.GetPointsNow(points);
      GetCrawlPoints(points, Point(turnOffset, turnOffset, 0));
      GetTurnPoints(points, -angle);
      GetCrawlPoints(points, Point(-turnOffset, -turnOffset, 0));
      // set leg 2 point
      points.leg2 = robot.bootPoints.leg2;
      GetTurnPoint(points.leg2, angle * 0);
      GetCrawlPoint(points.leg2, Point(-turnOffset, -turnOffset, robot.leg2.pointNow.z));
      // body move
      robot.SetSpeed(turnSpeedBody, turnSpeedLeg, turnSpeedBody, turnSpeedBody);
      LegsMoveTo(points);
      // leg 2 down
      LegMoveToRelatively(robot.leg2, Point(0, 0, -legLift), turnSpeedLeg);

      // body move to center
      LegsMoveToRelatively(Point(turnOffset, turnOffset, 0), turnSpeedBody);
    }
  }
  legsState = LegsState::Initial;
}

void RobotAction::GetTurnPoints(RobotLegsPoints & points, float angle)
{
  GetTurnPoint(points.leg1, angle);
  GetTurnPoint(points.leg2, angle);
  GetTurnPoint(points.leg3, angle);
  GetTurnPoint(points.leg4, angle);
}

void RobotAction::GetTurnPoint(Point & point, float angle)
{
  float radian = angle * PI / 180;
  float radius = sqrt(pow(point.x, 2) + pow(point.y, 2));

  float x = radius * cos(atan2(point.y, point.x) + radian);
  float y = radius * sin(atan2(point.y, point.x) + radian);

  point = Point(x, y, point.z);
}

void RobotAction::GetMoveBodyPoints(RobotLegsPoints & points, Point point)
{
  GetMoveBodyPoint(points.leg1, point);
  GetMoveBodyPoint(points.leg2, point);
  GetMoveBodyPoint(points.leg3, point);
  GetMoveBodyPoint(points.leg4, point);
}

void RobotAction::GetMoveBodyPoint(Point & point, Point direction)
{
  point = Point(point.x - direction.x, point.y - direction.y, point.z - direction.z);
}

void RobotAction::GetRotateBodyPoints(RobotLegsPoints & points, float x, float y, float z, float angle)
{
  float vectorLength = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  if (vectorLength == 0)
  {
    x = 0;
    y = 0;
    z = 1;
  }
  else
  {
    x = x / vectorLength;
    y = y / vectorLength;
    z = z / vectorLength;
  }

  GetRotateBodyPoint(points.leg1, x, y, z, angle);
  GetRotateBodyPoint(points.leg2, x, y, z, angle);
  GetRotateBodyPoint(points.leg3, x, y, z, angle);
  GetRotateBodyPoint(points.leg4, x, y, z, angle);
}

void RobotAction::GetRotateBodyPoint(Point & point, float x, float y, float z, float angle)
{
  float old_x = point.x;
  float old_y = point.y;
  float old_z = point.z;

  angle = angle * PI / 180;
  float c = cos(angle);
  float s = sin(angle);

  point.x = (x * x * (1 - c) + c) * old_x + (x * y * (1 - c) - z * s) * old_y + (x * z * (1 - c) + y * s) * old_z;
  point.y = (y * x * (1 - c) + z * s) * old_x + (y * y * (1 - c) + c) * old_y + (y * z * (1 - c) - x * s) * old_z;
  point.z = (x * z * (1 - c) - y * s) * old_x + (y * z * (1 - c) + x * s) * old_y + (z * z * (1 - c) + c) * old_z;
}

#endif
