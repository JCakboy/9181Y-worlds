#ifndef _PID_HPP_
#define _PID_HPP_

#include "main.h"

class PID {

  double wheelDiameter = 4; //in inches
  double wheelCircumference = PI * wheelDiameter;
  const int MAX_POWER = 90;
  const int MIN_POWER = 20;

public:
  void setBrakeMode();
  void resetEncoders();
  int checkPower(int power);
  void driveStraight(int power);
  void distancePID(int targetDistance);
  void pivotPID(int targetDegree);
};

#endif
