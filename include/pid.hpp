#ifndef _PID_HPP_
#define _PID_HPP_

#include "main.h"

class PID {
friend void ::opcontrol();
  // Power restraints
  const int MAX_POWER = 120;
  const int MIN_POWER = 20;

  // PID values
  double movekp = 0;
  double movekd = 0;
  double straightkp = 0;
  double pivotkp = 0;
  double pivotkd = 0;

  // Calculates and returns the gear ratio for the drive
  static double getGearRatio();

  // Sets the brake mode
  void setBrakeMode();
  // Resets the motor encoders
  void resetEncoders();
  // Returns the power given the minimum and maximum power restraints
  double checkPower(double power);
  // Ensures the robot drives straight
  void driveStraight(int power);
  // Sends the power commands to the motor
  void powerDrive(int powerLeft, int powerRight);
public:
  // Constructs the PID object
  PID();

  // Sets the move PID values
  void setMovePID(double movekp, double movekd, double straightkp);
  // Sets the pivot PID values
  void setPivotPID(double pivotkp, double pivotkd);

  // Moves the robot the given amount of inches to the desired location
  void move(double inches);
  // Pivots the robot the given amount of degrees
  void pivot(double degrees);

};

#endif
