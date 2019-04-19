#include "main.h"

// Dump ports namespace for ease of use
using namespace ports;

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void doubleShot() {
  flywheelMotor->move(127);
  while (flywheelMotor->get_actual_velocity() < 580)
    pros::delay(1);
  indexMotor->move(127);
  pros::delay(80);
  indexMotor->move(0);
  pros::delay(30);
  pid->move(25);
  indexMotor->move(127);
  intakeMotor->move(127);
  pros::delay(300);
  indexMotor->move(0);
  intakeMotor->move(0);
}

void visionAlign() {
  while (true) {
    if (flagVision != NULL) {
      // Prepare variables for decision
		int sigID = LCD::isAutonomousBlue() ? 1 : 2;
		pros::vision_object_s_t sig = flagVision->get_by_sig(0, sigID);
		int middle = util::sign(sig.x_middle_coord);
		int diff = middle - 158;
		// If a signature is detected, lock to it. Otherwise, give control back over to the driver
		if (middle > -2000)
			if (util::abs(diff) > 5) {
				int turnPower = diff / 3 + 10 * util::abs(diff) / diff;
        int leftPower = turnPower;
      	int rightPower = -turnPower;
      	frontLeftDrive->move(leftPower);
      	backLeftDrive->move(leftPower);
      	frontRightDrive->move(rightPower);
      	backRightDrive->move(rightPower);
      } else
				break;
		else;
  	}
  }
}

void autonomousSkills() {}

void autonomousBlueFlags() {
  flywheelMotor->move(127);
  intakeMotor->move(127);
  pid->move(44.4);
  pid->move(-34.4);
  pid->pivot(89);
  intakeMotor->move(0);

  // Shake the lift
  liftMotor->move(127);
  pros::delay(150);
  liftMotor->move(-127);
  pros::delay(350);
  liftMotor->move_absolute(265, 127);

  visionAlign();
  doubleShot();
  flywheelMotor->move(0);

  pid->move(10);
  pid->move(-6.25);
  intakeMotor->move(127);
  indexMotor->move(18);

  pid->pivot(-93);
  pid->resetEncoders();
  while (frontLeftDrive->get_position() < 180) {
    pid->driveStraight(50);
    pros::delay(10);
  }
  flywheelMotor->move(90);
  pid->move(-8.5);
  pros::delay(750);
  indexMotor->move(0);
  intakeMotor->move(-100);
  pid->move(29);
  pid->pivot(60);
  visionAlign();
  pid->move(5.5);
  intakeMotor->move(127);
  indexMotor->move(127);
  pros::delay(200);
  intakeMotor->move(0);
  indexMotor->move(0);
  pid->move(7.25);
}

void autonomousRedFlags() {
  flywheelMotor->move(127);
  intakeMotor->move(127);
  pid->move(44.4);
  pid->move(-35);
  pid->pivot(-90);
  intakeMotor->move(0);

  // Shake the lift
  liftMotor->move(127);
  pros::delay(150);
  liftMotor->move(-127);
  pros::delay(350);
  liftMotor->move_absolute(265, 127);

  visionAlign();
  doubleShot();
  flywheelMotor->move(0);

  pid->move(10);
  pid->move(-6.25);
  intakeMotor->move(127);
  indexMotor->move(18);

  pid->pivot(93);
  pid->resetEncoders();
  while (frontLeftDrive->get_position() < 180) {
    pid->driveStraight(50);
    pros::delay(10);
  }
  flywheelMotor->move(90);
  pid->move(-8.5);
  pros::delay(750);
  indexMotor->move(0);
  intakeMotor->move(-100);
  pid->move(29);
  pid->pivot(-60);
  visionAlign();
  pid->move(6.5);
  intakeMotor->move(127);
  indexMotor->move(127);
  pros::delay(200);
  intakeMotor->move(0);
  indexMotor->move(0);
  pid->move(7.25);

}

void autonomousBlueFar() {

}

void autonomousRedFar() {
  intakeMotor->move(127);
  pid->move(44.4);
  pid->move(-6);
  pid->pivot(68);

  // Shake the lift
  liftMotor->move(127);
  pros::delay(150);
  liftMotor->move(-127);
  pros::delay(800);

  pid->move(8.5);
  liftMotor->move(127);
  pros::delay(1500);
  pid->move(-31.25);

}

void autonomousOther(int selected) {

}

void autonomous() {
  switch (selectedAutonomous) {
    case 0:
      autonomousSkills();
      break;
    case 1:
      autonomousBlueFlags();
      break;
    case 2:
      autonomousRedFlags();
      break;
    case 3:
      autonomousBlueFar();
      break;
    case 4:
      autonomousRedFar();
      break;
    default:
      autonomousOther(selectedAutonomous);
      break;
  }
}
