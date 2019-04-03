#include "main.h"

#ifndef limit127
#define limit127(a) (a > 127 ? 127 : (a < -127 ? -127 : a))
#endif

// Dump ports namespace for ease of use
using namespace ports;

// Drives the robot based on the given controller
void drive(pros::Controller * controller) {
	int movePower = controller->get_analog(STICK_LEFT_Y);
	int turnPower = controller->get_analog(STICK_LEFT_X);
	int leftPower = movePower + turnPower;
	int rightPower = movePower - turnPower;
	frontLeftDrive->move(leftPower);
	backLeftDrive->move(leftPower);
	frontRightDrive->move(rightPower);
	backRightDrive->move(rightPower);
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {

	while (true) {
		// Drives the robot with the main controller
		drive(controllerMain);

		// Maps the right joystick to the lift
		liftMotor->move(controllerMain->get_analog(STICK_RIGHT_Y));

		// Maps the intake motor to the right triggers
		intakeMotor->move(limit127(controllerMain->get_digital(BUTTON_R1) * 2 * 127 - controllerMain->get_digital(BUTTON_R1) * 127));

		pros::delay(20);
	}
}
