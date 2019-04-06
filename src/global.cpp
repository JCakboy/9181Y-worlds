#include "main.h"

// Ports initialization
namespace ports {

  // Controllers
  pros::Controller * controllerMain = new pros::Controller(CONTROLLER_MASTER);
  pros::Controller * controllerPartner = new pros::Controller(CONTROLLER_PARTNER);

  // Motors
  pros::Motor * port1 = new pros::Motor(1, GEARSET_200, FWD, ENCODER_DEGREES);
  pros::Motor * port2 = new pros::Motor(2, GEARSET_200, FWD, ENCODER_DEGREES);
  pros::Motor * port3 = new pros::Motor(3, GEARSET_600, REV, ENCODER_DEGREES);
  pros::Motor * port4 = new pros::Motor(4, GEARSET_200, FWD, ENCODER_DEGREES);
  pros::Motor * port5 = NULL;
  pros::Motor * port6 = NULL;
  pros::Motor * port7 = new pros::Motor(7, GEARSET_200, REV, ENCODER_DEGREES);
  pros::Motor * port8 = new pros::Motor(8, GEARSET_600, REV, ENCODER_DEGREES);
  pros::Motor * port9 = new pros::Motor(9, GEARSET_200, REV, ENCODER_DEGREES);
  pros::Motor * port10 = new pros::Motor(10, GEARSET_200, REV, ENCODER_DEGREES);
  pros::Motor * port11 = NULL;
  pros::Motor * port12 = NULL;
  pros::Motor * port13 = NULL;
  pros::Motor * port14 = NULL;
  pros::Motor * port15 = NULL;
  pros::Motor * port16 = NULL;
  pros::Motor * port17 = NULL;
  pros::Motor * port18 = NULL;
  pros::Motor * port19 = NULL;
  pros::Motor * port20 = NULL;
  pros::Motor * port21 = NULL;

  // Port mapping
  pros::Motor * frontLeftDrive = port2;
  pros::Motor * backLeftDrive = port1;
  pros::Motor * frontRightDrive = port9;
  pros::Motor * backRightDrive = port10;
  pros::Motor * liftMotor = port4;
  pros::Motor * intakeMotor = port7;
  pros::Motor * indexMotor = port3;
  pros::Motor * flywheelMotor = port8;

  // Vision
  pros::Vision * flagVision = NULL;

  // ADI (3-wire) ports
  pros::ADIGyro * gyro = new pros::ADIGyro(21); //THE PORT FOR GYRO IS NOT CONFIRMED YET
}

// Selected autonomous routine
int selectedAutonomous = 0;
