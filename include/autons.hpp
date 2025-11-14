#pragma once
#include "pros/adi.hpp"
#include "pros/motors.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "robodash/api.h"

// motor constructors
extern pros::Motor Lintake;
extern pros::Motor Uintake;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

// pneumatic constructors
extern pros::adi::Pneumatics midGoal;
extern pros::adi::Pneumatics mLoader;

// chassis constructor
extern lemlib::Chassis chassis;

// match auton
void turnTest();
void turnRight(double votalge, int time);
void turnLeft(double voltage, int time);
void driveForward(double voltage, int time);
void driveBack(double voltage, int time);
void matchLoad();