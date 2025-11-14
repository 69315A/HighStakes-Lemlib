#include "autons.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "lemlib/api.hpp"

// ANGULAR MOTIONS
// turnToHeading --> turn to certain angle
// turntoPoint --> like turnToHeading but uses coordinates instead 
    // turn >10" of chassis only!!!
// swingToHeading --> turn only using half drivetrain
// swingToPoint --> swingToHeading but coordinates

// LATERAL MOTIONS
// moveToPoint --> move to point w/o specifying degree
// moveToPose --> majority lateral motion, ending with swing to certain heading
    // SLOWER than moveToPoint

// 37.5 inches
// 46 inches to goal
// .75 inches for start

void matchLoad(){
    // inital allignment to loader
    driveForward(315, 550);
    pros::delay(500);
    turnRight(137, 500);
    pros::delay(200);
    // loading
    mLoader.extend();
    pros::delay(865);
    driveForward(300, 575);
    Lintake.move_velocity(12000);
    pros::delay(900); 
    Lintake.move_velocity(0);
    // going towards goal
    driveBack(130, 2000);
    mLoader.retract();
    // quick adjusment to the left
    leftMotors.move_velocity(34);
    rightMotors.move_velocity(-34);
    pros::delay(400);
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
    pros::delay(600);
    // to goal
    driveBack(50,520);
    // scoring
    pros::delay(450);
    Lintake.move_velocity(12000);
    Uintake.move_velocity(12000);
    rightMotors.move_velocity(20);
    leftMotors.move_velocity(20);
    pros::delay(3000);
    rightMotors.move_velocity(0);
    leftMotors.move_velocity(0);
    Lintake.move_velocity(0);
    Uintake.move_velocity(0);
 
}

void turnTest() {
    // chassis.moveToPose(2,4,90,1000);
    chassis.turnToHeading(90, 100000);
}

void turnRight(double voltage, int time){
    leftMotors.move_velocity(-voltage);
    rightMotors.move_velocity(voltage);
    pros::delay(time);
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}

void turnLeft(double voltage, int time){
    rightMotors.move_velocity(voltage);
    leftMotors.move_velocity(voltage);
    pros::delay(time);
    rightMotors.move_velocity(0);
    leftMotors.move_velocity(0);
}

void driveForward(double voltage, int time){
    leftMotors.move_velocity(-voltage);
    rightMotors.move_velocity(-voltage);
    pros::delay(time);
    rightMotors.move_velocity(0);
    leftMotors.move_velocity(0);
}

void driveBack(double voltage, int time){
    leftMotors.move_velocity(voltage);
    rightMotors.move_velocity(voltage);
    pros::delay(time);
    leftMotors.move_velocity(0);
    rightMotors.move_velocity(0);
}
