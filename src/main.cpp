#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "autons.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "robodash/api.h"

// Chassis constructors
pros::MotorGroup leftMotors{{10, 9, 8}, pros::MotorGearset::blue};
pros::MotorGroup rightMotors{{-1, -3, -2}, pros::MotorGearset::blue};

// Motor constructors
pros::Motor Lintake(13, pros::MotorGearset::blue);
pros::Motor Uintake(-4, pros::MotorGearset::blue);

// Pneumatic constructors
pros::adi::Pneumatics midGoal('E', false);
pros::adi::Pneumatics mLoader('B', false);

// sensors for odometry
pros::Imu imu(7);
lemlib::OdomSensors sensorsIMUOnly(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// Controller constructor
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Robodash brain screen constructor
rd::Console brain;

rd::Selector selector({
    {"test", turnTest}
});

// drivetrain settings
lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11,
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(9, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              30, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              11, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     13, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  11, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

// create the chassis
lemlib::Chassis chassis(drivetrain, 
lateral_controller, 
angular_controller, 
sensorsIMUOnly, 
&throttleCurve, 
&steerCurve);

void initialize() {
    imu.reset();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(6000);
    chassis.calibrate();
    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            //print robot location to the brain screen
            brain.printf("\nX: %f", chassis.getPose().x); // x
            brain.printf("\nY: %f", chassis.getPose().y); // y
            brain.printf("\nTheta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });

}

void disabled() {}
void competition_initialize() {}

void autonomous() {
    // selector.run_auton();
    // chassis.calibrate();
    // matchLoad();
    chassis.setPose(0, 0, 0);
    brain.printf("\nX: %f", chassis.getPose().x); // x
    brain.printf("\nY: %f", chassis.getPose().y); // y
    brain.printf("\nTheta: %f", chassis.getPose().theta); // heading
    pros::delay(3000);
    chassis.turnToHeading(45, 400);
    brain.printf("\nX: %f", chassis.getPose().x); // x
    brain.printf("\nY: %f", chassis.getPose().y); // y
    brain.printf("\nTheta: %f", chassis.getPose().theta); // heading 
}

void opcontrol() {

    bool mLoaderToggle = false;
    bool midGoalToggle = false;

    while (true) {
       
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(-leftY, -rightX);

		// Intake controls
         if (controller.get_digital(DIGITAL_L1)){
            Lintake.move_velocity(-12000);
            Uintake.move_velocity(-12000);        
        } else if (controller.get_digital(DIGITAL_L2)){
            Lintake.move_velocity(12000);
            Uintake.move_velocity(12000);
        } else if (controller.get_digital(DIGITAL_R1)) {
            Lintake.move_velocity(-12000); 
            Uintake.move_velocity(0);
        } else if (controller.get_digital(DIGITAL_R2)){
            Lintake.move_velocity(12000);
            Uintake.move_velocity(0);
        } else {
            Lintake.move_velocity(0);
            Uintake.move_velocity(0);
        }

        // match loader pneumatics
        if(controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
            if(mLoaderToggle == false){
                mLoader.extend();
                mLoaderToggle = true;
            } else {
                mLoader.retract();
                mLoaderToggle = false;
            }
        }

        // middle goal pneumatics
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)){
            if(midGoalToggle == false){
                midGoal.extend();
                midGoalToggle = true;
            } else {
                midGoal.retract();
                midGoalToggle = false;
            }
        }
        

        // delay to save resources
        pros::delay(10);

    }
}
