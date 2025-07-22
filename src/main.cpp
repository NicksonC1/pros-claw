// <--------------------------------------------------------------- Includes --------------------------------------------------------------->
#include <bits/stdc++.h>
#include <vector>
#include <functional>
#include <string>
// #include <iomanip>
#include "main.h"
#include "genesis/api.hpp" // IWYU pragma: keep
#include "liblvgl/lvgl.h" //.
#include "liblvgl/llemu.hpp" //.
#include "brainScreenLVGL.h"
#include "pros/motors.h"
#include "pros/rtos.hpp"

// <--------------------------------------------------------------- Setup ------------------------------------------------------------------>
// controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// DriveTrain
pros::MotorGroup leftMotors({-10}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({1},pros::MotorGearset::green); 

namespace Motor{
    pros::Motor lift(8, pros::MotorGearset::green);
    pros::Motor claw(-3, pros::MotorGearset::green);
} // namespace Motor

namespace Sensor{
    pros::ADIAnalogIn line('H');
} // namspace Sensor

namespace Piston{
} // namespace Piston

// <------------------------------------------------------------- Odom Sensors ------------------------------------------------------------->
// pros::Imu imu(2);
pros::Rotation horizontalEnc(-16);
pros::Rotation verticalEnc(-15);
genesis::TrackingWheel vertical_tracking_wheel(&verticalEnc, genesis::Omniwheel::NEW_2 , 0.9); // Single
genesis::TrackingWheel horizontal_tracking_wheel(&horizontalEnc, 2.0 , -2.2); // Double Stacked

// <---------------------------------------------------------------- Config ---------------------------------------------------------------->
genesis::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              11.5, // 11.5 inch track width
                              genesis::Omniwheel::NEW_325, // using new 3.25" omnis
                              450, // drivetrain rpm is 450
                              2 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

genesis::ControllerSettings linearController(8, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             5, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in inches
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in inches
                                             500, // large error range timeout, in milliseconds
                                             5 // maximum acceleration (slew)
);

genesis::ControllerSettings angularController(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              15, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// sensors for odometry
genesis::OdomSensors sensors(nullptr, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            nullptr // inertial sensor
);

// input curve for throttle input during driver control
genesis::ExpoDriveCurve throttleCurve(1, // joystick deadband out of 127
                                     0, // minimum output where drivetrain will move out of 127
                                     1 // expo curve gain
);

// input curve for steer input during driver control
genesis::ExpoDriveCurve steerCurve(1, // joystick deadband out of 127
                                  0, // minimum output where drivetrain will move out of 127
                                  1 // expo curve gain
);

// create the chassis
genesis::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

LV_IMG_DECLARE(tdbg);
LV_IMG_DECLARE(logo);

lv_obj_t * sbg = lv_img_create(lv_scr_act());
lv_obj_t * slogo = lv_img_create(lv_scr_act());

LV_IMG_DECLARE(cp);
lv_obj_t * img = lv_img_create(lv_scr_act());

// <------------------------------------------------------------ Initialize --------------------------------------------------------------->
void initialize() {
    // lv_img_set_src(sbg, &tdbg);
	// lv_obj_set_pos(sbg,0,0);
	// lv_img_set_src(slogo, &logo);
	// lv_obj_set_pos(slogo,105,-15);
    chassis.setPose(0, 0, 0);
    chassis.calibrate();
    Sensor::line.calibrate();
    // pros::Task cataTask(cataV2);
    // pros::Task cataTask(catag);
    lv_init();
    lv_img_set_src(img, &cp);
    lv_obj_set_pos(img,0,0);
    // pros::lcd::initialize();
    // pros::Task screenTask([&]() {
    //     while (1) {
    //         // pros::lcd::print(0, "X: %f", chassis.getPose().x);
    //         pros::lcd::print(1, "Val: %d", Sensor::line.get_value());
    //         // pros::lcd::print(2, "Line: %d", Sensor::line.get_value());
    //         pros::delay(50);
    //     }
    // });
}
void disabled() {}
void competition_initialize() {}
ASSET(example_txt); // PP
// <------------------------------------------------------------- Autonomous ------------------------------------------------------------->
void autonomous() {
}

// <--------------------------------------------------------------- Driver --------------------------------------------------------------->
void opcontrol() {
	leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
	// rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    // Motor::lift.tare_position();
    // Motor::claw.tare_position();
    Motor::lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    Motor::claw.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    while(1) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        leftMotors.move(leftY-rightX);
        rightMotors.move(leftY+rightX);

        // leftMotors.move(rightX);
        // rightMotors.move(-rightX);


        // chassis.arcade(leftY, rightX);
        (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) ? Motor::lift.move(127) : (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) ? Motor::lift.move(-127) : Motor::lift.brake();
        (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) ? Motor::claw.move(127) : (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) ? Motor::claw.move(-127) : Motor::claw.brake();
        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) Motor::lift.move(127);
        // else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) Motor::lift.move(-127);
        // else Motor::lift.brake();

        // if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) Motor::claw.move(127);
        // else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) Motor::claw.move(-127);
        // else Motor::claw.brake();
        pros::delay(10);
    }
}