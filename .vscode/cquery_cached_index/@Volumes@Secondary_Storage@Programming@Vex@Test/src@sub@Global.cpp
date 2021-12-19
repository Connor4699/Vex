#include "main.h"

//Initializing Motors
pros::Motor DLF(1, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DRF(10, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DLB(11, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor DRB(20, pros::E_MOTOR_GEARSET_18, true, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor lift(12, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor claw(17, pros::E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

//Initializing okapi Motors
//okapi::MotorGroup left({13, 11});
//okapi::MotorGroup right({-18, 20});
okapi::Motor arm(-12);
okapi::Motor jaw(-17);

//Initializing Encoders
pros::ADIEncoder encoder_right(7, 8, false);
pros::ADIEncoder encoder_left(5, 6, false);
pros::ADIEncoder encoder_rear(3, 4, false);

//Initializing Sensors
pros::Imu imu_sensor(18);

//Initializing Controller
pros::Controller con(pros::E_CONTROLLER_MASTER);

//Initializing okapi Controller
Controller controller;

