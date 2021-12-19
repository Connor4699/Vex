#include "main.h"

void reset(){
    DLF.tare_position();
    DLB.tare_position();
    DRF.tare_position();
    DRB.tare_position();
}


void setDrive(int left, int right){
    DLF = left;
    DRF = right;
    DLB = left;
    DRB = right;
}

void setArm(int power){
    lift = power;
}


void drivemotors(bool state){
//    int xjoy = con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
//    int yjoy = con.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
////    int rjoy = con.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
//
//
//    if (abs(xjoy) < 10 && abs(yjoy) < 10){
//        setDrive(0, 0);
//    } else if (abs(xjoy) > 10 && -5 < yjoy && yjoy < 5) {
//        setDrive(xjoy, xjoy);
//    }
//    else {
//        int left = yjoy + xjoy;
//        int right = yjoy - xjoy;
//        setDrive(left, right);
//    }
//    okapi::MotorGroup left({13, 11});
//    okapi::MotorGroup right({-18, 20});
//    std::shared_ptr<ChassisController> drive =
//            ChassisControllerBuilder()
//                    .withMotors({13, 11}, {-18, 20})
//                    .withDimensions(AbstractMotor::gearset::green, {{4_in, 11.5_in}, imev5GreenTPR})
//                    .build();
//    drive->getModel()->arcade(controller.getAnalog(okapi::ControllerAnalog::leftY),
//                              controller.getAnalog(okapi::ControllerAnalog::leftX));
}


void liftArm(){
//    if(con.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
//        setArm(127);
//    } else {
//        setArm(0);
//    }
    ControllerButton armUpButton(ControllerDigital::A);
    ControllerButton armDownButton(ControllerDigital::B);
    if (armUpButton.isPressed()) {
        arm.moveVoltage(12000);
    } else if (armDownButton.isPressed()) {
        arm.moveVoltage(-12000);
    } else {
        arm.moveVoltage(0);
    }
}
//
//
std::string move(int units, int volt){
    // needs to be added before autonomous runs Sensors_reset();
    double *pos = position();
//    double final = pos[0] + units;
    double *val = straight();
    double final = val[0] + units;
    if (final < 0){
        return "invalid";
    }
    while(val[0] < final){
        setDrive(volt, volt);
        pros::lcd::set_text(0, std::to_string(val[0]));
        pros::delay(10);
        val = straight();
    }
    setDrive(0, 0);
//    while(pos[0] < final){
//        setDrive(volt, volt);
//        if (pos[0] >= final - 6){
//            for(int i = volt; i > 0; i--){
//                setDrive(i, i);
//                pros::delay(5);
//            }
//        }
//        pos = position();
//    }
//    while(pos[0] < final){
//        if(pos[0] == 0){
//            for(int i = 0; i < volt; i++){
//                setDrive(i, i);
//                pros::delay(10);
//            }
//        }else{
//            setDrive(volt, volt);
//        }
//        pos = position();
//    }
    return "works";
}
//
//
//std::string turn(double angle, int volt){
//    pros::delay(1000);
//    double *pos = position();
//    double final = pos[2] + angle;
//    pros::lcd::set_text(0, "bum");
//    pros::delay(1000);
//    if (angle > 0){
//        while(pos[2] < final){
//            pros::lcd::set_text(0, std::to_string(final));
//            setDrive(volt, -volt);
//            pros::lcd::set_text(1, std::to_string(pos[2]));
//            pos = position();
//        }
//        setDrive(0, 0);
//    } else if (angle < 0){
//        while(pos[2] > final){
//            pros::lcd::set_text(0, std::to_string(final));
//            setDrive(-volt, volt);
//            pros::lcd::set_text(1, std::to_string(pos[2]));
//            pos = position();
//        }
//        setDrive(0, 0);
//    }
//    return "work";
//}
//
//// forward distance, turn distance, rest of distance for *x
//std::string sTurn(const int *x, int y, int volt){
//    double *pos = position();
//    double finalX = pos[0];
//    for (int i = 0; i <= 2; i++){
//        finalX += x[i];
//    }
//    double finalY = pos[1] + y;
//    if (finalX < 0 || finalY < 0){
//        return "invalid";
//    }
//
//    move(x[0], volt);
//
//
//    return "work";
//}