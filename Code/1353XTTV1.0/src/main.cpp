// TODO
// Create a new file and move motor definitions in it
// Create a new file for every method definition and its type
// Autonomous functions

#include "vex.h"

using namespace vex;

vex::brain Brain;

vex::competition    Competition;

vex::controller Controller1 = vex::controller();

vex::motor backLeft = vex::motor(vex::PORT13, vex::gearSetting::ratio18_1, false);
vex::motor backRight = vex::motor(vex::PORT18, vex::gearSetting::ratio18_1, true);
vex::motor frontLeft = vex::motor(vex::PORT11, vex::gearSetting::ratio18_1, false);
vex::motor frontRight = vex::motor(vex::PORT20, vex::gearSetting::ratio18_1, true);
vex::motor armLift = vex::motor(vex::PORT1, vex::gearSetting::ratio36_1, false);
vex::motor pivot = vex::motor(vex::PORT8, vex::gearSetting::ratio36_1, true);
vex::motor intake1 = vex::motor(vex::PORT3, vex::gearSetting::ratio18_1, false);
vex::motor intake2 = vex::motor(vex::PORT5, vex::gearSetting::ratio18_1, true);

int controllerTollerance = 20; // 0-100 value assigned to create a deadzone in the joysticks, prevents accidental activation.

void stopAllMotors(){ // A method used to stop every motor, used in autonomous programs and unique circumstances.
  backLeft.stop(vex::brakeType::brake);
  backRight.stop(vex::brakeType::brake);
  frontLeft.stop(vex::brakeType::brake);
  frontRight.stop(vex::brakeType::brake);
  armLift.stop(vex::brakeType::hold);
  pivot.stop(vex::brakeType::coast);
  intake1.stop(vex::brakeType::hold);
  intake2.stop(vex::brakeType::hold);
}

void driverDriveControl(){ // Controls the drive train movement during the autonomous period. Arcade style movement on right joystick.
  if((abs(Controller1.Axis1.value()) > controllerTollerance) || (abs(Controller1.Axis2.value()) > controllerTollerance)){
    int leftSideValue = Controller1.Axis2.value() + Controller1.Axis1.value();
    int rightSideValue = Controller1.Axis2.value() - Controller1.Axis1.value();
    backLeft.spin(vex::directionType::fwd, leftSideValue, vex::velocityUnits::pct);
    frontLeft.spin(vex::directionType::fwd, leftSideValue, vex::velocityUnits::pct);
    backRight.spin(vex::directionType::fwd, rightSideValue, vex::velocityUnits::pct);
    frontRight.spin(vex::directionType::fwd, rightSideValue, vex::velocityUnits::pct);
  } else {
    backLeft.stop(vex::brakeType::brake);
    backRight.stop(vex::brakeType::brake);
    frontLeft.stop(vex::brakeType::brake);
    frontRight.stop(vex::brakeType::brake);
  }
}

void driverpivotControl(){ // Controls the pivot (ramp) on the right side analog buttons.
  if(Controller1.ButtonR1.pressing()){
    pivot.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  } else if (Controller1.ButtonR2.pressing()){
    pivot.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  } else {
    pivot.stop(vex::brakeType::coast);
  }
}

void driverIntakeControl(){ // Controls the flapped intake on the left side analog buttons.
  if(Controller1.ButtonL1.pressing()){
    intake1.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  } else if (Controller1.ButtonL2.pressing()){
    intake1.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
    intake2.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  } else {
    intake1.stop(vex::brakeType::hold);
    intake2.stop(vex::brakeType::hold);
  }
}

void driverArmLiftControl(){ // Controls the arm of the robot using the left side joystick.
  int axisValue = Controller1.Axis4.value();
  if(axisValue > controllerTollerance || axisValue < controllerTollerance){
    armLift.spin(vex::directionType::fwd, controllerTollerance, vex::velocityUnits::pct);
    } else {
    armLift.stop(vex::brakeType::hold);
  }
}

void pre_auton( void ) {
  stopAllMotors();
}

void autonomous( void ) {
  stopAllMotors();
}

void usercontrol( void ) { // Method that is called at the start of the driver controlled period of the match.
  while (true) {
    driverArmLiftControl();
    driverDriveControl();
    driverIntakeControl();
    driverpivotControl();
  }
}

int main() {
    Competition.autonomous( autonomous );
    Competition.drivercontrol( usercontrol );
    pre_auton();                      
    while(1) {
      vex::task::sleep(100);
    }    
       
}