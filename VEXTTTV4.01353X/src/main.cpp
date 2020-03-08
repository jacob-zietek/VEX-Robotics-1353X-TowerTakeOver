#include "vex.h"

using namespace vex;

competition Competition;

#include "vex.h"
#include <cmath>

using namespace vex;

//vex::brain Brain;

vex::controller Controller1 = vex::controller();

vex::motor backLeft = vex::motor(vex::PORT19, vex::gearSetting::ratio18_1, false);
vex::motor backRight = vex::motor(vex::PORT10, vex::gearSetting::ratio18_1, true);
vex::motor frontLeft = vex::motor(vex::PORT18, vex::gearSetting::ratio18_1, true);
vex::motor frontRight = vex::motor(vex::PORT5, vex::gearSetting::ratio18_1, false);
vex::motor armLift = vex::motor(vex::PORT4, vex::gearSetting::ratio36_1, false);
vex::motor pivot = vex::motor(vex::PORT15, vex::gearSetting::ratio36_1, true);
vex::motor intake1 = vex::motor(vex::PORT6, vex::gearSetting::ratio18_1, true);
vex::motor intake2 = vex::motor(vex::PORT9, vex::gearSetting::ratio18_1, false);

vex::inertial    Inertial( vex::PORT12 );

//vex::light       lightSensor( Brain.ThreeWirePort.A );

motor_group   leftDrive( frontLeft, backLeft );
motor_group   rightDrive(frontRight, backRight);

smartdrive    robotDrive( leftDrive, rightDrive, Inertial, (4 * M_PI) , 11, (9 * (5/8)), distanceUnits::in );

int controllerTollerance = 15; // 0-100 value assigned to create a deadzone in the joysticks, prevents accidental activation.

void stopAllMotors(){ // A method used to stop every motor, used in autonomous programs and unique circumstances.
  backLeft.stop(vex::brakeType::coast);
  backRight.stop(vex::brakeType::coast);
  frontLeft.stop(vex::brakeType::coast);
  frontRight.stop(vex::brakeType::coast);
  armLift.stop(vex::brakeType::hold);
  pivot.stop(vex::brakeType::coast);
  intake1.stop(vex::brakeType::brake);
  intake2.stop(vex::brakeType::brake);
}

void autoDriveTime(int left, int right, int t){ 
    backRight.spin(directionType::fwd, right, velocityUnits::pct);
    backLeft.spin(directionType::fwd, left, velocityUnits::pct);
    frontRight.spin(directionType::fwd, right, velocityUnits::pct);
    frontLeft.spin(directionType::fwd, left, velocityUnits::pct);
    vex::task::sleep(t);
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
    vex::task::sleep(20);
}

void autoDriveP(int desiredValue){
    float kP = .125;
    float PIDcorrection;
    float error;
    int maxSpeed = 40;
    backRight.resetRotation();
    while(fabs(backRight.rotation(rotationUnits::deg)) < abs(desiredValue)){
        error = backRight.rotation(rotationUnits::deg) - desiredValue;
        PIDcorrection = error * kP;
        if(fabs(error) < 50){
            if(desiredValue > 0)
                PIDcorrection = -10;
            if(desiredValue < 0)
                PIDcorrection = 10;
        }
        if(PIDcorrection > maxSpeed){
          PIDcorrection = maxSpeed;
        }
        if(PIDcorrection < -maxSpeed){
          PIDcorrection = -maxSpeed;
        }
        frontLeft.spin(directionType::fwd,  -PIDcorrection, velocityUnits::pct);
        backLeft.spin(directionType::fwd,   -PIDcorrection, velocityUnits::pct);
        frontRight.spin(directionType::fwd, -PIDcorrection, velocityUnits::pct);
        backRight.spin(directionType::fwd,  -PIDcorrection, velocityUnits::pct);
        task::sleep(1);
     }
     frontLeft.stop();
     frontRight.stop();
     backLeft.stop();
     backRight.stop();
}

void autoDriveEncoder(int left, int right, int e){ 
    backLeft.resetRotation();
    backRight.resetRotation();
    backRight.spin(directionType::fwd, left, velocityUnits::pct);
    backLeft.spin(directionType::fwd, left, velocityUnits::pct);
    frontRight.spin(directionType::fwd, right, velocityUnits::pct);
    frontLeft.spin(directionType::fwd, right, velocityUnits::pct);
    int averageEncoder = (fabs(backLeft.rotation(vex::rotationUnits::deg)) + fabs(backRight.rotation(vex::rotationUnits::deg)))/2;
    while(averageEncoder < e){
      vex::task::sleep(3);
      averageEncoder = (fabs(backLeft.rotation(vex::rotationUnits::deg)) + fabs(backRight.rotation(vex::rotationUnits::deg)))/2;
    }
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
    vex::task::sleep(20);
}

void autoTurnDeg(int deg){
  robotDrive.setHeading(0, vex::rotationUnits::deg);
  robotDrive.turnToHeading(deg, vex::rotationUnits::deg);
}

void driverDriveControl(){ // Controls the drive train movement during the autonomous period. Arcade style movement on right joystick.
  if((abs(Controller1.Axis4.value()) > controllerTollerance) || (abs(Controller1.Axis3.value()) > controllerTollerance)){
    int leftSideValue = (Controller1.Axis3.value() + Controller1.Axis4.value())/1;
    int rightSideValue = (Controller1.Axis3.value() - Controller1.Axis4.value())/1;
    backLeft.spin(vex::directionType::fwd, leftSideValue, vex::velocityUnits::pct);
    frontLeft.spin(vex::directionType::fwd, leftSideValue, vex::velocityUnits::pct);
    backRight.spin(vex::directionType::fwd, rightSideValue, vex::velocityUnits::pct);
    frontRight.spin(vex::directionType::fwd, rightSideValue, vex::velocityUnits::pct);
  } else {
    backLeft.stop(vex::brakeType::coast);
    backRight.stop(vex::brakeType::coast);
    frontLeft.stop(vex::brakeType::coast);
    frontRight.stop(vex::brakeType::coast);
  }
}

void driverpivotControl(){ // Controls the pivot (ramp) on the right side analog buttons.
  if(Controller1.ButtonR1.pressing()){
    pivot.spin(vex::directionType::fwd, 35, vex::velocityUnits::pct);
  } else if (Controller1.ButtonR2.pressing()){
    pivot.spin(vex::directionType::fwd, -35, vex::velocityUnits::pct);
  } else {
    pivot.stop(vex::brakeType::hold);
  }
}

void driverIntakeControl(){ // Controls the flapped intake on the left side analog buttons.
  if(Controller1.ButtonL1.pressing()){
    intake1.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  } else if (Controller1.ButtonL2.pressing()){
    intake1.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
    intake2.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
    Controller1.rumble(".");
  } else if (Controller1.ButtonX.pressing()) {
    intake1.spin(vex::directionType::fwd, -32, vex::velocityUnits::pct);
    intake2.spin(vex::directionType::fwd, -32, vex::velocityUnits::pct);
  } else {
    intake1.stop(vex::brakeType::hold);
    intake2.stop(vex::brakeType::hold);
  }
}

void driverArmLiftControl(){ // Controls the arm of the robot using the left side joystick.
  int axisValue = Controller1.Axis2.value();
  if(axisValue > controllerTollerance || axisValue < controllerTollerance){
    armLift.spin(vex::directionType::fwd, axisValue, vex::velocityUnits::pct);
    } else {
    armLift.stop(vex::brakeType::hold);
  }
}

void deploy(){
  intake1.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  pivot.spin(vex::directionType::fwd, -35, vex::velocityUnits::pct);
  armLift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  vex::task::sleep(600);
  pivot.stop();
  vex::task::sleep(250);
  armLift.spin(vex::directionType::fwd, -50, vex::velocityUnits::pct);
  vex::task::sleep(600);
  intake1.stop();
  intake2.stop();
  //armLift.stop();
  pivot.spin(vex::directionType::fwd, 35, vex::velocityUnits::pct);
  vex::task::sleep(750);
  pivot.stop();
  armLift.stop();
}

/*
void sleep(int t){
  vex::task::sleep(t);
}
*/
void pre_auton( void ) {
  //Inertial.calibrate(); 
  stopAllMotors();
}

void pushCube(){
  stopAllMotors();
  autoDriveEncoder(50, 50, 900);
  autoDriveEncoder(-50, -50, 900);
  stopAllMotors();
}

void dropStack(){
  intake1.spin(vex::directionType::fwd, -65, vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd, -65, vex::velocityUnits::pct);
  vex::task::sleep(500);
  intake1.stop();
  intake2.stop();
  pivot.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  vex::task::sleep(800);
  pivot.spin(vex::directionType::fwd, -30, vex::velocityUnits::pct);
  vex::task::sleep(1200);
  pivot.stop(vex::brakeType::hold);
}

void blueUnprotected(){
  deploy();
  intake1.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  armLift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  vex::task::sleep(275);
  armLift.stop();
  autoDriveEncoder(20, 20, 1050);
  vex::task::sleep(200);
  autoDriveEncoder(-50, -50, 500);
  autoDriveTime(-25, 25, 1350);
  vex::task::sleep(100);
  autoDriveTime(70, 70, 1250);
  dropStack();
  intake1.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  autoDriveTime(-50, -50, 1000);
  stopAllMotors();
}


void redUnprotected(){
  deploy();
  intake1.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  armLift.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
  vex::task::sleep(275);
  armLift.stop();
  autoDriveEncoder(20, 20, 1050);
  vex::task::sleep(200);
  autoDriveEncoder(-50, -50, 500);
  autoDriveTime(25, -25, 1350);
  vex::task::sleep(100);
  autoDriveTime(70, 70, 1250);
  dropStack();
  intake1.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  intake2.spin(vex::directionType::fwd, -100, vex::velocityUnits::pct);
  autoDriveTime(-50, -50, 1000);
  stopAllMotors();
}

void autonomous( void ) {
  backLeft.stop(vex::brakeType::hold);
  backRight.stop(vex::brakeType::hold);
  frontLeft.stop(vex::brakeType::hold);
  frontRight.stop(vex::brakeType::hold);
  //dropStack();
  //redUnprotected();
  pushCube();
  //autoTurnDeg(82);
  //autoDriveP(1000);
  //deploy();
}

void usercontrol( void ) { // Method that is called at the start of the driver controlled period of the match.
  stopAllMotors();
  while (true) {
    driverArmLiftControl();
    driverDriveControl();
    driverIntakeControl();
    driverpivotControl();
    if(Controller1.ButtonA.pressing()){
      deploy();
    }
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