/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Peter Voong and greydon ting                              */
/*    Created:      Fri Dec 3 2021                                            */
/*    Description:  8000E V2 - Pre-Tournament Code                            */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "math.h"
#include "vex.h"

using namespace vex;

//-------------------------------------------------------variables and
//helpers---------------------------------------------------------------------------------

// Middle of wheel to middle of wheel
const double CHASSIS_WIDTH = 15;
const double CHASSIS_LENGTH = 15;
const double WHEEL_RADIUS = 3.25 / 2.0;
const double pi = 3.1415927;
const double wheelCircumference = 2 * WHEEL_RADIUS * pi;
const double gearRatio = 1;

// Declarations:
const int encoderTicksPerRev = 300;
const double ticksPerTurn = 1000; // tbd value through testing
const double turningRadius =
    pow(pow(CHASSIS_LENGTH, 2) + pow(CHASSIS_WIDTH, 2), .5) / 2.0;

const double LEFT_SENS = 1;
const double RIGHT_SENS = 1;

//-------------------------------conversion
//methods-------------------------------------------------------------------------------
double inchToRotation(double inches) {

  double temp = (inches / (wheelCircumference));
  return temp;
}

double degreesToRadians(double degrees) {
  double temp = degrees * pi;
  return temp / 180.0;
}

//-------------------------------chassis
//methods-------------------------------------------------------------------------------

void leftDrive(vex::directionType type, int percentage) {
  chassisLF.spin(type, percentage, velocityUnits::pct);
  chassisLB.spin(type, percentage, velocityUnits::pct);
}

void rightDrive(vex::directionType type, int percentage) {
  chassisRF.spin(type, percentage, velocityUnits::pct);
  chassisRB.spin(type, percentage, velocityUnits::pct);
}

void leftMoveFor(double velocity, double inches) {
  double rotations = inchToRotation(inches);

  chassisLF.rotateFor(fwd, rotations, rotationUnits::rev, velocity,
                      velocityUnits::rpm, false);
  chassisLB.rotateFor(fwd, rotations, rotationUnits::rev, velocity,
                      velocityUnits::rpm, false);
}

void rightMoveFor(double velocity, double inches) {
  double rotations = inchToRotation(inches);

  chassisRF.rotateFor(fwd, rotations, rotationUnits::rev, velocity,
                      velocityUnits::rpm, false);
  chassisRB.rotateFor(fwd, rotations, rotationUnits::rev, velocity,
                      velocityUnits::rpm, false);
}

void setChassisBrakeType(vex::brakeType type = coast) {
  chassisLF.setStopping(type);
  chassisLB.setStopping(type);
  chassisRF.setStopping(type);
  chassisRB.setStopping(type);
}
void leftSpin(double velocity) {
  chassisLF.setStopping(vex::brakeType::coast);
  chassisLB.setStopping(vex::brakeType::coast);

  chassisLF.setVelocity(velocity, velocityUnits::rpm);
  chassisLB.setVelocity(velocity, velocityUnits::rpm);

  chassisLF.spin(vex::directionType::fwd);
  chassisLB.spin(vex::directionType::fwd);
}

void rightSpin(double velocity) {
  chassisRF.setStopping(vex::brakeType::coast);
  chassisRB.setStopping(vex::brakeType::coast);

  chassisRF.setVelocity(velocity, velocityUnits::rpm);
  chassisRB.setVelocity(velocity, velocityUnits::rpm);

  chassisRF.spin(vex::directionType::fwd);
  chassisRB.spin(vex::directionType::fwd);
}

void turn(double degrees, int velocity) {
  double inches = (pi * turningRadius * degrees) /
                  360; // simplifies to pi*turningRadius*degrees/180
  // inches = inches/2; //because for point turn, left and right side move, so
  // you don't want to run it twice
  double rotation = inches / wheelCircumference;

  chassisLF.rotateFor(rotation, rotationUnits::rev, velocity,
                      velocityUnits::pct, false);
  chassisLB.rotateFor(rotation, rotationUnits::rev, velocity,
                      velocityUnits::pct, false);

  chassisRB.rotateFor(-rotation, rotationUnits::rev, velocity,
                      velocityUnits::pct, false);
  chassisRF.rotateFor(-rotation, rotationUnits::rev, velocity,
                      velocityUnits::pct, true);
}

//-------------------------------auton----------------------------------------------------------------------------------

// PID Methods:
// unit conversion from inches to encoder ticks
int inch2Tick(float inch) {
  int ticks;
  ticks = inch * encoderTicksPerRev / (wheelCircumference);
  return ticks;
}

// function getting the sign of number
int signNum(double num) {
  if (num < 0)
    return -1;
  else if (num > 0)
    return 1;
  else
    return 0;
}

// unit conversion from degree angle to encoder ticks
// this function requires measurement of # ticks per robot turn
int degree2Tick(float degree) { // need to measure based on robot
  int ticks = degree * ticksPerTurn / encoderTicksPerRev;
  return ticks;
}

// unit conversion from degree angle to encoder ticks
// this function is based on wheel travel trajectory during a turn
// need fine tune of turningRadius to get correct angle conversion
int degree2Tick_2(float degree) {
  float turningCirc = 2 * pi * turningRadius;
  double ticksPerTurn = inch2Tick(turningCirc);
  int ticks = degree * ticksPerTurn / 360;
  return ticks;
}

// time delay at the end of a chassis move. Minumum is 250msec regardless what
// the setting is
int waitTime_msec(double rawSeconds) {
  int miliSeconds;
  miliSeconds = rawSeconds * 1000;
  if (miliSeconds < 150) // was 250
  {
    miliSeconds = 150; // was 250
  }
  return miliSeconds;
}

double Limit(double val, double min, double max) {
  if (val > max) {
    return max;
  } else if (val < min) {
    return min;
  } else {
    return val;
  }
}

//------------------------------------PID Control Move and
//Turn----------------------------------------

void setChassisLSmooth(int speed) {
  double inertia = 0.97; // was 0.5
  static int currentSpeed = 0;
  currentSpeed = inertia * currentSpeed + (1 - inertia) * speed;
  leftSpin(currentSpeed);
}

void setChassisRSmooth(int speed) {
  double inertia = 0.97; // was 0.5
  static int currentSpeed = 0;
  currentSpeed = inertia * currentSpeed + (1 - inertia) * speed;
  rightSpin(currentSpeed);
}

double kP = 1;  // was 1
double kD = 0.4; // was 1 , 0.5

void chassisPIDMoveMax(double inches, double maxSpeed) {
  double revolutions =
      inches / (4 * pi); // wheel circumference. Assumes radius of 2 in.
  double degrees = revolutions * 360; // How many degrees the wheels need to
                                      // turn

  // double mtrDegrees = (degrees * 6) / 5;//How many degrees the motors need to
  // spin

  chassisLF.resetRotation();
  chassisRF.resetRotation();

  double distanceL = chassisLF.rotation(rotationUnits::deg);
  double distanceR = chassisRF.rotation(rotationUnits::deg);

  double errorL = degrees - distanceL;
  double errorR = degrees - distanceL;

  double lastErrorL = errorL;
  double lastErrorR = errorR;

  double proportionalL;
  double proportionalR;

  double speedL;
  double speedR;

  double derivativeL;
  double derivativeR;

  double powerL;
  double powerR;

  while (fabs(errorL) > 20 || fabs(errorR) > 20) {
    distanceL = chassisLF.rotation(rotationUnits::deg);
    distanceR = chassisRF.rotation(rotationUnits::deg);

    errorL = degrees - distanceL;
    errorR = degrees - distanceL;

    proportionalL = errorL * kP;
    proportionalR = errorR * kP;

    speedL = lastErrorL - errorL;
    speedR = lastErrorR - errorR;

    derivativeL = -speedL * kD;
    derivativeR = -speedR * kD;

    lastErrorL = errorL;
    lastErrorR = errorR;

    powerL = Limit((proportionalL + derivativeL) * 0.6, maxSpeed * -1,
                   maxSpeed); //* .6
    powerR = Limit((proportionalR + derivativeR) * 0.6, maxSpeed * -1,
                   maxSpeed); //* .6

    setChassisLSmooth(powerL);
    setChassisRSmooth(powerR);

    vex::task::sleep(10);
  }

  leftSpin(0);
  rightSpin(0);
}

void chassisPIDMove(double inches) {

  double revolutions = inches / (wheelCircumference); // wheel circumference.
  double degrees = revolutions * 360; // How many degrees the wheels need to
                                      // turn

  // double mtrDegrees = (degrees * 6) / 5;//How many degrees the motors need to
  // spin

  chassisLF.resetRotation();
  chassisRF.resetRotation();

  double distanceL = chassisLF.rotation(rotationUnits::deg);
  double distanceR = chassisRF.rotation(rotationUnits::deg);

  double errorL = degrees - distanceL;
  double errorR = degrees - distanceL;

  double lastErrorL = errorL;
  double lastErrorR = errorR;

  double proportionalL;
  double proportionalR;

  double speedL;
  double speedR;

  double derivativeL;
  double derivativeR;

  double powerL;
  double powerR;

  while (fabs(errorL) > 20 || fabs(errorR) > 20) {
    distanceL = chassisLF.rotation(rotationUnits::deg);
    distanceR = chassisRF.rotation(rotationUnits::deg);

    errorL = degrees - distanceL;
    errorR = degrees - distanceL;

    proportionalL = errorL * kP;
    proportionalR = errorR * kP;

    speedL = lastErrorL - errorL;
    speedR = lastErrorR - errorR;

    derivativeL = -speedL * kD;
    derivativeR = -speedR * kD;

    lastErrorL = errorL;
    lastErrorR = errorR;

    powerL = Limit((proportionalL + derivativeL), -600, 600);
    powerR = Limit((proportionalR + derivativeR), -600, 600);

    setChassisLSmooth(powerL);
    setChassisRSmooth(powerR);

    vex::task::sleep(10);
  }

  leftSpin(0);
  rightSpin(0);
}

//-------------------------------------------------------buttons---------------------------------------------------------------------------------

void upPressed() {

}

void downPressed() {}
void leftPressed() {}

void rightPressed() {}

void xPressed() { // go down
}

void yPressed() { // go up
  intake1.setVelocity(100, velocityUnits::pct);
  intake2.setVelocity(100, velocityUnits::pct);
}

// void xPressed() {

// }

// void yPressed() {

// }

void aPressed() { // go down further
  intake1.setVelocity(100, velocityUnits::pct);
  intake2.setVelocity(100, velocityUnits::pct);
}

void bPressed() { // go down
  intake1.setVelocity(100, velocityUnits::pct);
  intake2.setVelocity(100, velocityUnits::pct);
}

void chassisControl() {
  int axis3 = Controller1.Axis3.position();
  int axis2 = Controller1.Axis2.position();

  leftDrive(vex::directionType::fwd, axis3);
  rightDrive(vex::directionType::fwd, axis2);
}

void trigger() {
    triggerM.setStopping(hold);
    triggerM.spinToPosition(25, deg, 100, velocityUnits::pct, true);
    wait(500, msec);
    triggerM.spinToPosition(0, deg, 100, velocityUnits::pct, false);
}

// void flywheelStart(int power) {
//     // flywheel.setVelocity(600, velocityUnits::rpm);
//     flywheel1.spin(fwd, power, velocityUnits::pct);
//     flywheel2.spin(fwd, power, velocityUnits::pct);
// }

void flywheelStart() {
    // flywheel.setVelocity(600, velocityUnits::rpm);
    flywheel1.spin(fwd, 100, velocityUnits::pct);
    flywheel2.spin(fwd, 100, velocityUnits::pct);
}


void flywheelStop() {
    // flywheel.setBrake(coast);
    flywheel1.stop(coast);
    flywheel2.stop(coast);
}

void intakeStart() {
   intake1.spin(fwd, 100, velocityUnits::pct); 
   intake2.spin(fwd, 100, velocityUnits::pct);
}

void intakeStop() {
   intake1.stop(coast);
   intake2.stop(coast);
}

void outtakeStart() {
   intake1.spin(reverse, 100, velocityUnits::pct);
   intake2.spin(reverse, 100, velocityUnits::pct);
}


int intake(void){
  task::sleep(100);
  intakeStart();
  task::sleep(300);
  intakeStop();
  return 0;
}


// Auton Methods
void Auton1(void) {
   task i = task(intake); //intake that will be used later
  //spin roller (code is not set up yet)
  turn(75, 100); // turn to goal
  flywheel1.setVelocity(400, velocityUnits::rpm); //set velocity of both flywheel motors
  flywheel2.setVelocity(400, velocityUnits::rpm);
  flywheelStart(); //shoot
  vex::task::sleep(250);
  turn(45, 100); // turn to the where the disks are set
  i.resume(); //intake while driving
  chassisPIDMove(136); //drive across line
  turn(-90, 100);
  chassisPIDMove(-4);
  // spin roller
}

void autonomous(void) { 
  Auton1();
}



//-------------------------------------------------------callbacks---------------------------------------------------------------------------------

void usercontrol(void) {
  setChassisBrakeType(coast);

  //flywheel
  Controller1.ButtonL1.pressed(*flywheelStart);

  // if(Controller1.ButtonL1.pressing()){flywheelStart;}
  Controller1.ButtonL1.released(*flywheelStop);

  //trigger
  Controller1.ButtonL2.pressed(*trigger);

  //intake
  Controller1.ButtonR1.pressed(*intakeStart);
  Controller1.ButtonR1.released(*intakeStop);

  //outtake
  Controller1.ButtonR2.pressed(*outtakeStart);
  Controller1.ButtonR2.released(*intakeStop);

 
  
  Controller1.ButtonUp.pressed(*upPressed);
  Controller1.ButtonDown.pressed(*downPressed);
  Controller1.ButtonLeft.pressed(*leftPressed);
  Controller1.ButtonRight.pressed(*rightPressed);
  Controller1.ButtonX.pressed(*xPressed);
  Controller1.ButtonY.pressed(*yPressed);
  Controller1.ButtonA.pressed(*aPressed);
  Controller1.ButtonB.pressed(*bPressed);
  Controller1.Axis2.changed(*chassisControl);
  Controller1.Axis3.changed(*chassisControl);

  // game tick
  while (1) {
//     if (Controller1.ButtonY.pressing()) {
//       liftL.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct);
//       liftR.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct );
//     }
//      if (Controller1.ButtonX.pressing()) {
//       liftL.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
//       liftR.spin(vex::directionType::fwd, 50, vex::velocityUnits::pct );
    
//     }

// else {
//   liftL.stop(vex::brakeType::hold);
//   liftR.stop(vex::brakeType::hold);
// }

    vex::task::sleep(20); // DO NOT DELETE -- Sleep the task for a short amount
                          // of time to prevent wasted resources.
  }
}

//-------------------------------------------------------main-------------------------------------------------------------------------------

int main() {
  vexcodeInit();
  triggerM.setTimeout(0.2, timeUnits::sec);
  //triggerM.spinToPosition(double rotation, rotationUnits units);
  triggerM.setTimeout(1, timeUnits::sec);
  // gyroSensor.startCalibration(1500);
  Competition.drivercontrol(usercontrol);
  Competition.autonomous(autonomous);
}