#include "vex.h"

using namespace vex;


brain Brain;

//Controller
competition Competition = competition();
controller  Controller1 = controller();

//Motors
/** 
BLUE = 6:1
GREEN = 18:1
RED = 36:1
**/
motor chassisLF = motor(vex::PORT1, vex::gearSetting::ratio18_1, false);
motor chassisLB = motor(vex::PORT2, vex::gearSetting::ratio18_1, true);
motor chassisRF = motor(vex::PORT5, vex::gearSetting::ratio18_1, true);
motor chassisRB = motor(vex::PORT6, vex::gearSetting::ratio18_1, false);


motor flywheel1 = motor(vex::PORT14, vex::gearSetting::ratio6_1, true);
motor flywheel2 = motor(vex::PORT15, vex::gearSetting::ratio6_1, false);
motor intake1 = motor(vex::PORT11, vex::gearSetting::ratio6_1, true);
motor intake2 = motor(vex::PORT12, vex::gearSetting::ratio6_1, false);
motor triggerM = motor(vex::PORT13, vex::gearSetting::ratio18_1, false);


//pneumatics 
/**
0 is close, 1 is open 
**/

void vexcodeInit(void) {
}