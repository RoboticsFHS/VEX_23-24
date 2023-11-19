using namespace vex;

extern brain Brain;

// VEXcode devices
extern smartdrive Drivetrain;
extern distance Distance;
extern rotation Rotation;
extern motor Intake;
extern gps GPS;
extern motor Arm;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );