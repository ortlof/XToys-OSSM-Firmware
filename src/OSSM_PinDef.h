/*
        Driver pins
*/
// Pin that pulses on servo/stepper steps - likely labelled PUL on drivers.
#define MOTOR_STEP_PIN 14
// Pin connected to driver/servo step direction - likely labelled DIR on drivers.
// N.b. to iHSV57 users - DIP switch #5 can be flipped to invert motor direction entirely
#define MOTOR_DIRECTION_PIN 27
// Pin for motor enable - likely labelled ENA on drivers.
#define MOTOR_ENABLE_PIN 26

/*
    Homing and safety pins
*/
// define the IO pin where the limit(homing) switch(es) are connected to (switches in
// series in normally closed setup) Switches wired from IO pin to ground.
#define LIMIT_SWITCH_PIN 12