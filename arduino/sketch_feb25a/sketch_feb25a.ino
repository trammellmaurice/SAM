
// Project SAM
// MIDN 1/C Anthony Chase-Hill
// MIDN 1/C Maurice Trammell
// MIDN 1/C Christopher Meacham
// Capstone Advisor - Capt. Caliph Lebrun, USMC

#include <Arduino.h>
#include <Servo.h>

// ============================================================================
// Define global variables
// ============================================================================

// Define Servos
Servo trigger; // Create servo object to control trigger servo              - Position Servo
Servo yaw;     // Create servo object to control yaw (turret base movement) - Continuous Rotation Servo
Servo pitch;   // Create servo object to control pitch                      - Continuous Rotation Servo

// Servo controller variables

int input_up         = 7;  // Pin that will read input Nano "Up" output
int input_down       = 6;  // Pin that will read input Nano "Down" output
int input_right      = 5;  // Pin that will read input Nano "Right" output
int input_left       = 4;  // Pin that will read input Nano "Left" output
int input_slow       = 3;  // Pin that will read input Nano "Left" output
int input_fire       = 2;  // Pin that will read input Nano "Fire" output


int input_val_up        = 0;  // Value for "Up" pin
int input_val_down      = 0;  // Value for "Down" pin
int input_val_right     = 0;  // Value for "Right" pin
int input_val_left      = 0;  // Value for "Left" pin
int input_val_slow      = 0;  // Value for "Slow" pin
int input_val_fire      = 0;  // Value for "Fire" pin

int curr_pos               = 0;
int prev_pos;
int clockwise              = 0;  // Used to rotate continuous servos clockwise (full speed)
int counterclockwise       = 180;// Used to rotate continuous servos counterclockwise (full speed)
int slowclockwise          = 45;  // Used to rotate continuous servos clockwise (full speed)
int slowcounterclockwise   = 135;// Used to rotate continuous servos counterclockwise (full speed)
int cc_stop                = 90; // Used to stop continuous servos
int pull                   = 130;// When trigger needs to be pulled, trigger servo moves
int trigger_ready          = 90; // When trigger does not need to be pulled, it rests in a ready position
int trigger_disengage      = 70; // When system is shutting down, move as far away from trigger as possible to prevent misfires
int fire                   = 0;  // Set fire equal to 1 when the trigger needs to be pulled

// Function prototypes
void startup_fcn();
void fire_fcn();
void controller_fcn();
void shutdown_fcn();


// ============================================================================
// Setup Process
// ============================================================================
void setup() {
  trigger.attach(8); // Attach servo to pin 8  - Trigger Pin
  pitch.attach(9);   // Attach servo to pin 9  - Pitch
  yaw.attach(10);    // Attach servo to pin 10 - Yaw (Turret base)

  pinMode(input_up,    INPUT);
  pinMode(input_down,  INPUT);
  pinMode(input_right, INPUT);
  pinMode(input_left,  INPUT);
  pinMode(input_slow,  INPUT);
  pinMode(input_fire,  INPUT);
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {

  startup_fcn();

  while(1) {
    controller_fcn();
  }

  shutdown_fcn(); // Once prompted to, shutdown the system

}

// ============================================================================
// Startup Function
// ============================================================================
void startup_fcn() {
  trigger.write(trigger_disengage);
  delay(2000);
  yaw.write(90);
  pitch.write(90);
  trigger.write(trigger_ready);
  delay(2000);
}

// ============================================================================
// Firing Function
// ============================================================================
void fire_fcn() {
  if(fire=1){
    trigger.write(pull);
    delay(300); // Wait to give servo time to complete motion
    trigger.write(trigger_ready);
  }
  else{
    trigger.write(trigger_ready);
  }
}

// ============================================================================
// Controller Function
// ============================================================================
void controller_fcn() {

input_val_up = digitalRead(input_up);
input_val_down = digitalRead(input_down);
input_val_right = digitalRead(input_right);
input_val_left = digitalRead(input_left);
input_val_slow = digitalRead(input_slow);
input_val_fire = digitalRead(input_fire);

  if (input_val_fire == HIGH){
    fire = 1;
    fire_fcn();
  }


  if (input_val_up == HIGH){
    if (input_val_slow == HIGH){
      pitch.write(slowcounterclockwise);
    }
    else{
      pitch.write(counterclockwise);
    }
  }
  else if (input_val_down == HIGH){
    if (input_val_slow == HIGH){
      pitch.write(slowclockwise);
    }
    else{
      pitch.write(clockwise);
    }
  }
  else if (input_val_up == LOW && input_val_down == LOW){
    pitch.write(cc_stop);
  }


  if (input_val_left == HIGH){
    if (input_val_slow == HIGH){
      yaw.write(slowcounterclockwise);
    }
    else{
      yaw.write(counterclockwise);
    }
  }
  else if (input_val_right == HIGH){
    if (input_val_slow == HIGH){
      yaw.write(slowclockwise);
    }
    else{
      yaw.write(clockwise);
    }
  }
  else if (input_val_right == LOW && input_val_left == LOW){
    yaw.write(cc_stop);
  }
}

// ============================================================================
// Shutdown Function
// ============================================================================
void shutdown_fcn() {
  delay(1000);
  trigger.write(trigger_disengage);
  yaw.write(90);
  pitch.write(90);
  while(1){
    }
}

// ============================================================================
// Reset Continuous Servo Position With This Code As Applicable
// ============================================================================

//pitch.write(clockwise);
//delay(1000);
//pitch.write(counterclockwise);
//delay(1000);
//pitch.write(cc_stop);

//yaw.write(clockwise);
//delay(1000);
//yaw.write(counterclockwise);
//delay(1000);
//yaw.write(cc_stop);
