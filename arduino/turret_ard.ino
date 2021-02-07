// Project SAM
// MIDN 1/C Anthony Chase-Hill
// MIDN 1/C Maurice Trammell
// MIDN 1/C Christopher Meacham
// Capstone Advisor - Capt. Caliph Lebrun, USMC


#include <Arduino.h>
#include <SoftwareSerial.h>
#include <HardwareSerial.h>
#include <Servo.h>



// ============================================================================
// Define global variables
// ============================================================================


// Define Servos
Servo trigger; // Create servo object to control trigger servo              - Position Servo
Servo yaw;     // Create servo object to control yaw (turret base movement) - Continuous Rotation Servo
Servo pitch;   // Create servo object to control pitch                      - Continuous Rotation Servo


// Servo controller variables
int jetson_fire       = 7;  // Pin that will read Jetson Nano "Fire" output
int jetson_up         = 6;  // Pin that will read Jetson Nano "Up" output
int jetson_down       = 5;  // Pin that will read Jetson Nano "Down" output
int jetson_right      = 4;  // Pin that will read Jetson Nano "Right" output
int jetson_left       = 3;  // Pin that will read Jetson Nano "Left" output
int jetson_val_7      = 0;  // Value for "Fire" pin
int jetson_val_6      = 0;  // Value for "Up" pin
int jetson_val_5      = 0;  // Value for "Down" pin
int jetson_val_4      = 0;  // Value for "Right" pin
int jetson_val_3      = 0;  // Value for "Left" pin
int curr_pos          = 0;
int prev_pos;
int cw                = 0;  // Used to rotate continuous servos clockwise (full speed)
int ccw               = 180;// Used to rotate continuous servos counterclockwise (full speed)
int cc_stop           = 90; // Used to stop continuous servos
int pull              = 160;// When trigger needs to be pulled, trigger servo moves to 20 degrees
int trigger_ready     = 90; // When trigger does not need to be pulled, it rests in a ready position (40 degrees)
int trigger_disengage = 90; // When system is shutting down, move as far away from trigger as possible to prevent misfires (90 degrees)
int fire              = 0;  // Set fire equal to 1 when the trigger needs to be pulled


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
  pinMode(jetson_fire,  INPUT);
  pinMode(jetson_up,    INPUT);
  pinMode(jetson_down,  INPUT);
  pinMode(jetson_right, INPUT);
  pinMode(jetson_left,  INPUT);
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {

  startup_fcn();

  while(1) {
   controller_fcn();
  }
  //fire =1;
  //fire_fcn();

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


jetson_val_7 = digitalRead(jetson_fire);
jetson_val_6 = digitalRead(jetson_up);
jetson_val_5 = digitalRead(jetson_down);
jetson_val_4 = digitalRead(jetson_right);
jetson_val_3 = digitalRead(jetson_left);

  if (jetson_val_7 == HIGH){
    fire = 1;
    fire_fcn();
  }


  if (jetson_val_6 == HIGH){
    pitch.write(ccw);
    delay(100);
    pitch.write(cc_stop);
  }

  if (jetson_val_5 == HIGH){
    pitch.write(cw);
    delay(100);
    pitch.write(cc_stop);
  }


  if (jetson_val_4 == HIGH){
    yaw.write(cw);
    delay(100);
    yaw.write(cc_stop);
  }


  if (jetson_val_3 == HIGH){
    yaw.write(ccw);
    delay(100);
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


//pitch.write(cw);
//delay(1000);
//pitch.write(ccw);
//delay(1000);
//pitch.write(cc_stop);


//yaw.write(cw);
//delay(1000);
//yaw.write(ccw);
//delay(1000);
//yaw.write(cc_stop);
Turret_trigger.txt
Displaying Turret_trigger.txt.
