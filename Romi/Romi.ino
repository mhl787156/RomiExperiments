/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Library Includes.
   Be sure to check each of these to see what variables/functions are made
   global and accessible.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "pins.h"
#include "utils.h"
#include "motors.h"
#include "pid.h"
#include "interrupts.h"
#include "kinematics.h"
#include "line_sensors.h"
#include "irproximity.h"
#include "beliefmapping.h"
#include "mapping.h"
#include "RF_Interface.h"
#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"
#include "Pushbutton.h"
#include "planner.h"

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Definitions.  Other definitions exist in the .h files above.
   Also ensure you check pins.h for pin/device definitions.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define BAUD_RATE 9600


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Class Instances.
   This list is complete for all devices supported in this code.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
Kinematics    Pose; //Kinematics class to store position and heading

LineSensor    LineCentre(LINE_CENTRE_PIN); //Centre line sensor

SharpIR       LeftIR(LEFT_IR_PIN, 50620 , -0.9465 , 54.72 );
SharpIR       CentreIR(CENTRE_IR_PIN, 46440 , -0.9306 , 48.85 ); //Distance sensor
SharpIR       RightIR(RIGHT_IR_PIN, 36740 , -0.8698 , 27.36 );

// Magnetometer  Mag; // Class for the magnetometer

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftPosControl(.15, 0.04, 0.000) ;
PID           RightPosControl(.15, 0.04, 0.000) ;
PID           LeftSpeedControl( 3.5, 20.9, 0 );
PID           RightSpeedControl( 3.5, 20.9, 0 );
PID           HeadingControl( 92 , 0, 0.00 );

BeliefMapper  Map; //Class for representing the map

Pushbutton    ButtonA( BUTTON_A, DEFAULT_STATE_HIGH);
Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

Planner       MotionPlanner(Map);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Global variables.
   These global variables are not mandatory, but are used for the example loop()
   routine below.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//Use these variables to set the demand of the speed controller
bool use_speed_controller = true;// false;
volatile float left_speed_demand = 0;
volatile float right_speed_demand = 0;
unsigned long map_timer = 0;

// Declare helper functions
void PauseAndPrintMap(bool raw=false);


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   This setup() routine initialises all class instances above and peripherals.
   It is recommended:
   - You keep this sequence of setup calls if you are to use all the devices.
   - Comment out those you will not use.
   - Insert new setup code after the below sequence.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void setup()
{

  // These two function set up the pin
  // change interrupts for the encoders.
  setupLeftEncoder();
  setupRightEncoder();
  startTimer();

  Pose.setDebug(false);

  //Set speed control maximum outputs to match motor
  LeftSpeedControl.setMax(30);
  RightSpeedControl.setMax(30);
  HeadingControl.setMax(50);

  // For this example, we'll calibrate only the
  // centre sensor.  You may wish to use more.
  LineCentre.calibrate();

  //Setup RFID card
  setupRFID();

  // The IMU calibration requires the Romi does not move.
  // See related lab sheets for more information.

  Wire.begin();

  // Set the random seed for the random number generator
  // from A0, which should itself be quite random.
  randomSeed(analogRead(A0));

  // Initialise Serial communication
  Serial1.begin( BAUD_RATE );
  delay(1000);
  Serial1.println("Board Reset");
  buzz2();

  // Romi will wait for you to press a button and then print
  // the current map.
  //
  // !!! A second button press will erase the map !!!
  ButtonB.waitForButton();

  Serial1.println("Calibrating IMU");
  Pose.calibrateIMU() ;
  Pose.setPose(900, 900, 0);

  Map.printMap();
  buzz();

  // Watch for second button press, then begin autonomous mode.
  ButtonB.waitForButton();

  Serial1.println("Map Erased - Mapping Started");
  Map.resetMap();
  map_timer = millis() ;
  
  // Your extra setup code is best placed here:
  // ...
  // ...
  // but not after the following:
  // Because code flow has been blocked, we need to reset the
  // last_time variable of the PIDs, otherwise we update the
  // PID with a large time elapsed since the class was
  // initialised, which will cause a big intergral term.
  // If you don't do this, you'll see the Romi accelerate away
  // very fast!
  LeftSpeedControl.reset();
  RightSpeedControl.reset();
  left_speed_demand = 0;
  right_speed_demand = 0;

  LeftPosControl.setMax( 30 ) ;
  RightPosControl.setMax( 30 ) ;
}



/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   This loop() demonstrates all devices being used in a basic sequence.
   The Romi should:
   - move forwards with random turns
   - log lines, RFID and obstacles to the map.

 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void loop() 
{
  unsigned long curr_time = millis();
  if ( curr_time - map_timer > 10000 ) {
    map_timer = curr_time;
    PauseAndPrintMap(false); // boolean indicates printing raw map
  }

  if(ButtonA.getSingleDebouncedPress()) {
    Serial1.print("Stopping");
    StopAndPrintMap();
  }
 
  // Print map to serial on button b press.
  if(ButtonB.getSingleDebouncedPress()) {
    Map.printMap();
  }

  Serial1.print("Pose: ");
  Pose.printPose();

  // Update kinematic model and print pose
  Pose.update(); 

  // Do Movement
  doMovement();
   
  // Update the map with sensor readings
  doMapping();

  delay(5);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
    Motion Control
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
static unsigned short movement_state = 0; // keeps track of motion state
static unsigned long movement_timer = millis(); // useful timer
static unsigned short movement_internal_state = 0; //keep track of boundary motion state
void doMovement() {
  unsigned short prev_mvmt_state = movement_state;

  // Serial1.print("mvmtstate: ");
  // Serial1.println(movement_state);

  // Check Current State and interrupt into other states
  // if non-normal motion is detected
  if (IRDetectObstacle()) {
    movement_state = 1;
  } else if (detectBoundary()) {
    movement_state = 2;
  } 

  // Reset timer if change of state
  if(prev_mvmt_state != movement_state) {
    movement_internal_state = 0;
    HeadingControl.reset();
    movement_timer = millis();
    buzz1();
  }

  // Based on state, run each behaviour
  if (movement_state == 1) {
    ObstacleAvoidanceState();
  } else if (movement_state == 2) {
    BoundaryAvoidanceState();
  } else {
    MotionPlanningState();
  }
}

void ObstacleAvoidanceState(){
  MotionPlanner.cancelCurrentMove();

  // Reverse
  float forward_bias = -20;
  float dist = (LeftIR.getDistanceRaw() - RightIR.getDistanceRaw()); // should be set to turn in opposite direction of obstacle
  float turn_bias = 15; // 0.2*dist;
  if (dist < 0){turn_bias *= -1;}
  
  left_speed_demand = forward_bias + turn_bias;
  right_speed_demand = forward_bias - turn_bias;
  
  if(millis() - movement_timer > 2000) {
    buzz();
    buzz1();
    movement_state = 0;
  }
}

void BoundaryAvoidanceState() {
  MotionPlanner.cancelCurrentMove();

  // Boundary avoidance state 0 is turning to face the origin
  if ( movement_internal_state == 0 ) {
    
    // Turn in direction of the origin of the map
    float Origin_Angle = atan2((Pose.getY()-900),(Pose.getX()-900)) ;
    float Boundary_Turn = wrapAngle((Origin_Angle + PI )) ;
    
    float turn_bias = HeadingControl.update(Boundary_Turn, Pose.getThetaRadians());
    left_speed_demand = -turn_bias;
    right_speed_demand = turn_bias;
     
    // If detect stability (i.e. finished turning), advance to state 1
    if ( HeadingControl.detectStability() || abs(turn_bias) < 3) {
      movement_timer = millis();
      movement_internal_state = 1;
      buzz();
      buzz2();
    }
  } 

  // Boundary avoidance state 1 is moving forward for 3 seconds towards origin.
  if(movement_internal_state == 1) {
    float forward_bias = 20;
    left_speed_demand = forward_bias;
    right_speed_demand = forward_bias;

    // If moved forward for more than 3 seconds, exit back to motion planning
    if(millis() - movement_timer > 2000) {
      movement_state = 0;
      movement_internal_state = 0;
      buzz();
      buzz();
      buzz2();
    }
  }
}

void MotionPlanningState() {
  
  // Check if previous move has been achieved, or was cancelled
  if(MotionPlanner.isPreviousMoveComplete(Pose)){    
      // If so plan next set of moves
      MotionPlanner.calculateNextMove(Pose);  
      movement_internal_state = 0;
  }

  // Otherwise recalculate demand based on current pose and continue with motionplanning
  MotionPlanner.calculateDemand(Pose);
  float forward_bias = MotionPlanner.nextMoveTargetDist();
  float turn_bias = MotionPlanner.nextMoveTargetAngle();
  float turn_control = HeadingControl.update(turn_bias, Pose.getThetaRadians());

  // Serial1.print("MP- f: ");
  // Serial1.print(forward_bias);
  // Serial1.print("\t t: ");
  // Serial1.print(turn_bias);
  // Serial1.print("\t tc: ");
  // Serial1.println(turn_control);

  if (movement_internal_state == 0) {  
    left_speed_demand = -turn_control;
    right_speed_demand = turn_control;
    if ( HeadingControl.detectStability() || abs(turn_bias - Pose.getThetaRadians()) < 0.05) {
      movement_timer = millis();
      movement_internal_state = 1;
      buzz();
      buzz2();
    }
  }

  if (movement_internal_state == 1) {
    left_speed_demand = forward_bias - turn_control;
    right_speed_demand = forward_bias + turn_control;
  }
 
}

bool IRDetectObstacle() {
  return CentreIR.getDistanceRaw() > 500 || LeftIR.getDistanceRaw() > 600 || RightIR.getDistanceRaw() > 600;
}

bool detectBoundary() {
  float xmin = 100;// 200;
  float xmax = 1700;// 1600;
  float ymin = 100;// 200;
  float ymax = 1700;// 1600;
  return Pose.getX() < xmin || Pose.getX() > xmax || Pose.getY() < xmin || Pose.getY() > xmax;
}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   This function groups up our sensor checks, and then
   encodes into the map.  To get you started, we are
   simply placing a character into the map.  However,
   you might want to look using a bitwise scheme to
   encode more information.  Take a look at mapping.h
   for more information on how we are reading and
   writing to eeprom memory.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void doMapping() {

  // Add own position to map first
  Map.updateMapFeature((byte)'*', Pose.getX(), Pose.getY());

  // Read IR sensors and add any detected obstacles to belief map
  IRaddToMap() ;

  // Check RFID scanner.
  // Look inside RF_interface.h for more info.
  if ( checkForRFID() ) {

    // Add card to map encoding.
    Map.updateMapFeature( (byte)'R', Pose.getY(), Pose.getX() );

    // you can check the position reference and
    // bearing information of the RFID Card in
    // the following way:
    // serialToBearing( rfid.serNum[0] );
    // serialToXPos( rfid.serNum[0] );
    // serialToYPos( rfid.serNum[0] );
    //
    // Note, that, you will need to set the x,y
    // and bearing information in rfid.h for your
    // experiment setup.  For the experiment days,
    // we will tell you the serial number and x y
    // bearing information for the cards in use.

  }

  // Basic uncalibrated check for a line.
  // Students can do better than this after CW1 ;)
  if ( LineCentre.readRaw() > 580 ) {
    Map.updateMapFeature( (byte)'L', Pose.getY(), Pose.getX() );
  }

}

void IRaddToMap() {
  float distance = CentreIR.getFilteredInMM();
  IRProjectOntoMap(distance, 0, 50, 300);
  
  distance = LeftIR.getFilteredInMM();
  IRProjectOntoMap(distance, -PI/4, 50, 300);

  distance = RightIR.getFilteredInMM();
  IRProjectOntoMap(distance, PI/4, 50, 300);
}

void IRProjectOntoMap(float distance, float angleoffset, float mindist, float maxdist) {
  float projected_x = 0 ;
  float projected_y = 0 ;
  if ( distance < maxdist && distance > mindist ) {

    distance += 80; // Adjust for sensor placement on body
    float cos_proj = distance * cos( Pose.getThetaRadians() + angleoffset);
    float sin_proj = distance * sin( Pose.getThetaRadians() + angleoffset);

    // Update all cells up to the obstacle position as non-obstacles
    for(float k = 0.0 ; k < 1.0 ; k=k+0.2) {
      projected_x = Pose.getX() + ( k * cos_proj );
      projected_y = Pose.getY() + ( k * sin_proj );
      Map.updateMapFeature( (byte)'.', projected_x, projected_y );
    }

    // Here we calculate the actual position of the obstacle we have detected
    projected_x = Pose.getX() + cos_proj;
    projected_y = Pose.getY() + sin_proj;
    Map.updateMapFeature( (byte)'O', projected_x, projected_y );
  } else if (distance > maxdist) {
    distance = maxdist;
    distance += 80; // Adjust for sensor placement on body
    float cos_proj = distance * cos( Pose.getThetaRadians() + angleoffset);
    float sin_proj = distance * sin( Pose.getThetaRadians() + angleoffset);

    // Update all cells up to maximum distance as non-obstacles
    for(float k = 0.0 ; k < 1.0 ; k=k+0.2) {
      projected_x = Pose.getX() + ( k * cos_proj );
      projected_y = Pose.getY() + ( k * sin_proj );
      Map.updateMapFeature( (byte)'.', projected_x, projected_y );
    }
  } else if (distance < mindist) {
    distance = mindist;
    distance += 80; // Adjust for sensor placement on body
    float cos_proj = distance * cos( Pose.getThetaRadians() + angleoffset);
    float sin_proj = distance * sin( Pose.getThetaRadians() + angleoffset); 
    // Here we calculate the actual position of the obstacle we have detected
    projected_x = Pose.getX() + cos_proj;
    projected_y = Pose.getY() + sin_proj;
    Map.updateMapFeature( (byte)'O', projected_x, projected_y );
  }
}

/***********************************************************************************
 * 
 *  Utilities 
 *  Motor control and buzzer and other things
 * 
 * 
 * */

void StartMoving(float ldemand, float rdemand) {
  LeftSpeedControl.reset();
  RightSpeedControl.reset();

  left_speed_demand = ldemand;
  right_speed_demand = rdemand;
}

void StopMoving() { 
  // Zero demands to stop speed controller motion
  left_speed_demand = 0;
  right_speed_demand = 0;
}

void PauseAndPrintMap(bool raw=false) {
  float prevleftdemand = left_speed_demand;
  float prevrightdemand = right_speed_demand;
  StopMoving();
  if(!raw) {
    Map.printMap();
  } else {
    Map.printRawMap();
  }
  StartMoving(prevleftdemand, prevrightdemand);
}

void StopAndPrintMap() {  
  // Stops moving and prints map to serial forever
  // If A is pressed in loop, human readable map will be sent.
  StopMoving();
  unsigned long start_time = millis();
  while(true){
    unsigned long curr_time = millis();
    if((curr_time - start_time) > 5000) {
      start_time = curr_time;
      Map.printRawMap();
    }
  }
}

void buzz() {
  analogWrite( BUZZER_PIN , 10 ) ;
  delay ( 25 ) ;
  analogWrite( BUZZER_PIN , 0 ) ;
}

void buzz1() {
  analogWrite( BUZZER_PIN , 30 ) ;
  delay ( 50 ) ;
  analogWrite( BUZZER_PIN , 0 ) ;
}

void buzz2() {
  analogWrite( BUZZER_PIN , 100 ) ;
  delay ( 50 ) ;
  analogWrite( BUZZER_PIN , 0 ) ;
}

ISR(TIMER3_COMPA_vect) {
  
   /*
     Calculate Speeds
  */
  signed int left_delta = left_encoder_count - last_count_left;
  signed int right_delta = right_encoder_count - last_count_right;

  last_count_left = left_encoder_count;
  last_count_right = right_encoder_count;

  left_speed =  left_delta;
  right_speed = right_delta;

  if (use_speed_controller)
  {
    float left_motor_demand = LeftSpeedControl.update(left_speed_demand, left_speed);
    float right_motor_demand = RightSpeedControl.update(right_speed_demand, right_speed);

    // Serial.print(left_speed);
    // Serial.print(" ");
    // Serial.print(left_speed_demand);
    // Serial.print(" ");
    // Serial.println(left_motor_demand);

    LeftMotor.setPower(left_motor_demand);
    RightMotor.setPower(right_motor_demand);
  }
}
