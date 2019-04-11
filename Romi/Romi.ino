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
#include "mapping.h"
#include "RF_Interface.h"
#include <Wire.h>
#include "imu.h"
#include "magnetometer.h"
#include "Pushbutton.h"



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

Imu           Imu;

Magnetometer  Mag; // Class for the magnetometer

Motor         LeftMotor(MOTOR_PWM_L, MOTOR_DIR_L);
Motor         RightMotor(MOTOR_PWM_R, MOTOR_DIR_R);

//These work for our Romi - We strongly suggest you perform your own tuning
PID           LeftPosControl(.15, 0.04, 0.001) ;
PID           RightPosControl(.15, 0.04, 0.001) ;
PID           LeftSpeedControl( 3.5, 20.9, 0.04 );
PID           RightSpeedControl( 3.5, 20.9, 0.04 );
PID           HeadingControl( 1.5, 0, 0.001 );

Mapper        Map; //Class for representing the map

Pushbutton    ButtonB( BUTTON_B, DEFAULT_STATE_HIGH);

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   Global variables.
   These global variables are not mandatory, but are used for the example loop()
   routine below.
 *                                                                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//Use these variables to set the demand of the speed controller
bool use_speed_controller = false;
float left_speed_demand = 0;
float right_speed_demand = 0;


    float readingC ;



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

  //Set speed control maximum outputs to match motor
  LeftSpeedControl.setMax(100);
  RightSpeedControl.setMax(100);

  // For this example, we'll calibrate only the
  // centre sensor.  You may wish to use more.
  LineCentre.calibrate();

  //Setup RFID card
  setupRFID();

  // These functions calibrate the IMU and Magnetometer
  // The magnetometer calibration routine require you to move
  // your robot around  in space.
  // The IMU calibration requires the Romi does not move.
  // See related lab sheets for more information.

  Wire.begin();
 // Mag.init();
 // buzz() ;
 // Mag.calibrate();
  //  Imu.init();
  //  Imu.calibrate();


  // Set the random seed for the random number generator
  // from A0, which should itself be quite random.
  randomSeed(analogRead(A0));


  // Initialise Serial communication
  Serial1.begin( BAUD_RATE );
  delay(1000);
  Serial1.println("Board Reset");

  // Romi will wait for you to press a button and then print
  // the current map.
  //
  // !!! A second button press will erase the map !!!
  ButtonB.waitForButton();

  Map.printMap();

  // Watch for second button press, then begin autonomous mode.
  ButtonB.waitForButton();

  Serial1.println("Map Erased - Mapping Started");
  Map.resetMap();

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
  left_speed_demand = 5;
  right_speed_demand = 5;

  LeftPosControl.setMax( 30 ) ;
  RightPosControl.setMax( 30 ) ;




}
void stop_moving() { // Returns the wheel speeds to zero

  LeftMotor.setPower( 0 ) ;
  RightMotor.setPower( 0 ) ;
  //


}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   This loop() demonstrates all devices being used in a basic sequence.
   The Romi should:
   - move forwards with random turns
   - log lines, RFID and obstacles to the map.

 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
void loop() {


readingC = CentreIR.getFilteredInMM() ; 
//readingC = getReading() ;
Serial1.println(readingC) ;
//float readingL = LeftIR.getFilteredInMM() ;
//float readingR = RightIR.getFilteredInMM() ; 
//Serial1.print("Left: ") ;
//Serial1.println(readingL) ;
//Serial1.print("Centre: ") ;

//Serial1.print("Right: ") ;
//Serial1.println(readingR) ;
//Serial1.println() ;

//float readingLine = LineCentre.readCalibrated() ;
//Serial1.print("Line: ") ;
//Serial1.println(readingLine) ;
//Serial1.println() ;


delay(100);
}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   We have implemented a random walk behaviour for you
   with a *very* basic obstacle avoidance behaviour.
   It is enough to get the Romi to drive around.  We
   expect that in your first week, should should get a
   better obstacle avoidance behaviour implemented for
   your Experiment Day 1 baseline test.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
float getReading() {
for (int i = 0 ; i < 100 ; i++ )
{ 
float readingC = CentreIR.getFilteredInMM() ;
readingC += ( readingC / 100 ) ;
return readingC ;
}
}

void doMovement() {

  // Static means this variable will keep
  // its value on each call from loop()
  static unsigned long walk_update = millis();

  // used to control the forward and turn
  // speeds of the robot.
  float forward_bias;
  float turn_bias;

  // Check if we are about to collide.  If so,
  // zero forward speed
  if ( CentreIR.getDistanceRaw() > 450 ) {
    forward_bias = 0;
  } else {
    forward_bias = 30;
  }

  // Periodically set a random turn.
  // Here, gaussian means we most often drive
  // forwards, and occasionally make a big turn.
  if ( millis() - walk_update > 500 ) {
    walk_update = millis();

    // randGaussian(mean, sd).  utils.h
    turn_bias = randGaussian(0, 35 );

    // Setting a speed demand with these variables
    // is automatically captured by a speed PID
    // controller in timer3 ISR. Check interrupts.h
    // for more information.
    left_speed_demand = forward_bias + turn_bias;
    right_speed_demand = forward_bias - turn_bias;

      if ( Pose.getX() < 75 || Pose.getX() > 1725 || Pose.getY() < 75 || Pose.getY() > 1725 ) {
    stop_moving() ;

    LeftMotor.setPower(40);
    RightMotor.setPower(-40);
    delay(2000);
    stop_moving() ;
    LeftMotor.setPower(30);
    RightMotor.setPower(30);
    delay(1000) ;
    stop_moving() ;
    

  } else {

    LeftMotor.setPower(left_speed_demand) ;
    RightMotor.setPower(right_speed_demand) ;
  }

}
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

  // Read the IR Sensor and determine distance in
  // mm.  Make sure you calibrate your own code!
  // We threshold a reading between 40mm and 12mm.
  // The rationale being:
  // We can't trust very close readings or very far.
  // ...but feel free to investigate this.
  float distance = CentreIR.getFilteredInMM();
  if ( distance < 40 && distance > 12 ) {

    // We know the romi has the sensor mounted
    // to the front of the robot.  Therefore, the
    // sensor faces along Pose.Theta.
    // We also add on the distance of the
    // sensor away from the centre of the robot.
    distance += 80;


    // Here we calculate the actual position of the obstacle we have detected
    float projected_x = Pose.getX() + ( distance * cos( Pose.getThetaRadians() ) );
    float projected_y = Pose.getY() + ( distance * sin( Pose.getThetaRadians() ) );
    Map.updateMapFeature( (byte)'O', projected_x, projected_y );


  }

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





void turn( float angle ) { // Turn the Romi by a specified angle ( in Radians )

  float arc_length = 75 * angle ;
  int turn_target = round ( arc_length / ( MM_PER_COUNT ) ) ;

  float L_PWM = LeftPosControl.update( (  - turn_target ) , left_encoder_count ) ;
  float R_PWM = RightPosControl.update( (  turn_target ) , right_encoder_count ) ;
  //float ADJ = wheelMatch.update( 0 , turn_error ) ;

  leftWheel_output(L_PWM) ;
  rightWheel_output(R_PWM) ;

 //  analogWrite(10,L_PWM);
 //   analogWrite(9,R_PWM);
  //
}




void buzz() {

  delay ( 25 ) ;
  analogWrite( 6 , 10 ) ;
  delay ( 25 ) ;
  analogWrite( 6 , 0 ) ;
  //
}



ISR(TIMER3_COMPA_vect)
{

  //Pose.update() ;

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

    LeftMotor.setPower(left_motor_demand);
    RightMotor.setPower(right_motor_demand);
  }
}


void leftWheel_output( float leftPower ) { // Writes left wheel power signal

  if ( leftPower < 0 ) {
    leftPower *= -1 ;
    digitalWrite( MOTOR_DIR_L , HIGH ) ;
    analogWrite( MOTOR_PWM_L , leftPower ) ;
  } else {
    digitalWrite( MOTOR_DIR_L , LOW ) ;
    analogWrite( MOTOR_PWM_L , leftPower ) ;
  }
  //
}


void rightWheel_output( float rightPower ) { // Writes right wheel power signal

  if ( rightPower < 0 ) {
    rightPower *= -1 ;
    digitalWrite( MOTOR_DIR_R , HIGH ) ;
    analogWrite( MOTOR_PWM_R , rightPower ) ;
  } else {
    digitalWrite( MOTOR_DIR_R , LOW ) ;
    analogWrite( MOTOR_PWM_R , rightPower ) ;
  }
  //
}
