#ifndef _Kinematics_h
#define _Kinematics_h
#include "imu.h"


#define GEAR_RATIO 120.0
#define COUNTS_PER_SHAFT_REVOLUTION 12.0
const float WHEEL_RADIUS = 35.0;
const float WHEEL_DISTANCE = 143;
const float COUNTS_PER_WHEEL_REVOLUTION = GEAR_RATIO * COUNTS_PER_SHAFT_REVOLUTION;
const float MM_PER_COUNT = ( 2 * WHEEL_RADIUS * PI ) / COUNTS_PER_WHEEL_REVOLUTION;

class Kinematics
{
     public:

         void  update();
         float getThetaDegrees();
         float getThetaRadians();
         float getX();
         float getY();
         void  resetPose();
         void  setPose(float X, float Y, float theta);
         void  printPose();
         void  setDebug(bool state);
         float getDistanceFromOrigin();
         float getAngularVelocity();
         void  calibrateIMU() ;
        
    private:

         float  x = 900 ;
         float  y = 900 ;
         float  theta ;
         float  last_theta = 0 ;
         float  theta_enc_d = 0 ;
         float  theta_imu_d = 0 ; 
         float  angular_velocity = 0 ;
         long   last_left_encoder_count = 0 ;
         long   last_right_encoder_count = 0 ;
         bool   debug = true ;
         unsigned long last_update = 0 ;
         Imu    IMU ;

};

void Kinematics::calibrateIMU() 
{

    IMU.init() ;
    IMU.calibrate() ;

}


void Kinematics::update()
{

    float time_elapsed = millis() - last_update;
    last_update = millis();

    IMU.readFiltered() ;

    //Calculate delta since last update
    float left_delta = (left_encoder_count - last_left_encoder_count)*MM_PER_COUNT;
    float right_delta = (right_encoder_count - last_right_encoder_count)*MM_PER_COUNT;
    float mean_delta = (left_delta + right_delta) / 2;  
    
    //Store counts
    last_left_encoder_count = left_encoder_count;
    last_right_encoder_count = right_encoder_count;  

    theta_enc_d = ( ( ( right_delta - left_delta ) / WHEEL_DISTANCE ) )  ;  // Heading from Encoders
    theta_imu_d = ( IMU.gz_rad * ( time_elapsed / 1000 ) ) ;                // Heading from IMU
    theta += theta_enc_d + ( theta_imu_d - theta_enc_d ) ;                  // Complimentary filter for current heading 
    
    //Wrap theta between -PI and PI.
    if (theta > PI)
    {
        theta -=2*PI;
    }
    else if(theta < -PI)
    {
        theta += 2*PI;
    } 

    last_theta = theta; // Store theta for next iteration

    x += ( mean_delta * cos ( theta ) ) ;   // X position update
    y += ( mean_delta * sin ( theta ) ) ;   // Y position update


    angular_velocity = ( (left_delta-right_delta) / WHEEL_DISTANCE );
    angular_velocity -= last_theta;
    angular_velocity /= time_elapsed;
    

    if (debug)
    {
        printPose();
    }
  
}


float Kinematics::getThetaDegrees()
{
    return rad2deg(theta);
}
float Kinematics::getThetaRadians()
{
    return (theta);
}

float Kinematics::getAngularVelocity() {
    return rad2deg( angular_velocity );
}


float Kinematics::getX()
{
    return x;
}


float Kinematics::getY()
{
    return y;
}


void Kinematics::resetPose()
{

    x = 900;
    y = 900;
    theta = 0;

}


void Kinematics::setPose(float newX, float newY, float newTheta)
{

    x = newX;
    y = newY;
    theta = newTheta;

}

void Kinematics::printPose()
{

    Serial1.print(F("X: "));
    Serial1.print(x);
    Serial1.print(F(" Y: "));
    Serial1.print(y);
    Serial1.print(F(" H: "));
    Serial1.println(theta);

}


void Kinematics::setDebug(bool state)
{
    debug = state;
}


float Kinematics::getDistanceFromOrigin()
{
    return sqrt(x*x + y*y);
}

#endif
