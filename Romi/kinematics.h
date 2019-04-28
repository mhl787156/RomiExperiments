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
         float  theta = 0;
         float  last_theta = 0 ;
         float  theta_enc_d = 0 ;
         float  theta_imu_d = 0 ; 
         float  angular_velocity = 0 ;
         long   last_left_encoder_count = 0 ;
         long   last_right_encoder_count = 0 ;
         float  filter_gain = 0;
         float  g = 0.85;
         float  h = 0.05;
         bool   debug = true ;
         unsigned long last_update = 0 ;
         Imu    IMU ;

};

void Kinematics::calibrateIMU() 
{

    // IMU.init() ;
    // IMU.calibrate() ;

}


void Kinematics::update()
{
    unsigned long time_elapsed = millis() - last_update;
    last_update = millis();

    // Calculate delta (distance) since last update
    float left_delta = (left_encoder_count - last_left_encoder_count) * MM_PER_COUNT;
    float right_delta = (right_encoder_count - last_right_encoder_count) * MM_PER_COUNT;  
    
    // Store counts
    last_left_encoder_count = left_encoder_count;
    last_right_encoder_count = right_encoder_count;  
    last_theta = theta; // Store theta for next iteration


    // Calculate ICR
    float diff = right_delta - left_delta;
    float sum = right_delta + left_delta;
    theta_enc_d = diff / WHEEL_DISTANCE;                                    // Heading from Encoders
    if(abs(diff) < 1e-6) { // Forward Motion
        // Serial1.println("Forward update");
        float mean_delta = sum / 2;             // Average
        x += ( mean_delta * cos ( theta ) ) ;   // X position update
        y += ( mean_delta * sin ( theta ) ) ;   // Y position update
    } else if (abs(sum) < 1e-6) { // On spot turn
        // Serial1.println("On Spot update");
        theta += theta_enc_d;
    } else { // Using ICC
        // Serial1.println("ICC update");
        float R = (WHEEL_DISTANCE / 2) * (sum / diff);
        float ICCx = x - R*sin(theta);
        float ICCy = y + R*cos(theta);

        float x_new = cos(theta_enc_d)*(x - ICCx) - sin(theta_enc_d)*(y - ICCy) + ICCx;
        float y_new = sin(theta_enc_d)*(x - ICCx) + cos(theta_enc_d)*(y - ICCy) + ICCy;
        x = x_new;
        y = y_new;
        theta += theta_enc_d;
    }

    // Update angular vel;
    angular_velocity = (theta - last_theta) / time_elapsed;

    // Serial1.print("theta:");
    // Serial1.print(theta_enc_d, 4);
    // Serial1.print(" ");
    // Serial1.print(theta);
    // Serial1.print(" ");

    // Encorporate IMU using GH filter
    // g = 0.85; h = 0.005;
    // IMU.readFiltered() ;
    // theta_imu_d = IMU.gz_rad * ( time_elapsed / 1000 );                     // Heading from IMU (effectively complementary as no accel
    // theta += filter_gain * time_elapsed;
    // diff = theta_imu_d - theta;
    // theta += g * diff;
    // filter_gain += h * diff / time_elapsed;
    // filter_gain = filter_gain + h * (theta_imu_d - theta)/ time_elapsed;
    // float predict_theta = theta + filter_gain;
    // theta = predict_theta + g * (theta_imu_d - predict_theta);
  
    // Serial1.print(theta_imu_d, 6);
    // Serial1.print(" ");
    // Serial1.print(filter_gain,4);
    // Serial1.print(" ");
    // Serial1.print(theta,4);
    Serial1.print("\n");

    //  Wrap theta between -PI and PI.
    if (theta > PI)
    {
        theta -=2*PI;
    }
    else if(theta < -PI)
    {
        theta += 2*PI;
    } 

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
