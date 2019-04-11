#ifndef _IMU_h
#define _IMU_h

#include <LSM6.h>

const int NUM_CALIBRATIONS_IMU = 100;

class Imu
{
    public:
        void  init();
        void  readRaw();
        void  readCalibrated();
        void  readFiltered();
        void  calibrate();
        LSM6 imu;
        float ax = 0;
        float ay = 0;
        float az = 0;
        float gx = 0;
        float gy = 0;
        float gz = 0;
        float gx_deg = 0;
        float gy_deg = 0;
        float gz_deg = 0;
        float gx_rad = 0;
        float gy_rad = 0;
        float gz_rad = 0;

    private:
        float a_sensitivity = 0.061;
        float g_sensitivity = 8.75;
        float gx_offset = 0;
        float gy_offset = 0;
        float gz_offset = 0;
        double alpha = 0.75 ;

        float gx_deg_last = 0 ;
        float gy_deg_last = 0 ;
        float gz_deg_last = 0 ;


};

void Imu::init()
{
    if (!imu.init())
    {
        Serial1.println("Failed to detect and initialize magnetometer!");
        while (1);
    }

    imu.enableDefault();
}

void Imu::readRaw()
{
  
  imu.read();
  
  gx = imu.g.x;
  gy = imu.g.y;
  gz = imu.g.z;

  ax = imu.a.x;
  ay = imu.a.y;
  az = imu.a.z;
  
}

void Imu::readCalibrated()
{

  imu.read();
  
  gx_deg = ( g_sensitivity * (imu.g.x - gx_offset) ) / 1000 ;
  gy_deg = ( g_sensitivity * (imu.g.y - gy_offset) ) / 1000 ;
  gz_deg = ( g_sensitivity * (imu.g.z - gz_offset) ) / 1000 ;

  gx_rad = gx_deg * PI / 180 ;
  gy_rad = gy_deg * PI / 180 ;
  gz_rad = gz_deg * PI / 180 ;
  
  ax = a_sensitivity * imu.a.x;
  ay = a_sensitivity * imu.a.y;
  az = a_sensitivity * imu.a.z;

   
}

void Imu::readFiltered()
{

  Imu::readCalibrated();

  gx_deg = ( gx_deg * alpha ) + (( 1 - alpha ) * gx_deg_last) ;
  gy_deg = ( gy_deg * alpha ) + (( 1 - alpha ) * gy_deg_last) ;
  gz_deg = ( gz_deg * alpha ) + (( 1 - alpha ) * gz_deg_last) ;
  gx_deg_last = gx_deg ;
  gy_deg_last = gy_deg ;
  gz_deg_last = gz_deg ;

  gx_rad = gx_deg * PI / 180 ;
  gy_rad = gy_deg * PI / 180 ;
  gz_rad = gz_deg * PI / 180 ;


}


void Imu::calibrate()
{

  for (int i=0;i<NUM_CALIBRATIONS_IMU;i++)
  {
    delay(50);
    
    imu.read();

    gx_offset += ((float)imu.g.x / NUM_CALIBRATIONS_IMU);
    gy_offset += ((float)imu.g.y / NUM_CALIBRATIONS_IMU);
    gz_offset += ((float)imu.g.z / NUM_CALIBRATIONS_IMU);
    
  }

  
}




#endif
