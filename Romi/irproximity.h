#ifndef _IRProximity_h
#define _IRProximity_h

class SharpIR
{
  public:
    SharpIR(byte pin, float a, float b, float c);
    int     getDistanceRaw();
    float   getDistanceInMM();
    float   getFilteredInMM() ;
    void    calibrate();

  private:
    byte  pin;
    float a ;
    float b ;
    float c ;
    double alpha = 0.1 ;
    float last_output ;

};

SharpIR::SharpIR(byte _pin, float _a, float _b, float _c )
{
  pin = _pin;
  a = _a ;
  b = _b ;
  c = _c ;
}

int SharpIR::getDistanceRaw()
{
  return analogRead(pin);
}


/*
   This piece of code is quite crucial to mapping
   obstacle distance accurately, so you are encouraged
   to calibrate your own sensor by following the labsheet.
   Also remember to make sure your sensor is fixed to your
   Romi firmly and has a clear line of sight!
*/
float SharpIR::getDistanceInMM()
{

  float distance = (float)analogRead( pin );

  distance = ( pow( (( distance - c ) / a  ),  (1/b) ) );
  
  return distance;
}

float   SharpIR::getFilteredInMM() 
{

float output = ( alpha * SharpIR::getDistanceInMM() ) + ( ( 1 - alpha ) * last_output )  ;
last_output = output ;

return output ;
      
}


#endif
