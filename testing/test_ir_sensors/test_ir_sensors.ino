byte pin = A6;
byte pin2 = A0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  float distance = (float)analogRead( pin );
    
//  // map this to 0 : 5v range.
//  distance *= 0.0048;
//
//  const float exponent = (1/-0.616);
//  distance = pow( ( distance / 12.494 ), exponent);

    // put your main code here, to run repeatedly:
  float distance2 = (float)analogRead( pin2 );
//    
//  // map this to 0 : 5v range.
//  distance2 *= 0.0048;
//
//  const float exponent2 = (1/-0.616);
//  distance2 = pow( ( distance2 / 12.494 ), exponent2);
  
  Serial.print(distance);
  Serial.print(" ");
  Serial.println(distance2);
  delay(10);
}
