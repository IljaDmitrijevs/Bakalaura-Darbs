#include "GP2Y0A02YK0F.h"

GP2Y0A02YK0F irSensor;
int distance;

void setup()
{
  Serial.begin(9600);
  irSensor.begin(A0);  //  Assign A0 as sensor pin
}

void loop()
{
  distance = irSensor.getDistanceCentimeter();
 unsigned int val = analogRead(A0);
  Serial.print("\nDistance in centimeters: ");
  Serial.print(distance);  
  Serial.print("\nanalog: ");
  Serial.print(val);
  
  delay(750); 
}
