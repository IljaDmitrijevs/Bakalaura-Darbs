#define trigPin 10
#define echoPin 13
float duration;
int distance;

void setup() 
{
  Serial.begin (9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  }
  void loop()
  {
    digitalWrite(trigPin, LOW);
    delay(1);
    digitalWrite(trigPin, HIGH);
    delay(100);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration/2)* 0.0343;
    Serial.print("Distance = ");
    if (distance >=500 || distance <=2)
    {
      Serial.println("Our of range");
      }
      else
      {
        Serial.print(distance);
        Serial.println("CM");
        delay(500);
        }
        delay(500);
    }
