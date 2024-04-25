/*
  Ch32v003-f4p6

  - blinks D4 -> LED on board 
*/

void setup() 
{
//  Serial.begin(9600);
  Serial.begin(115200);

  Serial.println("Hello");

  pinMode(D4, OUTPUT);

  }

void loop() 
{
  Serial.println("Blink");

  digitalWrite(D4, LOW);
   delay(500);
  digitalWrite(D4, HIGH);
   delay(500);

}
