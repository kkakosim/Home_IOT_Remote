/*Arduino Whitsle Detector Switch Program
   Detects the whistles from pin 8 and toggles pin 13
   Dated: 31-5-2019
   Website: www.circuitdigest.com
*/

#include <FreqMeasure.h>//https://github.com/PaulStoffregen/FreqMeasure
double sum = 0;
int count = 0;
bool state = false;
bool stateClap = false;
float frequency;
int continuity = 0;
//int soundSensor=8;
int pinOut = 10;

void setup() {
  Serial.begin(9600);
  FreqMeasure.begin(); //Measures on pin 8 by default
  Serial.println("StartUp");
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pinOut, OUTPUT); //Relay Signal
  digitalWrite(pinOut, LOW);
}



void loop() {
  //int SensorData = digitalRead(soundSensor);
  if (FreqMeasure.available()) {
    // average several reading together
    sum = sum + FreqMeasure.read();
    count = count + 1;
  }
  if (count > 30) {
    frequency = FreqMeasure.countToFrequency(sum / count);
    Serial.println(frequency);
    sum = 0;
    count = 0;
  }

  if (frequency > 10 && frequency < 20000)
  {
    continuity++;
    Serial.print("Continuity -> ");
    Serial.println(continuity);
    frequency = 0;
  }

  if (continuity >= 3 && state == false)
  {
    state = true;
    continuity = 0;
    //digitalWrite(pinOut, HIGH);
    Serial.println("Light Turned ON");
    delay(1000);
  }

  if (continuity >= 3 && state == true)
  {
    state = false;
    continuity = 0;
    //digitalWrite(pinOut, LOW);
    Serial.println("Light Turned OFF");
    delay(1000);
  }

  digitalWrite(LED_BUILTIN, state);
  digitalWrite(pinOut, state);

}
