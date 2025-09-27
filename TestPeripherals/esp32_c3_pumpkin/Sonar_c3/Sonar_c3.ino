#include <HCSR04.h>

//6 trigger, 5 echo
UltraSonicDistanceSensor distanceSensor(6, 5);  // Initialize sensor
#define LED_CAP_CTRL 7 //default is pin 8 for esp32-c3

void setup () {
    Serial.begin(9600);  // We initialize serial connection so that we could print values from sensor.
      // initialize digital pin LED_CAP_CTRL as an output.
  pinMode(LED_CAP_CTRL, OUTPUT);

}

void loop () {
    // Every 500 miliseconds, do a measurement using the sensor and print the distance in centimeters.
    int distance = distanceSensor.measureDistanceCm();
    Serial.println(distance);
    if(distance < 20)
    {
      //OFF
      digitalWrite(LED_CAP_CTRL, LOW);  // turn the LED on (HIGH is the voltage level)
      delay(1000);                      // wait for a second
                     // wait for a second
    }else
    {
      //ON
      digitalWrite(LED_CAP_CTRL, HIGH);   // turn the LED off by making the voltage LOW
      delay(1000); 
    }
    delay(500);
}
