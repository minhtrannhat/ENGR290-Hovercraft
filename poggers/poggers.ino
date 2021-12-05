 /* MPU9250 Basic Example Code

   mods to get working out of the box (sjr)
  1. choose AD0 below instead of AD1
  2. edit out LCD code
  3. activate magnetometer calibration
  4. add delays for prints
  5. shorten prints

  Corrected wrong hand of magnetometer data in quaternion_update call.

  by: Kris Winer
  date: April 1, 2014
  license: Beerware - Use this code however you'd like. If you
  find it useful you can buy me a beer some time.
  Modified by Brent Wilkins July 19, 2016

  Demonstrate basic MPU-9250 functionality including parameterizing the register
  addresses, initializing the sensor, getting properly scaled accelerometer,
  gyroscope, and magnetometer data out. Added display functions to allow display
  to on breadboard monitor. Addition of 9 DoF sensor fusion using open source
  Madgwick and Mahony filter algorithms. Sketch runs on the 3.3 V 8 MHz Pro Mini
  and the Teensy 3.1.

  SDA and SCL should have external pull-up resistors (to 3.3V).
  10k resistors are on the EMSENSR-9250 breakout board.

  Hardware setup:
  MPU9250 Breakout --------- Arduino
  VDD ---------------------- 3.3V
  VDDI --------------------- 3.3V
  SDA ----------------------- A4
  SCL ----------------------- A5
  GND ---------------------- GND
*/
#include "init_290.c"
#include <Servo.h>

//180/1023*adc reading

#define AHRS true         // Set to false for basic data read
#define SerialDebug false  // Set to true to get Serial output for debugging

Servo myservo;  // Creates a servo object
int pos = 0;    // Variable to store the servos angle

#define I2Cclock 400000
#define I2Cport Wire

void setup()
{
  Serial.begin(38400);
  myservo.attach(PB1);
  cli();
}

void loop()
{
  


  for(int pos = 180; pos <= 0; pos--){
  myservo.write(pos); //turn thrust fan to the right 45 degrees
  delay(15);
  }
  /*
  // IR sensor
  int valIR =analogRead(PC3); // read the sensor
 
  // US sensors
  int valUS = analogRead(PC4);

  //Serial.print("IR sensor reading: ");
  Serial.println("IR sensor reading: ");
  Serial.println(valIR);
  delay(100000);

  Serial.println("US sensor reading: ");
  Serial.println(valUS);
  delay(100000);

  while (valIR < 550) {
    //OCR1B = D1B(190); // 75% speed
    if(valUS > 26){
      for (uint8_t i = 20; i > 0; i--) Serial.println("Turning right");
      // myservo.write(135);; //turn thrust fan to the right 45 degrees
    }
    else {
      for (uint8_t i = 2000; i > 0; i--) Serial.println("Turning left");
      //myservo.write(45); //turn thrust fan to the left 45 degrees
    }
  }*/
}
