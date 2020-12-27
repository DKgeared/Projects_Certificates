#include<Motor2.h>

int tm = 25;
int threshold = 5;
float pwmmul = 1;
float baseSpeed = (100 * pwmmul) - threshold;
float maxSpeed =  (100 * pwmmul) + threshold;
float motorSpeed = 100 * pwmmul;

float kpp = 0.9;
float kii = 0.07;
float kdd = 0.1;

int roll;
int pitch;
int ang1;
int ang2;
int ang3;
int ang4;
int angmul = 3;

int now;
int limit = 500;

//#define left
//#define right

int l = 0;
int r = 0;
int reverse = 0;

//.............................................................................................................................................................................................................................//

//general walking gait
//stride=180mm
//stance=400mm
//flight height=70mm

float leg11a[] = { 79.69, 80.51, 81.43, 82.46, 83.60, 84.84, 86.17, 87.60, 89.13, 90.75, 92.46, 94.27, 96.16, 98.15, 100.22, 102.38, 104.63, 106.97, 109.41, 98.76, 92.06, 86.71, 82.14, 78.14, 74.63, 71.55, 68.90, 66.69, 64.94, 63.66, 62.91, 62.74, 63.24, 64.54, 66.93, 71.03 };
float leg22a[] = { 124.68, 122.46, 120.31, 118.22, 116.19, 114.23, 112.32, 110.48, 108.69, 106.97, 105.30, 103.69, 102.15, 100.66, 99.24, 97.89, 96.60, 95.39, 94.24, 88.94, 86.86, 86.00, 85.87, 86.29, 87.14, 88.35, 89.89, 91.70, 93.78, 96.10, 98.66, 101.48, 104.59, 108.05, 112.02, 116.92 };

//.............................................................................................................................................................................................................................//

//flight Height =150mm
//stride Length=300mm
//stance Height =400mm
//Total 40 points

//float leg11a[] = {86.88, 87.73, 88.74, 90.02, 91.58, 93.42, 95.51, 97.85, 100.44, 103.15, 103.41, 106.36, 109.69, 113.28, 117.18, 121.45, 126.20, 131.67, 138.53, 150.42, 136.13, 127.34, 120.89, 114.27, 108.88, 102.91, 97.16, 90.57, 82.56, 72.82, 60.38, 56.81, 55.62, 56.01, 57.58, 60.38, 63.84, 69, 74.46, 81.27};
//float leg22a[] = {138.71, 129.67, 123.04, 117.45, 112.52, 108.09, 104.06, 100.39, 97.05, 94.15, 93.90, 91.31, 88.93, 86.87, 85.15, 83.79, 82.81, 82.27, 82.21, 82.63, 74.63, 65.43, 57.94, 50.54, 45.19, 40.28, 36.75, 34.27, 33.70, 37.09, 51.38, 61.35, 70.07, 78.54, 85.99, 93.73, 100.67, 109.06, 116.97, 127.01};

//..........................................................150mm stride and 52mm height.........................................................................//

//float leg11a[] = {129.09, 133.03, 137.29, 141.94, 147.03, 153.63, 158.87, 166.01, 174.76, 189.73, 165.76, 153.36, 142.80, 133.63, 126.41, 121.89, 120.68, 123.09};
//float leg22a[] = {152.43, 143.30, 135.56, 128.89, 123.15, 118.29, 114.32, 111.26, 109.14, 108.05, 92.88, 82.98, 78.41, 79.90, 87.45, 99.71, 114.79, 131.77};

//..........................................................Left Gait.........................................................................//
//..........................................................100mm stride and 32mm height.........................................................................//

//float leg11a[] = {148.42, 153.39, 158.82, 164.86, 171.84, 180.73, 191.66, 193.66, 198.61, 178.15, 169.94, 161.86, 156.62, 147.56, 145.32, 146.68};
//float leg22a[] = {116.80, 113.02, 109.88, 107.39, 105.60, 104.53, 104.23, 102.16, 98.29, 90.40, 85.26, 81.54, 80.88, 87.70, 102.96, 111.09};

//..........................................................Right Gait.........................................................................//
//..........................................................100mm stride and 32mm height.........................................................................//

//float leg11a[] = {119.69, 123.17, 126.29, 129.54, 133.01, 136.76, 140.80, 145.17, 141.34, 136.17, 130.53, 124.45, 114.73, 114.70, 116.02, 118.33};
//float leg22a[] = {173.28, 160.61, 151.18, 143.26, 136.32, 130.16, 124.68, 119.82, 116.18, 112.46, 110.37, 111.12, 129.17, 140.11, 149.82, 162.12};

//...................................................................................................................................................//

const int arraySize = sizeof(leg11a) / sizeof(leg11a[0]);
const int arraySize2 = sizeof(leg22a) / sizeof(leg22a[0]);

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   IMU   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   IMU   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   IMU   ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   IMU   ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//#ifndef _BV
//#define _BV(n) (1<<(n))
//#endif
//
//// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
//// for both classes must be in the include path of your project
//#include "I2Cdev.h"
//
//#include "MPU6050_6Axis_MotionApps20.h"
//
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#include "Wire.h"
//#endif
//
//// class default I2C address is 0x68
//// specific I2C addresses may be passed as a parameter here
//// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
//// AD0 high = 0x69
//MPU6050 mpu;
////MPU6050 mpu(0x69); // <-- use for AD0 high
//
///* =========================================================================
//  NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
//  depends on the MPU-6050's INT pin being connected to the Arduino's
//  external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
//  digital I/O pin 2.
//  ========================================================================= */
//
////#define OUTPUT_READABLE_QUATERNION
//
////#define OUTPUT_READABLE_EULER
//
//#define OUTPUT_READABLE_YAWPITCHROLL
//
////#define OUTPUT_READABLE_REALACCEL
//
////#define OUTPUT_READABLE_WORLDACCEL
//
////#define OUTPUT_TEAPOT
//
//
//
//#define INTERRUPT_PIN 23  // use pin 2 on Arduino Uno & most boards
//
//
//// MPU control/status vars
//bool dmpReady = false;  // set true if DMP init was successful
//uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
//uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
//uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
//uint16_t fifoCount;     // count of all bytes currently in FIFO
//uint8_t fifoBuffer[64]; // FIFO storage buffer
//
//// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
//float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
//
//// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };
//
//
//
//// ================================================================
//// ===               INTERRUPT DETECTION ROUTINE                ===
//// ================================================================
//
//volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
//void dmpDataReady() {
//  mpuInterrupt = true;
//}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   IMU End   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   IMU End   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   IMU End   ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   IMU End   ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

float leg11[arraySize];
float leg22[arraySize];

float motorpwm[arraySize];
float leg1[arraySize];
float leg2[arraySize];

float diff1[arraySize];
float diff2[arraySize];
float rat1[arraySize];
float rat2[arraySize];

//#define ir1 33
//#define ir2 35
//#define ir3 37
//#define ir4 39
//#define ir5 A11

//#define myservo 5

//#define Button1 A9
//#define Button2 A10

int FR1;
int FR2;
int FL1;
int FL2;
int HR1;
int HR2;
int HL1;
int HL2;

#define encoderFL1PinA 45
#define encoderFL1PinB 47

#define encoderFL2PinA 51
#define encoderFL2PinB 49

#define encoderFR1PinA 29
#define encoderFR1PinB 31

#define encoderFR2PinA 25
#define encoderFR2PinB 27

#define encoderHL1PinA 48
#define encoderHL1PinB 50

#define encoderHL2PinA 53
#define encoderHL2PinB 52

#define encoderHR1PinA 26
#define encoderHR1PinB 28

#define encoderHR2PinA 22
#define encoderHR2PinB 24

//#define encoderXPinA 42
//#define encoderXPinB 40

volatile long encoderTicksFL1 = 0;
volatile long encoderTicksFL2 = 0;
volatile long encoderTicksFR1 = 0;
volatile long encoderTicksFR2 = 0;
volatile long encoderTicksHL1 = 0;
volatile long encoderTicksHL2 = 0;
volatile long encoderTicksHR1 = 0;
volatile long encoderTicksHR2 = 0;

#define lsFL1 A5
#define lsFL2 A6

#define lsFR1 A8
#define lsFR2 A7

#define lsHL1 A1
#define lsHL2 A2

#define lsHR1 A4
#define lsHR2 A3

//#define lsX A0

Motor motorFL1(9, 43);
Motor motorFL2(8, 41);
Motor motorFR1(7, 15);
Motor motorFR2(6, 14);

Motor motorHL1(10, 46);
Motor motorHL2(11, 44);
Motor motorHR1(12, 30);
Motor motorHR2(13, 32);

//Motor motorUp(4, 34);
//Motor motorGrip(3, 36);
//Motor motorX(2, 38);

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   Setup   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   Setup   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   Setup   ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------   Setup   ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void setup() {

  //  imuSetup();

  pinMode(lsFL1, INPUT_PULLUP);
  pinMode(lsFR1, INPUT_PULLUP);
  pinMode(lsHL1, INPUT_PULLUP);
  pinMode(lsHR1, INPUT_PULLUP);
  pinMode(lsFL2, INPUT_PULLUP);
  pinMode(lsFR2, INPUT_PULLUP);
  pinMode(lsHL2, INPUT_PULLUP);
  pinMode(lsHR2, INPUT_PULLUP);

  pinMode(encoderFL1PinA, INPUT_PULLUP);
  pinMode(encoderFL1PinB, INPUT_PULLUP);
  pinMode(encoderFL2PinA, INPUT_PULLUP);
  pinMode(encoderFL2PinB, INPUT_PULLUP);

  pinMode(encoderFR1PinA, INPUT_PULLUP);
  pinMode(encoderFR1PinB, INPUT_PULLUP);
  pinMode(encoderFR2PinA, INPUT_PULLUP);
  pinMode(encoderFR2PinB, INPUT_PULLUP);

  pinMode(encoderHL1PinA, INPUT_PULLUP);
  pinMode(encoderHL1PinB, INPUT_PULLUP);
  pinMode(encoderHL2PinA, INPUT_PULLUP);
  pinMode(encoderHL2PinB, INPUT_PULLUP);

  pinMode(encoderHR1PinA, INPUT_PULLUP);
  pinMode(encoderHR1PinB, INPUT_PULLUP);
  pinMode(encoderHR2PinA, INPUT_PULLUP);
  pinMode(encoderHR2PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderFL1PinA), doEncoderFL1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFL1PinB), doEncoderFL1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFL2PinA), doEncoderFL2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFL2PinB), doEncoderFL2B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderFR1PinA), doEncoderFR1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFR1PinB), doEncoderFR1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFR2PinA), doEncoderFR2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderFR2PinB), doEncoderFR2B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderHL1PinA), doEncoderHL1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderHL1PinB), doEncoderHL1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderHL2PinA), doEncoderHL2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderHL2PinB), doEncoderHL2B, CHANGE);

  attachInterrupt(digitalPinToInterrupt(encoderHR1PinA), doEncoderHR1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderHR1PinB), doEncoderHR1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderHR2PinA), doEncoderHR2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderHR2PinB), doEncoderHR2B, CHANGE);

  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------   PWM Calculations   ----------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------   PWM Calculations   ----------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------   PWM Calculations   --------------------------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------   PWM Calculations   --------------------------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  delay(2000);
  Serial.begin(115200);
  Serial.println("Commencing Homeing Sequence ... ");

  Serial.print("Size of array: ");
  Serial.print(arraySize);
  Serial.print("\t");
  Serial.println(arraySize2);

  if (reverse == 1) {
    for (int i = 0 ; i < arraySize ; i++) {
      leg11[i] = leg11a[arraySize - i - 1];
      leg22[i] = leg22a[arraySize - i - 1];
    }
  }
  else {
    for (int i = 0 ; i < arraySize ; i++) {
      leg11[i] = leg11a[i];
      leg22[i] = leg22a[i];
    }
  }

  for (int i = 0 ; i < arraySize ; i++) {

    leg1[i] = leg11[i] / 0.15;
    leg2[i] = leg22[i] / 0.15;
    diff1[i] = leg11[(i - 1 + arraySize) % arraySize] - leg11[i];
    diff2[i] = leg22[(i - 1 + arraySize) % arraySize] - leg22[i];
    rat1[i] = abs(diff1[i] / diff2[i]);
    rat2[i] = abs(diff2[i] / diff1[i]);

    if (rat1[i] < 1) {
      motorpwm[i] = rat1[i] * motorSpeed;
    }
    else {
      motorpwm[i] = rat2[i] * motorSpeed;
    }

    Serial.print(motorpwm[i]);
    Serial.print("\t");
    Serial.print(rat1[i]);
    Serial.print("\t");
    Serial.println(rat2[i]);

  }

  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //-----------------------  Serial Prints  ------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //-----------------------  Serial Prints  ------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //-----------------------  Serial Prints  -----------------------------------------------------------------------------------------------------------------------------------------------------------------
  //-----------------------  Serial Prints  -----------------------------------------------------------------------------------------------------------------------------------------------------------------
  //----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  delay(1000);
  Serial.println("5 ...\t");
  delay(500);
  Serial.println("4 ...\t");
  delay(500);
  Serial.println("3 ...\t");
  delay(500);
  Serial.println("2 ...\t");
  delay(500);
  Serial.println("1 ...");
  delay(500);
  Serial.println(" ... Motors Mobilized ... ");
  delay(500);

  homingonce();
  displayTicks();
  delay(1000);
  zeroTicks();
  now = millis();
  while (millis() - now < 3000) {
    botdumb(limit);
  }
  Serial.println("Setup Done");
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------   Main Loop   ------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------   Main Loop   ------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------   Main Loop   ------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------   Main Loop   ------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void loop() {
  Serial.println("Loop In");
  //botbalance(0, 800, 800, 0);

  //  l = digitalRead(left);
  //  r = digitalRead(right);

  //bottest();
  //angles();
  //botdumb(700);

  if (l == 0 && r == 0) {
    Serial.println("^^^^");
    botstraight();
  }
  else if (l == 1 && r == 0) {
    Serial.println("<<<<");
    botleft();
    //twoleg(800);
  }
  else if (l == 0 && r == 1) {
    botright();
    Serial.println(">>>>");
  }
  else {
    Serial.println("||||");
    botstop(500);
  }
  displayTicks();
  //lsRead();

}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------   Homing   -----------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------   Homing   -----------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------   Homing   -----------------------------------------------------------------------------------------------------------------------------------------------------------------
//-----------------------   Homing   -----------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void homingonce()
{
  while (digitalRead(lsFL1) != 0)
  {
    //lsRead();
    motorFL1.motorRun(60, LOW);
  }
  Serial.println("MotorFL1 Set");
  motorFL1.motorRun(0, HIGH);
  delay(160);

  //lsRead();
  while (digitalRead(lsFL2) != 0)
  {
    //lsRead();
    motorFL2.motorRun(60, LOW);
  }
  Serial.println("MotorFL2 Set");
  motorFL2.motorRun(0, HIGH);
  delay(160);


  //lsRead();
  while (digitalRead(lsFR1) != 0)
  {
    //lsRead();
    motorFR1.motorRun(60, LOW);
  }
  Serial.println("MotorFR1 Set");
  motorFR1.motorRun(0, HIGH);
  delay(160);

  //lsRead();
  while (digitalRead(lsFR2) != 0)
  {
    //lsRead();
    motorFR2.motorRun(60, LOW);
  }
  Serial.println("MotorFR2 Set");
  motorFR2.motorRun(0, HIGH);
  delay(160);

  //lsRead();
  while (digitalRead(lsHL1) != 0)
  {
    //lsRead();
    motorHL1.motorRun(60, LOW);
  }
  Serial.println("MotorHL1 Set");
  motorHL1.motorRun(0, HIGH);
  delay(160);

  //lsRead();
  while (digitalRead(lsHL2) != 0)
  {
    //lsRead();
    motorHL2.motorRun(60, LOW);
  }
  Serial.println("MotorHL2 Set");
  motorHL2.motorRun(0, HIGH);
  delay(100);


  //lsRead();
  while (digitalRead(lsHR1) != 0)
  {
    //lsRead();
    motorHR1.motorRun(60, LOW);
  }
  Serial.println("MotorHR1 Set");
  motorHR1.motorRun(0, HIGH);
  delay(100);

  //lsRead();
  while (digitalRead(lsHR2) != 0)
  {
    //lsRead();
    motorHR2.motorRun(60, LOW);
  }
  Serial.println("MotorHR2 Set");
  motorHR2.motorRun(0, HIGH);
  delay(1000);

}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------   Limit Switch Reads   --------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------   Limit Switch Reads   --------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------   Limit Switch Reads   --------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------------   Limit Switch Reads   --------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void lsRead()
{
  Serial.print(digitalRead(lsFL1));
  Serial.print(" ");
  Serial.print(digitalRead(lsFL2));
  Serial.print(" ");
  Serial.print(digitalRead(lsFR1));
  Serial.print(" ");
  Serial.print(digitalRead(lsFR2));
  Serial.print(" ");
  Serial.print(digitalRead(lsHL1));
  Serial.print(" ");
  Serial.print(digitalRead(lsHL2));
  Serial.print(" ");
  Serial.print(digitalRead(lsHR1));
  Serial.print(" ");
  Serial.println(digitalRead(lsHR2));
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Zero Ticks   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Zero Ticks   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Zero Ticks   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Zero Ticks   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void zeroTicks()
{
  encoderTicksFL1 = 0;
  encoderTicksFL2 = 0;
  encoderTicksFR1 = 0;
  encoderTicksFR2 = 0;
  encoderTicksHL1 = 0;
  encoderTicksHL2 = 0;
  encoderTicksHR1 = 0;
  encoderTicksHR2 = 0;
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------   Encoder Ticks   ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------   Encoder Ticks   ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------   Encoder Ticks   ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------   Encoder Ticks   ------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void displayTicks()
{
  Serial.print(encoderTicksFL1);
  Serial.print("\t");
  Serial.print(encoderTicksFL2);
  Serial.print("\t");

  Serial.print(encoderTicksFR1);
  Serial.print("\t");
  Serial.print(encoderTicksFR2);
  Serial.print("\t");

  Serial.print(encoderTicksHL1);
  Serial.print("\t");
  Serial.print(encoderTicksHL2);
  Serial.print("\t");

  Serial.print(encoderTicksHR1);
  Serial.print("\t");
  Serial.println(encoderTicksHR2);
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Encoder Calculations    -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Encoder Calculations    -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Encoder Calculations    -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Encoder Calculations    -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void doEncoderFL1A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderFL1PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFL1PinB) == LOW) {
      encoderTicksFL1 = encoderTicksFL1 + 1;         // CW
    }
    else {
      encoderTicksFL1 = encoderTicksFL1 - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFL1PinB) == HIGH) {
      encoderTicksFL1 = encoderTicksFL1 + 1;          // CW
    }
    else {
      encoderTicksFL1 = encoderTicksFL1 - 1;          // CCW
    }
  }
}

void doEncoderFL1B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoderFL1PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoderFL1PinA) == HIGH) {
      encoderTicksFL1 = encoderTicksFL1 + 1;         // CW
    }
    else {
      encoderTicksFL1 = encoderTicksFL1 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFL1PinA) == LOW) {
      encoderTicksFL1 = encoderTicksFL1 + 1;          // CW
    }
    else {
      encoderTicksFL1 = encoderTicksFL1 - 1;          // CCW
    }
  }
}

void doEncoderFL2A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderFL2PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFL2PinB) == LOW) {
      encoderTicksFL2 = encoderTicksFL2 + 1;         // CW
    }
    else {
      encoderTicksFL2 = encoderTicksFL2 - 1;        // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFL2PinB) == HIGH) {
      encoderTicksFL2 = encoderTicksFL2 + 1;          // CW
    }
    else {
      encoderTicksFL2 = encoderTicksFL2 - 1;          // CCW
    }
  }
}

void doEncoderFL2B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoderFL2PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoderFL2PinA) == HIGH) {
      encoderTicksFL2 = encoderTicksFL2 + 1;         // CW
    }
    else {
      encoderTicksFL2 = encoderTicksFL2 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFL2PinA) == LOW) {
      encoderTicksFL2 = encoderTicksFL2 + 1;          // CW
    }
    else {
      encoderTicksFL2 = encoderTicksFL2 - 1;          // CCW
    }
  }
}

void doEncoderFR1A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderFR1PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFR1PinB) == LOW) {
      encoderTicksFR1 = encoderTicksFR1 + 1;         // CW
    }
    else {
      encoderTicksFR1 = encoderTicksFR1 - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFR1PinB) == HIGH) {
      encoderTicksFR1 = encoderTicksFR1 + 1;          // CW
    }
    else {
      encoderTicksFR1 = encoderTicksFR1 - 1;          // CCW
    }
  }
}

void doEncoderFR1B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoderFR1PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoderFR1PinA) == HIGH) {
      encoderTicksFR1 = encoderTicksFR1 + 1;         // CW
    }
    else {
      encoderTicksFR1 = encoderTicksFR1 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFR1PinA) == LOW) {
      encoderTicksFR1 = encoderTicksFR1 + 1;          // CW
    }
    else {
      encoderTicksFR1 = encoderTicksFR1 - 1;          // CCW
    }
  }
}

void doEncoderFR2A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderFR2PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFR2PinB) == LOW) {
      encoderTicksFR2 = encoderTicksFR2 + 1;         // CW
    }
    else {
      encoderTicksFR2 = encoderTicksFR2 - 1;        // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFR2PinB) == HIGH) {
      encoderTicksFR2 = encoderTicksFR2 + 1;          // CW
    }
    else {
      encoderTicksFR2 = encoderTicksFR2 - 1;          // CCW
    }
  }
}

void doEncoderFR2B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoderFR2PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoderFR2PinA) == HIGH) {
      encoderTicksFR2 = encoderTicksFR2 + 1;         // CW
    }
    else {
      encoderTicksFR2 = encoderTicksFR2 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderFR2PinA) == LOW) {
      encoderTicksFR2 = encoderTicksFR2 + 1;          // CW
    }
    else {
      encoderTicksFR2 = encoderTicksFR2 - 1;          // CCW
    }
  }
}

void doEncoderHL1A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderHL1PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHL1PinB) == LOW) {
      encoderTicksHL1 = encoderTicksHL1 + 1;         // CW
    }
    else {
      encoderTicksHL1 = encoderTicksHL1 - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHL1PinB) == HIGH) {
      encoderTicksHL1 = encoderTicksHL1 + 1;          // CW
    }
    else {
      encoderTicksHL1 = encoderTicksHL1 - 1;          // CCW
    }
  }
}

void doEncoderHL1B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoderHL1PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoderHL1PinA) == HIGH) {
      encoderTicksHL1 = encoderTicksHL1 + 1;         // CW
    }
    else {
      encoderTicksHL1 = encoderTicksHL1 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHL1PinA) == LOW) {
      encoderTicksHL1 = encoderTicksHL1 + 1;          // CW
    }
    else {
      encoderTicksHL1 = encoderTicksHL1 - 1;          // CCW
    }
  }
}

void doEncoderHL2A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderHL2PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHL2PinB) == LOW) {
      encoderTicksHL2 = encoderTicksHL2 + 1;         // CW
    }
    else {
      encoderTicksHL2 = encoderTicksHL2 - 1;        // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHL2PinB) == HIGH) {
      encoderTicksHL2 = encoderTicksHL2 + 1;          // CW
    }
    else {
      encoderTicksHL2 = encoderTicksHL2 - 1;          // CCW
    }
  }
}

void doEncoderHL2B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoderHL2PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoderHL2PinA) == HIGH) {
      encoderTicksHL2 = encoderTicksHL2 + 1;         // CW
    }
    else {
      encoderTicksHL2 = encoderTicksHL2 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHL2PinA) == LOW) {
      encoderTicksHL2 = encoderTicksHL2 + 1;          // CW
    }
    else {
      encoderTicksHL2 = encoderTicksHL2 - 1;          // CCW
    }
  }
}

void doEncoderHR1A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderHR1PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHR1PinB) == LOW) {
      encoderTicksHR1 = encoderTicksHR1 + 1;         // CW
    }
    else {
      encoderTicksHR1 = encoderTicksHR1 - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHR1PinB) == HIGH) {
      encoderTicksHR1 = encoderTicksHR1 + 1;          // CW
    }
    else {
      encoderTicksHR1 = encoderTicksHR1 - 1;          // CCW
    }
  }
}

void doEncoderHR1B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoderHR1PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoderHR1PinA) == HIGH) {
      encoderTicksHR1 = encoderTicksHR1 + 1;         // CW
    }
    else {
      encoderTicksHR1 = encoderTicksHR1 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHR1PinA) == LOW) {
      encoderTicksHR1 = encoderTicksHR1 + 1;          // CW
    }
    else {
      encoderTicksHR1 = encoderTicksHR1 - 1;          // CCW
    }
  }
}

void doEncoderHR2A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoderHR2PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHR2PinB) == LOW) {
      encoderTicksHR2 = encoderTicksHR2 + 1;         // CW
    }
    else {
      encoderTicksHR2 = encoderTicksHR2 - 1;        // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHR2PinB) == HIGH) {
      encoderTicksHR2 = encoderTicksHR2 + 1;          // CW
    }
    else {
      encoderTicksHR2 = encoderTicksHR2 - 1;          // CCW
    }
  }
}

void doEncoderHR2B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoderHR2PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoderHR2PinA) == HIGH) {
      encoderTicksHR2 = encoderTicksHR2 + 1;         // CW
    }
    else {
      encoderTicksHR2 = encoderTicksHR2 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoderHR2PinA) == LOW) {
      encoderTicksHR2 = encoderTicksHR2 + 1;          // CW
    }
    else {
      encoderTicksHR2 = encoderTicksHR2 - 1;          // CCW
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------   Bot Stanz   --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------   Bot Stanz   --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------   Bot Stanz   --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//--------------   Bot Stanz   --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void botdumb(int stan) {
  int now = millis();
  while (millis() - now < tm) {
    motorFR1.singleMotorPID(stan, encoderTicksFR1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorFR2.singleMotorPID(stan, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorFL1.singleMotorPID(stan, encoderTicksFL1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorFL2.singleMotorPID(stan, encoderTicksFL2, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHR1.singleMotorPID(stan, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHR2.singleMotorPID(stan, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHL1.singleMotorPID(stan, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHL2.singleMotorPID(stan, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Straight   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Straight   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Straight   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Straight   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void botstraight() {
  Serial.println("Botstraight Function");
  for (int pos = 0 ; pos < arraySize; pos++)
  {
    Serial.println("Pos Botstraight");
    Serial.println(pos);
    int txt = (pos + arraySize / 2) % arraySize;
    Serial.println(txt);
    now = millis();
    while (millis() - now < tm)
    {
      if (rat1[pos] < 1)
      {
        motorHL1.singleMotorPID(leg1[pos], encoderTicksHL1, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[pos], encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFR1.singleMotorPID(leg1[pos], encoderTicksFR1, motorpwm[pos]  - threshold, motorpwm[pos]  + threshold, kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[pos], encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else
      {
        motorHL1.singleMotorPID(leg1[pos], encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[pos], encoderTicksHL2, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
        motorFR1.singleMotorPID(leg1[pos], encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[pos], encoderTicksFR2, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
      }
      if (rat1[txt] < 1)
      {
        motorHR1.singleMotorPID(leg1[txt], encoderTicksHR1, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[txt], encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFL1.singleMotorPID(leg1[txt], encoderTicksFL1, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[txt], encoderTicksFL2, baseSpeed , maxSpeed , kpp, kii, kdd);
      }
      else
      {
        motorHR1.singleMotorPID(leg1[txt], encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[txt], encoderTicksHR2, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorFL1.singleMotorPID(leg1[txt], encoderTicksFL1, baseSpeed, maxSpeed , kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[txt], encoderTicksFL2, motorpwm[txt]  - threshold, motorpwm[txt]  + threshold, kpp, kii, kdd);
      }
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------   Bot Right Turn   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------   Bot Right Turn   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------   Bot Right Turn   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//------------------   Bot Right Turn   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void botright() {
  Serial.println("botright");
  for (int pos = arraySize / 2 ; pos < arraySize; pos++) {
    Serial.println("loop of right");
    Serial.println(pos);
    int txt = ((arraySize / 2 - pos)  + arraySize - 1) % arraySize;
    Serial.println(txt);
    now = millis();
    while (millis() - now < tm) {
      if (rat1[pos] < 1) {
        motorHR1.singleMotorPID(leg1[pos], encoderTicksHR1, motorpwm[pos] - threshold,  motorpwm[pos] + threshold, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[pos], encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFR1.singleMotorPID(leg1[pos], encoderTicksFR1, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[pos], encoderTicksFR2, baseSpeed , maxSpeed , kpp, kii, kdd);
      }
      else {
        motorHR1.singleMotorPID(leg1[pos], encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[pos], encoderTicksHR2,  motorpwm[pos] - threshold,  motorpwm[pos] + threshold, kpp, kii, kdd);
        motorFR1.singleMotorPID(leg1[pos], encoderTicksFR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[pos], encoderTicksFR2, motorpwm[pos]  - threshold, motorpwm[pos]  + threshold, kpp, kii, kdd);
      }
      if (rat1[txt] < 1) {
        motorHL1.singleMotorPID(leg1[txt], encoderTicksHL1, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[txt], encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFL1.singleMotorPID(leg1[txt], encoderTicksFL1,  motorpwm[txt]  - threshold,  motorpwm[txt]  + threshold, kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[txt], encoderTicksFL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else {
        motorHL1.singleMotorPID(leg1[txt], encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[txt], encoderTicksHL2, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorFL1.singleMotorPID(leg1[txt], encoderTicksFL1, baseSpeed , maxSpeed , kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[txt], encoderTicksFL2,  motorpwm[txt] - threshold,  motorpwm[txt] + threshold, kpp, kii, kdd);
      }
    }
  }
  Serial.println("out stride");
  botstop(500);

  for (int pos = 0; pos < arraySize / 2; pos++) {
    Serial.println("HR");
    Serial.println(pos);
    now = millis();
    while (millis() - now < tm) {
      motorFL1.singleMotorPID(FL1, encoderTicksFL1, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorFL2.singleMotorPID(FL2, encoderTicksFL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorFR1.singleMotorPID(FR1, encoderTicksFR1, baseSpeed, maxSpeed , kpp, kii, kdd);
      motorFR2.singleMotorPID(FR2, encoderTicksFR2, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorHL1.singleMotorPID(HL1, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL2.singleMotorPID(HL2, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      if (rat1[pos] < 1) {
        motorHR1.singleMotorPID(leg1[pos], encoderTicksHR1,  motorpwm[pos] - threshold,  motorpwm[pos] + threshold, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[pos], encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else {
        motorHR1.singleMotorPID(leg1[pos], encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[pos], encoderTicksHR2,  motorpwm[pos] - threshold,  motorpwm[pos] + threshold, kpp, kii, kdd);
      }
    }
  }
  botstop(500);

  for (int pos = 0 ; pos < arraySize / 2; pos++) {
    Serial.println("FR");
    Serial.println(pos);
    now = millis();
    while (millis() - now < tm) {
      motorFL1.singleMotorPID(FL1, encoderTicksFL1, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorFL2.singleMotorPID(FL2, encoderTicksFL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL1.singleMotorPID(HL1, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL2.singleMotorPID(HL2, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR1.singleMotorPID(HR1, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR2.singleMotorPID(HR2, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      if (rat1[pos] < 1) {
        motorFR1.singleMotorPID(leg1[pos], encoderTicksFR1, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[pos], encoderTicksFR2, baseSpeed , maxSpeed , kpp, kii, kdd);
      }
      else {
        motorFR1.singleMotorPID(leg1[pos], encoderTicksFR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[pos], encoderTicksFR2, motorpwm[pos]  - threshold, motorpwm[pos]  + threshold, kpp, kii, kdd);
      }
    }
  }
  botstop(500);

  for (int pos = 0 ; pos < arraySize / 2; pos++) {
    Serial.println("HL");
    int txt = ((arraySize / 2 - pos)  + arraySize - 1) % arraySize;
    Serial.println(txt);
    now = millis();
    while (millis() - now < tm) {
      motorHR1.singleMotorPID(HR1, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR2.singleMotorPID(HR2, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorFL1.singleMotorPID(FL1, encoderTicksFL1, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorFL2.singleMotorPID(FL2, encoderTicksFL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorFR1.singleMotorPID(FR1, encoderTicksFR1, baseSpeed, maxSpeed , kpp, kii, kdd);
      motorFR2.singleMotorPID(FR2, encoderTicksFR2, baseSpeed , maxSpeed , kpp, kii, kdd);
      if (rat1[txt] < 1) {
        motorHL1.singleMotorPID(leg1[txt], encoderTicksHL1, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[txt], encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else {
        motorHL1.singleMotorPID(leg1[txt], encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[txt], encoderTicksHL2, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
      }
    }
  }
  botstop(500);

  for (int pos = 0 ; pos < arraySize / 2; pos++) {
    Serial.println("FL");
    int txt = ((arraySize / 2 - pos)  + arraySize - 1) % arraySize;
    Serial.println(txt);
    now = millis();
    while (millis() - now < tm) {
      motorFR1.singleMotorPID(FR1, encoderTicksFR1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorFR2.singleMotorPID(FR2, encoderTicksFR2, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorHL1.singleMotorPID(HL1, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL2.singleMotorPID(HL2, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR1.singleMotorPID(HR1, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR2.singleMotorPID(HR2, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      if (rat1[txt] < 1) {
        motorFL1.singleMotorPID(leg1[txt], encoderTicksFL1,  motorpwm[txt]  - threshold,  motorpwm[txt] + threshold, kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[txt], encoderTicksFL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else {
        motorFL1.singleMotorPID(leg1[txt], encoderTicksFL1, baseSpeed , maxSpeed , kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[txt], encoderTicksFL2,  motorpwm[txt] - threshold,  motorpwm[txt] + threshold, kpp, kii, kdd);
      }
    }
  }
  botstop(1000);

  Serial.println("right out");
}


//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Bor Left Turn   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Bot Left Turn   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Bor Left Turn   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------   Bot Left Turn   -------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


void botleft() {
  Serial.println("botleft");
  for (int pos = arraySize / 2 ; pos < arraySize; pos++) {
    Serial.println("loop of left");
    Serial.println(pos);
    int txt = ((arraySize / 2 - pos)  + arraySize - 1) % arraySize;
    Serial.println(txt);
    now = millis();
    while (millis() - now < tm) {
      if (rat1[pos] < 1) {
        motorHL1.singleMotorPID(leg1[pos], encoderTicksHL1, motorpwm[pos] - threshold,  motorpwm[pos] + threshold, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[pos], encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFL1.singleMotorPID(leg1[pos], encoderTicksFL1, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[pos], encoderTicksFL2, baseSpeed , maxSpeed , kpp, kii, kdd);
      }
      else {
        motorHL1.singleMotorPID(leg1[pos], encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[pos], encoderTicksHL2,  motorpwm[pos] - threshold,  motorpwm[pos] + threshold, kpp, kii, kdd);
        motorFL1.singleMotorPID(leg1[pos], encoderTicksFL1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[pos], encoderTicksFL2, motorpwm[pos]  - threshold, motorpwm[pos]  + threshold, kpp, kii, kdd);
      }
      if (rat1[txt] < 1) {
        motorHR1.singleMotorPID(leg1[txt], encoderTicksHR1, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[txt], encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFR1.singleMotorPID(leg1[txt], encoderTicksFR1,  motorpwm[txt]  - threshold,  motorpwm[txt]  + threshold, kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[txt], encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else {
        motorHR1.singleMotorPID(leg1[txt], encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[txt], encoderTicksHR2, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorFR1.singleMotorPID(leg1[txt], encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[txt], encoderTicksFR2,  motorpwm[txt] - threshold,  motorpwm[txt] + threshold, kpp, kii, kdd);
      }
    }
  }
  Serial.println("out stride");
  botstop(500);

  for (int pos = 0; pos < arraySize / 2; pos++) {
    Serial.println("HL");
    Serial.println(pos);
    now = millis();
    while (millis() - now < tm) {
      motorFR1.singleMotorPID(FR1, encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorFR2.singleMotorPID(FR2, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorFL1.singleMotorPID(FL1, encoderTicksFL1, baseSpeed, maxSpeed , kpp, kii, kdd);
      motorFL2.singleMotorPID(FL2, encoderTicksFL2, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorHR1.singleMotorPID(HR1, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR2.singleMotorPID(HR2, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      if (rat1[pos] < 1) {
        motorHL1.singleMotorPID(leg1[pos], encoderTicksHL1,  motorpwm[pos] - threshold,  motorpwm[pos] + threshold, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[pos], encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else {
        motorHL1.singleMotorPID(leg1[pos], encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[pos], encoderTicksHL2,  motorpwm[pos] - threshold,  motorpwm[pos] + threshold, kpp, kii, kdd);
      }
    }
  }
  botstop(500);

  for (int pos = 0 ; pos < arraySize / 2; pos++) {
    Serial.println("FL");
    Serial.println(pos);
    now = millis();
    while (millis() - now < tm) {
      motorFR1.singleMotorPID(FR1, encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorFR2.singleMotorPID(FR2, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR1.singleMotorPID(HR1, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR2.singleMotorPID(HR2, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL1.singleMotorPID(HL1, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL2.singleMotorPID(HL2, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      if (rat1[pos] < 1) {
        motorFL1.singleMotorPID(leg1[pos], encoderTicksFL1, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[pos], encoderTicksFL2, baseSpeed , maxSpeed , kpp, kii, kdd);
      }
      else {
        motorFL1.singleMotorPID(leg1[pos], encoderTicksFL1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[pos], encoderTicksFL2, motorpwm[pos]  - threshold, motorpwm[pos]  + threshold, kpp, kii, kdd);
      }
    }
  }
  botstop(500);

  for (int pos = 0 ; pos < arraySize / 2; pos++) {
    Serial.println("HR");
    int txt = ((arraySize / 2 - pos)  + arraySize - 1) % arraySize;
    Serial.println(txt);
    now = millis();
    while (millis() - now < tm) {
      motorHL1.singleMotorPID(HL1, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL2.singleMotorPID(HL2, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorFR1.singleMotorPID(FR1, encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorFR2.singleMotorPID(FR2, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorFL1.singleMotorPID(FL1, encoderTicksFL1, baseSpeed, maxSpeed , kpp, kii, kdd);
      motorFL2.singleMotorPID(FL2, encoderTicksFL2, baseSpeed , maxSpeed , kpp, kii, kdd);
      if (rat1[txt] < 1) {
        motorHR1.singleMotorPID(leg1[txt], encoderTicksHR1, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[txt], encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else {
        motorHR1.singleMotorPID(leg1[txt], encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHR2.singleMotorPID(leg2[txt], encoderTicksHR2, motorpwm[txt] - threshold, motorpwm[txt] + threshold, kpp, kii, kdd);
      }
    }
  }
  botstop(500);

  for (int pos = 0 ; pos < arraySize / 2; pos++) {
    Serial.println("FR");
    int txt = ((arraySize / 2 - pos)  + arraySize - 1) % arraySize;
    Serial.println(txt);
    now = millis();
    while (millis() - now < tm) {
      //      Serial.println("ohh yeah !!!");
      motorFL1.singleMotorPID(FL1, encoderTicksFL1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorFL2.singleMotorPID(FL2, encoderTicksFL2, baseSpeed , maxSpeed , kpp, kii, kdd);
      motorHR1.singleMotorPID(HR1, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHR2.singleMotorPID(HR2, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL1.singleMotorPID(HL1, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
      motorHL2.singleMotorPID(HL2, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
      if (rat1[txt] < 1) {
        motorFR1.singleMotorPID(leg1[txt], encoderTicksFR1,  motorpwm[txt]  - threshold,  motorpwm[txt] + threshold, kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[txt], encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else {
        motorFR1.singleMotorPID(leg1[txt], encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
        motorFR2.singleMotorPID(leg2[txt], encoderTicksFR2,  motorpwm[txt] - threshold,  motorpwm[txt] + threshold, kpp, kii, kdd);
      }
    }
  }
  botstop(1000);

  Serial.println("left out");
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Position Retaining   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Position Retaining   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Position Retaining   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Position Retaining   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void botstop(int stp) {

  FR1 = encoderTicksFR1;
  FR2 = encoderTicksFR2;
  FL1 = encoderTicksFL1;
  FL2 = encoderTicksFL2;
  HR1 = encoderTicksHR1;
  HR2 = encoderTicksHR2;
  HL1 = encoderTicksHL1;
  HL2 = encoderTicksHL2;

  now = millis();
  while (millis() - now < stp) {
    motorFR1.singleMotorPID(FR1, encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
    motorFR2.singleMotorPID(FR2, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorFL1.singleMotorPID(FL1, encoderTicksFL1, baseSpeed, maxSpeed , kpp, kii, kdd);
    motorFL2.singleMotorPID(FL2, encoderTicksFL2, baseSpeed , maxSpeed , kpp, kii, kdd);
    motorHR1.singleMotorPID(HR1, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHR2.singleMotorPID(HR2, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHL1.singleMotorPID(HL1, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHL2.singleMotorPID(HL2, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Test   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Test   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Test   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Test   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void bottest() {
  now = millis();
  while (millis() - now < 5000) {
    botstraight();
  }

  now = millis();
  while (millis() - now < 10000) {
    botleft();
  }

  now = millis();
  while (millis() - now < 5000) {
    botstraight();
  }

  now = millis();
  while (millis() - now < 10000) {
    botright();
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Two Leg Turn   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Two Leg Turn   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Two Leg Turn   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Two Leg Turn   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void twoleg(int stan) {
  Serial.println("Botstraight continous");
  for (int pos = 0 ; pos < arraySize; pos++)
  {
    Serial.println("FOR LOOP");
    Serial.println(pos);
    int txt = (pos + arraySize / 2) % arraySize;
    Serial.println(txt);
    now = millis();
    while (millis() - now < tm)
    {
      if (rat1[pos] < 1)
      {
        motorHL1.singleMotorPID(leg1[pos], encoderTicksHL1, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[pos], encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFR1.singleMotorPID(stan, encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
        motorFR2.singleMotorPID(stan, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      else
      {
        motorHL1.singleMotorPID(leg1[pos], encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHL2.singleMotorPID(leg2[pos], encoderTicksHL2, motorpwm[pos] - threshold, motorpwm[pos] + threshold, kpp, kii, kdd);
        motorFR1.singleMotorPID(stan, encoderTicksFR1, baseSpeed , maxSpeed , kpp, kii, kdd);
        motorFR2.singleMotorPID(stan, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
      }
      if (rat1[(pos + arraySize / 2) % arraySize] < 1)
      {
        motorHR1.singleMotorPID(stan, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHR2.singleMotorPID(stan, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFL1.singleMotorPID(leg1[(pos + arraySize / 2) % arraySize], encoderTicksFL1, motorpwm[(pos + arraySize / 2) % arraySize] - threshold, motorpwm[(pos + arraySize / 2) % arraySize] + threshold, kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[(pos + arraySize / 2) % arraySize], encoderTicksFL2, baseSpeed , maxSpeed , kpp, kii, kdd);
      }
      else
      {
        motorHR1.singleMotorPID(stan, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorHR2.singleMotorPID(stan, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
        motorFL1.singleMotorPID(leg1[(pos + arraySize / 2) % arraySize], encoderTicksFL1, baseSpeed, maxSpeed , kpp, kii, kdd);
        motorFL2.singleMotorPID(leg2[(pos + arraySize / 2) % arraySize], encoderTicksFL2, motorpwm[(pos + arraySize / 2) % arraySize]  - threshold, motorpwm[(pos + arraySize / 2) % arraySize]  + threshold, kpp, kii, kdd);
      }
    }
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Balancing   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Balancing   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Balancing   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Balancing   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void botbalance(int r1, int r2, int l1, int l2) {
  int now = millis();
  while (millis() - now < tm) {
    motorFR1.singleMotorPID(r1, encoderTicksFR1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorFR2.singleMotorPID(r1, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorFL1.singleMotorPID(l1, encoderTicksFL1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorFL2.singleMotorPID(l1, encoderTicksFL2, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHR1.singleMotorPID(r2, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHR2.singleMotorPID(r2, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHL1.singleMotorPID(l2, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
    motorHL2.singleMotorPID(l2, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
  }
}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Tilting   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Tilting   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Tilting   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   Bot Tilting   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//void tilt() {
//
//  //  Serial.print(ang1);
//  //  Serial.print("\t");
//  //  Serial.print(ang2);
//  //  Serial.print("\t");
//  //  Serial.print(ang3);
//  //  Serial.print("\t");
//  //  Serial.println(ang4);
//
//
//  motorFL1.singleMotorPID(ang1, encoderTicksFL1, baseSpeed, maxSpeed, kpp, kii, kdd);
//  motorFL2.singleMotorPID(ang1, encoderTicksFL2, baseSpeed, maxSpeed, kpp, kii, kdd);
//  motorFR1.singleMotorPID(ang2, encoderTicksFR1, baseSpeed, maxSpeed, kpp, kii, kdd);
//  motorFR2.singleMotorPID(ang2, encoderTicksFR2, baseSpeed, maxSpeed, kpp, kii, kdd);
//  motorHL1.singleMotorPID(ang3, encoderTicksHL1, baseSpeed, maxSpeed, kpp, kii, kdd);
//  motorHL2.singleMotorPID(ang3, encoderTicksHL2, baseSpeed, maxSpeed, kpp, kii, kdd);
//  motorHR1.singleMotorPID(ang4, encoderTicksHR1, baseSpeed, maxSpeed, kpp, kii, kdd);
//  motorHR2.singleMotorPID(ang4, encoderTicksHR2, baseSpeed, maxSpeed, kpp, kii, kdd);
//
//}

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   IMU angle Calculations   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   IMU angle Calculations   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   IMU angle Calculations   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//---------------------   IMU angle Calculations   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

//void angles() {
//  // if programming failed, don't try to do anything
//  if (!dmpReady) return;
//
//  // wait for MPU interrupt or extra packet(s) available
//  while (!mpuInterrupt && fifoCount < packetSize) {
//    if (mpuInterrupt && fifoCount < packetSize) {
//      // try to get out of the infinite loop
//      fifoCount = mpu.getFIFOCount();
//    }
//    // other program behavior stuff here
//  }
//
//  // reset interrupt flag and get INT_STATUS byte
//  mpuInterrupt = false;
//  mpuIntStatus = mpu.getIntStatus();
//
//  // get current FIFO count
//  fifoCount = mpu.getFIFOCount();
//
//  // check for overflow (this should never happen unless our code is too inefficient)
//  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
//    // reset so we can continue cleanly
//    mpu.resetFIFO();
//    fifoCount = mpu.getFIFOCount();
//    Serial.println(F("FIFO overflow!"));
//
//    // otherwise, check for DMP data ready interrupt (this should happen frequently)
//  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
//    // wait for correct available data length, should be a VERY short wait
//    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
//
//    // read a packet from FIFO
//    mpu.getFIFOBytes(fifoBuffer, packetSize);
//
//    // track FIFO count here in case there is > 1 packet available
//    // (this lets us immediately read more without waiting for an interrupt)
//    fifoCount -= packetSize;
//
//#ifdef OUTPUT_READABLE_YAWPITCHROLL
//    // display Euler angles in degrees
//    mpu.dmpGetQuaternion(&q, fifoBuffer);
//    mpu.dmpGetGravity(&gravity, &q);
//    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//
//    roll = ypr[1] * 180 / M_PI;
//    pitch = ypr[2] * 180 / M_PI;
//
//    Serial.print(roll);
//    Serial.print("\t");
//    Serial.println(pitch);
//
//    ang1 = limit + (-roll - pitch) * angmul;
//    ang3 = limit + ( roll - pitch) * angmul;
//    ang2 = limit + (-roll + pitch) * angmul;
//    ang4 = limit + ( roll + pitch) * angmul;
//
//#endif
//
//
//    int now = millis();
//    while ((millis() - now) < tm) {
//      tilt();
//    }
//  }
//}
//
////----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
////---------------------   IMU Setup   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
////---------------------   IMU Setup   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
////---------------------   IMU Setup   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
////---------------------   IMU Setup   -------------------------------------------------------------------------------------------------------------------------------------------------------------------
////----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
//
//void imuSetup() {
//  // join I2C bus (I2Cdev library doesn't do this automatically)
//#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//  Wire.begin();
//  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
//#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
//  Fastwire::setup(400, true);
//#endif
//
//  // initialize serial communication
//  // (115200 chosen because it is required for Teapot Demo output, but it's
//  // really up to you depending on your project)
//  Serial.begin(115200);
//  while (!Serial); // wait for Leonardo enumeration, others continue immediately
//
//  Serial.println(F("Initializing I2C devices..."));
//  mpu.initialize();
//  pinMode(INTERRUPT_PIN, INPUT);
//
//  // verify connection
//  //Serial.println(F("Testing device connections..."));
//  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
//
//  // wait for ready
//  //  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
//  //  while (Serial.available() && Serial.read()); // empty buffer
//  //  while (!Serial.available());                 // wait for data
//  //  while (Serial.available() && Serial.read()); // empty buffer again
//
//  // load and configure the DMP
//  Serial.println(F("Initializing DMP..."));
//
//
//  devStatus = mpu.dmpInitialize();
//
//  // make sure it worked (returns 0 if so)
//  if (devStatus == 0) {
//    // turn on the DMP, now that it's ready
//    //Serial.println(F("Enabling DMP..."));
//    mpu.setDMPEnabled(true);
//
//
//    // enable Arduino interrupt detection
//    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
//    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
//    //Serial.println(F(")..."));
//    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
//    mpuIntStatus = mpu.getIntStatus();
//
//    // set our DMP Ready flag so the main loop() function knows it's okay to use it
//    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
//    dmpReady = true;
//
//    // get expected DMP packet size for later comparison
//    packetSize = mpu.dmpGetFIFOPacketSize();
//
//
//  } else {
//    // ERROR!
//    // 1 = initial memory load failed
//    // 2 = DMP configuration updates failed
//    // (if it's going to break, usually the code will be 1)
//    Serial.print(F("DMP Initialization failed (code "));
//    Serial.print(devStatus);
//    Serial.println(F(")"));
//  }
//}
