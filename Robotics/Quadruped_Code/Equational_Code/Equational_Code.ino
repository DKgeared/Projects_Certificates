#include<math.h>
double Ti = 1;
double dt = 0.025;
double stroke = 150; //dia of gait
double dc = 0.8;
double ht = 10; //height of the gait
double flight = -180;  //hight from motor tothe stroke
double l1 = 150;
double l2 = 225;
double l3 = 225;
double l4 = 150;
double l5 = 66;
double l6 = 95;
double a0;
double a1;
double a2;
double a3;
double K1;
double K2;
double K3;
double P1;
double P2;
double P3;

double theta_1;
double theta_4;

int tm = 15;

double kpp = 0.07;
double kii = 0.1;
double kdd = 0.9;
double baseSpeed = 30;
double maxSpeed = 50;

#include <Motor2.h>

Motor motor1(3, 2);
Motor motor2(5, 4);

//#define ls1 33
//#define ls2 A9

#define encoder1PinA 18
#define encoder1PinB 19

#define encoder2PinA 21
#define encoder2PinB 20

volatile long encoderTicks1 = 0;
volatile long encoderTicks2 = 0;

int i;
double t;

//double tt[10000];
//double x[10000];
//double y[10000];

double x = 0;
double y = 0;


void kinematics() {
  K1 = sq(x - l5) + sq(y) + sq(l4) - sq(l3 + l6);
  K2 = 2 * l4 * (x - l5);
  K3 = 2 * l4 * y;
  double delta_2 = (sq(2 * K1 * K2) - 4 * (sq(K2) + sq(K3)) * (sq(K2) - sq(K3)));
  theta_4 = 57.296 * acos((((2 * K1 * K2) + sqrt(delta_2)) / (2 * (sq(K2) + sq(K3)))));
  double theta_31 = 57.296 * asin((abs(y) - l1 * sin(theta_4 / 57.296)) / (l2 + l6));
  double theta_3 = abs(180 - theta_31);
  P1 = sq(y - l6 * cos(theta_3 / 57.296)) + sq(y - l6 * sin(theta_3 / 57.296)) + (sq(l1) - sq(l2));
  P2 = 2 * l1 * (x - l6 * cos(theta_3 / 57.296));
  P3 = 2 * l1 * (y - l6 * sin(theta_3 / 57.296));
  double delta_1 = (sq(2 * P1 * P2) - 4 * (sq(P2) + sq(P3)) * (sq(P2) - sq(P3)));
  theta_1 = abs(57.296 * acos((((2 * P1 * P2) + sqrt(delta_1)) / (2 * (sq(P2) + sq(P3))))));
}


void prnt() {
  Serial.print(x);
  Serial.print("\t");
  Serial.println (y);
  //  Serial.print("\t");
  //  Serial.print(theta_1);
  //  Serial.print("\t");
  //  Serial.println(theta_4);
}

void gait() {
  for (double t2 = 0 ; t2 <= 10; t2 = t2 + 0.025) {
    //tt[i] = t2;
    t = mod(t2);
    if (t <= (dc + dt)) {
      x = stroke / 2 + stroke / dc * (-t); //stroke with velocity extension for
    }
    else if (t <= (Ti - dt)) {
      x = a0 + a1 * (t - (dc + dt)) + a2 * pow((t - (dc + dt)), 2) + a3 * pow((t - (dc + dt)), 3);
    }
    else {
      x = stroke / 2 + stroke / dc * (Ti - t); // velocity extension for 'dt'seconds
    }

    if (t <= dc) {
      y = flight;
    }
    else {
      y = -180 + 0 + 16 * ht / sq(Ti - dc) * sq(t - dc) - 32 * ht / pow((Ti - dc), 3) * pow((t - dc), 3) + 16 * ht / pow((Ti - dc), 4) * pow((t - dc), 4);
    }

    kinematics();

    prnt();
  /*  int now = millis();
    while (millis() - now < tm) {
      motor1.singleMotorPID(theta_1, encoderTicks1 , baseSpeed, maxSpeed, kpp, kii, kdd);
      motor2.singleMotorPID(theta_4, encoderTicks2 , baseSpeed, maxSpeed, kpp, kii, kdd);
    }*/
    i = i + 1;
    //    delay(1500);
  }
}



void lsHome() {

  //  int pwm1 = 60;
  //
  //  while (digitalRead(ls1) != 0) {
  //    motor2.motorRun(pwm1, HIGH);
  //  }
  //  Serial.println("Motor2 Set");
  //  motor2.motorRun(0, LOW);
  //  delay(150);

  //  while (digitalRead(ls2) != 0) {
  //    motor1.motorRun(pwm1, LOW);
  //  }
  //  Serial.println("Motor1 Set");
  //  motor1.motorRun(0, HIGH);
  //  delay(150);
}

void ZeroTicks() {
  encoderTicks1 = 0;
  encoderTicks2 = 0;
}


void doEncoder1A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder1PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == LOW) {
      encoderTicks1 = encoderTicks1 + 1;         // CW
    }
    else {
      encoderTicks1 = encoderTicks1 - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinB) == HIGH) {
      encoderTicks1 = encoderTicks1 + 1;          // CW
    }
    else {
      encoderTicks1 = encoderTicks1 - 1;          // CCW
    }
  }
}

void doEncoder1B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder1PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoder1PinA) == HIGH) {
      encoderTicks1 = encoderTicks1 + 1;         // CW
    }
    else {
      encoderTicks1 = encoderTicks1 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder1PinA) == LOW) {
      encoderTicks1 = encoderTicks1 + 1;          // CW
    }
    else {
      encoderTicks1 = encoderTicks1 - 1;          // CCW
    }
  }
}

void doEncoder2A() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder2PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder2PinB) == LOW) {
      encoderTicks2 = encoderTicks2 + 1;         // CW
    }
    else {
      encoderTicks2 = encoderTicks2 - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder2PinB) == HIGH) {
      encoderTicks2 = encoderTicks2 + 1;          // CW
    }
    else {
      encoderTicks2 = encoderTicks2 - 1;          // CCW
    }
  }
}

void doEncoder2B() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder2PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoder2PinA) == HIGH) {
      encoderTicks2 = encoderTicks2 + 1;         // CW
    }
    else {
      encoderTicks2 = encoderTicks2 - 1;        // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder2PinA) == LOW) {
      encoderTicks2 = encoderTicks2 + 1;          // CW
    }
    else {
      encoderTicks2 = encoderTicks2 - 1;          // CCW
    }
  }
}

void setup() {

  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder1PinA), doEncoder1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder1PinB), doEncoder1B, CHANGE);

  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoder2PinA), doEncoder2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinB), doEncoder2B, CHANGE);

  //  pinMode(ls1, INPUT_PULLUP);
  //  pinMode(ls2, INPUT_PULLUP);

  Serial.begin(9600);
  a3 = (-4 * (stroke / 2 + stroke / dc * dt) - 2 * stroke / dc * (Ti - dc - 2 * dt)) / pow((Ti - dc - 2 * dt), 3);
  a2 = -3 / 2 * a3 * (Ti - dc - 2 * dt);
  a1 = -stroke / dc;
  a0 = -(stroke / 2 + stroke / dc * dt);
  i = 1;
  gait();
  delay(1000);

  //  lsHome(); 
  ZeroTicks();

  delay(1000);

}

double mod(double T2) {
  while (T2 > 1)
    T2 = T2 - 1;
  return T2;

}
void loop() {

  //  gait();

  //  Serial.print(encoderTicks1);
  //  Serial.print("\t");
  //  Serial.println(encoderTicks2);
}
