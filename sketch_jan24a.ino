//Vagarth's commits
int x = 0;
int e[2], n[2];
int flg = 0;
const float Kp = 2;   // Kp value that you have to change
const float Kd = 6;   // Kd value that you have to change
const float setPoint = 35;    // Middle point of sensor
int pulse1, pulse2, pulse3;
int baseSpeed = 235;
int positionVal1;
int positionVal2;
int positionVal3;
int lastpositionVal1;
int lastpositionVal2;
int lastpositionVal3;

int maxSpeed = 255;

const byte serialEn1 = 2; // Connect UART output enable of LSA08 to pin 2
const byte serialEn2 = 3;
const byte serialEn3 = 4;
const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
//const byte serialEn = 2;    // Connect UART output enable of LSA08 to pin 2

#define m1a 22
#define m1b 23
#define m2a 26
#define m2b 27
#define m3a 30
#define m3b 31
#define m4a 34
#define m4b 35
#define en1 9
#define en2 10
#define en3 11
#define en4 12
#define jp1 50
#define jp2 51
#define jp3 52
int pulsecount1 = 0, pulsecount2 = 0;
int flag = 0;
//const byte pwm2 = 10;   // Connect PWM1 of motor driver to pin 10
//const byte pwm1 = 12;   // Connect PWM2 of motor driver to pin 12


void setup() {
  pinMode(serialEn1, OUTPUT);  // Setting serialEn as digital output pin
  pinMode(serialEn2, OUTPUT);
  pinMode(serialEn3, OUTPUT);
  pinMode(jp1, INPUT);  // Setting junctionPulse as digital input pin
  pinMode(jp2, INPUT);
  pinMode(jp3, INPUT);
  // Setting pin 10 - 13 as digital output pin
  for (byte i = 10; i <= 33; i++) {
    pinMode(i, OUTPUT);

  }


  // Setting initial condition of serialEn pin to HIGH
  digitalWrite(serialEn1, HIGH);
  digitalWrite(serialEn2, HIGH);
  digitalWrite(serialEn3, HIGH);


  // Setting the initial condition of motors
  // make sure both PWM pins are LOW
  analogWrite(en1, 0);
  analogWrite(en1, 0);
  analogWrite(en3, 0);
  analogWrite(en4, 0);

  // State of DIR pins are depending on your physical connection
  // if your robot behaves strangely, try changing thses two values


  // Begin serial communication with baudrate 9600
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);


}

int lastError = 0;    // Declare a variable to store previous error



void clockwise()
{
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, HIGH);
  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  digitalWrite(m4a, HIGH);
  digitalWrite(m4b, LOW);
  analogWrite(en1, 200);
  analogWrite(en2, 200);
  analogWrite(en3, 200);
  analogWrite(en4, 200);
  Serial.println("\t\t\t\t\tclockwise");

}
void anticlockwise()
{
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, HIGH);
  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, LOW);
  digitalWrite(m3a, HIGH);
  digitalWrite(m3b, LOW);
  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(en1, 200);
  analogWrite(en2, 200);
  analogWrite(en3, 200);
  analogWrite(en4, 200);
  Serial.println("\t\t\t\t\tanticlockwise");

}

void lsa1()
{ Serial.println("\t\t\t\t LSA 1 READ");
  digitalWrite(serialEn2, HIGH);   // Stop requesting for UART data
  digitalWrite(serialEn3, HIGH);
  digitalWrite(serialEn1, LOW);  // Set serialEN to LOW to request UART data
  if (positionVal1 != 255)
    lastpositionVal1 = positionVal1;
  while (Serial.available() <= 0);  // Wait for data to be available

  positionVal1 = Serial.read();    // Read incoming data and store in variable positionVal
  digitalWrite(serialEn1, HIGH);   // Stop requesting for UART data

  Serial.print("sensor1");
  Serial.print(positionVal1);
  Serial.print("\t\t");
  pulse1 = digitalRead(jp1);

}
void lsa2()
{ Serial.println("\t\t\t\t LSA 2 READ");
  digitalWrite(serialEn1, HIGH);   // Stop requesting for UART data
  digitalWrite(serialEn3, HIGH);
  digitalWrite(serialEn2, LOW);  // Set serialEN to LOW to request UART data
  if (positionVal2 != 255)
    lastpositionVal2 = positionVal2;
  while (Serial1.available() <= 0);  // Wait for data to be available
  positionVal2 = Serial1.read();    // Read incoming data and store in variable positionVal
  digitalWrite(serialEn2, HIGH);   // Stop requesting for UART data

  Serial.print("\t\t\tsensor2\t\t");
  Serial.print(positionVal2);
  Serial.println("\t\t");
  pulse2 = digitalRead(jp2);
  Serial.println("lsa2 khatam");
}
void lsa3()
{ Serial.println("\t\t\t\t LSA 3 READ");
  digitalWrite(serialEn1, HIGH);   // Stop requesting for UART data
  digitalWrite(serialEn2, HIGH);   // Stop requesting for UART data
  digitalWrite(serialEn3, LOW);  // Set serialEN to LOW to request UART data
  if (positionVal3 != 255)
    lastpositionVal3 = positionVal3;
  while (Serial2.available() <= 0);  // Wait for data to be available
  positionVal3 = Serial2.read();    // Read incoming data and store in variable positionVal
  digitalWrite(serialEn3, HIGH);   // Stop requesting for UART data

  Serial.print("\t\t\tsensor3\t\t");
  Serial.print(positionVal3);
  Serial.println("\t\t");
  pulse3 = digitalRead(jp3);
  Serial.println("lsa3 khatam");
}
void lfreast()
{
  Serial.println("lfr east entered");
  lsa1();
  lsa2();
  lsa3();
  if (positionVal2 != 255)
  {
    int error = positionVal2 - setPoint;   // Calculate the deviation from position to the set point
    int motorSpeed = Kp * error + Kd * (error - lastError);   // Applying formula of PID
    lastError = error;    // Store current error as previous error for next iteration use

    // Adjust the motor speed based on calculated value
    // You might need to interchange the + and - sign if your robot move in opposite direction
    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;

    // If the speed of motor exceed max speed, set the speed to max speed
    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

    // If the speed of motor is negative, set it to 0
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;

    // Writing the motor speed value as output to hardware motor

    //**************************************writing speed on l298**********
    Serial.println(leftMotorSpeed);
    Serial.println(rightMotorSpeed);
    digitalWrite(m1a, HIGH);
    digitalWrite(m1b, LOW);
    digitalWrite(m2a, HIGH);
    digitalWrite(m2b, LOW);
    digitalWrite(m3a, HIGH);
    digitalWrite(m3b, LOW);
    digitalWrite(m4a, HIGH);
    digitalWrite(m4b, LOW);
    analogWrite(en1, leftMotorSpeed);
    analogWrite(en2, rightMotorSpeed);
    analogWrite(en3, rightMotorSpeed);
    analogWrite(en4, leftMotorSpeed);
    //*********************************************************************

  }
}


void lfrnorth()
{ Serial.println("north lfr entered");
  lsa1();
  lsa2();
  lsa3();
  if (positionVal3 != 255)
  {
    int error = positionVal3 - setPoint;   // Calculate the deviation from position to the set point
    if (abs(lastError - error) > 15)
    {
      error = lastError;
    }
    int motorSpeed = Kp * error + Kd * (error - lastError);   // Applying formula of PID
    lastError = error;    // Store current error as previous error for next iteration use

    // Adjust the motor speed based on calculated value
    // You might need to interchange the + and - sign if your robot move in opposite direction
    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;

    // If the speed of motor exceed max speed, set the speed to max speed
    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

    // If the speed of motor is negative, set it to 0
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;

    // Writing the motor speed value as output to hardware motor

    //**************************************writing speed on l298**********

    digitalWrite(m1a, LOW);
    digitalWrite(m1b, HIGH);
    digitalWrite(m2a, HIGH);
    digitalWrite(m2b, LOW);
    digitalWrite(m3a, LOW);
    digitalWrite(m3b, HIGH);
    digitalWrite(m4a, HIGH);
    digitalWrite(m4b, LOW);
    analogWrite(en1, rightMotorSpeed);
    analogWrite(en2, rightMotorSpeed);
    analogWrite(en3, leftMotorSpeed);
    analogWrite(en4, leftMotorSpeed);
    //*********************************************************************
    Serial.println(rightMotorSpeed);
    Serial.println(leftMotorSpeed);

  }
}
void lfrnorth1()
{ Serial.println("north lfr entered");
  lsa1();
  lsa2();
  lsa3();
  if (positionVal3 != 255)
  {
    int error = positionVal3 - setPoint;   // Calculate the deviation from position to the set point
    if (abs(lastError - error) > 15)
    {
      error = lastError;
    }
    int motorSpeed = Kp * error + Kd * (error - lastError);   // Applying formula of PID
    lastError = error;    // Store current error as previous error for next iteration use

    // Adjust the motor speed based on calculated value
    // You might need to interchange the + and - sign if your robot move in opposite direction
    int rightMotorSpeed = 160 - motorSpeed;
    int leftMotorSpeed = 160 + motorSpeed;

    // If the speed of motor exceed max speed, set the speed to max speed
    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = 200;
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = 200;

    // If the speed of motor is negative, set it to 0
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;

    // Writing the motor speed value as output to hardware motor

    //**************************************writing speed on l298**********

    digitalWrite(m1a, LOW);
    digitalWrite(m1b, HIGH);
    digitalWrite(m2a, HIGH);
    digitalWrite(m2b, LOW);
    digitalWrite(m3a, LOW);
    digitalWrite(m3b, HIGH);
    digitalWrite(m4a, HIGH);
    digitalWrite(m4b, LOW);
    analogWrite(en1, rightMotorSpeed);
    analogWrite(en2, rightMotorSpeed);
    analogWrite(en3, leftMotorSpeed);
    analogWrite(en4, leftMotorSpeed);
    //*********************************************************************
    Serial.println(rightMotorSpeed);
    Serial.println(leftMotorSpeed);

  }
}
/*
  void lfrwest()
  { Serial.println("lfr west entered");
  lsa1();
  lsa2();
  lsa3();
  int error = positionVal3 - setPoint;    // Calculate the deviation from position to the set point
  int motorSpeed = Kp * error + Kd * (error - lastError);   // Applying formula of PID
  lastError = error;    // Store current error as previous error for next iteration use

  // Adjust the motor speed based on calculated value
  // You might need to interchange the + and - sign if your robot move in opposite direction
  int rightMotorSpeed = baseSpeed - motorSpeed;
  int leftMotorSpeed = baseSpeed + motorSpeed;

  // If the speed of motor exceed max speed, set the speed to max speed
  if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
  if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

  // If the speed of motor is negative, set it to 0
  if (rightMotorSpeed < 0) rightMotorSpeed = 0;
  if (leftMotorSpeed < 0) leftMotorSpeed = 0;

  // Writing the motor speed value as output to hardware motor

  //**************************************writing speed on l298**********

  digitalWrite(m1a, LOW);
  digitalWrite(m1b, HIGH);
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, HIGH);
  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(en1, rightMotorSpeed);
  analogWrite(en2, leftMotorSpeed);
  analogWrite(en3, leftMotorSpeed);
  analogWrite(en4, rightMotorSpeed);
  //*********************************************************************


  }
*/

void lfrsouth()
{
  lsa1();
  lsa2();
  lsa3();
  if (positionVal1 != 255)
  {
    int error = positionVal1 - setPoint;   // Calculate the deviation from position to the set point
    int motorSpeed = Kp * error + Kd * (error - lastError);   // Applying formula of PID
    lastError = error;    // Store current error as previous error for next iteration use

    // Adjust the motor speed based on calculated value
    // You might need to interchange the + and - sign if your robot move in opposite direction
    int rightMotorSpeed = baseSpeed - motorSpeed;
    int leftMotorSpeed = baseSpeed + motorSpeed;

    // If the speed of motor exceed max speed, set the speed to max speed
    if (rightMotorSpeed > maxSpeed) rightMotorSpeed = maxSpeed;
    if (leftMotorSpeed > maxSpeed) leftMotorSpeed = maxSpeed;

    // If the speed of motor is negative, set it to 0
    if (rightMotorSpeed < 0) rightMotorSpeed = 0;
    if (leftMotorSpeed < 0) leftMotorSpeed = 0;

    // Writing the motor speed value as output to hardware motor

    //**************************************writing speed on l298**********

    digitalWrite(m1a, HIGH);
    digitalWrite(m1b, LOW);
    digitalWrite(m2a, LOW);
    digitalWrite(m2b, HIGH);
    digitalWrite(m3a, HIGH);
    digitalWrite(m3b, LOW);
    digitalWrite(m4a, LOW);
    digitalWrite(m4b, HIGH);
    analogWrite(en1, leftMotorSpeed);
    analogWrite(en2, leftMotorSpeed);
    analogWrite(en3, rightMotorSpeed);
    analogWrite(en4, rightMotorSpeed);
    //*********************************************************************
  }
}

void stopbot()
{ Serial.println("stopbot");
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, LOW);
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, LOW);
  digitalWrite(m3a, LOW);
  digitalWrite(m3b, LOW);
  digitalWrite(m4a, LOW);
  digitalWrite(m4b, LOW);
  analogWrite(en1, 0);
  analogWrite(en2, 0);
  analogWrite(en3, 0);
  analogWrite(en4, 0);
}
void alignbot()
{ while (positionVal2 != positionVal3)
  {
    lsa2();
    lsa3();
    if (positionVal2 < positionVal3)
    {
      clockwise();
    }
    if (positionVal2 > positionVal3)
    {
      anticlockwise();
    }
  }
}
void align1()
{ int tmkc = 0;
  int motorspeed14;
  int motorspeed23;
  int a, b, c, d;
  //while (tmkc == 0)
  while ((positionVal1 > 32 && positionVal1 < 38) && (positionVal3 > 32 && positionVal3 < 38) != 1)
  {

    lsa1();
    lsa2();
    lsa3();
    if (positionVal1 == 35 && positionVal3 == 35)
    {
      analogWrite(en1, 0);
      analogWrite(en2, 0);
      analogWrite(en3, 0);
      analogWrite(en4, 0);
    }
    if (positionVal3 > 35)
    {
      a = positionVal3 - 35;
      motorspeed14 = map(a, 0, 35, 60, 150);
      digitalWrite(m1a, HIGH);
      digitalWrite(m1b, LOW);
      digitalWrite(m4a, HIGH);
      digitalWrite(m4b, LOW);
      analogWrite(en1, motorspeed14);
      analogWrite(en4, motorspeed14);
    }
    if (positionVal3 < 35)
    {
      b = positionVal3;
      motorspeed14 = map(b, 35, 0, 60, 150);
      digitalWrite(m1a, LOW);
      digitalWrite(m1b, HIGH);
      digitalWrite(m4a, LOW);
      digitalWrite(m4b, HIGH);
      analogWrite(en1, motorspeed14);
      analogWrite(en4, motorspeed14);
    }


    if (positionVal1 > 35)
    {
      c = positionVal1 - 35;
      motorspeed23 = map(c, 0, 35, 60, 150);
      digitalWrite(m2a, LOW);
      digitalWrite(m2b, HIGH);
      digitalWrite(m3a, LOW);
      digitalWrite(m3b, HIGH);
      analogWrite(en2, motorspeed23);
      analogWrite(en3, motorspeed23);
    }
    if (positionVal1 < 35)
    {
      d = positionVal1;
      motorspeed23 = map(d, 35, 0, 60, 150);
      digitalWrite(m2a, HIGH);
      digitalWrite(m2b, LOW);
      digitalWrite(m3a, HIGH);
      digitalWrite(m3b, LOW);
      analogWrite(en2, motorspeed23);
      analogWrite(en3, motorspeed23);
    }
    lsa2();
    lsa1();
    lsa3();
    if ((positionVal1 > 29) && (positionVal1 < 42) && (positionVal3 > 29) && (positionVal3 < 42))
    {
      tmkc = 1;
      stopbot();
    }
  }
}
void toandfro()
{
  int msall;
  while (positionVal2 != 35)
  { lsa2();
    if (positionVal2 < 35)
    { msall = map(positionVal2, 35, 0, 100, 255);
      digitalWrite(m1a, LOW);
      digitalWrite(m1b, HIGH);
      digitalWrite(m2a, HIGH);
      digitalWrite(m2b, LOW);
      digitalWrite(m3a, LOW);
      digitalWrite(m3b, HIGH);
      digitalWrite(m4a, HIGH);
      digitalWrite(m4b, LOW);
      analogWrite(en1, msall);
      analogWrite(en2, msall);
      analogWrite(en3, msall);
      analogWrite(en4, msall);
    }
    if (positionVal2 > 35 && positionVal2 != 255)
    { msall = map(positionVal2, 35, 70, 100, 255);
      digitalWrite(m1a, HIGH);
      digitalWrite(m1b, LOW);
      digitalWrite(m2a, LOW);
      digitalWrite(m2b, HIGH);
      digitalWrite(m3a, HIGH);
      digitalWrite(m3b, LOW);
      digitalWrite(m4a, LOW);
      digitalWrite(m4b, HIGH);
      analogWrite(en1, msall);
      analogWrite(en2, msall);
      analogWrite(en3, msall);
      analogWrite(en4, msall);
    }
  }
}
void align2()
{
  int motorspeed14;
  int motorspeed23;
  int a, b, c, d;
  lsa1();
  lsa2();
  lsa3();
  //***********************************
  lsa1();
  lsa2();
  lsa3();

  if (positionVal2 == 255)
  {
    if (lastpositionVal2 > 60)
    { while (lastpositionVal2 == 255)
      { lfrsouth();

        lsa1();
        lsa2();
        lsa3();
      }
    }
    if (lastpositionVal2 < 10)
    { while (lastpositionVal2 == 255)
      { lfrnorth();
        lsa1();
        lsa2();
        lsa3();
      }
    }
  }
  //*****************************
  if (positionVal1 == 255)
  { while (positionVal1 == 255)
    {
      if (lastpositionVal1 < 5)
      { //anticlk
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, HIGH);
        digitalWrite(m2a, HIGH);
        digitalWrite(m2b, LOW);
        digitalWrite(m3a, HIGH);
        digitalWrite(m3b, LOW);
        digitalWrite(m4a, LOW);
        digitalWrite(m4b, HIGH);
        analogWrite(en1, 200);
        analogWrite(en2, 200);
        analogWrite(en3, 200);
        analogWrite(en4, 200);
      }
      if (lastpositionVal1 < 65)
      { //clk
        digitalWrite(m1a, HIGH);
        digitalWrite(m1b, LOW);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, HIGH);
        digitalWrite(m3a, LOW);
        digitalWrite(m3b, HIGH);
        digitalWrite(m4a, HIGH);
        digitalWrite(m4b, LOW);
        analogWrite(en1, 200);
        analogWrite(en2, 200);
        analogWrite(en3, 200);
        analogWrite(en4, 200);
      }
    }
  }

  if (positionVal3 == 255)
  { while (positionVal3 == 255)
    {
      if (lastpositionVal3 < 5)
      { //anticlk
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, HIGH);
        digitalWrite(m2a, HIGH);
        digitalWrite(m2b, LOW);
        digitalWrite(m3a, HIGH);
        digitalWrite(m3b, LOW);
        digitalWrite(m4a, LOW);
        digitalWrite(m4b, HIGH);
        analogWrite(en1, 200);
        analogWrite(en2, 200);
        analogWrite(en3, 200);
        analogWrite(en4, 200);
      }
      if (lastpositionVal3 < 65)
      { //clk
        digitalWrite(m1a, HIGH);
        digitalWrite(m1b, LOW);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, HIGH);
        digitalWrite(m3a, LOW);
        digitalWrite(m3b, HIGH);
        digitalWrite(m4a, HIGH);
        digitalWrite(m4b, LOW);
        analogWrite(en1, 200);
        analogWrite(en2, 200);
        analogWrite(en3, 200);
        analogWrite(en4, 200);
      }
    }
  }
  while (((positionVal1 >= 34 && positionVal1 <= 36) && (positionVal3 >= 34 && positionVal3 <= 36)) != 1)
  { //***********************************
    lsa1();
    lsa2();
    lsa3();

    if (positionVal2 == 255)
    {
      if (lastpositionVal2 > 60)
      { while (lastpositionVal2 == 255)
        { lfrsouth();

          lsa1();
          lsa2();
          lsa3();
        }
      }
      if (lastpositionVal2 < 10)
      { while (lastpositionVal2 == 255)
        { lfrnorth();
          lsa1();
          lsa2();
          lsa3();
        }
      }
    }
    //*****************************
    lsa1();
    lsa2();
    lsa3();
    if (positionVal3 > 35)
    {
      a = positionVal3 - 35;
      motorspeed14 = map(a, 0, 35, 100, 180);
      digitalWrite(m1a, HIGH);
      digitalWrite(m1b, LOW);
      digitalWrite(m4a, HIGH);
      digitalWrite(m4b, LOW);
      analogWrite(en1, motorspeed14);
      analogWrite(en4, motorspeed14);
    }
    if (positionVal3 < 35)
    {
      b = positionVal3;
      motorspeed14 = map(b, 35, 0, 80, 180);
      digitalWrite(m1a, LOW);
      digitalWrite(m1b, HIGH);
      digitalWrite(m4a, LOW);
      digitalWrite(m4b, HIGH);
      analogWrite(en1, motorspeed14);
      analogWrite(en4, motorspeed14);
    }
    if (positionVal1 > 35)
    {
      c = positionVal1 - 35;
      motorspeed23 = map(c, 0, 35, 100, 180);
      digitalWrite(m2a, LOW);
      digitalWrite(m2b, HIGH);
      digitalWrite(m3a, LOW);
      digitalWrite(m3b, HIGH);
      analogWrite(en2, motorspeed23);
      analogWrite(en3, motorspeed23);
    }
    if (positionVal1 < 35)
    {
      d = positionVal1;
      motorspeed23 = map(d, 35, 0, 100, 180);
      digitalWrite(m2a, HIGH);
      digitalWrite(m2b, LOW);
      digitalWrite(m3a, HIGH);
      digitalWrite(m3b, LOW);
      analogWrite(en2, motorspeed23);
      analogWrite(en3, motorspeed23);
    }
  }
  toandfro();

}




void align()
{ // int m1, m2, m3, m4;
  if ((positionVal1 != 255) && (positionVal2 != 255) && ((positionVal1 != 35) || (positionVal2 != 35)))
  {
    int x = map(positionVal1, 70, 0, -1000, 1000);
    int y = map(positionVal2, 0 , 70, 1000, -1000);
    Serial.print("X,Y= ");
    Serial.print(x);
    Serial.print(",");
    Serial.print(y);
    Serial.println(".");
    int m4 = (x + y) * cos(45);
    int m3 = (x - y) * cos(45);
    int m1 = (x - y) * cos(45);
    int m2 = (x + y) * cos(45);
    if (m1 >= 0)
    {
      m1 = map(m1, 0, 1000, 120, 250);
      if (m1 > 255) {
        m1 = 255;
      }
      digitalWrite(m1a, HIGH);
      digitalWrite(m1b, LOW);
    }
    else
    {
      m1 = map(m1, 0, -1000, 120, 250);
      if (m1 > 255) {
        m1 = 255;
      }
      digitalWrite(m1a, LOW);
      digitalWrite(m1b, HIGH);
    }
    analogWrite(en1, m1);
    if (m2 >= 0)
    {
      m2 = map(m2, 0, 1000, 120, 250);
      if (m2 > 255) {
        m2 = 255;
      }
      digitalWrite(m2a, HIGH);
      digitalWrite(m2b, LOW);
    }
    else
    {
      m2 = map(m2, 0, -1000, 120, 250);
      if (m2 > 255) {
        m2 = 255;
      }
      digitalWrite(m2a, LOW);
      digitalWrite(m2b, HIGH);
    }
    analogWrite(en2, m2);
    if (m3 >= 0)
    {
      m3 = map(m3, 0, 1000, 120, 250);
      if (m3 > 255) {
        m3 = 255;
      }
      digitalWrite(m3a, HIGH);
      digitalWrite(m3b, LOW);
    }
    else
    {
      m3 = map(m3, 0, -1000, 120, 250);
      if (m3 > 255) {
        m3 = 255;
      }
      digitalWrite(m3a, LOW);
      digitalWrite(m3b, HIGH);
    }
    analogWrite(en3, m3);
    if (m4 >= 0)
    {
      m4 = map(m4, 0, 1000, 120, 250);
      if (m4 > 255) {
        m4 = 255;
      }
      digitalWrite(m4a, HIGH);
      digitalWrite(m4b, LOW);
    }
    else
    {
      m4 = map(m4, 0, -1000, 120, 250);
      if (m4 > 255) {
        m4 = 255;
      }
      digitalWrite(m4a, LOW);
      digitalWrite(m4b, HIGH);
    }
    analogWrite(en4, m4);
    Serial.print(m1);
    Serial.print(",");
    Serial.print(m2);
    Serial.print(",");
    Serial.print(m3);
    Serial.print(",");
    Serial.print(m4);
    Serial.print(",");
  } else {
    stopbot();
  }
}


//***************loop***************

void loop() {
  int im = 0;
  /*  while (1)
    {
      lsa1();
      lsa2();
      lsa3();
      align2();
      // toandfro();
      //if(positionVal1 == 35 && positionVal2 == 35 && positionVal3 == 35)
      //break;
    } */



  lsa1();
  lsa2();
  lsa3();
  //Serial.println(pulsecount1);
  // Serial.println(pulsecount2);
  if (x == 0)
  { Serial.print("x=");
    Serial.println(x);
    while (((positionVal2 <= 70) && (positionVal2 >= 0)) != 1)
    { Serial.print("x=");
      Serial.println(x);
      Serial.println("moving north....");

      //first line
      lfrnorth();
      lsa2();
      if ((positionVal2 <= 70) && (positionVal2 >= 0))
      {
        //second line, first turn
        stopbot();
        digitalWrite(m1a, HIGH);
        digitalWrite(m1b, LOW);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, HIGH);
        digitalWrite(m3a, HIGH);
        digitalWrite(m3b, LOW);
        digitalWrite(m4a, LOW);
        digitalWrite(m4b, HIGH);
        analogWrite(en1, 200);
        analogWrite(en2, 200);
        analogWrite(en3, 200);
        analogWrite(en4, 200);
        delay(200);
        //   lfreast();
        stopbot();
        align2();
      }
    }
    lsa1();
    lsa2();
    lsa3();
    x = 10;
  }
  if (x = 10)
  { //***********************************
    lsa1();
    lsa2();
    lsa3();

    if (positionVal2 == 255)
    {
      if (lastpositionVal2 > 60)
      { while (lastpositionVal2 == 255)
        { lfrsouth();

          lsa1();
          lsa2();
          lsa3();
        }
      }
      if (lastpositionVal2 < 10)
      { while (lastpositionVal2 == 255)
        { lfrnorth();
          lsa1();
          lsa2();
          lsa3();
        }
      }
    }
    //*****************************
    while (positionVal2 != 255)
      align2();
  }
  //if ((x == 1)&&(positionVal2 != 255))
  if (x == 1)
  { Serial.print("x=");
    Serial.println(x);
    lsa1();
    lsa2();
    lsa3();
    lfreast();
    while (positionVal3 == 255)
    { //second line
      Serial.print("x=");
      Serial.println(x);
      lfreast();
      lsa3();
    }
    /* if(positionVal3==70)
      {int e=positionVal2[0];
      }
      if(positionVal2==70)
      {int n=positionVal3[0];
      }
      if(positionVal3==0)
      {int e=positionVal2[1];
      }
      if(positionVal2==0)
      {int n=positionVal3[1];
      }
      while((positionVal1<70 && positionVal2<70 && positionVal3<70)&&(positionVal1>0 && positionVal2 > 0 && positionVal3>0)!=1)
      {if(positionVal2[0])
       }*/
    while ((positionVal1 < 70 && positionVal2 < 70 && positionVal3 < 70) && (positionVal1 > 0 && positionVal2 > 0 && positionVal3 > 0) != 1)
    { digitalWrite(m1a, HIGH);
      digitalWrite(m1b, LOW);
      digitalWrite(m2a, LOW);
      digitalWrite(m2b, HIGH);
      digitalWrite(m3a, HIGH);
      digitalWrite(m3b, LOW);
      digitalWrite(m4a, LOW);
      digitalWrite(m4b, HIGH);
      analogWrite(en1, 160);
      analogWrite(en2, 160);
      analogWrite(en3, 160);
      analogWrite(en4, 160);
    }
    if (positionVal3 != 255)
    {
      stopbot();
      digitalWrite(m1a, LOW);
      digitalWrite(m2a, LOW);
      digitalWrite(m2b, HIGH);
      digitalWrite(m3a, LOW);
      digitalWrite(m3b, HIGH);
      digitalWrite(m4a, LOW);
      digitalWrite(m4b, HIGH);
      analogWrite(en1, 255);
      analogWrite(en2, 255);
      analogWrite(en3, 255);
      analogWrite(en4, 255);
      delay(1000);
      // lfrnorth();
    }
    /*  while ((positionVal3 == 255))
      { digitalWrite(m1a, LOW);
        digitalWrite(m1b, HIGH);
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, LOW);
        digitalWrite(m3a, LOW);
        digitalWrite(m3b, HIGH);
        digitalWrite(m4a, LOW);
        digitalWrite(m4b, LOW);
        analogWrite(en1, 220);
        analogWrite(en2, 0);
        analogWrite(en3, 220);
        analogWrite(en4, 0);

        //wait for loading signal
        while (positionVal2 != 255)
        {
          digitalWrite(m1a, LOW);
          digitalWrite(m1b, HIGH);
          digitalWrite(m2a, LOW);
          digitalWrite(m2b, LOW);
          digitalWrite(m3a, LOW);
          digitalWrite(m3b, HIGH);
          digitalWrite(m4a, LOW);
          digitalWrite(m4b, LOW);
          analogWrite(en1, 220);
          analogWrite(en2, 0);
          analogWrite(en3, 220);
          analogWrite(en4, 0);

        }
      }*/

    x = 2;

  }
  if ( x == 2)
  {
    // while (pulse2 != 1)
    int flg1 = 0;
    while (positionVal2 == 255)
    { lsa3();
      lsa2();

      /* while ((positionVal3 == 255) && (flg1 == 0))
        {
         digitalWrite(m1a, LOW);
         digitalWrite(m1b, HIGH);
         digitalWrite(m2a, LOW);
         digitalWrite(m2b, LOW);
         digitalWrite(m3a, LOW);
         digitalWrite(m3b, HIGH);
         digitalWrite(m4a, LOW);
         digitalWrite(m4b, LOW);
         analogWrite(en1, 220);
         analogWrite(en2, 0);
         analogWrite(en3, 220);
         analogWrite(en4, 0);
         flg1 = 2;
        }*/
      Serial.print("x=");
      Serial.println(x);
      lfrnorth();
    }
    x = 3;
  }
  //*************************************************************************************
  if (x == 3)
  {
    stopbot();
    delay(1000);
    for (int fllg = 0; fllg < 3000; fllg++)
    {
      Serial.print("x=");
      Serial.println(x);
      align();
      delay(1);
    }
    //wait for shuttle
    x = 4;
  }
  //***********************************************
  if (x == 4)
  { stopbot();
    //shoot
    Serial.print("x=");
    Serial.println(x);
    Serial.println("shooting.............");
    delay(4000);
  }
  //Serial.println(flag);
  // Serial.println(digitalRead(junctionPulse));
  //if(digitalRead(junctionPulse)==1)
  //flag=flag+1;


  //lfrnorth();

}

