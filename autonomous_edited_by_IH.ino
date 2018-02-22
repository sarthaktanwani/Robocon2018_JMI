int x = 4,u=0;
int mspeed = 0;

const float Kp = 0.9;   // Kp value that you have to change
const float Kd = 2;  // Kd value that you have to change
const float setPoint = 35;    // Middle point of sensor
int pulse1, pulse2, pulse3;

//************BASE SPEED AND MAX SPEED**********************
int baseSpeed = 180;
int maxSpeed = 215;
int lastError = 0;
//*********************************************************

int positionVal1;
int positionVal2;
int positionVal3;
int lastpositionVal1;
int lastpositionVal2;
int lastpositionVal3;



const byte serialEn1 = 2; // Connect UART output enable of LSA08 to pin 2
const byte serialEn2 = 25;
const byte serialEn3 = 53;
const byte rx = 0;    // Defining pin 0 as Rx
const byte tx = 1;    // Defining pin 1 as Tx
//const byte serialEn = 2;    // Connect UART output enable of LSA08 to pin 2

#define m1a 35
#define m1b 37
#define m2a 39
#define m2b 41
#define m3a 49
#define m3b 45
#define m4a 47
#define m4b 43
#define en1 11
#define en2 10
#define en3 9
#define en4 8
#define jp1 3
#define jp2 23
#define jp3 51
#define R1 22
#define s1 24
//****************************************************************SETUP****************************************************
void setup() {
  pinMode(serialEn1, OUTPUT);
  pinMode(serialEn2, OUTPUT);
  pinMode(serialEn3, OUTPUT);
  pinMode(jp1, INPUT);
  pinMode(jp2, INPUT);
  pinMode(jp3, INPUT);
  // Setting initial condition of serialEn pin to HIGH
  digitalWrite(serialEn1, HIGH);
  digitalWrite(serialEn2, HIGH);
  digitalWrite(serialEn3, HIGH);

  pinMode(m1a, OUTPUT);
  pinMode(m1b, OUTPUT);
  pinMode(m2a, OUTPUT);
  pinMode(m2b, OUTPUT);
  pinMode(m3a, OUTPUT);
  pinMode(m3b, OUTPUT);
  pinMode(m4a, OUTPUT);
  pinMode(m4b, OUTPUT);
  pinMode(en1, OUTPUT);
  pinMode(en2, OUTPUT);
  pinMode(en3, OUTPUT);
  pinMode(en4, OUTPUT);


  // Setting the initial condition of motors
  // make sure both PWM pins are LOW
  analogWrite(en1, 0);
  analogWrite(en1, 0);
  analogWrite(en3, 0);
  analogWrite(en4, 0);

  // Begin serial communication with baudrate 9600
  Serial.begin(9600);
  Serial1.begin(9600);
  Serial2.begin(9600);
  Serial3.begin(9600);
}
//**************************************************SETUP END**********************************************************************

//**********************************CALLING VALUES FROM LSA08******************************
void lsa1()
{ /* Serial.println("\t\t\t\t LSA 1 READ");
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
    Serial.print(pulse1); if (pulse1 == 1)
     Serial.println("junction 1");
    Serial.println("lsa1 khatam");*/
}


void lsa2()
{ Serial.println("\t\t\t\t LSA 2 READ");
  digitalWrite(serialEn1, HIGH);   // Stop requesting for UART data
  digitalWrite(serialEn3, HIGH);
  digitalWrite(serialEn2, LOW);  // Set serialEN to LOW to request UART data
  if (positionVal2 != 255)
    lastpositionVal2 = positionVal2;
  while (Serial2.available() <= 0);  // Wait for data to be available
  positionVal2 = Serial2.read();    // Read incoming data and store in variable positionVal
  digitalWrite(serialEn2, HIGH);   // Stop requesting for UART data

  Serial.print("\t\t\tsensor2\t\t");
  Serial.print(positionVal2);
  Serial.println("\t\t");
  pulse2 = digitalRead(jp2);
  Serial.print(pulse2);
  if (pulse2 == 1)
    Serial.println("junction 2");
  Serial.println("lsa2 khatam");
}
void lsa3()
{ Serial.println("\t\t\t\t LSA 3 READ");
  digitalWrite(serialEn1, HIGH);   // Stop requesting for UART data
  digitalWrite(serialEn2, HIGH);   // Stop requesting for UART data
  digitalWrite(serialEn3, LOW);  // Set serialEN to LOW to request UART data
  if (positionVal3 != 255)
    lastpositionVal3 = positionVal3;
  while (Serial3.available() <= 0);  // Wait for data to be available
  positionVal3 = Serial3.read();    // Read incoming data and store in variable positionVal
  digitalWrite(serialEn3, HIGH);   // Stop requesting for UART data

  Serial.print("\t\t\tsensor3\t\t");
  Serial.print(positionVal3);
  Serial.println("\t\t");
  pulse3 = digitalRead(jp3);
  if (pulse3 == 1)
    Serial.println("junction 3");
  Serial.print(pulse3);
  Serial.println("lsa3 khatam");
}
//******************************************************************************


//*********************************LFR EAST***********************************
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
//*********************************************************************

//*************************LFR NORTH***********************************
void lfrnorth()
{ Serial.println("north lfr entered");
  lsa1();
  lsa2();
  lsa3();
  if (positionVal3 != 255)
  {
    int error = positionVal3 - setPoint;   // Calculate the deviation from position to the set point

    error = lastError;

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
//*********************************************************************


//*******************************LFR NORTH 1 ****************************
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
//*******************************************************************

//*************************LFR SOUTH*********************************
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
//***********************************************************

//*************************STOP BOT**************************
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
//***********************************************************

//************EAST STOP*************************************
void eaststop(int mspeed)
{
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, HIGH);
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, HIGH);
  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(en1, mspeed);
  analogWrite(en2, mspeed);
  analogWrite(en3, mspeed);
  analogWrite(en4, mspeed);
}
//****************************************

//************NORTH STOP*************************************
void northstop(int mspeed)
{
  digitalWrite(m1a, HIGH);
  digitalWrite(m1b, LOW);
  digitalWrite(m2a, LOW);
  digitalWrite(m2b, HIGH);
  digitalWrite(m3a, HIGH);
  digitalWrite(m3b, LOW);
  digitalWrite(m4a, LOW);
  digitalWrite(m4b, HIGH);
  analogWrite(en1, mspeed);
  analogWrite(en2, mspeed);
  analogWrite(en3, mspeed);
  analogWrite(en4, mspeed);
}
//****************************************

//************SOUTH STOP*************************************
void southstop(int mspeed)
{
  digitalWrite(m1a, LOW);
  digitalWrite(m1b, HIGH);
  digitalWrite(m2a, HIGH);
  digitalWrite(m2b, LOW);
  digitalWrite(m3a, LOW);
  digitalWrite(m3b, HIGH);
  digitalWrite(m4a, HIGH);
  digitalWrite(m4b, LOW);
  analogWrite(en1, mspeed);
  analogWrite(en2, mspeed);
  analogWrite(en3, mspeed);
  analogWrite(en4, mspeed);
} 
void shoot()
{digitalWrite(R1,HIGH);
 delay(400);
 digitalWrite(R2,LOW);
 delay(400);
  }
//****************************************





//**********************************************************LOOP******************************************
void loop() {


  if (x == 0)
  { Serial.print("x=");
    Serial.println(x);
    lastError = 0;
    while (((positionVal2 <= 70) && (positionVal2 >= 0)) != 1)
    {
      Serial.print("x=");
      Serial.println(x);
      Serial.println("moving north....");

      //first line
      lfrnorth();
      lsa2();
      if ((positionVal2 <= 70) && (positionVal2 >= 0))
      {
       while((positionVal2 <= 70) && (positionVal2 >= 0) && u==0)
       {lfrnorth();
         }
       u++;
        //second line, first turn
        if(u!=0)
       {stopbot();
        northstop(200);
        delay(200);
        stopbot();
        }
      }
    }
    lsa1();
    lsa2();
    lsa3();
    x = 1;
  }
  lastError = 0;
  if (x == 1)
  {
    Serial.print("x=");
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

    eaststop(200);
    delay(200);
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
  lastError = 0;
  if (x == 2)
  {
    for (int op = 0; op < 1000; op++)
    {
      lfrnorth();
      delay(1);
    }
    //x = 3;
    stopbot();
  }
  
  
  //*************waiting for loading**********
  if(s1==HIGH)
  {x = 3;
    }

    
  //*************getcontrol from pi***********
  if (x == 3)
  {
    if (analogRead(A0) > 200)
      x = 4;
    if (analogRead(A1) > 200)
      x = 10;
  }
  //*******************************************



  //********************go and shoot from tz1************************
  if ( x == 4)
  {
    lsa2();
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
    for (int r = 0; r < 200; r++)
    {
      northstop(200);
      delay(1);
      lsa1();
      lsa2();
      lsa3();
    }
    stopbot();

    x = 5;
  }


  //*************************************************************************************
  if (x == 5)
  {
    stopbot();
    delay(100);
    for (int fllg = 0; fllg < 3000; fllg++)
    {
      Serial.print("x=");
      Serial.println(x);
      //    align();
      delay(1);
    }
    shoot();
    if(s1==LOW)
     {stopbot();
      delay(400);}
    x=6;
  }
//*************************************GOING BACK FOR LOADING ***************************
  if (x == 6)
  { for(i=0;i<400;i++)
     {lfrsouth();
      delay(1);
       }
    while(positionVal1<70 && positionVal2<70 && positionVal1>0 &&positionVal2>0)
     {lfrsouth();
       }   
   x = 3;
    }

}
