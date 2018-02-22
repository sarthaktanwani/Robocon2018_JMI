#include "PS2X_lib.h"  //for v1.6
#include <Wire.h>
#include<Servo.h>         //orange -> data , red -> 5V , brown -> GND 
                          //OR red 5V , black GND , white Data 
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT 12  //14    
#define PS2_CMD 11  //15
#define PS2_SEL 10  //16
#define PS2_CLK 13  //17
#define m1a 36
#define m1b 37
#define m2a 38
#define m2b 39
#define m3a 35
#define m3b 34
#define m4a 40
#define m4b 41
#define en1 5
#define en2 3
#define en3 8
#define en4 9
#define servo1 6
#define servo2 7
/******************************************************************
 * select modes of PS2 controller:
 *   - pressures = analog reading of push-butttons 
 *   - rumble    = motor rumbling
 * uncomment 1 of the lines for each mode selection
 ******************************************************************/
//#define pressures   true
#define pressures   true 
//#define rumble      true
#define rumble      true

PS2X ps2x; // create PS2 Controller Class
Servo myservo1,myservo2,myservo3;       //myservo3 is for hook
int pos1 = 0,pos2 = 0;    // variable to store the servo position

//right now, the library does NOT support hot pluggable controllers, meaning 
//you must always either restart your Arduino after you connect the controller, 
//or call config_gamepad(pins) again after connecting the controller.

int error = 0;
byte type = 0;
byte vibrate = 0;

int lx = 0;
int ly=  0;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61); 

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *L_M3 = AFMS.getMotor(3);
Adafruit_DCMotor *R_M4 = AFMS.getMotor(4);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);


void setup(){
  pinMode(m1a,OUTPUT);
  pinMode(m1b,OUTPUT);
  pinMode(m2a,OUTPUT);
  pinMode(m2b,OUTPUT);
  pinMode(m3a,OUTPUT);
  pinMode(m3b,OUTPUT);
  pinMode(m4a,OUTPUT);
  pinMode(m4b,OUTPUT);
  pinMode(en1,OUTPUT);
  pinMode(en2,OUTPUT);
  pinMode(en3,OUTPUT);
  pinMode(en4,OUTPUT);
  myservo1.attach(servo1);
  myservo2.attach(servo2);
  Serial.begin(9600);
  
  delay(5000);  //added delay to give wireless ps2 module some time to startup, before configuring it
  Serial.print("===================================");
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************
  
  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);
  
  if(error == 0){
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
  if (pressures)
    Serial.println("true ");
  else
    Serial.println("false");
  Serial.print("rumble = ");
  if (rumble)
    Serial.println("true)");
  else
    Serial.println("false");
//    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
//    Serial.println("holding L1 or R1 will print out the analog stick values.");
//    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }  
  else if(error == 1)
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
   
  else if(error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if(error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");
  
//  Serial.print(ps2x.Analog(1), HEX);
  
  type = ps2x.readType(); 
  switch(type) {
    case 0:
      Serial.print("Unknown Controller type found ");
      break;
    case 1:
      Serial.print("DualShock Controller found ");
      break;
    case 2:
      Serial.print("GuitarHero Controller found ");
      break;
  case 3:
      Serial.print("Wireless Sony DualShock Controller found ");
      break;
   }
   
  AFMS.begin(50);  // create with the default frequency 1.6KHz

}
void loop() {
  
  int y14,y23,xVal1, yVal1, buttonVal1,xVal2, yVal2, buttonVal2;
 
  /* You must Read Gamepad to get new values and set vibration values
     ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
     if you don't enable the rumble, use ps2x.read_gamepad(); with no values
     You should call this at least once a second
   */  
   //Serial.print("==================444=================\n");
 // if(error == 1) //skip loop if no controller found
   // return; 
  
  /*if(type == 2){ //Guitar Hero Controller
   ps2x.read_gamepad();          //read controller
    }*/

      ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
      Serial.println("Start is being held");
    if(ps2x.Button(PSB_SELECT))
      Serial.println("Select is being held");      

    if(ps2x.Button(PSB_PAD_UP)) {      //will be TRUE as long as button is pressed
      Serial.print("Up held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);
      L_M3->run(FORWARD);
      R_M4->run(FORWARD);
      L_M3->setSpeed(ps2x.Analog(PSAB_PAD_UP));
      R_M4->setSpeed(ps2x.Analog(PSAB_PAD_UP));  
    }else if(ps2x.ButtonReleased(PSB_PAD_UP))  {
      L_M3->run(RELEASE);
      R_M4->run(RELEASE);      
    }
    if(ps2x.Button(PSB_PAD_DOWN)){
      Serial.print("DOWN held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
      L_M3->run(BACKWARD);
      R_M4->run(BACKWARD);
      L_M3->setSpeed(ps2x.Analog(PSAB_PAD_DOWN));
      R_M4->setSpeed(ps2x.Analog(PSAB_PAD_DOWN));  
    }else if(ps2x.ButtonReleased(PSB_PAD_DOWN))  {
      L_M3->run(RELEASE);
      R_M4->run(RELEASE);      
    }
    if(ps2x.Button(PSB_PAD_RIGHT)){
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
      R_M4->run(RELEASE);
    }
    if(ps2x.Button(PSB_PAD_LEFT)){
      Serial.print("LEFT held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
      L_M3->run(RELEASE);
    }
   

    vibrate = ps2x.Analog(PSAB_CROSS);  //this will set the large motor vibrate speed based on how hard you press the blue (X) button
    if (ps2x.NewButtonState()) {        //will be TRUE if any button changes state (on to off, or off to on)
      if(ps2x.Button(PSB_L3))
        Serial.println("L3 pressed");
      if(ps2x.Button(PSB_R3))
        Serial.println("R3 pressed");
      if(ps2x.Button(PSB_L2))
        Serial.println("L2 pressed");
      if(ps2x.Button(PSB_R2))
        Serial.println("R2 pressed");
      if(ps2x.Button(PSB_TRIANGLE))
        Serial.println("Triangle pressed");        
    }

    if(ps2x.ButtonPressed(PSB_CIRCLE))               //will be TRUE if button was JUST pressed
      Serial.println("Circle just pressed");
    if(ps2x.NewButtonState(PSB_CROSS)) {              //will be TRUE if button was JUST pressed OR released
      Serial.println("X just changed");
      ps2x.read_gamepad(true, vibrate);   
  }
    if(ps2x.ButtonReleased(PSB_SQUARE))              //will be TRUE if button was JUST released
      Serial.println("Square just released");       
   // if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
    while(1)
      {
      ps2x.read_gamepad();
      //ps2x.read_gamepad(false, vibrate);
      //vibrate = ps2x.Analog(PSAB_CROSS);  
      //Serial.print("Stick Values:");
     // Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      //Serial.print(",\t");
      //Serial.print(ps2x.Analog(PSS_LX), DEC); 
      //Serial.print(",\t");
      //Serial.print(ps2x.Analog(PSS_RY), DEC); 
      //Serial.print(",\t");
      //Serial.println(ps2x.Analog(PSS_RX), DEC); 
     if(ps2x.Button(PSB_R2))
    {
      
  
      if(ps2x.Button(PSB_PAD_UP))       //will be TRUE as long as button is pressed
      {
        for (pos1 = 0; pos1 <= 180; pos1 += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
      delay(10);                       // waits 15ms for the servo to reach the position
    }
      for (pos1 = 180; pos1 >= 0; pos1 -= 1) { // goes from 180 degrees to 0 degrees
    myservo1.write(pos1);              // tell servo to go to position in variable 'pos'
    delay(10);                       // waits 15ms for the servo to reach the position
        /*Serial.print("Up held this hard: ");
        Serial.println(ps2x.Analog(PSAB_PAD_UP), DEC);*/
      }
      if(ps2x.Button(PSB_PAD_DOWN))
      {
        Serial.print("DOWN held this hard: ");
        Serial.println(ps2x.Analog(PSAB_PAD_DOWN), DEC);
      }
      if(ps2x.Button(PSB_PAD_RIGHT))
      {
      Serial.print("Right held this hard: ");
      Serial.println(ps2x.Analog(PSAB_PAD_RIGHT), DEC);
      }
      if(ps2x.Button(PSB_PAD_LEFT))
      {
        Serial.print("LEFT held this hard: ");
        Serial.println(ps2x.Analog(PSAB_PAD_LEFT), DEC);
      }
      } 
    }
    else
    Serial.println("NOT WORKING!!!");
     int x,y;
     /*if (ps2x.Button(PSB_L1)) // OMNI wheel
    {
      xVal1 = ps2x.Analog(PSS_LX);
      yVal1 = ps2x.Analog(PSS_LY);
      xVal2 = ps2x.Analog(PSS_RX);
      yVal2 = ps2x.Analog(PSS_RY);
      
      
      Serial.print(yVal1); 
      Serial.print(",\t");
      Serial.print(yVal2); 
      Serial.print(",\t");
      Serial.print(xVal1); 
      Serial.print(",\t");
      Serial.println(xVal2); 
      
      x = map(xVal1, 0, 255, -1000, 1000);    //rx
      y = map(yVal1, 0, 255, 1000, -1000);    //ry
      Serial.print("X,Y= ");
      Serial.print(x);
      Serial.print(",");
      Serial.print(y);
      Serial.println(".");
      int m2 = (-x + y) * cos(45);
      int m1 = (-x - y) * cos(45);
      int m4 = (x - y) * cos(45);
      int m3 = (x + y) * cos(45);
      if (m1 >= 0)
      {
        m1 = map(m1, 0, 1000, 0, 254);
        if (m1 > 255) {
          m1 = 255;
        }
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, HIGH);
      }
      else
      {
        m1 = map(m1, 0, -1000, 0, 254);
        if (m1 > 255) {
          m1 = 255;
        }
        digitalWrite(m1a, HIGH);
        digitalWrite(m1b, LOW);
      }
      analogWrite(en1, m1);
      if (m2 >= 0)
      {
        m2 = map(m2, 0, 1000, 0, 254);
        if (m2 > 255) {
          m2 = 255;
        }
        digitalWrite(m2a, HIGH);
        digitalWrite(m2b, LOW);
      }
      else
      {
        m2 = map(m2, 0, -1000, 0, 254);
        if (m2 > 255) {
          m2 = 255;
        }
        digitalWrite(m2a, LOW);
        digitalWrite(m2b, HIGH);
      }
      analogWrite(en2, m2);
      if (m3 >= 0)
      {
        m3 = map(m3, 0, 1000, 0, 254);
        if (m3 > 255) {
          m3 = 255;
        }
        digitalWrite(m3a, LOW);
        digitalWrite(m3b, HIGH);
      }
      else
      {
        m3 = map(m3, 0, -1000, 0, 254);
        if (m3 > 255) {
          m3 = 255;
        }
        digitalWrite(m3a, HIGH);
        digitalWrite(m3b, LOW);
      }
      analogWrite(en3, m3);
      if (m4 >= 0)
      {
        m4 = map(m4, 0, 1000, 0, 254);
        if (m4 > 255) {
          m4 = 255;
        }
        digitalWrite(m4a, LOW);
        digitalWrite(m4b, HIGH);
      }
      else
      {
        m4 = map(m4, 0, -1000, 0, 254);
        if (m4 > 255) {
          m4 = 255;
        }
        digitalWrite(m4a, HIGH);
        digitalWrite(m4b, LOW);
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
      if (xVal2 < 50)   //lx
      {
        digitalWrite(m1a, LOW);
        digitalWrite(m1b, HIGH);
        digitalWrite(m2a, HIGH);
        digitalWrite(m2b, LOW);
        digitalWrite(m3a, LOW);
        digitalWrite(m3b, HIGH);
        digitalWrite(m4a, LOW);
        digitalWrite(m4b, HIGH);
        digitalWrite(en1, HIGH);
        digitalWrite(en2, HIGH);
        digitalWrite(en3, HIGH);
        digitalWrite(en4, HIGH);
        Serial.println("Anticlockwise rotate");
      }
      else if (xVal2 > 190)   //lx
      {
        digitalWrite(m1b, LOW);
        digitalWrite(m1a, HIGH);
        digitalWrite(m2b, HIGH);
        digitalWrite(m2a, LOW);
        digitalWrite(m3b, LOW);
        digitalWrite(m3a, HIGH);
        digitalWrite(m4b, LOW);
        digitalWrite(m4a, HIGH);
        digitalWrite(en1, HIGH);
        digitalWrite(en2, HIGH);
        digitalWrite(en3, HIGH);
        digitalWrite(en4, HIGH);
        Serial.println("Clockwise rotate");
      }
    }
    else
    { Serial.println("Omni Off");
      digitalWrite(en1, LOW);
      digitalWrite(en2, LOW);
      digitalWrite(en3, LOW);
      digitalWrite(en4, LOW);
    }*/
  }
  delay(50);  
//      L_M3->run(RELEASE);
//      R_M4->run(RELEASE);
}
