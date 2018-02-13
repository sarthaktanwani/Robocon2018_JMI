#include "PS2X_lib.h"  //for v1.6
#include <Wire.h>
#include "Adafruit_MotorShield.h"
#include "Adafruit_MS_PWMServoDriver.h"

/******************************************************************
 * set pins connected to PS2 controller:
 *   - 1e column: original 
 *   - 2e colmun: Stef?
 * replace pin numbers by the ones you use
 ******************************************************************/
#define PS2_DAT        12  //14    
#define PS2_CMD        11  //15
#define PS2_SEL        10  //16
#define PS2_CLK        13  //17
#define m1a 0
#define m1b 1
#define m2a 2
#define m2b 3
#define m3a 4
#define m3b 5
#define m4a 6
#define m4b 7
#define en1 A0
#define en2 A1
#define en3 A2
#define en4 A3

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
 
  Serial.begin(9600);
  
  delay(3000);  //added delay to give wireless ps2 module some time to startup, before configuring it
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
  // Serial.print("==================444=================\n");
 /* if(error == 1) //skip loop if no controller found
    return; 
  
  if(type == 2){ //Guitar Hero Controller
//    ps2x.read_gamepad();          //read controller
}*/
//   
//    if(ps2x.ButtonPressed(GREEN_FRET))
//      Serial.println("Green Fret Pressed");
//    if(ps2x.ButtonPressed(RED_FRET))
//      Serial.println("Red Fret Pressed");
//    if(ps2x.ButtonPressed(YELLOW_FRET))
//      Serial.println("Yellow Fret Pressed");
//    if(ps2x.ButtonPressed(BLUE_FRET))
//      Serial.println("Blue Fret Pressed");
//    if(ps2x.ButtonPressed(ORANGE_FRET))
//      Serial.println("Orange Fret Pressed"); 
//
//    if(ps2x.ButtonPressed(STAR_POWER))
//      Serial.println("Star Power Command");
//    
//    if(ps2x.Button(UP_STRUM))          //will be TRUE as long as button is pressed
//      Serial.println("Up Strum");
//    if(ps2x.Button(DOWN_STRUM))
//      Serial.println("DOWN Strum");
// 
//    if(ps2x.Button(PSB_START))         //will be TRUE as long as button is pressed
//      Serial.println("Start is being held");
//    if(ps2x.Button(PSB_SELECT))
//      Serial.println("Select is being held");
//    
//    if(ps2x.Button(ORANGE_FRET)) {     // print stick value IF TRUE
//      Serial.print("Wammy Bar Position:");
//      Serial.println(ps2x.Analog(WHAMMY_BAR), DEC); 
//    } 
  
  //else { //DualShock Controller
    /*ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed
    
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
//    lx= ps2x.Analog(PSS_LX);
//    ly= ps2x.Analog(PSS_LY);
//
//   if(ly>130){
//      ly=(ly-128)*2;
////      L_M3->setSpeed(ly);
//      L_M3->run(FORWARD);
////      L_M3->setSpeed(ly);
//      R_M4->run(FORWARD);
//    }else if(ly<125){
//      ly=(127-ly)*2;
//  //    L_M3->setSpeed(ly);
//      L_M3->run(BACKWARD);
//      R_M4->run(BACKWARD);
//    }else{
//      L_M3->run(RELEASE);
//      R_M4->run(RELEASE);
//    }
//    
//    if(lx>130){
//       lx=(lx-128)*2;
//       L_M3->setSpeed(ly);
//       R_M4->setSpeed(ly-lx);
//    }else if(lx<125){
//       lx=(127-lx)*2;
//       L_M3->setSpeed(ly-lx);
//       R_M4->setSpeed(ly);       
//    }else{
//       L_M3->setSpeed(ly);
//       R_M4->setSpeed(ly);  
//    }
//
*/
  
    if(ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) { //print stick values if either is TRUE
      Serial.print("Stick Values:");
      Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX  
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_LX), DEC); 
      Serial.print(",");
      Serial.print(ps2x.Analog(PSS_RY), DEC); 
      Serial.print(",");
      Serial.println(ps2x.Analog(PSS_RX), DEC); 
      xVal1 = ps2x.Analog(PSS_LX);
  yVal1 = ps2x.Analog(PSS_LY);

  xVal2 = ps2x.Analog(PSS_RX);
  yVal2 = ps2x.Analog(PSS_RY);
  Serial.println(yVal2);

  if(yVal1 == 127 && yVal2 == 127)
  {
    //go backward
    y14=map(yVal1,127,255,0,255);
    y23=map(yVal2,127,255,0,255);
    digitalWrite(m2a,LOW);
    digitalWrite(m2b,LOW);
    digitalWrite(m3a,LOW);
    digitalWrite(m3b,LOW);
    analogWrite(en2,y23);
    analogWrite(en3,y23);
      
   }     
  else if(yVal1 < 127 && yVal2 < 127)
  {
    //go forward
    y14=map(yVal1,127,0,0,255);
    y23=map(yVal2,127,0,0,255);
    digitalWrite(m2a,LOW);
    digitalWrite(m2b,HIGH);
    digitalWrite(m3a,LOW);
    digitalWrite(m3b,HIGH);
    //Serial.print(yVal2);
    //Serial.print("\n");
    analogWrite(en2,y23);
    analogWrite(en3,y23);
    
    }

    else if(yVal1 < 127 && yVal2 > 127)
  {
    //go clockwise
    y14=map(yVal1,127,0,0,255);
    y23=map(yVal2,127,255,0,255);
    digitalWrite(m2a,HIGH);
    digitalWrite(m2b,LOW);
    digitalWrite(m3a,HIGH);
    digitalWrite(m3b,LOW);
    analogWrite(en2,y23);
    analogWrite(en3,y23);
    //Serial.print(yVal2);
    //Serial.print("\n");
    }

    else if(yVal1 > 127 && yVal2 < 127)
  {
    //go anti-clockwise
    y14=map(yVal1,127,255,0,255);
    y23=map(yVal2,127,0,0,255);
    digitalWrite(m2a,LOW);
    digitalWrite(m2b,HIGH);
    digitalWrite(m3a,LOW);
    digitalWrite(m3b,HIGH);
    analogWrite(en2,y23);
    analogWrite(en3,y23);
    //Serial.print(yVal2);
    //Serial.print("\n");
    }

    else if(yVal1 > 127 && yVal2 > 127)
  {
    //go backward
    y14=map(yVal1,127,255,0,255);
    y23=map(yVal2,127,255,0,255);
    digitalWrite(m2a,HIGH);
    digitalWrite(m2b,LOW);
    digitalWrite(m3a,HIGH);
    digitalWrite(m3b,LOW);
    analogWrite(en2,y23);
    analogWrite(en3,y23);
    //Serial.print(yVal2);
    //Serial.print("\n");
  }
  
 // }
  delay(50);  
//      L_M3->run(RELEASE);
//      R_M4->run(RELEASE);
}}
