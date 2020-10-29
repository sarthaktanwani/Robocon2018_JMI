// Henry's Bench
// Module KY023

#define m1a 30
#define m1b 31
#define m2a 34
#define m2b 35
#define m3a 38
#define m3b 39
#define m4a 42
#define m4b 43
#define en1 32
#define en2 36
#define en3 40
#define en4 44

uint8_t Xin1= A0; // X Input Pin
uint8_t Yin1= A1; // Y Input Pin
uint8_t KEYin1 = 3; // Push Button

uint8_t Xin2= A2; // X Input Pin
uint8_t Yin2= A3; // Y Input Pin
uint8_t KEYin2 = 4; // Push Button
void setup ()
{
  pinMode (KEYin1, INPUT);
  pinMode (KEYin2, INPUT);
  Serial.begin (9600); 
}
void loop ()
{
  uint8_t y14,y23,xVal1, yVal1, buttonVal1,xVal2, yVal2, buttonVal2;
  
  xVal1 = analogRead (Xin1);
  yVal1 = analogRead (Yin1);
  buttonVal1 = digitalRead (KEYin1);

  xVal2 = analogRead (Xin2);
  yVal2 = analogRead (Yin2);
  Serial.println(yVal2);
  buttonVal2 = digitalRead (KEYin2);

 /* while(1)
  {
    digitalWrite(m2a,HIGH);
    digitalWrite(m2b,LOW);
    digitalWrite(m3a,HIGH);
    digitalWrite(m3b,LOW);
    analogWrite(en2,200);
    analogWrite(en3,200);
    }*/
  if(yVal1 >= 512 && yVal2 >= 512)
  {
    //go forward
    y14=map(yVal1,512,1024,0,255);
    y23=map(yVal2,512,1024,0,255);
    digitalWrite(m2a,LOW);
    digitalWrite(m2b,HIGH);
    digitalWrite(m3a,LOW);
    digitalWrite(m3b,HIGH);
    //Serial.print(yVal2);
    //Serial.print("\n");
    /*analogWrite(en2,y23);
    analogWrite(en3,y23);*/
    
    }

    if(yVal1 >= 512 && yVal2 <= 512)
  {
    //go clockwise
    y14=map(yVal1,512,1024,0,255);
    y23=map(yVal2,0,512,0,255);
    digitalWrite(m2a,HIGH);
    digitalWrite(m2b,LOW);
    digitalWrite(m3a,HIGH);
    digitalWrite(m3b,LOW);
    /*analogWrite(en2,y23);
    analogWrite(en3,y23);*/
    //Serial.print(yVal2);
    //Serial.print("\n");
    }

    if(yVal1 <= 512 && yVal2 >= 512)
  {
    //go anti-clockwise
    y14=map(yVal1,0,512,0,255);
    y23=map(yVal2,512,1024,0,255);
    digitalWrite(m2a,LOW);
    digitalWrite(m2b,HIGH);
    digitalWrite(m3a,LOW);
    digitalWrite(m3b,HIGH);
    /*analogWrite(en2,y23);
    analogWrite(en3,y23);*/
    //Serial.print(yVal2);
    //Serial.print("\n");
    }

    if(yVal1 <= 512 && yVal2 <= 512)
  {
    //go backward
    y14=map(yVal1,0,512,0,255);
    y23=map(yVal2,0,512,0,255);
    digitalWrite(m2a,HIGH);
    digitalWrite(m2b,LOW);
    digitalWrite(m3a,HIGH);
    digitalWrite(m3b,LOW);
    /*analogWrite(en2,y23);
    analogWrite(en3,y23);*/
    //Serial.print(yVal2);
    //Serial.print("\n");
  }
}
