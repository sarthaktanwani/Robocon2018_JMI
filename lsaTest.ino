#define SERIAL_ENABLE_1 8
#define SERIAL_ENABLE_2 9
#define SERIAL_ENABLE_3 10

void setup() 
{
  // put your setup code here, to run once:
  pinMode(SERIAL_ENABLE_1,OUTPUT);
  Serial.begin(9600);
}
void lsa1()
{ 
  Serial.println("\t\t\t\t LSA 1 READ");
  digitalWrite(SERIAL_ENABLE_2, HIGH);   // Stop requesting for UART data
  digitalWrite(SERIAL_ENABLE_3, HIGH);
  digitalWrite(SERIAL_ENABLE_1, LOW);  // Set serialEN to LOW to request UART data

  while (Serial.available() <= 0);  // Wait for data to be available

  positionVal1 = Serial.read();    // Read incoming data and store in variable positionVal
  digitalWrite(SERIAL_ENABLE_1, HIGH);   // Stop requesting for UART data

  Serial.print("sensor1");
  Serial.print(positionVal1);
  Serial.print("\t\t");
  pulse1 = digitalRead(jp1);

}
void loop() {
  // put your main code here, to run repeatedly:
  lsa1();
}
