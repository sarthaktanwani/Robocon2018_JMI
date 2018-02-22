#define serialEn1 8

void setup() {
  // put your setup code here, to run once:
pinMode(serialEn1,OUTPUT);
Serial.begin(9600);
}
void lsa1()
{ Serial.println("\t\t\t\t LSA 1 READ");
  digitalWrite(serialEn2, HIGH);   // Stop requesting for UART data
  digitalWrite(serialEn3, HIGH);
  digitalWrite(serialEn1, LOW);  // Set serialEN to LOW to request UART data

  while (Serial.available() <= 0);  // Wait for data to be available

  positionVal1 = Serial.read();    // Read incoming data and store in variable positionVal
  digitalWrite(serialEn1, HIGH);   // Stop requesting for UART data

  Serial.print("sensor1");
  Serial.print(positionVal1);
  Serial.print("\t\t");
  pulse1 = digitalRead(jp1);

}
void loop() {
  // put your main code here, to run repeatedly:
  lsa1();
}
