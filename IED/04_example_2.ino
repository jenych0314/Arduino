void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  while (!Serial) {
    ; //wait for serial port to connect. Needed for native USB
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Hello World!");
  delay(1000);
}
