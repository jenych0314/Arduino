#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10
#define PIN_IR A0
#define PIN_LED 9

// configurable parameters
#define _DUTY_MIN 547 // servo full clockwise position (0 degree)
#define _DUTY_NEU 1482 // servo neutral position (90 degree)
#define _DUTY_MAX 2401 // servo full counterclockwise position (180 degree)

// global variables
Servo myservo;
int a, b;

void setup() {
// initialize GPIO pins
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, 1);
  myservo.attach(PIN_SERVO);

  myservo.writeMicroseconds(_DUTY_NEU);
  
// initialize serial port
  Serial.begin(57600);
  
  a = 68; // 값 측정 후 변경
  b = 263; // 값 측정 후 변경
}

float ir_distance(float volt){ // return value unit: mm (cm2mm)
  float val;
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return val;
}

void loop() {
  float volt = float(analogRead(PIN_IR));
  float raw_dist = ir_distance(volt);
  float dist_cali = 100.0 + 300.0/(b-a) * (raw_dist - a);

//  if (200 <= raw_dist && raw_dist <= 350) {
//    digitalWrite(PIN_LED, 0);
//  }
//  else digitalWrite(PIN_LED, 1);

  if (raw_dist >= 255) {
    myservo.writeMicroseconds(1182);
  }
  else {
    myservo.writeMicroseconds(1782);
  }

  // output the read value to the serial port
  Serial.print("min:0, max:500, dist: ");
  Serial.print(raw_dist);
  Serial.print(", volt: ");
  Serial.print(volt);
  Serial.print(", dist_cali: ");
  Serial.println(dist_cali);
  delay(20);
}
