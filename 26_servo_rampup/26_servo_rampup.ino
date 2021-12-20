#include <math.h>
#include <Servo.h>

// Arduino pin assignment
#define PIN_SERVO 10

// configurable parameters
#define _DUTY_MIN (1635 - 500) // servo full clockwise position (0 degree)
#define _DUTY_NEU 1635 // servo neutral position (90 degree)
#define _DUTY_MAX (1635 + 300) // servo full counterclockwise position (180 degree)

#define _SERVO_SPEED 1000 // servo speed limit (unit: degree/second)
#define _RAMPUP_TIME 360 // servo speed rampup (0 to max) time (unit: ms)
#define INTERVAL 20  // servo update interval

#define START _DUTY_MIN + 100
#define END _DUTY_MAX - 100

// global variables
unsigned long last_sampling_time; // unit: ms
int duty_chg_max; // maximum speed, i.e., duty difference per interval (unit: us/interval)
int duty_chg_per_interval; // current speed (unit: us/interval)
int duty_chg_adjust; // duty accelration per interval during ramp up/down period (unit: us/interval^2)
int toggle_interval, toggle_interval_cnt;
float pause_time; // unit: sec
Servo myservo;
int duty_target, duty_curr;

void setup() {
// initialize GPIO pins
  myservo.attach(PIN_SERVO); 
  duty_target = duty_curr = START;
  myservo.writeMicroseconds(duty_curr);
  
// initialize serial port
  Serial.begin(57600);

// servo related variables
  duty_chg_max = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / 180 * INTERVAL / 1000;
  duty_chg_adjust = (float) duty_chg_max * INTERVAL / _RAMPUP_TIME;
  duty_chg_per_interval = 0; // initial speed is set to 0.

// remove next three lines after finding answers
//  Serial.print("duty_chg_per_interval:");
//  Serial.println(duty_chg_per_interval);
//  while(1) {}

// initialize variables for duty_target update.
  pause_time = 0.5;
  toggle_interval = (180.0 / _SERVO_SPEED + pause_time) * 1000 / INTERVAL;
  //toggle_interval = 1; // to demonstrate overshoot
  toggle_interval_cnt = toggle_interval;
  
// initialize last sampling time
  last_sampling_time = 0;
}

void loop() {
  if(millis() < last_sampling_time + INTERVAL) return;

// adjust duty_curr toward duty_target
  if(duty_target > duty_curr) {
    if(duty_chg_per_interval < duty_chg_max) {
      duty_chg_per_interval += duty_chg_adjust;
      if(duty_chg_per_interval > duty_chg_max) duty_chg_per_interval = duty_chg_max;
    }
    duty_curr += duty_chg_per_interval;
    if(duty_curr > duty_target) duty_curr = duty_target;
  }
  else if(duty_target < duty_curr) {
    if(duty_chg_per_interval > -duty_chg_max) {
      duty_chg_per_interval -= duty_chg_adjust;
      if(duty_chg_per_interval < -duty_chg_max) duty_chg_per_interval = -duty_chg_max;
    }
    duty_curr += duty_chg_per_interval;
    if(duty_curr < duty_target) duty_curr = duty_target;
  }
  else {
    duty_chg_per_interval = 0;
  }

// update servo position
  myservo.writeMicroseconds(duty_curr);

// output the read value to the serial port
  Serial.print("Min:400,duty_target:");
  Serial.print(duty_target);
  Serial.print(",duty_curr:");
  Serial.print(duty_curr);
  Serial.print(",duty_chg_max:");
  Serial.print(duty_chg_max);
  Serial.print(",duty_chg_adjust:");
  Serial.print(duty_chg_adjust);  
  Serial.print(",duty_chg_per_interval:");
  Serial.print(duty_chg_per_interval);  
  Serial.println(",Max:2400");

// toggle duty_target between _DUTY_MIN and _DUTY_MAX.
  if(toggle_interval_cnt >= toggle_interval) {
    toggle_interval_cnt = 0;
    if(duty_curr < START + 200) duty_target = END;
    else if(duty_curr > END - 200) duty_target = START;
  }
  else {
    toggle_interval_cnt++;
  }

// update last sampling time
  last_sampling_time += INTERVAL;
}
