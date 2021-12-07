#include <Servo.h>

// Configurable parameters

// Arduino pin assignment
#define PIN_LED 9
#define PIN_SERVO 10
#define PIN_IR A0

// Framework setting
#define _DIST_TARGET 255
#define _DIST_MIN 100
#define _DIST_MAX 410

// Distance sensor
// 수정 필요
#define _DIST_ALPHA 0.2

// Servo range
// min, max 값 수정 필요
#define _DUTY_MIN (1635 - 500)
#define _DUTY_NEU 1635
#define _DUTY_MAX (1635 + 75)

// Servo speed control
#define _SERVO_ANGLE 30
#define _SERVO_SPEED 30

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERVO 20
#define _INTERVAL_SERIAL 100

// PID parameters
// 수정 필요
#define _KP 1.0
#define _KD 20.0
//#define _KP 1.0
//#define _KD 30.0

// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_cali, dist_ema;

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_servo, last_sampling_time_serial;
bool event_dist, event_servo, event_serial;

// Servo speed control
int duty_chg_per_interval;
int duty_target, duty_curr;

// PID variables
float error_curr, error_prev, control, pterm, dterm, iterm;

void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);

  // move servo to neutral position
  duty_curr = _DUTY_NEU;
  myservo.writeMicroseconds(duty_curr);

  // initialize serial port
  Serial.begin(57600);

  // initialize global variables
  dist_target = _DIST_TARGET;
  dist_raw = dist_cali = dist_ema = ir_distance();
  error_curr = error_prev = dist_target - dist_raw;

  last_sampling_time_dist = 0;
  last_sampling_time_servo = 0;
  last_sampling_time_serial = 0;

  event_dist = event_servo = event_serial = false;

  // convert angle speed into duty change per interval.
  duty_chg_per_interval = (float)(_DUTY_MAX - _DUTY_MIN) * _SERVO_SPEED / ((float)_SERVO_ANGLE) * _INTERVAL_SERVO / 1000;
}

void loop() {
  // Event generator
  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
  }
  if (time_curr >= last_sampling_time_servo + _INTERVAL_SERVO) {
    last_sampling_time_servo += _INTERVAL_SERVO;
    event_servo = true;
  }
  if (time_curr >= last_sampling_time_serial + _INTERVAL_SERIAL) {
    last_sampling_time_serial += _INTERVAL_SERIAL;
    event_serial = true;
  }

  // Event handlers
  if (event_dist) {
    event_dist = false;

    // get a distance reading from the distance sensor
    dist_raw = ir_distance();
    dist_cali = dist_filter(dist_raw);
    dist_ema = _DIST_ALPHA * dist_cali + (1 - _DIST_ALPHA) * dist_ema;

    // PID control logic
    error_curr = dist_target - dist_ema;
    pterm = _KP * error_curr;
    dterm = _KD * (error_curr - error_prev);
    control = pterm + dterm;

    // duty_target = f(duty_neutral, control)
    duty_target = _DUTY_NEU + control;

    // keep duty_target value within the range of [_DUTY_MIN, _DUTY_MAX]
    if (duty_target < _DUTY_MIN) duty_target = _DUTY_MIN;
    else if (duty_target > _DUTY_MAX) duty_target = _DUTY_MAX;

    // error update
    error_prev = error_curr;
  }

  if (event_servo) {
    event_servo = false;

    // adjust duty_curr toward duty_target by duty_chg_per_interval
    if (duty_target > duty_curr) {
      duty_curr += duty_chg_per_interval;
      if (duty_curr > duty_target) duty_curr = duty_target;
    }
    else {
      duty_curr -= duty_chg_per_interval;
      if (duty_curr < duty_target) duty_curr = duty_target;
    }

    // update servo position
    myservo.writeMicroseconds(duty_curr);
  }

  if (event_serial) {
    event_serial = false;
    Serial.print("dist_ir: ");
    Serial.print(dist_raw);
    Serial.print(", dist_cali: ");
    Serial.print(dist_cali);
    Serial.print(", dist_ema: ");
    Serial.print(dist_ema);
    Serial.print(", pterm: ");
    Serial.print(map(pterm, -1000, 1000, 510, 610));
    Serial.print(", dterm: ");
    Serial.print(map(dterm, -1000, 1000, 510, 610));
    Serial.print(" ,duty_target: ");
    Serial.print(map(duty_target, 1000, 2000, 410, 510));
    Serial.print(", duty_curr: ");
    Serial.print(map(duty_curr, 1000, 2000, 410, 510));
    Serial.println(", Min: 100, Low: 200, dist_target: 255, High: 310, Max: 410");
    Serial.println("");
  }
}

float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return val;
}

float dist_filter(float raw_dist) { // return value unit: mm
//  float raw_values[7] = {80, 122, 173, 217, 271, 303, 317};
  float raw_values[7] = {72, 112, 155, 190, 232, 252, 263};
  float original_values[7] = {100, 150, 200, 250, 300, 350, 400};
  float check_point_size = sizeof(raw_values)/sizeof(float);

  float filtered_val = 0;
  
  for(int i = 0; i < check_point_size; i++) {
      float p = 1;
      for(int j = 0; j < check_point_size; j++) {
          if(i != j) {
            p = p * (raw_dist - raw_values[j]) / (raw_values[i] - raw_values[j]);
          }
      }
      filtered_val += p * original_values[i];
  }

  if (filtered_val < _DIST_MIN) filtered_val = _DIST_MIN;
  else if (filtered_val > _DIST_MAX) filtered_val = _DIST_MAX;
  
  return filtered_val;
}
