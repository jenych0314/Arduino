#include <Servo.h>
#include <math.h>

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
#define _DIST_ALPHA 0.16

#define _DUTY_NEU 1635

// Event periods
#define _INTERVAL_DIST 20
#define _INTERVAL_SERIAL 100

#define num 20
// Servo instance
Servo myservo;

// Distance sensor
float dist_target; // location to send the ball
float dist_raw, dist_raw_cancel, dist_cali, dist_ema;
float sensorValues[num];

// Event periods
unsigned long last_sampling_time_dist, last_sampling_time_serial;
bool event_dist, event_serial;

void setup() {
  // initialize GPIO pins for LED and attach servo
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);

  // move servo to neutral position
  myservo.writeMicroseconds(_DUTY_NEU);

  // initialize serial port
  Serial.begin(57600);

  // initialize global variables
  dist_raw = dist_raw_cancel = dist_cali = dist_ema = ir_distance();

  last_sampling_time_dist = 0;
  last_sampling_time_serial = 0;

  event_dist = event_serial = false;
}

void loop() {
  // Event generator
  unsigned long time_curr = millis();
  if (time_curr >= last_sampling_time_dist + _INTERVAL_DIST) {
    last_sampling_time_dist += _INTERVAL_DIST;
    event_dist = true;
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
    
    for (int i = 0; i < num - 1; i++) {
      sensorValues[i] = sensorValues[i + 1];
    }
    sensorValues[num - 1] = dist_raw;
    for (int i = 0; i < num; i++) {
      dist_cali += sensorValues[i];
    }
    dist_cali /= num;
    
    //    dist_raw_cancel = dist_noise_filter(dist_raw);
    dist_cali = dist_filter(dist_cali);
    dist_ema = _DIST_ALPHA * dist_cali + (1 - _DIST_ALPHA) * dist_ema;
  }

  if (event_serial) {
    event_serial = false;

    Serial.print(", RAW: ");
    Serial.print(dist_raw);
//    Serial.print(", Noise: ");
//    Serial.print(dist_raw_cancel);
    Serial.print(", EMA: ");
    Serial.print(dist_ema);
    Serial.println("");
  }
}

float ir_distance(void) { // return value unit: mm
  float val;
  float volt = float(analogRead(PIN_IR));
  val = ((6762.0 / (volt - 9.0)) - 4.0) * 10.0;
  return val;
}

//float dist_noise_filter(float raw_dist) {
//  for (int i = 0; i < 20 - 1; i++) {
//    sensorValues[i] = sensorValues[i - 1];
//  }
//
//  sensorValues[20 - 1] = raw_dist;
//
//  float filteredValue;
//
//  for (int i = 0; i < 20; i++) {
//    filteredValue += sensorValues[i];
//  }
//
//  filteredValue /= 20;
//
//  return filteredValue;
//}

float dist_filter(float raw_dist) { // return value unit: mm
  float raw_values[7] = {69, 106, 146, 178, 212, 238, 246};
  float original_values[7] = {100, 150, 200, 250, 300, 350, 400};
  float check_point_size = sizeof(raw_values) / sizeof(float);

  float filtered_val = 0;

  for (int i = 0; i < check_point_size; i++) {
    float p = 1;
    for (int j = 0; j < check_point_size; j++) {
      if (i != j) {
        p = p * (raw_dist - raw_values[j]) / (raw_values[i] - raw_values[j]);
      }
    }
    filtered_val += p * original_values[i];
  }

  if (filtered_val < _DIST_MIN) filtered_val = _DIST_MIN;
  else if (filtered_val > _DIST_MAX) filtered_val = _DIST_MAX;

  return filtered_val;
}
