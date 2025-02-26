#define PUL 3
#define DIR 2
#define STEERING_SENSOR A5
const float MAX_DUTY_CYCLE = 0.15;

float output = 0.5;
float time_low = 3;
float time_high = time_low / (output * MAX_DUTY_CYCLE);
float timer;
float last_time = 0;
float current_time = 0;
float v_out = HIGH;

void setup() {
  Serial.begin(9600);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(PUL, HIGH); // default speed 0
  digitalWrite(DIR, LOW); // default direction CCW
}

void loop() {
  current_time = micros();
  timer = (v_out == LOW) ? time_low : time_high;
  if ((current_time - last_time) >= timer) { 
    v_out = !v_out;
    digitalWrite(PUL, v_out);
    last_time = current_time;
  }

  // Serial.println(time_high);
  // writePercent(output);
}

void writePercent(float value) {
  value = constrain(value, -1, 1);

  if (value < 0) { 
    digitalWrite(DIR, LOW); // spin CCW for negative values
  } else if (value > 0) { 
    digitalWrite(DIR, HIGH); // spin CW for positive values
  }

  // convert value to proportion of max duty cycle, then to duty cycle delay
  float high = 3 / (abs(value) * MAX_DUTY_CYCLE);
  digitalWrite(PUL, LOW);
  delayMicroseconds(3); // Minimum 2.5us
  digitalWrite(PUL, HIGH);
  delayMicroseconds(high);
}
