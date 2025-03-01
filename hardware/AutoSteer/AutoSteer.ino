#define PUL 3
#define DIR 2
#define SENSOR A5
const float MAX_DUTY_CYCLE = 0.15;

float time_low = 3;
float time_high;
float timer;
float last_time = 0;
float last_read_time = 0;
float current_time = 0;
float current_time_sensor = 0;
float v_out = HIGH;
int position;
int read_interval = 1000000; // read every 1s

void setup() {
  Serial.begin(9600);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(SENSOR, INPUT);
  digitalWrite(PUL, HIGH); // default speed 0
  digitalWrite(DIR, LOW); // default direction CCW
  Serial.println("Begin");
}

void loop() {
  // writePercent(-1);
  read();
}

void read() {
  current_time_sensor = micros();
  if (current_time_sensor - last_read_time >= read_interval) {
    position = analogRead(SENSOR);
    last_read_time = current_time_sensor;
    Serial.println(position);
  }
}

void writePercent(float value) {
  value = constrain(value, -1, 1);
    
  if (value < 0) { 
    digitalWrite(DIR, LOW); // spin CCW for negative values
  } else if (value > 0) { 
    digitalWrite(DIR, HIGH); // spin CW for positive values
  }

  time_high = time_low / (abs(value) * MAX_DUTY_CYCLE);
  current_time = micros();
  timer = (v_out == LOW) ? time_low : time_high;
  if ((current_time - last_time) >= timer) { 
    v_out = !v_out;
    digitalWrite(PUL, v_out);
    last_time = current_time;
  }
}
