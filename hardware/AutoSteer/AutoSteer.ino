#define PUL 3
#define DIR 2
#define POTENTIOMETER A0
#define SENSOR A5
const float MAX_PERIOD = 5000;
const float MIN_PERIOD = 225;
// const float LIMIT = 1000;

float period = MAX_PERIOD;
float last_time = 0;
float current_time = 0;
float v_out = HIGH;

float position = 170;
float potentiometer_value;

float absolute_position;

float current_time_pot = 0;
float last_time_pot = 0;
float read_interval = 100000; // read every 0.1s

float output;

void setup() {
  Serial.begin(9600);
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(POTENTIOMETER, INPUT);
  digitalWrite(PUL, HIGH); // default speed 0
  digitalWrite(DIR, LOW); // default direction CCW
  Serial.println("Begin");
}

void loop() {
  read();
  writePercent(output);
}

void read() {
  current_time_pot = micros();

  if (current_time_pot - last_time_pot >= read_interval) {
    potentiometer_value = analogRead(POTENTIOMETER);
    absolute_position = analogRead(SENSOR);
    last_time_pot = current_time_pot;
    Serial.println(absolute_position);
  }
  
  if (potentiometer_value < 341) // lowest third of the potentiometer's range (0-340)
  {                  
    output = -( 1 - potentiometer_value / 340 ); // normalize to percent, negative for CCW
  }
  else if (potentiometer_value < 682) // middle third of potentiometer's range (341-681)
  {
    output = 0; // no movement when in center
  }
  else  // upper third of potentiometer"s range (682-1023)
  {
    output = (potentiometer_value-683) / 340; // normalize to percent, positive for CW
  }
}

void writePercent(float value) {
  value = constrain(value, -1, 1);

  // if (position <= -LIMIT && value < 0) {
  //   value = 0;
  // } else if (position >= LIMIT && value > 0) {
  //   value = 0;
  // }
    
  if (value < 0) { 
    digitalWrite(DIR, LOW); // spin CCW for negative values
  } else if (value > 0) { 
    digitalWrite(DIR, HIGH); // spin CW for positive values
  }

  if (value != 0) {
    period = MAX_PERIOD-((MAX_PERIOD-MIN_PERIOD) * abs(value));
    current_time = micros();
    if ((current_time - last_time) >= period) { 
      v_out = !v_out;
      digitalWrite(PUL, v_out);
      last_time = current_time;
      if (v_out == LOW)
        position += (value > 0 ? 1 : -1);
    }
  } else {
    digitalWrite(PUL, HIGH);
  }
}
