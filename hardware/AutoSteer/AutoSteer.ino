#define PUL 3
#define DIR 2
#define POTENTIOMETER A0
const float MAX_DUTY_CYCLE = 0.15;
// const float LIMIT = 

float time_low = 3;
float time_high;
float timer;
float last_time = 0;
float current_time = 0;
float v_out = HIGH;

float position = 0;
float potentiometer_value;
float time = 0;
float dt = 0;

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
  read_potentiometer();
  // update_position();
  writePercent(output);
}

void read_potentiometer() {
  current_time_pot = micros();

  if (current_time_pot - last_time_pot >= read_interval) {
    potentiometer_value = analogRead(POTENTIOMETER);
    last_time_pot = current_time_pot;
    Serial.println(position);
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

void update_position() {
  dt = millis() - time;
  position += (output * dt);
  time = millis();
}

void writePercent(float value) {
  value = constrain(value, -1, 1);

  // if (abs(position) > LIMIT) 
  //   value = 0;
    
  if (value < 0) { 
    digitalWrite(DIR, LOW); // spin CCW for negative values
  } else if (value > 0) { 
    digitalWrite(DIR, HIGH); // spin CW for positive values
  }

  if (value != 0) {
    time_high = time_low / (abs(value) * MAX_DUTY_CYCLE);
    current_time = micros();
    timer = (v_out == LOW) ? time_low : time_high;
    if ((current_time - last_time) >= timer) { 
      v_out = !v_out;
      digitalWrite(PUL, v_out);
      last_time = current_time;
      if (v_out == LOW)
        position += value / 100;
    }
  } else {
    digitalWrite(PUL, HIGH);
  }
}
