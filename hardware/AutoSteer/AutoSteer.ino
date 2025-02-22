const int PUL = 3;
const int DIR = 2;
const int MAX_DUTY_CYCLE = 0.15;
float output = 0;

void setup() {
  // put your setup code here, to run once:
  pinMode(PUL, OUTPUT);
  pinMode(DIR, OUTPUT);
  digitalWrite(PUL, HIGH);
  digitalWrite(DIR, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  output = 0.5;
  writePercent(output);
}

void writePercent(float value) {
  value = constrain(value, -1, 1);

  if (value < 0) { 
    digitalWrite(DIR, LOW); // spin CCW for negative values
  } else if (value > 0) { 
    digitalWrite(DIR, HIGH); // spin CW for positive values
  }

  // convert value to proportion of max duty cycle, then to integer between 0 and 255
  // int actual_output = int(value * MAX_DUTY_CYCLE * 255); 
  // analogWrite(PUL, actual_output);
  float high = 3 / (abs(value)*0.15);
  digitalWrite(PUL, LOW);
  delayMicroseconds(3); // Minimum 2.5us
  digitalWrite(PUL, HIGH);
  delayMicroseconds(high);
}
