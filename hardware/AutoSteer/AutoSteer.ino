#define PUL 3
#define DIR 2
#define POTENTIOMETER A0
#define SENSOR A5
const float MAX_PERIOD = 5000;
const float MIN_PERIOD = 225;
const float MIN_POSITION = 130;
const float MAX_POSITION = 450;

const float MIN_STEERING_ANGLE = -0.35;
const float MAX_STEERING_ANGLE = 0.35;

float goal_position;
const float kP = 0.03;

float period = MAX_PERIOD;
float last_time = 0;
float current_time = 0;
float v_out = HIGH;

float position = 170;
float potentiometer_value;

float absolute_position;

float current_time_pot = 0;
float last_time_pot = 0;
float read_interval = 100000;  // read every 0.1s

float output;

float last_target = 290;

void setup() {
    Serial.begin(115200);
    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(POTENTIOMETER, INPUT);
    digitalWrite(PUL, HIGH);  // default speed 0
    digitalWrite(DIR, LOW);   // default direction CCW
    // Serial.println("Begin");
}

void loop() {
    goal_position = read();
    output = calculate();
    writePercent(output);
}

float read() {
    current_time_pot = micros();

    if (current_time_pot - last_time_pot >= read_interval) {
        // potentiometer_value = analogRead(POTENTIOMETER);
        absolute_position = analogRead(SENSOR);
        last_time_pot = current_time_pot;
        // Serial.println(absolute_position);
        // Serial.println(output);

        if (Serial.available() > 0) {
          // Read the incoming data as a string
          String received_data = Serial.readStringUntil('\n');
          parseMessage(received_data);
        }

        Serial.print(current_time_pot);        // time
        Serial.print(" ");
        Serial.print(last_target, 4);                    // vel
        Serial.print(" ");
        Serial.print(absolute_position, 4);  // steer TODO: FIX
        Serial.print("\n");

    }

    // velocity control
    // if (potentiometer_value < 341) // lowest third of the potentiometer's range (0-340)
    // {
    //   return = -( 1 - potentiometer_value / 340 ); // normalize to percent, negative for CCW
    // }
    // else if (potentiometer_value < 682) // middle third of potentiometer's range (341-681)
    // {
    //   return = 0; // no movement when in center
    // }
    // else  // upper third of potentiometer"s range (682-1023)
    // {
    //   return = (potentiometer_value-683) / 340; // normalize to percent, positive for CW
    // }

    // positional control
    // return map(potentiometer_value, 0, 1023, 130, 450);

    return last_target;
}

float calculate() {
    return kP * (goal_position - absolute_position);
}

void parseMessage(String received_data) {
    // Split the string based on spaces
    int first_space = received_data.indexOf(' ');
    int second_space = received_data.indexOf(' ', first_space + 1);

    if (first_space > 0 && second_space > 0) {
        String angle_str = received_data.substring(first_space + 1, second_space);

        // Serial.print("New message: \n");
        // Serial.print(angle_str);
        // Serial.print("\n");
        // Serial.print(angle_str.toFloat());
        // Serial.print("\n");

        float parsed = angle_str.toFloat();

        parsed = constrain(parsed, MIN_STEERING_ANGLE, MAX_STEERING_ANGLE);
        last_target = (parsed - MIN_STEERING_ANGLE) * (MAX_POSITION - MIN_POSITION) / (MAX_STEERING_ANGLE - MIN_STEERING_ANGLE) + MIN_POSITION;

        // Convert to float
        // last_target = map(angle_str.toFloat(), MIN_STEERING_ANGLE, MAX_STEERING_ANGLE, MIN_POSITION, MAX_POSITION);
    }
}

void writePercent(float value) {
    value = constrain(value, -1, 1);

    if (absolute_position <= MIN_POSITION && value < 0) {
        value = 0;
    } else if (absolute_position >= MAX_POSITION && value > 0) {
        value = 0;
    }

    if (value < 0) {
        digitalWrite(DIR, LOW);   // spin CCW for negative values
    } else if (value > 0) {
        digitalWrite(DIR, HIGH);  // spin CW for positive values
    }

    if (abs(value) > 0.8) {
        period = MAX_PERIOD - ((MAX_PERIOD - MIN_PERIOD) * abs(value));
        current_time = micros();
        if ((current_time - last_time) >= period) {
            v_out = !v_out;
            digitalWrite(PUL, v_out);
            last_time = current_time;
            if (v_out == LOW) position += (value > 0 ? 1 : -1);
        }
    } else {
        digitalWrite(PUL, HIGH);
    }
}
