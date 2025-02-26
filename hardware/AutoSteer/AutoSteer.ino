const int PUL = 3;
const int DIR = 2;
// const int ENCODER = A5;
const int MAX_DUTY_CYCLE = 0.15;
float output = 0;
int current_pos;

int iters = 0;

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(9600);
    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    //    pinMode(ENCODER, INPUT);
    digitalWrite(PUL, HIGH);
    digitalWrite(DIR, LOW);
    TCCR1B = (TCCR1B & 0b11111000) | 0x05; // Set prescaler to 1
}

void loop()
{
    // put your main code here, to run repeatedly:
    //    current_pos = analogRead(ENCODER);

    //    if (iters > 100) {
    //      iters = 0;
    //      Serial.println(current_pos);
    //    }
    //    iters += 1;

    output = -0.25;
    //    writePercent(output);
    //    analogWriteFrequency(PUL, 100000);  // 100 kHz
    analogWrite(PUL, 128); // 50% duty cycle
}

void writePercent(float value)
{
    value = constrain(value, -1, 1);

    if (value < 0)
    {
        digitalWrite(DIR, LOW); // spin CCW for negative values
    }
    else if (value > 0)
    {
        digitalWrite(DIR, HIGH); // spin CW for positive values
    }

    // convert value to proportion of max duty cycle, then to integer between 0 and 255
    // int actual_output = int(value * MAX_DUTY_CYCLE * 255);
    // analogWrite(PUL, actual_output);
    float high = 3 / (abs(value) * 0.15);
    digitalWrite(PUL, LOW);
    delayMicroseconds(3); // Minimum 2.5us
    digitalWrite(PUL, HIGH);
    delayMicroseconds(high);
}

// #include <Arduino.h>
//
// const int PUL = 3;
// const int DIR = 2;
// const float MAX_DUTY_CYCLE = 0.15;
// volatile uint16_t highTime = 0;  // Duration PUL stays HIGH
// volatile bool isLowPhase = true; // Track if we're in the LOW phase
//
// void setup()
//{
//     Serial.begin(9600);
//     pinMode(PUL, OUTPUT);
//     pinMode(DIR, OUTPUT);
//     digitalWrite(PUL, HIGH); // Start in HIGH state (LOW to motor)
//     digitalWrite(DIR, LOW);  // Default direction
//
//     // Configure Timer1 for CTC mode
//     noInterrupts();
//     TCCR1A = 0;               // Normal mode
//     TCCR1B = (1 << WGM12);    // CTC mode
//     TCCR1B |= (1 << CS11);    // Prescaler 8 (for 0.5 µs per tick)
//     TIMSK1 |= (1 << OCIE1A);  // Enable compare match interrupt
//     OCR1A = 6; // Start with 3µs LOW phase (6 timer ticks @ 0.5µs/tick)
//     interrupts();
// }
//
// void loop()
//{
//     float output = .5; // Example duty cycle
//     writePercent(output);
// }
//
// void writePercent(float value)
//{
//     value = constrain(value, -1, 1);
//
//     digitalWrite(DIR, (value >= 0) ? HIGH : LOW);
//
//     if (value == 0)
//     {
//         highTime = 0;
//     }
//     else
//     {
//         highTime = (3.0 / (abs(value) * MAX_DUTY_CYCLE)) * 2; // Convert to timer ticks
//     }
// }
//
//// Timer1 ISR: Handles pulse timing
// ISR(TIMER1_COMPA_vect)
//{
//     if (highTime == 0) {
//         digitalWrite(PUL, HIGH);
//         return; // Stop toggling if no movement needed
//     }
//
//     if (isLowPhase)
//     {
//         digitalWrite(PUL, LOW);  // LOW for 3µs (Motor sees HIGH)
//         OCR1A = 6;  // 3µs LOW (6 ticks @ 0.5µs per tick)
//     }
//     else
//     {
//         digitalWrite(PUL, HIGH); // HIGH for variable time (Motor sees LOW)
//         OCR1A = highTime;
//     }
//
//     isLowPhase = !isLowPhase;
// }
//
