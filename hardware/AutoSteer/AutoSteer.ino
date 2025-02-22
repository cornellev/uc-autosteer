#define PUL_PIN 3 // Pulse pin connected to PUL+
#define DIR_PIN 2 // Direction pin connected to DIR+

void setup()
{
    pinMode(PUL_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    digitalWrite(PUL_PIN, HIGH);
    digitalWrite(DIR_PIN, LOW);
}
void loop()
{
    // put your main code here, to run repeatedly:
    // Serial.println("what the fudge");
    digitalWrite(PUL_PIN, LOW);
    delayMicroseconds(3); // Minimum 2.5us
    digitalWrite(PUL_PIN, HIGH);
    delayMicroseconds(20); // Increasing HIGH time reduces speed
}
