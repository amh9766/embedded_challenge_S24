#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

float X, Y, Z;

void setup() {
    Serial.begin(9600);
    CircuitPlayground.begin();
}

void loop() {
    X = CircuitPlayground.motionX();
    Y = CircuitPlayground.motionY();
    Z = CircuitPlayground.motionZ();

    Serial.print(">X:");
    Serial.println(X);
    Serial.print(">Y:");
    Serial.println(Y);
    Serial.print(">Z:");
    Serial.println(Z);

    delay(1000);
}
