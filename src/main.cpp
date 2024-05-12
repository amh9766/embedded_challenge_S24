#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

#include <math.h>

#define GRAVITY 9.8f
#define ARR_SIZE 10

float data[ARR_SIZE] = {0};

float calcMagnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

void setup() {
    Serial.begin(9600);
    CircuitPlayground.begin();
}

void loop() {
    float X = CircuitPlayground.motionX();
    float Y = CircuitPlayground.motionY();
    float Z = CircuitPlayground.motionZ();

    float accelerationMag = calcMagnitude(X, Y, Z) - GRAVITY;
    Serial.print(">Magnitude:");
    Serial.println(accelerationMag);

    float sum = 0;

    for(int i = ARR_SIZE - 2; i >= 0; i--)
    {
        data[i + 1] = data[i];
        sum += data[i + 1];
    }

    data[0] = accelerationMag;
    sum += data[0];

    Serial.print(">Average:");
    Serial.println(sum / ARR_SIZE);

    delay(100);
}


