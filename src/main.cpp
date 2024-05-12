#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

#include <math.h>

#define GRAVITY 9.8f
#define ARR_SIZE 100
#define AVG_SIZE 10

int second = 0;
int minute = 0;

float data[ARR_SIZE] = {0};
float movingAvg[AVG_SIZE] = {0};

float calcMagnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

void init2SecTimer()
{
    // Using Timer/Counter1, which is a 16-bit counter
    
    // Setting bits for TCCR1A and TCCR1B registers (pg. 131)
    //      * TCCR1A -> 0b 0000 0000
    //          * COM1A, OC1A -> disconnected
    //          * COM1b, OC1B -> disconnected
    //          * COM1C, OC1C -> disconnected
    //          * Waveform Generation Mode -> CTC
    //      * TCCR1B -> 0b XXX0 1100
    //          * Waveform Generation Mode -> CTC
    //          * Clock Prescaler -> 256
    //      * TIMSK1 -> 0b XXXX 0010
    //          * Timer/Counter1 Output Compare A Match Interrupt -> Enable
    TCCR1A = 0x00;
    TCCR1B = 0b00001100;
    TIMSK1 = 0b00000010;

    // For the microcontroller's clock frequency of 8 MHz, a tick is 1.25e-7
    // seconds. Note that we wish to count to 2 s. The amount of ticks necessary
    // to reach 2 s can be calculated by 2 s / 1.25e-7 = 16e6. Because this
    // value does not fit within 16 bits (max value of 65535), a prescaler is
    // needed. A prescaler value of 256 was found to reduce the amount of ticks
    // to 62500, which fit within 16 bits.
    OCR1A = 62500;
}

ISR(TIMER1_COMPA_vect)
{
    // Should trigger every two seconds
    Serial.println("Hello World!");
}

void setup() {
    Serial.begin(9600);
    CircuitPlayground.begin();
    init2SecTimer();
    sei();
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
