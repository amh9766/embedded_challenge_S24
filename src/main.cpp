#include <Arduino.h>
#include <math.h>
#include <Adafruit_CircuitPlayground.h>

#include <EmbeddedChallenge.h>

int second = 0;
int minute = 0;
int index = 0;
int timesSampled = 0;

int16_t data[DATA_SIZE] = {0};
complex_t dataImg[DATA_SIZE];
uint16_t spectrum[DATA_SIZE / 2] = {0};
float movingAvg[AVG_SIZE] = {0};

uint8_t meterColors[10][3] = {
    {0, 255, 0},    // Green, high
    {85, 85, 0},    // Yellow, low
    {170, 170, 0},  // Yellow, mid
    {255, 255, 0},  // Yellow, high
    {85, 55, 0},    // Orange, low
    {170, 110, 0},  // Orange, mid
    {255, 165, 0},  // Orange, high
    {85, 0, 0},     // Red, low
    {170, 0, 0},    // Red, mid 
    {255, 0, 0},    // Red, high
};

void initTimer()
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
    //              * Note however that it will be off until enabled by the
    //                switch.
    //          * Clock Prescaler -> 256
    //      * TIMSK1 -> 0b XXXX 0010
    //          * Timer/Counter1 Output Compare A Match Interrupt -> Enable
    TCCR1A = 0x00;
    TCCR1B = 0b00001000;
    TIMSK1 = 0b00000010;

    // For the microcontroller's clock frequency of 8 MHz, a tick is 1.25e-7
    // seconds. Note that we wish to count to 2 s. The amount of ticks necessary
    // to reach 2 s can be calculated by 2 s / 1.25e-7 = 16e6. Because this
    // value does not fit within 16 bits (max value of 65535), a prescaler is
    // needed. A prescaler value of 256 was found to reduce the amount of ticks
    // to 62500, which fit within 16 bits.
    OCR1A = 62500 / 2;
}

void initSwitch()
{
    // Note that A3_SWITCH, which is on Port F, Bit 4
    
    // Set bit for DDRF register:
    //      * DDRDF -> 0b XXX1 XXXX
    DDRF &= ~(0x10);
}

void setNeoPixels()
{
    CircuitPlayground.clearPixels();

    for(int i = 0; i < minute; i++)
    {
        CircuitPlayground.setPixelColor(i, meterColors[i][0], 
                meterColors[i][1], meterColors[i][2]);
    }
}

float calcMagnitude(float x, float y, float z)
{
    return sqrtf(x * x + y * y + z * z);
}

void enableTimer()
{
    TCCR1B |= 0b00000100;
}

void disableTimer()
{
    TCNT1 = 0;
    TCCR1B &= ~(0b00000111);
}

ISR(TIMER1_COMPA_vect)
{
    // Disable interrupts to prevent calculation-related interleavings 
    // on the shared data array 
    cli();

    if(minute == 10)
    {
        // Resting tremor was conclusively detected!
        disableTimer();

        for(int i = 0; i < 5; i++)
        {
            setNeoPixels();
            CircuitPlayground.playTone(100, 250);
            CircuitPlayground.clearPixels();
            delay(500);
        }

        second = 0;
        minute = 0;

        enableTimer();
    }
    else
    {
        // Should trigger every two seconds
        if(second < 60)
        {
            // DEBUG: Print the amount of samples taken in the last interval
            //Serial.println(timesSampled);

            // Calculate FFT
            fft_input(data, dataImg);
            fft_execute(dataImg);
            fft_output(dataImg, spectrum);

            for(int i = 0; i < DATA_SIZE / 2; i++){
                // Calculate the center frequency of the current bin
                float curBin = FFT_BIN(i, timesSampled, DATA_SIZE);

                int lowerFreqBound = 3;
                int upperFreqBound = 6;

                // Calculate shift in bounds since FFT binning will not be
                // perfectly centered on the frequencies near the frequencies we
                // want to observe.
                float errorShift = ERROR_BOUND(lowerFreqBound, 10);
                
                // Observe desired range and then break once past it
                if((curBin >= lowerFreqBound - errorShift) 
                        && (curBin <= upperFreqBound + errorShift))
                {
                Serial.print(curBin);
                Serial.print(" Hz: ");

                //print the corresponding FFT output
                Serial.println(spectrum[i]);
                }
                else if(curBin > (upperFreqBound + errorShift))
                    break;
            }

            if(true/* final FFT boolean check */)
                second++;
            else
            {
                // Reset counters if FFT does not indicate a tremor
                second = 0;
                minute = 0;
            }

            timesSampled = 0;
            index = 0;
        }
        
        if(second == 60)
        {

            second = 0;
            minute++;
            timesSampled = 0;

            setNeoPixels();
        }
    }

    sei();
}

void setup() {
    Serial.begin(9600);
    CircuitPlayground.begin();

    initSwitch();
    initTimer();

    sei();
}

void loop() {
    float x = CircuitPlayground.motionX();
    float y = CircuitPlayground.motionY();
    float z = CircuitPlayground.motionZ();

    // Calculate magnitude of acceleration using the x, y, z components 
    // provided by the accelerometer and subtract the acceleration of 
    // gravity to get only the contribution from the user moving the
    // microcontroller.
    float accelMagnitude = calcMagnitude(x, y, z) - GRAVITY;
    
    // DEBUG: Print current acceleration magnitude
    //Serial.print(">Magnitude:");
    //Serial.println(accelMagnitude);

    float sum = 0;

    for(int i = AVG_SIZE - 2; i >= 0; i--)
    {
        movingAvg[i + 1] = movingAvg[i];
        sum += movingAvg[i + 1];
    }

    movingAvg[0] = accelMagnitude;
    sum += movingAvg[0];

    float finalSample = sum / AVG_SIZE;

    // DEBUG: Print average sample value
    //Serial.print(">Average:");
    //Serial.println(finalSample);
    
    // Insert moving average value into the "capture window" data array and
    // increment the index. 
    //  
    // If the array is completely filled, the addition data is not entered in
    // the array to avoid an out-of-bounds index related crash. Moreover, the
    // switch must be activated to start the data compilation process.
    bool switchOn = TEST_BIT_MASK(PINF, 0x10);
    if(switchOn)
    {
        enableTimer();
        if(index < DATA_SIZE)
            data[index++] = (int16_t) (finalSample * 100);
        timesSampled++;
    }
    else
    {
        disableTimer();
        CircuitPlayground.clearPixels();
        second = 0;
        minute = 0;
        index = 0;
    }

    // Because we want to get about 64 samples per second, we would normally
    // want a delay of 15.62 ms to give a loop frequency of 64 Hz; however, we
    // choose a shorter delay to get more samples and increase bin size based on
    // sampling rate since the speed at
    // which loop cycles occur are not exactly the same every time. As such,
    // the timesSampled variable effectively provides the sample rate for each
    // "capture window."
    delay(8);
}
