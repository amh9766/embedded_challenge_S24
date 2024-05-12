#include <Arduino.h>
#include <Adafruit_CircuitPlayground.h>

float X, Y, Z;

void setup() {
    Serial.begin(9600);
    CircuitPlayground.begin();
}

void makeTone(unsigned char speakerPin, int frequencyInHertz, long timeInMilliseconds) {
  int x;   
  long delayAmount = (long)(1000000/frequencyInHertz);
  long loopTime = (long)((timeInMilliseconds*1000)/(delayAmount*2));
  for (x=0; x<loopTime; x++) {        // the wave will be symetrical (same time high & low)
     digitalWrite(speakerPin,HIGH);   // Set the pin high
     delayMicroseconds(delayAmount);  // and make the tall part of the wave
     digitalWrite(speakerPin,LOW);    // switch the pin back to low
     delayMicroseconds(delayAmount);  // and make the bottom part of the wave
  }  
}

void loop() {
    boolean on_switch = CircuitPlayground.slideSwitch();

    X = CircuitPlayground.motionX();
    Y = CircuitPlayground.motionY();
    Z = CircuitPlayground.motionZ();


    if (on_switch) {
        Serial.print(">X:");
        Serial.println(X);
        Serial.print(">Y:");
        Serial.println(Y);
        Serial.print(">Z:");
        Serial.println(Z);
    }

    delay(1000);
}

void lightUpNeoPixels(int pixel, int red, int green, int blue) {
  for (int i = 0; i < 10; i++) {
    CircuitPlayground.setPixelColor(i, red, green, blue);
  }
}

void meter() {
  // defining meter for tremor intensity
  uint8_t meter_colors[10][3] = {
    {0, 255, 0}, // green
    {85, 85, 0}, // low intensity yellow
    {170, 170, 0}, // mid intensity yellow
    {255, 255, 0}, // high intensity yellow
    {85, 55, 0}, // low intensity orange
    {170, 110, 0}, // mid intensity orange
    {255, 165, 0}, // high intensity orange
    {85, 0, 0}, // low intensity red
    {170, 0, 0}, //mid intensity red
    {255, 0, 0}, // high intensity red
  };

  if (/* tremor intensity? */) {
    int pixel_num = 0;
    for (int i = 0; i < pixel_num; i++) {
      lightUpNeoPixels(i, meter_colors[i][0], meter_colors[i][1], meter_colors[i][2]);
    }
  }
}

