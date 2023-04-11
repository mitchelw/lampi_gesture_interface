#include <Arduino.h>
#include <skywriter.h>
#include "color_conversion.hpp"

#define PIN_TRFR  2    // TRFR Pin of Skywriter
#define PIN_RESET 3    // Reset Pin of Skywriter

void gesture(unsigned char type);
void touch(unsigned char type);
void xyz(unsigned int x, unsigned int y, unsigned int z);
void brightnessWheel(int deg);

long touch_timeout = 0;
unsigned int max_x, max_y, max_z;
unsigned int min_x, min_y, min_z;

const int adc_bitwidth = 256;

const int blue_led = 4;

bool delta_rst = true;
int old_delta = 0;

int brightness = 255;

void setup() {
    Skywriter.begin(PIN_TRFR, PIN_RESET);
    Skywriter.onGesture(gesture);
//    Skywriter.onTouch(touch);
//    Skywriter.onXYZ(xyz);
    Skywriter.onAirwheel(brightnessWheel);
//    analogWriteRes(12);

    pinMode(blue_led, OUTPUT);

    Serial.begin(9600);
//    while(!Serial);
}

void loop() {
    Skywriter.poll();
    if( touch_timeout > 0 ) touch_timeout--;
}

void gesture(unsigned char type){
    Serial.println("Got gesture ");
    Serial.print(type,DEC);
    Serial.print('\n');

    if( type == SW_FLICK_WEST_EAST ){
        if (brightness > 0) {
            analogWrite(blue_led, 0);
            brightness = 0;
        }
        else {
            analogWrite(blue_led, adc_bitwidth - 1);
            brightness = adc_bitwidth - 1;
        }
    }
}

void touch(unsigned char type) {
    Serial.println("Got touch ");
    Serial.print(type,DEC);
    Serial.print('\n');

    if( type == SW_TOUCH_CENTER ){
        touch_timeout = 100;
        if (brightness > 0) {
            analogWrite(blue_led, 0);
            brightness = 0;
        }
        else {
            analogWrite(blue_led, adc_bitwidth - 1);
            brightness = adc_bitwidth - 1;
        }
    } else if (type == SW_GESTURE_GARBAGE) {
        for (int i = 0; i < 4; i++) {
            analogWrite(blue_led, adc_bitwidth - 1);
            delayMicroseconds(250);
            analogWrite(blue_led, 0);
            delayMicroseconds(250);
        }
    }
}

void xyz(unsigned int x, unsigned int y, unsigned int z) {
    if( touch_timeout > 0 ) return;

    if (x < min_x) min_x = x;
    if (y < min_y) min_y = y;
    if (z < min_z) min_z = z;
    if (x > max_x) max_x = x;
    if (y > max_y) max_y = y;
    if (z > max_z) max_z = z;

    char buf[18];
    sprintf(buf, "%05u:%05u:%05u", x, y, z);
    Serial.println(buf);
}

void brightnessWheel(int deg) {
    double delta = (double)deg / 360. * adc_bitwidth;

    double coeff = 0.5;
    int newBrightness = brightness + (int)(delta * coeff);
    if (newBrightness >= 0 && newBrightness < adc_bitwidth) {
        brightness = newBrightness;
        analogWrite(blue_led, brightness);
    }
    char buf[128];
    sprintf(buf, "Deg: %d, Delta: %d, Brightness: %d", deg, delta, brightness);
    Serial.println(buf);
}