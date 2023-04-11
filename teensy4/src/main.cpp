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

int brightness = 255;

void setup() {
    Skywriter.begin(PIN_TRFR, PIN_RESET);
    Skywriter.onGesture(gesture);
    Skywriter.onTouch(touch);
    Skywriter.onXYZ(xyz);
    Skywriter.onAirwheel(brightnessWheel);
    analogWriteRes(12);

    pinMode(13, OUTPUT);

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
            analogWrite(13, 0);
            brightness = 0;
        }
        else {
            analogWrite(13, 4095);
            brightness = 4095;
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
            analogWrite(13, 0);
            brightness = 0;
        }
        else {
            analogWrite(13, 4095);
            brightness = 4095;
        }
    } else if (type == SW_GESTURE_GARBAGE) {
        for (int i = 0; i < 4; i++) {
            analogWrite(13, 4095);
            delayMicroseconds(250);
            analogWrite(13, 0);
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
    int delta = (int)((double )deg / 360. * 4096);
    int newBrightness = brightness + delta;
    if (newBrightness >= 0 && newBrightness < 4096) {
        brightness = newBrightness;
        analogWrite(13, newBrightness);
    }
    char buf[128];
    sprintf(buf, "Delta: %d Brightness: %d", delta, brightness);
    Serial.println(buf);
}