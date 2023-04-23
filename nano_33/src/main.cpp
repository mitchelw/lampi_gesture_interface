#include <Arduino.h>
#include <skywriter.h>
#include "color_conversion.hpp"

#define PIN_TRFR  21    // TRFR Pin of Skywriter
#define PIN_RESET 20    // Reset Pin of Skywriter

void gesture(unsigned char type);
void touch(unsigned char type);
void xyz(unsigned int x, unsigned int y, unsigned int z);
void brightnessWheel(int deg);
void setLampHsv(float h, float s, float v);

// Config
long touch_timeout = 0;
unsigned int max_x, max_y, max_z;
unsigned int min_x, min_y, min_z;

// Pin constants
const int adc_bitwidth = 256;
const int red_led = 5;
const int green_led = 6;
const int blue_led = 4;

bool delta_rst = true;
int old_delta = 0;

// State
bool is_on = true;
float hue = 1;
float saturation = 1;
float brightness = 1;
int mode = 0;

void setup() {
    Skywriter.begin(PIN_TRFR, PIN_RESET);
    Skywriter.onGesture(gesture);
//    Skywriter.onTouch(touch);
//    Skywriter.onXYZ(xyz);
    Skywriter.onAirwheel(brightnessWheel);
//    analogWriteRes(12);

    pinMode(red_led, OUTPUT);
    pinMode(green_led, OUTPUT);
    pinMode(blue_led, OUTPUT);

    Serial.begin(9600);
//    while(!Serial);

    setLampHsv(hue, saturation, brightness);
}

void loop() {
    Skywriter.poll();
    if( touch_timeout > 0 ) touch_timeout--;
}

void gesture(unsigned char type){
    Serial.println("Got gesture ");
    Serial.print(type,DEC);
    Serial.print('\n');

    if (type == SW_FLICK_SOUTH_NORTH) {
        is_on = true;
        setLampHsv(hue, saturation, brightness);
    } else if (type == SW_FLICK_NORTH_SOUTH) {
        is_on = false;
        setLampHsv(0, 0, 0);
    } else if (type == SW_FLICK_WEST_EAST) {
        mode = (mode + 1) % 3;
    }
}

void brightnessWheel(int deg) {
    double delta = (double)deg / 360.;

    if (mode == 0) {
        double coeff = 0.25;
        hue += delta * coeff;
        if (hue > 1)
            hue = 0;
        else if (hue < 0)
            hue = 1;
    } else if (mode == 1) {
        double coeff = 0.35;
        saturation += delta * coeff;
        if (saturation > 1)
            saturation = 1;
        else if (saturation < 0)
            saturation = 0;
    } else if (mode == 2) {
        double coeff = 0.5;
        brightness += delta * coeff;
        if (brightness > 1)
            brightness = 1;
        else if (brightness < 0)
            brightness = 0;
//        char buf[128];
//        sprintf(buf, "Deg: %d, Delta: %d, Brightness: %d", deg, delta, brightness);
//        Serial.println(buf);
    }
    setLampHsv(hue, saturation, brightness);
}

void setLampHsv(float h, float s, float v) {
    float rgb[3];
    hsv2rgb(h, s, v, rgb);
    analogWrite(red_led, (int)(rgb[0] * 255));
    analogWrite(green_led, (int)(rgb[1] * 255));
    analogWrite(blue_led, (int)(rgb[2] * 255));
}