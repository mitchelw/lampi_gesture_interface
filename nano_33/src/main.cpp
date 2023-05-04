#include <Arduino.h>
#include <skywriter.h>
#include "color_conversion.hpp"
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "wifi_config.hpp"
#include <ArduinoJson.h>


// Pin constants
const int dac_bitwidth = 256;
const int red_led = 5;
const int green_led = 6;
const int blue_led = 4;
const int reset_pin = 20;  // TRFR Pin of Skywriter
const int trfr_pin = 21;   // Reset Pin of Skywriter
const int hue_led = 9;
const int saturation_led = 8;
const int brightness_led = 7;

// State
bool is_on = true;
float hue = 1;
float saturation = 1;
float brightness = 1;
int mode = 0;

// WIFI, MQTT
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
const char broker[] = "ec2-107-22-75-242.compute-1.amazonaws.com";
int port = 50001;
const char pub_topic[] = "devices/08b61f821470/lamp/changed";
const char sub_topic[] = "devices/08b61f821470/lamp/set_config";
const char willTopic[] = "lamp/connection/08b61f821470/";
String willPayload = "0";
bool willRetain = true;
int willQos = 2;
int subscribeQos = 1;


void gesture(unsigned char type);
void brightnessWheel(int deg);
void onMqttMessage(int messageSize);
void setLampHsv(float h, float s, float v);
void setLampState();
void publishState();


void setup() {
    Skywriter.begin(trfr_pin, reset_pin);
    Skywriter.onGesture(gesture);
    Skywriter.onAirwheel(brightnessWheel);

    pinMode(red_led, OUTPUT);
    pinMode(green_led, OUTPUT);
    pinMode(blue_led, OUTPUT);
    pinMode(hue_led, OUTPUT);
    pinMode(saturation_led, OUTPUT);
    pinMode(brightness_led, OUTPUT);

    digitalWrite(hue_led, LOW);
    digitalWrite(saturation_led, LOW);
    digitalWrite(brightness_led, LOW);


    Serial.begin(9600);
//    while(!Serial);

    // Set the initial state to red
    setLampHsv(hue, saturation, brightness);

    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
        Serial.print(".");
        delay(5000);
    }
    Serial.println("You're connected to the network!");


    mqttClient.beginWill(willTopic, willPayload.length(), willRetain, willQos);
    mqttClient.print(willPayload);
    mqttClient.endWill();

    Serial.print("Attempting to connect to the MQTT broker: ");
    Serial.println(broker);

    if (!mqttClient.connect(broker, port)) {
        Serial.print("MQTT connection failed! Error code = ");
        Serial.println(mqttClient.connectError());

        while (1);
    }
    Serial.println("You're connected to the MQTT broker!");

    mqttClient.onMessage(onMqttMessage);

    mqttClient.subscribe(sub_topic, subscribeQos);

    // Send the initial state to the broker
    publishState();

    // Set hue pin as confirmation
    digitalWrite(hue_led, HIGH);
}


void loop() {
    Skywriter.poll();
    mqttClient.poll();
}


void gesture(unsigned char type) {
    Serial.println("Got gesture ");
    Serial.print(type,DEC);
    Serial.print('\n');

    if (type == SW_FLICK_SOUTH_NORTH) {         // Swipe Up
        is_on = true;
        setLampState();
    } else if (type == SW_FLICK_NORTH_SOUTH) {  // Swipe Down
        is_on = false;
        setLampState();
    } else if (type == SW_FLICK_WEST_EAST) {    // Swipe Right
        mode = (mode + 1) % 3;
    } else if (type == SW_FLICK_EAST_WEST) {    // Swipe Left
        mode = (mode - 1) % 3;
    }

    if (type == SW_FLICK_WEST_EAST || type == SW_FLICK_EAST_WEST) {
        digitalWrite(hue_led, LOW);
        digitalWrite(saturation_led, LOW);
        digitalWrite(brightness_led, LOW);
        if (mode == 0)
            digitalWrite(hue_led, HIGH);
        else if (mode == 1)
            digitalWrite(saturation_led, HIGH);
        else
            digitalWrite(brightness_led, HIGH);
    }
}

void brightnessWheel(int deg) {
    float delta = (float)deg / 360.;  // Angular change scaled 0-1
    float coeff;                      // Coefficient to adjust the amount of change reflected in a given delta for each action

    if (mode == 0) {            // Hue Select
        coeff = 0.25;
        hue += delta * coeff;
        if (hue > 1)       // Wrap hue back to 0 after exceeding 1
            hue = 0;
        else if (hue < 0)  // And vice versa
            hue = 1;
    } else if (mode == 1) {     // Saturation Select
        coeff = 0.35;
        saturation += delta * coeff;
        // Bound Saturation from 0-1
        if (saturation > 1)
            saturation = 1;
        else if (saturation < 0)
            saturation = 0;
    } else if (mode == 2) {     // Brightness Select
        coeff = 0.5;
        brightness += delta * coeff;
        // Bound Brightness from 0-1
        if (brightness > 1)
            brightness = 1;
        else if (brightness < 0)
            brightness = 0;
    }
    setLampState();
}


void onMqttMessage(int messageSize) {
    char json[1024];
    int i = 0;
    while (mqttClient.available() && i < 1023) {
        char c = (char)mqttClient.read();
        Serial.print(c);
        json[i++] = c;  // Read characters into the json buffer
    }
    json[i] = 0;

    DynamicJsonDocument doc(1024);
    deserializeJson(doc, String(json));

    brightness = doc[String("brightness")];
    hue = doc[String("color")][String("h")];
    saturation = doc[String("color")][String("s")];
    is_on = doc[String("on")];

    setLampState();
}


void setLampHsv(float h, float s, float v) {
    float rgb[3];
    hsv2rgb(h, s, v, rgb);
    analogWrite(red_led, (int)(rgb[0] * (dac_bitwidth - 1)));
    analogWrite(green_led, (int)(rgb[1] * (dac_bitwidth - 1)));
    analogWrite(blue_led, (int)(rgb[2] * (dac_bitwidth - 1)));
}


void setLampState() {
    if (is_on) {
        setLampHsv(hue, saturation, brightness);
    } else {
        setLampHsv(0, 0, 0);
    }
    publishState();
}


void publishState() {
    char json_out[1024];
    snprintf(json_out, 1024, R"({"color": {"h": %f, "s": %f}, "brightness": %f, "on": %s, "client": "08b61f821470"})",
             hue, saturation, brightness, is_on ? "true" : "false");

    mqttClient.beginMessage(pub_topic);
    mqttClient.print(json_out);
    mqttClient.endMessage();
}
