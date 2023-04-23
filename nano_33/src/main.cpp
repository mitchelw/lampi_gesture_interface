#include <Arduino.h>
#include <skywriter.h>
#include "color_conversion.hpp"
#include <ArduinoMqttClient.h>
#include <WiFiNINA.h>
#include "wifi_config.hpp"
#include <ArduinoJson.h>

#define PIN_TRFR  21    // TRFR Pin of Skywriter
#define PIN_RESET 20    // Reset Pin of Skywriter

void gesture(unsigned char type);
void touch(unsigned char type);
void xyz(unsigned int x, unsigned int y, unsigned int z);
void brightnessWheel(int deg);
void onMqttMessage(int messageSize);
void setLampHsv(float h, float s, float v);
void setLampState();
void publishState();

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

// WIFI, MQTT
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);
const char broker[] = "ec2-107-22-75-242.compute-1.amazonaws.com";
int        port     = 50001;
const char pub_topic[]  = "devices/08b61f821470/lamp/changed";
const char sub_topic[]  = "devices/08b61f821470/lamp/set_config";
const char willTopic[] = "lamp/connection/08b61f821470/";
String willPayload = "0";
bool willRetain = true;
int willQos = 2;
int subscribeQos = 1;

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

    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
        // failed, retry
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

    publishState();
}

void loop() {
    Skywriter.poll();
    if( touch_timeout > 0 ) touch_timeout--;
    mqttClient.poll();
}

void gesture(unsigned char type){
    Serial.println("Got gesture ");
    Serial.print(type,DEC);
    Serial.print('\n');

    if (type == SW_FLICK_SOUTH_NORTH) {
        is_on = true;
        setLampState();
    } else if (type == SW_FLICK_NORTH_SOUTH) {
        is_on = false;
        setLampState();
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
    setLampState();
}

void onMqttMessage(int messageSize) {
    // we received a message, print out the topic and contents
    Serial.print("Received a message with topic '");
    Serial.print(mqttClient.messageTopic());
    Serial.print("', duplicate = ");
    Serial.print(mqttClient.messageDup() ? "true" : "false");
    Serial.print(", QoS = ");
    Serial.print(mqttClient.messageQoS());
    Serial.print(", retained = ");
    Serial.print(mqttClient.messageRetain() ? "true" : "false");
    Serial.print("', length ");
    Serial.print(messageSize);
    Serial.println(" bytes:");

    char json[1024];
    int i = 0;
    while (mqttClient.available()) {
        char c = (char)mqttClient.read();
        Serial.print(c);
        json[i++] = c;
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
    analogWrite(red_led, (int)(rgb[0] * 255));
    analogWrite(green_led, (int)(rgb[1] * 255));
    analogWrite(blue_led, (int)(rgb[2] * 255));
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
    sprintf(json_out, R"({"color": {"h": %f, "s": %f}, "brightness": %f, "on": %s, "client": "08b61f821470"})",
            hue, saturation, brightness, is_on ? "true" : "false");

    mqttClient.beginMessage(pub_topic);
    mqttClient.print(json_out);
    mqttClient.endMessage();
}