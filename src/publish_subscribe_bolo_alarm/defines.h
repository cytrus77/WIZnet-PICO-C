#pragma once

#include <string>

using namespace std;

/**
 * ----------------------------------------------------------------------------------------------------
 * Macros
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
#define PLL_SYS_KHZ (133 * 1000)

/* Buffer */
#define ETHERNET_BUF_MAX_SIZE (1024 * 2)

/* Socket */
#define SOCKET_MQTT 0

/* Timeout */
#define DEFAULT_TIMEOUT 1000 // 1 second

// #include "AnalogSensor.h"

// TimerOne configLed
const unsigned long CYCLES_PER_SECOND = 8000;
const unsigned int PIR_PERIOD_MS = 300;
const unsigned int ANALOG_PERIOD_MS = 3000;
const float MAX_ADC_V = 5.0;
// const AnalogSensor::SScaler ANALOG_SCALER = AnalogSensor::SScaler(0, 1024, 100, 0);

// Ethernet config
#define MAC      { 0x00, 0x08, 0xDC, 0x16, 0x34, 0x02 }
#define IP       { 192, 168, 89, 102 }
#define SUBNET   { 255, 255, 254, 0 }
#define GATEWAY  { 192, 168, 88, 1 }
#define DNS      { 8, 8, 8, 8 }

// test parameters
// #define IP       { 192, 168, 116, 188 }
// #define SUBNET   { 255, 255, 255, 0 }
// #define GATEWAY  { 192, 168, 116, 1 }

/* MQTT IP */
#define MQTT_PUBLISH_PERIOD (1000 * 1) // 1 seconds
#define MQTT_KEEP_ALIVE 1000 // 1000 milliseconds
#define MQTT_SERVER { 192, 168, 89, 100 }
const uint16_t MQTT_PORT = 1883;

// MQTT
const string MQTT_CLIENT_ID = "CT-alarm-2";
const string MQTT_USERNAME = "wiznet";
const string MQTT_PASSWORD = "0123456789";

const string deviceName = "CT01";
const string cmndSufix  = "/cmnd";

const string statusTopic      = deviceName + "/status";
const string uptimeTopic      = deviceName + "/uptime";

const string pir1Topic        = deviceName + "/move1";
const string pir2Topic        = deviceName + "/move2";
const string pir3Topic        = deviceName + "/move3";
const string pir4Topic        = deviceName + "/move4";
const string pir5Topic        = deviceName + "/move5";
const string pir6Topic        = deviceName + "/move6";

const string tapper1Topic     = deviceName + "/tapper1";
const string tapper2Topic     = deviceName + "/tapper2";
const string tapper3Topic     = deviceName + "/tapper3";
const string tapper4Topic     = deviceName + "/tapper4";
const string tapper5Topic     = deviceName + "/tapper5";
const string tapper6Topic     = deviceName + "/tapper6";

const int PWM1Pin = 3;
const int PWM2Pin = 22;
const int PWM3Pin = 21;
const int PWM4Pin = 11;
const int PWM5Pin = 13;
const int PWM6Pin = 14;

const int Port1PirLPin = 15;   // [x][][][][][] PIN 3 Left
const int Port1PirRPin = 12;   // [x][][][][][] PIN 4 Right
const int Port2PirLPin = 10;   // [][x][][][][] PIN 3 Left
const int Port2PirRPin = 9;    // [][x][][][][] PIN 4 Right
const int Port3PirLPin = 8;    // [][][x][][][] PIN 3 Left
const int Port3PirRPin = 7;    // [][][x][][][] PIN 4 Right
const int Port4PirLPin = 26;   // [][][][x][][] PIN 3 Left
const int Port4PirRPin = 6;    // [][][][x][][] PIN 4 Right
const int Port5PirLPin = 27;   // [][][][][x][] PIN 3 Left
const int Port5PirRPin = 5;    // [][][][][x][] PIN 4 Right
const int Port6PirLPin = 28;   // [][][][][][x] PIN 3 Left
const int Port6PirRPin = 4;    // [][][][][][x] PIN 4 Right


typedef struct PIR
{
    bool initDone;
    bool measuredValue;
    bool lastReportedValue;
    const string& topicStat;
    const uint8_t pinNo;

    PIR(const string& _topicStat, const uint8_t _pinNo)
     : initDone(false) 
     , measuredValue(false)
     , lastReportedValue(false)
     , topicStat(_topicStat)
     , pinNo(_pinNo)
    {}
} PIR;

PIR pir1 = PIR(pir1Topic, Port1PirRPin);
PIR pir2 = PIR(pir2Topic, Port2PirRPin);
PIR pir3 = PIR(pir3Topic, Port3PirRPin);
PIR pir4 = PIR(pir4Topic, Port4PirRPin);
PIR pir5 = PIR(pir5Topic, Port5PirRPin);
PIR pir6 = PIR(pir6Topic, Port6PirRPin);
PIR tapper1 = PIR(tapper1Topic, Port1PirLPin);
PIR tapper2 = PIR(tapper2Topic, Port2PirLPin);
PIR tapper3 = PIR(tapper3Topic, Port3PirLPin);
PIR tapper4 = PIR(tapper4Topic, Port4PirLPin);
PIR tapper5 = PIR(tapper5Topic, Port5PirLPin);
PIR tapper6 = PIR(tapper6Topic, Port6PirLPin);
