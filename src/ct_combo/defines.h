#pragma once

#include <string>
#include <vector>

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
#define MAX_HTTP_PARAM_SIZE 64
#define HTTP_SOCKET_MAX_NUM 4

/* Socket */
#define SOCKET_MQTT 0
#define SOCKET_DHCP 0
#define SOCKET_DNS 1

/* Retry count */
#define DHCP_RETRY_COUNT 5
#define DNS_RETRY_COUNT 5

/* Timeout */
#define DEFAULT_TIMEOUT 1000 // 1 second
#define NETWORK_RECOVERY_THRESHOLD 100

// TimerOne configLed
const unsigned long CYCLES_PER_SECOND = 8000;
const unsigned int PIR_PERIOD_MS = 300;
const unsigned int ANALOG_PERIOD_MS = 3000;
const float MAX_ADC_V = 5.0;

std::string charToHexString(char input) {
    const char hexDigits[] = "0123456789ABCDEF";
    std::string result;
    result += hexDigits[(input >> 4) & 0x0F];  // Extract the high nibble
    result += hexDigits[input & 0x0F];         // Extract the low nibble
    return result;
}

// GENERIC CONF
#define IP       { 192, 168, 38, 111 }
#define SUBNET   { 255, 255, 255, 0 }
#define GATEWAY  { 192, 168, 38, 1 }
#define DNS      { 8, 8, 8, 8 }
#define MAC      { 0x2C, 0x08, 0x01, 0x48, 0x15, 0x30 }

// MQTT config
// Seraf - Kobaltowa
// #define MQTT_SERVER { 192, 168, 13, 40 }
// WLKP
// #define MQTT_SERVER { 192, 168, 116, 135 }
// MARCIN - RYCERSKA
// #define MQTT_SERVER { 192, 168, 101, 10 }
// KLECZKO
#define MQTT_SERVER { 192, 168, 38, 11 }
// KMINKOWA
// #define MQTT_SERVER { 192, 168, 18, 108 }


//======================================================================================

/* MQTT IP */
#define MQTT_PUBLISH_PERIOD (1000 * 1) // 1 seconds
#define MQTT_KEEP_ALIVE 10 // milliseconds
const uint16_t MQTT_PORT = 1883;

// MQTT
std::vector<unsigned char> macAddr = MAC;
string MQTT_CLIENT_ID = "CT-pico-combo-" + charToHexString(macAddr[3]) + charToHexString(macAddr[4]) + charToHexString(macAddr[5]);
string deviceName = "CTpicoCombo_" + charToHexString(macAddr[3]) + charToHexString(macAddr[4]) + charToHexString(macAddr[5]);
const string MQTT_USERNAME = "wiznet";
const string MQTT_PASSWORD = "0123456789";

const string cmndSufix  = "/cmnd";

string statusTopic      = deviceName + "/status";
string uptimeTopic      = deviceName + "/uptime";
string willTopic        = deviceName + "/LWT";
const string willMessageOff   = "offline";
const string willMessageOn    = "online";

string pir1Topic        = deviceName + "/move1";
string pir2Topic        = deviceName + "/move2";
string pir3Topic        = deviceName + "/move3";
string pir4Topic        = deviceName + "/move4";
string pir5Topic        = deviceName + "/move5";
string pir6Topic        = deviceName + "/move6";
string pir7Topic        = deviceName + "/move7";
string pir8Topic        = deviceName + "/move8";
string pir9Topic        = deviceName + "/move9";

string light1Topic      = deviceName + "/light1";
string light2Topic      = deviceName + "/light2";
string light3Topic      = deviceName + "/light3";

string pwm1TopicStat    = deviceName + "/pwm1";
string pwm2TopicStat    = deviceName + "/pwm2";
string pwm3TopicStat    = deviceName + "/pwm3";
string pwm4TopicStat    = deviceName + "/pwm4";
string pwm5TopicStat    = deviceName + "/pwm5";
string pwm6TopicStat    = deviceName + "/pwm6";
string pwm1TopicCmnd    = pwm1TopicStat + cmndSufix;
string pwm2TopicCmnd    = pwm2TopicStat + cmndSufix;
string pwm3TopicCmnd    = pwm3TopicStat + cmndSufix;
string pwm4TopicCmnd    = pwm4TopicStat + cmndSufix;
string pwm5TopicCmnd    = pwm5TopicStat + cmndSufix;
string pwm6TopicCmnd    = pwm6TopicStat + cmndSufix;

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
const int Port4PirRPin = 6;    // [][][][x][][] PIN 4 Right
const int Port5PirRPin = 5;    // [][][][][x][] PIN 4 Right
const int Port6PirRPin = 4;    // [][][][][][x] PIN 4 Right

const int Port4ADCLPin = 26;   // [][][][x][][] PIN 3 Left
const int Port5ADCLPin = 27;   // [][][][][x][] PIN 3 Left
const int Port6ADCLPin = 28;   // [][][][][][x] PIN 3 Left

const int Port4ADCinput = 0;
const int Port5ADCinput = 1;
const int Port6ADCinput = 2;

typedef struct ADCchannel
{
    bool initDone;
    uint16_t measuredValue;
    uint16_t lastReportedValue;
    const string& topicStat;
    const uint8_t pinNo;
    const uint8_t channelNo;

    ADCchannel(const string& _topicStat, const uint8_t _pinNo, const uint8_t _channelNo)
     : initDone(false)
     , measuredValue(0)
     , lastReportedValue(0)
     , topicStat(_topicStat)
     , pinNo(_pinNo)
     , channelNo(_channelNo)
    {}
} ADCchannel;

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

typedef struct PWMchannel
{
    uint16_t targetPWM;
    uint16_t currentPWM;
    const string& topicStat;
    const string& topicCmnd;
    const uint8_t pinNo;

    PWMchannel(const string& _topicStat, const string& _topicCmnd, const uint8_t _pinNo)
     : targetPWM(0)
     , currentPWM(0)
     , topicStat(_topicStat)
     , topicCmnd(_topicCmnd)
     , pinNo(_pinNo)
    {}
} PWMchannel;

PWMchannel pwm1 = PWMchannel(pwm1TopicStat, pwm1TopicCmnd, PWM1Pin);
PWMchannel pwm2 = PWMchannel(pwm2TopicStat, pwm2TopicCmnd, PWM2Pin);
PWMchannel pwm3 = PWMchannel(pwm3TopicStat, pwm3TopicCmnd, PWM3Pin);
PWMchannel pwm4 = PWMchannel(pwm4TopicStat, pwm4TopicCmnd, PWM4Pin);
PWMchannel pwm5 = PWMchannel(pwm5TopicStat, pwm5TopicCmnd, PWM5Pin);
PWMchannel pwm6 = PWMchannel(pwm6TopicStat, pwm6TopicCmnd, PWM6Pin);

ADCchannel adc1 = ADCchannel(light1Topic, Port4ADCLPin, Port4ADCinput);
ADCchannel adc2 = ADCchannel(light2Topic, Port5ADCLPin, Port5ADCinput);
ADCchannel adc3 = ADCchannel(light3Topic, Port6ADCLPin, Port6ADCinput);

PIR pir1 = PIR(pir1Topic, Port1PirLPin);
PIR pir2 = PIR(pir2Topic, Port1PirRPin);
PIR pir3 = PIR(pir3Topic, Port2PirLPin);
PIR pir4 = PIR(pir4Topic, Port2PirRPin);
PIR pir5 = PIR(pir5Topic, Port3PirLPin);
PIR pir6 = PIR(pir6Topic, Port3PirRPin);
PIR pir7 = PIR(pir7Topic, Port4PirRPin);
PIR pir8 = PIR(pir8Topic, Port5PirRPin);
PIR pir9 = PIR(pir9Topic, Port6PirRPin);

