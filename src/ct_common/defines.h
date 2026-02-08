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

/* Socket */
#define SOCKET_MQTT 0
#define SOCKET_DHCP 0
#define SOCKET_DNS 1

/* Retry count */
#define DHCP_RETRY_COUNT 5
#define DNS_RETRY_COUNT 5

/* Timeout */
#define DEFAULT_TIMEOUT 1000 // 1 second

//======================================================================================

std::string charToHexString(char input) {
    const char hexDigits[] = "0123456789ABCDEF";
    std::string result;
    result += hexDigits[(input >> 4) & 0x0F];  // Extract the high nibble
    result += hexDigits[input & 0x0F];         // Extract the low nibble
    return result;
}

// GENERIC CONF
#define IP       { 192, 168, 21, 11 }
#define SUBNET   { 255, 255, 255, 0 }
#define GATEWAY  { 192, 168, 21, 1 }
#define DNS      { 8, 8, 8, 8 }
#define MAC      { 0x2C, 0x08, 0x01, 0x70, 0x61, 0x21 }

// MQTT config
// Seraf - Kobaltowa
// #define MQTT_SERVER { 192, 168, 13, 40 }
// WLKP
#define MQTT_SERVER { 192, 168, 21, 10 }

//======================================================================================

/* MQTT IP */
#define MQTT_PUBLISH_PERIOD (1000 * 1) // 1 seconds
#define MQTT_KEEP_ALIVE 10 // milliseconds
const uint16_t MQTT_PORT = 1883;

// MQTT
std::vector<unsigned char> macAddr = MAC;
string MQTT_CLIENT_ID = "CT-common-" + charToHexString(macAddr[3]) + charToHexString(macAddr[4]) + charToHexString(macAddr[5]);
string deviceName = "CTcommon_" + charToHexString(macAddr[3]) + charToHexString(macAddr[4]) + charToHexString(macAddr[5]);
const string MQTT_USERNAME = "wiznet";
const string MQTT_PASSWORD = "0123456789";
