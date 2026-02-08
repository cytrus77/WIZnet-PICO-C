/**
 * Copyright (c) 2021 WIZnet Co.,Ltd
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream> 

#include "port_common.h"

#include "wizchip_conf.h"
// #include "w5x00_spi.h" // covered by extern "C
// #include "timer.h"     // covered by extern "C
#include "mqtt_interface.h"
#include "MQTTClient.h"

#include "defines.h"

/* pico include */
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
/* pico include end */

/**
 * ----------------------------------------------------------------------------------------------------
 * Variables
 * ----------------------------------------------------------------------------------------------------
 */
/* Network */
static wiz_NetInfo g_net_info =
    {
        .mac = MAC,                         // MAC address
        .ip  = IP,                          // IP address
        .sn  = SUBNET,                      // Subnet Mask
        .gw  = GATEWAY,                     // Gateway
        .dns = DNS,                         // DNS server
        .dhcp = NETINFO_STATIC              // DHCP enable/disable
};

/* MQTT */
static uint8_t g_mqtt_send_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_mqtt_recv_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_mqtt_broker_ip[4] = MQTT_SERVER;
static Network g_mqtt_network;
static MQTTClient g_mqtt_client;
static MQTTPacket_connectData g_mqtt_packet_connect_data = MQTTPacket_connectData_initializer;
static MQTTMessage g_mqtt_message;

/* Timer  */
static volatile uint32_t g_msec_cnt = 0;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void);

/* Timer  */
static void repeating_timer_callback(void);
static time_t millis(void);

void networkConfig();
void mqttConnect();
bool sendMqtt(const string& topic, const string& data);
bool sendMqtt(const string& topic, const int value);

/* GPIO */
void pirInit(uint8_t pinNo, gpio_irq_callback_t callback);
void gpio_callback(uint gpio, uint32_t events);
void processPIR(PIR& pir, bool state);

void log(const string& domain, const string& log) { std::cout << domain << ": " << log << std::endl; }
void log(const string& domain, const uint32_t value) { std::cout << domain << ": " << value << std::endl; }

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main()
{
    set_clock_khz();
    stdio_init_all();

    // if (watchdog_caused_reboot()) {
    //     log("System", "Rebooted by Watchdog!");
    //     return 0;
    // } else {
    //     log("System", "Clean boot");
    // }
    // watchdog_enable(1000, 1);

    pirInit(pir1.pinNo, &gpio_callback);
    pirInit(pir2.pinNo, &gpio_callback);
    pirInit(pir3.pinNo, &gpio_callback);
    pirInit(pir4.pinNo, &gpio_callback);
    pirInit(pir5.pinNo, &gpio_callback);
    pirInit(pir6.pinNo, &gpio_callback);
    pirInit(tapper1.pinNo, &gpio_callback);
    pirInit(tapper2.pinNo, &gpio_callback);
    pirInit(tapper3.pinNo, &gpio_callback);
    pirInit(tapper4.pinNo, &gpio_callback);
    pirInit(tapper5.pinNo, &gpio_callback);
    pirInit(tapper6.pinNo, &gpio_callback);

    networkConfig();

    /* Initialize */
    int32_t retval = 0;
    uint32_t start_ms = 0;
    uint32_t end_ms = 0;

    start_ms = millis();

    /* Infinite loop */
    while (1)
    {
        // watchdog_update();

        if (g_mqtt_client.isconnected)
        {
            bool result = true;
            if ((retval = MQTTYield(&g_mqtt_client, g_mqtt_packet_connect_data.keepAliveInterval)) < 0)
            {
                log("MQTT", "Yield error: ");// + retval);
                result = false;
            }

            end_ms = millis();

            if (end_ms > start_ms + MQTT_PUBLISH_PERIOD)
            {
                result = sendMqtt(uptimeTopic, millis()/1000);
                start_ms = millis();
            }

            if (!result)
            {
                g_mqtt_client.isconnected = 0;
            }
        }
        else
        {
            mqttConnect();
        }
    }
}

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void)
{
    // set a system clock frequency in khz
    set_sys_clock_khz(PLL_SYS_KHZ, true);

    // configure the specified clock
    clock_configure(
        clk_peri,
        0,                                                // No glitchless mux
        CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLKSRC_PLL_SYS, // System PLL on AUX mux
        PLL_SYS_KHZ * 1000,                               // Input frequency
        PLL_SYS_KHZ * 1000                                // Output (must be same as no divider)
    );
}

/* Timer */
static void repeating_timer_callback(void)
{
    g_msec_cnt++;

    MilliTimer_Handler();
}

static time_t millis(void)
{
    return g_msec_cnt;
}

/* GPIO */
#define LEVEL_LOW  0x1
#define LEVEL_HIGH 0x2
#define EDGE_FALL  0x4
#define EDGE_RISE  0x8

void pirInit(uint8_t pinNo, gpio_irq_callback_t callback)
{
    gpio_init(pinNo);
    gpio_set_dir(pinNo, GPIO_IN);
    gpio_pull_up(pinNo);
    gpio_set_irq_enabled_with_callback(pinNo, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, callback);
}

void gpio_callback(uint gpio, uint32_t events)
{
    bool state = events & EDGE_RISE ? true : false;
    state = events & EDGE_FALL ? false : true;

    if (gpio == pir1.pinNo) processPIR(pir1, state);
    else if (gpio == pir2.pinNo) processPIR(pir2, state);
    else if (gpio == pir3.pinNo) processPIR(pir3, state);
    else if (gpio == pir4.pinNo) processPIR(pir4, state);
    else if (gpio == pir5.pinNo) processPIR(pir5, state);
    else if (gpio == pir6.pinNo) processPIR(pir6, state);
    else if (gpio == tapper1.pinNo) processPIR(tapper1, state);
    else if (gpio == tapper2.pinNo) processPIR(tapper2, state);
    else if (gpio == tapper3.pinNo) processPIR(tapper3, state);
    else if (gpio == tapper4.pinNo) processPIR(tapper4, state);
    else if (gpio == tapper5.pinNo) processPIR(tapper5, state);
    else if (gpio == tapper6.pinNo) processPIR(tapper6, state);
}

void processPIR(PIR& pir, bool state)
{
    pir.measuredValue = state;
    if (pir.measuredValue != pir.lastReportedValue || !pir.initDone)
    {
        string mqttState = state ? "HIGH" : "LOW";
        log("PIR", pir.topicStat);
        log("PIR", state);
        auto result = sendMqtt(pir.topicStat, mqttState);
        if (result)
        {
            pir.lastReportedValue = pir.measuredValue;
            pir.initDone = true;
        }
    }
}

// MQTT
extern "C" void wizchip_spi_initialize(void);
extern "C" void wizchip_cris_initialize(void);
extern "C" void wizchip_reset(void);
extern "C" void wizchip_initialize(void);
extern "C" void wizchip_check(void);
extern "C" void network_initialize(wiz_NetInfo net_info);
extern "C" void print_network_information(wiz_NetInfo net_info);
extern "C" void wizchip_1ms_timer_initialize(void (*callback)(void));

///////////===================================================================

void networkConfig()
{
    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    wizchip_1ms_timer_initialize(repeating_timer_callback);

    network_initialize(g_net_info);

    /* Get network information */
    print_network_information(g_net_info);

    NewNetwork(&g_mqtt_network, SOCKET_MQTT);
}

string getNetInfo(wiz_NetInfo net_info)
{
    stringstream stream;
    ctlnetwork(CN_GET_NETINFO, (void *)&net_info);

    if (net_info.dhcp == NETINFO_DHCP)
    {
        stream << "network config : DHCP\n";
    }
    else
    {
        stream << "network config : static\n";
    }

    stream << " MAC         : " << net_info.mac << "\n";
    // printf(" IP          : %d.%d.%d.%d\n", net_info.ip[0], net_info.ip[1], net_info.ip[2], net_info.ip[3]);
    // printf(" Subnet Mask : %d.%d.%d.%d\n", net_info.sn[0], net_info.sn[1], net_info.sn[2], net_info.sn[3]);
    // printf(" Gateway     : %d.%d.%d.%d\n", net_info.gw[0], net_info.gw[1], net_info.gw[2], net_info.gw[3]);
    // printf(" DNS         : %d.%d.%d.%d\n", net_info.dns[0], net_info.dns[1], net_info.dns[2], net_info.dns[3]);
    string str;
    stream >> str;
    return str;
}

void mqttConnect()
{
    int32_t retval = 0;

    retval = ConnectNetwork(&g_mqtt_network, g_mqtt_broker_ip, MQTT_PORT);

    if (retval != 1)
    {
        log("Network", "Network connect failed");
        return;
    }

    log("MQTT", "mqttConnect start");
    /* Initialize MQTT client */
    MQTTClientInit(&g_mqtt_client, &g_mqtt_network, DEFAULT_TIMEOUT, g_mqtt_send_buf, ETHERNET_BUF_MAX_SIZE, g_mqtt_recv_buf, ETHERNET_BUF_MAX_SIZE);

    /* Connect to the MQTT broker */
    g_mqtt_packet_connect_data.MQTTVersion = 3;
    g_mqtt_packet_connect_data.cleansession = 1;
    g_mqtt_packet_connect_data.willFlag = 0;
    g_mqtt_packet_connect_data.keepAliveInterval = MQTT_KEEP_ALIVE;
    g_mqtt_packet_connect_data.clientID.cstring = const_cast<char*>(MQTT_CLIENT_ID.c_str());
    g_mqtt_packet_connect_data.username.cstring = const_cast<char*>(MQTT_USERNAME.c_str());
    g_mqtt_packet_connect_data.password.cstring = const_cast<char*>(MQTT_PASSWORD.c_str());

    retval = MQTTConnect(&g_mqtt_client, &g_mqtt_packet_connect_data);

    if (retval < 0)
    {
        log("MQTT", "MQTT connect failed: ");// + retval);
        return;
    }

    log("MQTT", "MQTT connected");

    sendMqtt(statusTopic, getNetInfo(g_net_info));

    if (retval < 0)
    {
        log("MQTT", "Subscribe failed: ");//+ retval);
        return;
    }

    log("MQTT", "Subscribed");
}

bool sendMqtt(const string& topic, const string& data)
{
    /* Configure publish message */
    g_mqtt_message.qos = QOS0;
    g_mqtt_message.retained = 0;
    g_mqtt_message.dup = 0;
    g_mqtt_message.payload = const_cast<char*>(data.c_str());
    g_mqtt_message.payloadlen = strlen(data.c_str());

    int32_t retval = 0;
    /* Publish */
    retval = MQTTPublish(&g_mqtt_client, topic.c_str(), &g_mqtt_message);
    if (retval < 0)
    {
        log("MQTT", "Publish failed");
        return false;
    }
    return true;
}

bool sendMqtt(const string& topic, const int value)
{
  char dataChar[6];
  itoa(value, dataChar, 10);
  const string dataStr(dataChar);
  return sendMqtt(topic, dataStr);
}