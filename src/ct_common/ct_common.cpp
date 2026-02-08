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
#include "dhcp.h"
#include "dns.h"

#include "defines.h"
#include "all-sense-config.h"

/* pico include */
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/watchdog.h"
/* pico include end */

// Ethernet
extern "C" void wizchip_spi_initialize(void);
extern "C" void wizchip_cris_initialize(void);
extern "C" void wizchip_reset(void);
extern "C" void wizchip_initialize(void);
extern "C" void wizchip_check(void);
extern "C" void network_initialize(wiz_NetInfo net_info);
extern "C" void print_network_information(wiz_NetInfo net_info);
extern "C" void wizchip_1ms_timer_initialize(void (*callback)(void));
extern "C" void wizchip_delay_ms(uint32_t ms);
extern "C" void DNS_init(uint8_t s, uint8_t * buf);
extern "C" uint8_t DHCP_run(void);

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
        .dhcp = NETINFO_DHCP                // DHCP enable/disable
        // .dhcp = NETINFO_STATIC              // DHCP enable/disable
};

static uint8_t g_ethernet_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
}; // common buffer

/* DHCP */
static uint8_t g_dhcp_get_ip_flag = 0;

static void wizchip_dhcp_init(void);
static void wizchip_dhcp_assign(void);
static void wizchip_dhcp_conflict(void);

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
// Last Will Testament for MQTT
static MQTTPacket_willOptions g_mqtt_will_options = MQTTPacket_willOptions_initializer;

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
void inputInit(uint8_t pinNo, gpio_irq_callback_t callback);
void gpio_callback(uint gpio, uint32_t events);
void processInput(Input& input, bool state);

void log(const string& domain, const string& log) { std::cout << domain << ": " << log << std::endl; }
void log(const string& domain, const uint32_t value) { std::cout << domain << ": " << value << std::endl; }

/* Utils */
std::vector<unsigned char> getUniqueMAC();
void reinitMacAndTopics();

/**
 * ----------------------------------------------------------------------------------------------------
 * Main
 * ----------------------------------------------------------------------------------------------------
 */
int main()
{
    set_clock_khz();
    stdio_init_all();

    watchdog_enable(10000, false);
    reinitMacAndTopics();

    inputInit(in1a.pinNo, &gpio_callback);
    inputInit(in1b.pinNo, &gpio_callback);
    inputInit(in2a.pinNo, &gpio_callback);
    inputInit(in2b.pinNo, &gpio_callback);
    inputInit(in3a.pinNo, &gpio_callback);
    inputInit(in3b.pinNo, &gpio_callback);
    inputInit(in4a.pinNo, &gpio_callback);
    inputInit(in4b.pinNo, &gpio_callback);
    inputInit(in5a.pinNo, &gpio_callback);
    inputInit(in5b.pinNo, &gpio_callback);
    inputInit(in6a.pinNo, &gpio_callback);
    inputInit(in6b.pinNo, &gpio_callback);
    inputInit(in7a.pinNo, &gpio_callback);
    inputInit(in7b.pinNo, &gpio_callback);
    inputInit(in8a.pinNo, &gpio_callback);
    inputInit(in8b.pinNo, &gpio_callback);
    inputInit(in9a.pinNo, &gpio_callback);
    inputInit(in9b.pinNo, &gpio_callback);

    networkConfig();

    /* Initialize */
    int32_t retval = 0;
    uint32_t start_ms = 0;
    uint32_t end_ms = 0;
    bool initDone = false;
    uint8_t dhcp_retry = 0;

    start_ms = millis();

    /* Infinite loop */
    while (1)
    {
        if (g_mqtt_client.isconnected)
        {
            watchdog_update();

            if (!initDone)
            {
                auto initRet = sendMqtt(willTopic, willMessageOn);
                initDone = initRet;
            }

            bool result = true;
            if ((retval = MQTTYield(&g_mqtt_client, g_mqtt_packet_connect_data.keepAliveInterval)) < 0)
            {
                log("MQTT", "Yield error: ");// + retval);
                result = false;
            }

            end_ms = millis();

            if (end_ms > start_ms + MQTT_PUBLISH_PERIOD)
            {
                result = sendMqtt(uptimeTopic, millis()/1000) && result;
                start_ms = millis();
            }

            if (!result)
            {
                g_mqtt_client.isconnected = 0;
            }
        }
        else
        {
            /* Assigned IP through DHCP */
            if (g_net_info.dhcp == NETINFO_DHCP)
            {
                retval = DHCP_run();

                if (retval == DHCP_IP_LEASED)
                {
                    if (g_dhcp_get_ip_flag == 0)
                    {
                        printf(" DHCP success\n");
                        g_dhcp_get_ip_flag = 1;
                    }
                }
                else if (retval == DHCP_FAILED)
                {
                    g_dhcp_get_ip_flag = 0;
                    dhcp_retry++;

                    if (dhcp_retry <= DHCP_RETRY_COUNT)
                    {
                        printf(" DHCP timeout occurred and retry %d\n", dhcp_retry);
                    }
                }

                if (dhcp_retry > DHCP_RETRY_COUNT)
                {
                    printf(" DHCP failed\n");
                    DHCP_stop();
                }

                wizchip_delay_ms(1000); // wait for 1 second
            }

            mqttConnect();
            initDone = false;
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

void inputInit(uint8_t pinNo, gpio_irq_callback_t callback)
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

    if (gpio == in1a.pinNo) processInput(in1a, state);
    else if (gpio == in1b.pinNo) processInput(in1b, state);
    else if (gpio == in2a.pinNo) processInput(in2a, state);
    else if (gpio == in2b.pinNo) processInput(in2b, state);
    else if (gpio == in3a.pinNo) processInput(in3a, state);
    else if (gpio == in3b.pinNo) processInput(in3b, state);
    else if (gpio == in4a.pinNo) processInput(in4a, state);
    else if (gpio == in4b.pinNo) processInput(in4b, state);
    else if (gpio == in5a.pinNo) processInput(in5a, state);
    else if (gpio == in5b.pinNo) processInput(in5b, state);
    else if (gpio == in6a.pinNo) processInput(in6a, state);
    else if (gpio == in6b.pinNo) processInput(in6b, state);
    else if (gpio == in7a.pinNo) processInput(in7a, state);
    else if (gpio == in7b.pinNo) processInput(in7b, state);
    else if (gpio == in8a.pinNo) processInput(in8a, state);
    else if (gpio == in8b.pinNo) processInput(in8b, state);
    else if (gpio == in9a.pinNo) processInput(in9a, state);
    else if (gpio == in9b.pinNo) processInput(in9b, state);
}

void processInput(Input& input, bool state)
{
    input.measuredValue = state;
    if (input.measuredValue != input.lastReportedValue || !input.initDone)
    {
        const string mqttState = state ? "HIGH" : "LOW";
        log("input", input.topicStat);
        log("input", state);
        auto result = sendMqtt(input.topicStat, mqttState);
        if (result)
        {
            input.lastReportedValue = input.measuredValue;
            input.initDone = true;
        }
    }
}

///////////===================================================================

void networkConfig()
{
    wizchip_spi_initialize();
    wizchip_cris_initialize();

    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    wizchip_1ms_timer_initialize(repeating_timer_callback);

    if (g_net_info.dhcp == NETINFO_DHCP) // DHCP
    {
        wizchip_dhcp_init();
    }
    else // static
    {
        network_initialize(g_net_info);

        /* Get network information */
        print_network_information(g_net_info);
    }

    DNS_init(SOCKET_DNS, g_ethernet_buf);

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
    g_mqtt_packet_connect_data.willFlag = 1;
    g_mqtt_packet_connect_data.keepAliveInterval = MQTT_KEEP_ALIVE;
    g_mqtt_packet_connect_data.clientID.cstring = const_cast<char*>(MQTT_CLIENT_ID.c_str());
    g_mqtt_packet_connect_data.username.cstring = const_cast<char*>(MQTT_USERNAME.c_str());
    g_mqtt_packet_connect_data.password.cstring = const_cast<char*>(MQTT_PASSWORD.c_str());
    g_mqtt_packet_connect_data.will = g_mqtt_will_options;
    g_mqtt_will_options.topicName.cstring = const_cast<char*>(willTopic.c_str());
    g_mqtt_will_options.message.cstring = const_cast<char*>(willMessageOff.c_str());
    g_mqtt_will_options.retained = 1;
    
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

/* DHCP */
static void wizchip_dhcp_init(void)
{
    printf(" DHCP client running\n");

    DHCP_init(SOCKET_DHCP, g_ethernet_buf);

    reg_dhcp_cbfunc(wizchip_dhcp_assign, wizchip_dhcp_assign, wizchip_dhcp_conflict);
}

static void wizchip_dhcp_assign(void)
{
    getIPfromDHCP(g_net_info.ip);
    getGWfromDHCP(g_net_info.gw);
    getSNfromDHCP(g_net_info.sn);
    getDNSfromDHCP(g_net_info.dns);

    g_net_info.dhcp = NETINFO_DHCP;

    /* Network initialize */
    network_initialize(g_net_info); // apply from DHCP

    print_network_information(g_net_info);
    printf(" DHCP leased time : %ld seconds\n", getDHCPLeasetime());
}

static void wizchip_dhcp_conflict(void)
{
    printf(" Conflict IP from DHCP\n");

    // halt or reset or any...
    while (1)
        ; // this example is halt.
}

std::vector<unsigned char> getUniqueMAC()
{
    const size_t id_size = 8;
    size_t macByteCounter = 0;
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    std::vector<unsigned char> mac(6);
    mac.at(macByteCounter++) = 0x2C;
    mac.at(macByteCounter++) = 0x08;
    printf("Unique ID: ");
    for (int len = id_size - 4; len < id_size; len++)
    {
        mac.at(macByteCounter++) = id.id[len];
        printf("%02X", id.id[len]);
    }
    printf("\n");

    printf("MAC Unquie: ");
    for (auto& macBye : mac)
    {
        printf("%02X", macBye);
    }
        printf("\n");

    return mac;
}

void reinitMacAndTopics()
{
    macAddr = getUniqueMAC();
    MQTT_CLIENT_ID = "CT-pico-sens-" + charToHexString(macAddr[3]) + charToHexString(macAddr[4]) + charToHexString(macAddr[5]);
    deviceName = "CTpicoSens_" + charToHexString(macAddr[3]) + charToHexString(macAddr[4]) + charToHexString(macAddr[5]);

    printf("MAC: ");
    for (auto& macBye : macAddr)
    {
        printf("%02X", macBye);
    }
    printf("\n");
    printf(MQTT_CLIENT_ID.c_str());
    printf("\n");
    printf(deviceName.c_str());
    printf("\n");

    g_net_info =
    {
        .mac = { macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5] },             // MAC address
        .ip  = IP,                          // IP address
        .sn  = SUBNET,                      // Subnet Mask
        .gw  = GATEWAY,                     // Gateway
        .dns = DNS,                         // DNS server
        .dhcp = NETINFO_DHCP                // DHCP enable/disable
    };

    statusTopic      = deviceName + "/status";
    uptimeTopic      = deviceName + "/uptime";
    willTopic        = deviceName + "/LWT";

    In1A_Topic       = deviceName + "/in1a";
    In1B_Topic       = deviceName + "/in1b";
    In2A_Topic       = deviceName + "/in2a";
    In2B_Topic       = deviceName + "/in2b";
    In3A_Topic       = deviceName + "/in3a";
    In3B_Topic       = deviceName + "/in3b";
    In4A_Topic       = deviceName + "/in4a";
    In4B_Topic       = deviceName + "/in4b";
    In5A_Topic       = deviceName + "/in5a";
    In5B_Topic       = deviceName + "/in5b";
    In6A_Topic       = deviceName + "/in6a";
    In6B_Topic       = deviceName + "/in6b";
    In7A_Topic       = deviceName + "/in7a";
    In7B_Topic       = deviceName + "/in7b";
    In8A_Topic       = deviceName + "/in8a";
    In8B_Topic       = deviceName + "/in8b";
    In9A_Topic       = deviceName + "/in9a";
    In9B_Topic       = deviceName + "/in9b";
}