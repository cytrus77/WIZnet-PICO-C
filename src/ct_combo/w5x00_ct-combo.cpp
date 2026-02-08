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
#include <ctime>

#include "port_common.h"

#include "wizchip_conf.h"
// #include "w5x00_spi.h" // covered by extern "C
// #include "timer.h"     // covered by extern "C
#include "mqtt_interface.h"
#include "MQTTClient.h"
#include "dhcp.h"
#include "dns.h"

#include "defines.h"

/* pico include */
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/unique_id.h"
#include "hardware/adc.h"
#include "hardware/flash.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
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

// HTTP server
extern "C" void httpServer_init(uint8_t * tx_buf, uint8_t * rx_buf, uint8_t cnt, uint8_t * socklist, uint8_t * param_buf);
extern "C" void reg_httpServer_cbfunc(void(*mcu_reset)(void), void(*wdt_reset)(void));
extern "C" void httpServer_run(uint8_t seqnum);

extern "C" void reg_httpServer_webContent(const char * content_name, const char * content);
extern "C" uint8_t find_userReg_webContent(uint8_t * content_name, uint16_t * content_num, uint32_t * file_len);
extern "C" uint16_t read_userReg_webContent(uint16_t content_num, uint8_t * buf, uint32_t offset, uint16_t size);
extern "C" uint8_t display_reg_webContent_list(void);
extern "C" uint8_t * get_http_param_value(char* uri, char* param_name);

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

/* HTTP */
static uint8_t g_http_param_buf[MAX_HTTP_PARAM_SIZE] = {
    0,
};
static uint8_t g_http_send_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_http_recv_buf[ETHERNET_BUF_MAX_SIZE] = {
    0,
};
static uint8_t g_http_socket_num_list[HTTP_SOCKET_MAX_NUM] = {0, 1, 2, 3};
static char index_page[512];

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
static uint32_t networkRecoveryCounter = 0;
/* Timer  */
static volatile uint32_t g_msec_cnt = 0;

/* Flash */
 // Flash layout constants
#define FLASH_TARGET_OFFSET (2 * 1024 * 1024 - FLASH_SECTOR_SIZE)
// ^ Last sector of a 2MB flash (default Pico)

// Magic number to verify valid data
#define FLASH_CONFIG_MAGIC 0xDEADBEEF
// Data to store
struct ConfigData {
    uint32_t magic;
    uint8_t mqtt_broker_ip[4];
};

/* Initialize */
int32_t retval = 0;
uint32_t start_ms = 0;
uint32_t end_ms = 0;
bool initDone = false;
uint8_t dhcp_retry = 0;

/**
 * ----------------------------------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------------------------------
 */
/* Clock */
static void set_clock_khz(void);
void core1_entry(void);

/* HTTP */
static void updateHttpIndexPage(void);

/* MQTT */
static void message_arrived_pwm1(MessageData *msg_data);
static void message_arrived_pwm2(MessageData *msg_data);
static void message_arrived_pwm3(MessageData *msg_data);
static void message_arrived_pwm4(MessageData *msg_data);
static void message_arrived_pwm5(MessageData *msg_data);
static void message_arrived_pwm6(MessageData *msg_data);

/* Timer  */
static void repeating_timer_callback(void);
static time_t millis(void);

void networkConfig();
void mqttConnect();
bool sendMqtt(const string& topic, const string& data, const bool retained = false);
bool sendMqtt(const string& topic, const int value, const bool retained = false);

void configPWM(const uint8_t pinNo);
void on_pwm_wrap();

/* GPIO */
void pirInit(uint8_t pinNo, gpio_irq_callback_t callback);
void gpio_callback(uint gpio, uint32_t events);
uint16_t readADC(ADCchannel& channel);
void processADC(ADCchannel& channel);
void processPIR(PIR& pir, bool state);

void log(const string& domain, const string& log) { std::cout << domain << ": " << log << std::endl; }
void log(const string& domain, const uint32_t value) { std::cout << domain << ": " << value << std::endl; }

/* FLASH */
void save_to_flash(const ConfigData& data);
ConfigData read_from_flash(void);

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

    log("Info", "Setup start");

    watchdog_enable(50000, false);
    sleep_ms(3000);

    // Read config from Flash
    ConfigData loaded = read_from_flash();
    if (loaded.magic == FLASH_CONFIG_MAGIC) {
        log("Flash", "Read OK");
        log("Flash", "Broker IP = " + std::to_string(loaded.mqtt_broker_ip[0]) + "." +
            std::to_string(loaded.mqtt_broker_ip[1]) + "." +
            std::to_string(loaded.mqtt_broker_ip[2]) + "." +
            std::to_string(loaded.mqtt_broker_ip[3]));
        g_mqtt_broker_ip[0] = loaded.mqtt_broker_ip[0];
        g_mqtt_broker_ip[1] = loaded.mqtt_broker_ip[1];
        g_mqtt_broker_ip[2] = loaded.mqtt_broker_ip[2];
        g_mqtt_broker_ip[3] = loaded.mqtt_broker_ip[3];
    } else {
        log("Flash", "Invalid flash data");
    }

    reinitMacAndTopics();

    pirInit(pir1.pinNo, &gpio_callback);
    pirInit(pir2.pinNo, &gpio_callback);
    pirInit(pir3.pinNo, &gpio_callback);
    pirInit(pir4.pinNo, &gpio_callback);
    pirInit(pir5.pinNo, &gpio_callback);
    pirInit(pir6.pinNo, &gpio_callback);
    pirInit(pir7.pinNo, &gpio_callback);
    pirInit(pir8.pinNo, &gpio_callback);
    pirInit(pir9.pinNo, &gpio_callback);

    adc_init();
    adc_gpio_init(Port4ADCLPin);
    adc_gpio_init(Port5ADCLPin);
    adc_gpio_init(Port6ADCLPin);
    gpio_pull_up(Port4ADCLPin);
    gpio_pull_up(Port5ADCLPin);
    gpio_pull_up(Port6ADCLPin);

    configPWM(pwm1.pinNo);
    configPWM(pwm2.pinNo);
    configPWM(pwm3.pinNo);
    configPWM(pwm4.pinNo);
    configPWM(pwm5.pinNo);
    configPWM(pwm6.pinNo);

    networkConfig();

    /* HTTP */
    httpServer_init(g_http_send_buf, g_http_recv_buf, HTTP_SOCKET_MAX_NUM, g_http_socket_num_list, g_http_param_buf);
    /* Build index page that exposes IP, subnet and gateway from g_net_info */
    updateHttpIndexPage();
    reg_httpServer_webContent("index.html", index_page);

    /* Simple web page to send a value (form sends value as GET parameter "value") */
    static const char set_page[] =
        "<!doctype html>"
        "<html><head><meta charset=\"utf-8\"><title>Set MQTT server IP Address</title></head>"
        "<body>"
        "<h1>Set the MQTT server IP address</h1>"
        "<form action=\"/set.html\" method=\"get\">"
        "  <label>IP Address: <input type=\"text\" name=\"ip\" placeholder=\"192.168.0.100\" pattern=\"^([0-9]{1,3}\\.){3}[0-9]{1,3}$\" required /></label>"
        "  <input type=\"submit\" value=\"Set\" />"
        "</form>"
        "<p>Enter IPv4 address in dotted decimal format.</p>"
        "</body></html>";

    reg_httpServer_webContent("set.html", set_page);
    /* HTTP END */

    /* Launch core 1 */
    multicore_launch_core1(core1_entry);

    start_ms = millis();
    g_mqtt_client.isconnected = 0;
    log("Info", "Setup complete");

    /* Infinite loop */
    while (1)
    {
        watchdog_update();

        if (g_mqtt_client.isconnected)
        {
            log("Info", "MQTT loop");
            if (!initDone)
            {
                auto initRet = sendMqtt(willTopic, willMessageOn, true);
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

                processADC(adc1);
                processADC(adc2);
                processADC(adc3);
            }

            if (!result)
            {
                g_mqtt_client.isconnected = 0;
            }
        }

        if (retval != DHCP_IP_LEASED)
        {
            log("Info", "Network reconnect loop");

            /* Assigned IP through DHCP */
            if (g_net_info.dhcp == NETINFO_DHCP)
            {
                retval = DHCP_run();

                if (retval == DHCP_IP_LEASED)
                {
                    if (g_dhcp_get_ip_flag == 0)
                    {
                        log("DHCP", "Success");
                        g_dhcp_get_ip_flag = 1;
                        g_mqtt_client.isconnected = 0; // force reconnect
                        networkRecoveryCounter = 0;
                    }
                }
                else if (retval == DHCP_FAILED)
                {
                    g_dhcp_get_ip_flag = 0;
                    dhcp_retry++;

                    if (dhcp_retry <= DHCP_RETRY_COUNT)
                    {
                        log("DHCP", "Timeout occurred and retry:");
                        log("DHCP", dhcp_retry);
                    }
                }

                if (dhcp_retry > DHCP_RETRY_COUNT)
                {
                    networkRecoveryCounter++;
                    log("DHCP", "Failed");
                    DHCP_stop();
                }

                wizchip_delay_ms(1000); // wait for 1 second
            }

            if(networkRecoveryCounter == 0)
            {
                updateHttpIndexPage();
                initDone = false;
            }
            else
            {
                networkRecoveryCounter++;
            }
        }
        else if (!g_mqtt_client.isconnected)
        {
            log("Info", "MQTT reconnect");
            mqttConnect();
            initDone = false;
        }

        if (networkRecoveryCounter > NETWORK_RECOVERY_THRESHOLD)
        {
            log("Info", "Reset recovery triggered");
            break;
        }
    }
}

void core1_entry()
{
    while (1)
    {
        if(retval != DHCP_IP_LEASED)
            continue;

        /* Run HTTP server */
        for (int i = 0; i < HTTP_SOCKET_MAX_NUM; i++)
        {
            httpServer_run(i);
            {
                char* param = (char *)g_http_param_buf;
                if (param[0] != 0)
                {
                    log("Http", "Received HTTP value:");
                    log("Http", param);
                    if (strstr(param, "ip=") != NULL) {
                        // IP parameter present — extract and process it
                        int b0=0, b1=0, b2=0, b3=0;
                        char* ip_start = strstr(param, "ip=");
                        if (ip_start) {
                            ip_start += 3; // skip "ip="
                            if (sscanf(ip_start, "%d.%d.%d.%d", &b0, &b1, &b2, &b3) == 4) {
                                g_mqtt_broker_ip[0] = static_cast<uint8_t>(b0);
                                g_mqtt_broker_ip[1] = static_cast<uint8_t>(b1);
                                g_mqtt_broker_ip[2] = static_cast<uint8_t>(b2);
                                g_mqtt_broker_ip[3] = static_cast<uint8_t>(b3);
                                log("MQTT", "Broker IP updated");
                                g_mqtt_client.isconnected = 0; // force reconnect with new IP
                                // Save new configuration to Flash
                                ConfigData config;
                                config.magic = FLASH_CONFIG_MAGIC;
                                config.mqtt_broker_ip[0] = g_mqtt_broker_ip[0];
                                config.mqtt_broker_ip[1] = g_mqtt_broker_ip[1];
                                config.mqtt_broker_ip[2] = g_mqtt_broker_ip[2];
                                config.mqtt_broker_ip[3] = g_mqtt_broker_ip[3];
                                // Save data
                                save_to_flash(config);
                                log("Flash", "Data written to flash");

                                updateHttpIndexPage();
                                sleep_ms(1000);
                                {
                                    log("Info", "Config updated. resetting...");
                                    while(1) {}
                                }
                            } else {
                                log("Http", "Invalid IP parameter");
                            }
                        }
                    } 
                    /* clear buffer so repeated parsing doesn't re-read same request */
                    memset(g_http_param_buf, 0, sizeof(g_http_param_buf));
                }
            }
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
    else if (gpio == pir7.pinNo) processPIR(pir7, state);
    else if (gpio == pir8.pinNo) processPIR(pir8, state);
    else if (gpio == pir9.pinNo) processPIR(pir9, state);
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

void processADC(ADCchannel& channel)
{
    static uint16_t MAX_ADC = 4095;
    uint32_t tempVal = readADC(channel) * 100;
    channel.measuredValue = tempVal / MAX_ADC; // value in %
    if (abs(channel.measuredValue - channel.lastReportedValue) > 5 || !channel.initDone)
    {
        log("ADC", channel.topicStat);
        log("ADC", channel.lastReportedValue);
        log("ADC", channel.measuredValue);
        auto result = sendMqtt(channel.topicStat, channel.measuredValue);
        if (result)
        {
            channel.lastReportedValue = channel.measuredValue;
            channel.initDone = true;
        }
    }
}

uint16_t readADC(ADCchannel& channel)
{
    adc_select_input(channel.channelNo);
    return adc_read();
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

    /* HTTP update*/
    updateHttpIndexPage();

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

    log("MQTT", "mqttConnect connecting");
    log("MQTT", "broker IP=" + std::to_string(g_mqtt_broker_ip[0]) + "." +
        std::to_string(g_mqtt_broker_ip[1]) + "." +
        std::to_string(g_mqtt_broker_ip[2]) + "." +
        std::to_string(g_mqtt_broker_ip[3]));

    retval = ConnectNetwork(&g_mqtt_network, g_mqtt_broker_ip, MQTT_PORT);

    if (retval != 1)
    {
        log("MQTT", "Broker connect failed");
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
    g_mqtt_will_options.message.cstring   = const_cast<char*>(willMessageOff.c_str());
    g_mqtt_will_options.retained = 1;
    
    retval = MQTTConnect(&g_mqtt_client, &g_mqtt_packet_connect_data);

    if (retval < 0)
    {
        log("MQTT", "MQTT connect failed: ");// + retval);
        return;
    }

    log("MQTT", "MQTT connected");

    sendMqtt(statusTopic, getNetInfo(g_net_info));

    /* Subscribe */
    retval = MQTTSubscribe(&g_mqtt_client, pwm1.topicCmnd.c_str(), QOS0, message_arrived_pwm1);
    retval = MQTTSubscribe(&g_mqtt_client, pwm2.topicCmnd.c_str(), QOS0, message_arrived_pwm2);
    retval = MQTTSubscribe(&g_mqtt_client, pwm3.topicCmnd.c_str(), QOS0, message_arrived_pwm3);
    retval = MQTTSubscribe(&g_mqtt_client, pwm4.topicCmnd.c_str(), QOS0, message_arrived_pwm4);
    retval = MQTTSubscribe(&g_mqtt_client, pwm5.topicCmnd.c_str(), QOS0, message_arrived_pwm5);
    retval = MQTTSubscribe(&g_mqtt_client, pwm6.topicCmnd.c_str(), QOS0, message_arrived_pwm6);

    if (retval < 0)
    {
        log("MQTT", "Subscribe failed: ");//+ retval);
        return;
    }

    log("MQTT", "Subscribed");
}

bool sendMqtt(const string& topic, const string& data, const bool retained)
{
    /* Configure publish message */
    g_mqtt_message.qos = QOS0;
    g_mqtt_message.retained = retained ? 1 : 0;
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

bool sendMqtt(const string& topic, const int value, const bool retained)
{
  char dataChar[6];
  itoa(value, dataChar, 10);
  const string dataStr(dataChar);
  return sendMqtt(topic, dataStr, retained);
}

/* MQTT */
void setMqttPwmTarget(PWMchannel& channel, uint16_t targetPwm)
{
    channel.targetPWM = targetPwm;
}

uint8_t getValueFromMsg(MessageData *msg_data)
{
    MQTTMessage *message = msg_data->message;
    string payload(static_cast<char*>(message->payload), message->payloadlen);
    return stoi(payload);
}

static void handle_pwm_message(MessageData *msg_data, PWMchannel& channel)
{
    log("MQTT", "received: " + channel.topicCmnd);
    uint16_t value = getValueFromMsg(msg_data);
    log("MQTT", value);
    setMqttPwmTarget(channel, value * 655.35);
    sendMqtt(channel.topicStat, value);
}

static void message_arrived_pwm1(MessageData *msg_data) {handle_pwm_message(msg_data, pwm1);};
static void message_arrived_pwm2(MessageData *msg_data) {handle_pwm_message(msg_data, pwm2);};
static void message_arrived_pwm3(MessageData *msg_data) {handle_pwm_message(msg_data, pwm3);};
static void message_arrived_pwm4(MessageData *msg_data) {handle_pwm_message(msg_data, pwm4);};
static void message_arrived_pwm5(MessageData *msg_data) {handle_pwm_message(msg_data, pwm5);};
static void message_arrived_pwm6(MessageData *msg_data) {handle_pwm_message(msg_data, pwm6);};

void message_arrived(MessageData *msg_data)
{
    log("MQTT", "received");
    MQTTMessage *message = msg_data->message;
    string topic(msg_data->topicName->lenstring.data, msg_data->topicName->lenstring.len);
    log("MQTT", topic);
    string payload(static_cast<char*>(message->payload), message->payloadlen);
    log("MQTT", payload);

    int value = stoi(payload);
}

void configPWM(const uint8_t pinNo)
{
    // Tell the LED pin that the PWM is in charge of its value.
    gpio_set_function(pinNo, GPIO_FUNC_PWM);
    // Figure out which slice we just connected to the LED pin
    uint slice_num = pwm_gpio_to_slice_num(pinNo);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // Get some sensible defaults for the slice configuration. By default, the
    // counter is allowed to wrap over its maximum range (0 to 2**16-1)
    pwm_config config = pwm_get_default_config();
    // Set wrap to 65535 for 16-bit resolution
    pwm_config_set_wrap(&config, 65535);
    // Set divider, reduces counter clock to sysclock/this value
    pwm_config_set_clkdiv(&config, 4.f);
    // Load the configuration into our PWM slice, and set it running.
    pwm_init(slice_num, &config, true);

    // Everything after this point happens in the PWM interrupt handler, so we
    // can twiddle our thumbs
}

uint16_t calculatePWMValue(PWMchannel& channel)
{
    if (channel.targetPWM != channel.currentPWM)
    {
        const uint16_t STEP = 40;
        if (abs(channel.targetPWM - channel.currentPWM) > STEP)
        {
            if (channel.targetPWM > channel.currentPWM)
                channel.currentPWM = channel.currentPWM + STEP;
            else if (channel.targetPWM < channel.currentPWM)
                channel.currentPWM = channel.currentPWM - STEP;;
        }
        else
        {
            channel.currentPWM = channel.targetPWM;
        }
    }

    return channel.currentPWM;
}

void setPWMValue(PWMchannel& channel)
{
    pwm_clear_irq(pwm_gpio_to_slice_num(channel.pinNo));
    uint16_t value = calculatePWMValue(channel);
    pwm_set_gpio_level(channel.pinNo, value);
}

void on_pwm_wrap() {
    static int fade = 0;
    static bool going_up = true;
    // Clear the interrupt flag that brought us here
    setPWMValue(pwm1);
    setPWMValue(pwm2);
    setPWMValue(pwm3);
    setPWMValue(pwm4);
    setPWMValue(pwm5);
    setPWMValue(pwm6);
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
    log("DHCP", "Conflict IP from DHCP");

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
    for (int len = id_size - 4; len < id_size; len++)
    {
        mac.at(macByteCounter++) = id.id[len];
    }

    return mac;
}

void reinitMacAndTopics()
{
    macAddr = getUniqueMAC();
    MQTT_CLIENT_ID = "CT-pico-combo-" + charToHexString(macAddr[3]) + charToHexString(macAddr[4]) + charToHexString(macAddr[5]);
    deviceName = "CTpicoCombo_" + charToHexString(macAddr[3]) + charToHexString(macAddr[4]) + charToHexString(macAddr[5]);

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

    pir1Topic        = deviceName + "/move1";
    pir2Topic        = deviceName + "/move2";
    pir3Topic        = deviceName + "/move3";
    pir4Topic        = deviceName + "/move4";
    pir5Topic        = deviceName + "/move5";
    pir6Topic        = deviceName + "/move6";
    pir7Topic        = deviceName + "/move7";
    pir8Topic        = deviceName + "/move8";
    pir9Topic        = deviceName + "/move9";

    light1Topic      = deviceName + "/light1";
    light2Topic      = deviceName + "/light2";
    light3Topic      = deviceName + "/light3";

    pwm1TopicStat    = deviceName + "/pwm1";
    pwm2TopicStat    = deviceName + "/pwm2";
    pwm3TopicStat    = deviceName + "/pwm3";
    pwm4TopicStat    = deviceName + "/pwm4";
    pwm5TopicStat    = deviceName + "/pwm5";
    pwm6TopicStat    = deviceName + "/pwm6";
    pwm1TopicCmnd    = pwm1TopicStat + cmndSufix;
    pwm2TopicCmnd    = pwm2TopicStat + cmndSufix;
    pwm3TopicCmnd    = pwm3TopicStat + cmndSufix;
    pwm4TopicCmnd    = pwm4TopicStat + cmndSufix;
    pwm5TopicCmnd    = pwm5TopicStat + cmndSufix;
    pwm6TopicCmnd    = pwm6TopicStat + cmndSufix;
}

void updateHttpIndexPage()
{
    snprintf(index_page, sizeof(index_page),
             "<!doctype html>"
             "<html><head><meta charset=\"utf-8\"><title>CitrusTec MQTT module</title></head>"
             "<body>"
             "<h1>CitrusTec Device Network Info</h1>"
             "<p>Device Name: %s</p>"
             "<p>MQTT Client ID: %s</p>"
             "<p>MAC Address: %02X:%02X:%02X:%02X:%02X:%02X</p>"
             "<p>IP Address: %u.%u.%u.%u</p>"
             "<p>Subnet Mask: %u.%u.%u.%u</p>"
             "<p>Gateway: %u.%u.%u.%u</p>"
             "<p>MQTT Broker IP: %u.%u.%u.%u</p>"
             "</body></html>",
             deviceName.c_str(),
             MQTT_CLIENT_ID.c_str(),
             g_net_info.mac[0], g_net_info.mac[1], g_net_info.mac[2], g_net_info.mac[3], g_net_info.mac[4], g_net_info.mac[5],
             g_net_info.ip[0], g_net_info.ip[1], g_net_info.ip[2], g_net_info.ip[3],
             g_net_info.sn[0], g_net_info.sn[1], g_net_info.sn[2], g_net_info.sn[3],
             g_net_info.gw[0], g_net_info.gw[1], g_net_info.gw[2], g_net_info.gw[3],
             g_mqtt_broker_ip[0], g_mqtt_broker_ip[1], g_mqtt_broker_ip[2], g_mqtt_broker_ip[3]);
}

// Write data to flash
void save_to_flash(const ConfigData& data) {
    uint32_t interrupts = save_and_disable_interrupts();

    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(
        FLASH_TARGET_OFFSET,
        (const uint8_t*)&data,
        sizeof(ConfigData)
    );

    restore_interrupts(interrupts);
}

// Read data from flash
ConfigData read_from_flash() {
    const ConfigData* flash_data =
        (const ConfigData*)(XIP_BASE + FLASH_TARGET_OFFSET);
    return *flash_data;
}
