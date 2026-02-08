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

/* pico include */
#include "pico/stdlib.h"
#include "pico/time.h"
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
bool sendMqtt(const string& topic, const string& data);
bool sendMqtt(const string& topic, const int value);

void configPWM(const uint8_t pinNo);
void on_pwm_wrap();

/* GPIO */
void pirInit(uint8_t pinNo, gpio_irq_callback_t callback);
void rpmInit(uint8_t pinNo, gpio_irq_callback_t callback);
void gpio_callback(uint gpio, uint32_t events);
void rpm_callback(uint gpio, uint32_t events);
uint16_t readADC(ADCchannel& channel);
void processADC(ADCchannel& channel);
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

    watchdog_enable(10000, false);

    // pirInit(pir1.pinNo, &gpio_callback);
    // pirInit(pir2.pinNo, &gpio_callback);
    // pirInit(pir3.pinNo, &gpio_callback);
    // pirInit(pir4.pinNo, &gpio_callback);
    pirInit(pir5.pinNo, &gpio_callback);
    pirInit(pir6.pinNo, &gpio_callback);
    pirInit(pir7.pinNo, &gpio_callback);
    pirInit(pir8.pinNo, &gpio_callback);
    pirInit(pir9.pinNo, &gpio_callback);

    pirInit(rpm1.pinNo, &rpm_callback);
    pirInit(rpm2.pinNo, &rpm_callback);

    adc_init();
    adc_gpio_init(Port4ADCLPin);
    adc_gpio_init(Port5ADCLPin);
    adc_gpio_init(Port6ADCLPin);

    configPWM(pwm1.pinNo);
    configPWM(pwm2.pinNo);
    configPWM(pwm3.pinNo);
    configPWM(pwm4.pinNo);
    configPWM(pwm5.pinNo);
    configPWM(pwm6.pinNo);

    networkConfig();

    /* Initialize */
    int32_t retval = 0;
    uint32_t start_ms = 0;
    uint32_t end_ms = 0;
    uint32_t rpm_start_ms = 0;
    uint32_t rpm_end_ms = 0;
    bool initDone = false;
    uint8_t dhcp_retry = 0;

    start_ms = millis();
    rpm_start_ms = millis();

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
            rpm_end_ms = millis();

            if (end_ms > start_ms + MQTT_PUBLISH_PERIOD)
            {
                result = sendMqtt(uptimeTopic, millis()/1000) && result;
                start_ms = millis();

                processADC(adc1);
                processADC(adc2);
                processADC(adc3);
            }

            if (rpm_end_ms > rpm_start_ms + RPM_PUBLISH_PERIOD)
            {
                sendMqtt(rpm1.topicStat, rpm1.counter * 4/(RPM_PUBLISH_PERIOD/1000));
                sendMqtt(rpm2.topicStat, rpm2.counter * 4/(RPM_PUBLISH_PERIOD/1000));
                rpm1.counter = rpm2.counter = 0;
                rpm_start_ms = millis();
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

void pirInit(uint8_t pinNo, gpio_irq_callback_t callback)
{
    gpio_init(pinNo);
    gpio_set_dir(pinNo, GPIO_IN);
    gpio_pull_up(pinNo);
    gpio_set_irq_enabled_with_callback(pinNo, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, callback);
}

void rpmInit(uint8_t pinNo, gpio_irq_callback_t callback)
{
    gpio_init(pinNo);
    gpio_set_dir(pinNo, GPIO_IN);
    gpio_pull_up(pinNo);
    gpio_set_irq_enabled_with_callback(pinNo, GPIO_IRQ_EDGE_FALL, true, callback);
}

void gpio_callback(uint gpio, uint32_t events)
{
    bool state = events & EDGE_RISE ? true : false;
    state = events & EDGE_FALL ? false : true;

    // if (gpio == pir1.pinNo) processPIR(pir1, state);
    // else if (gpio == pir2.pinNo) processPIR(pir2, state);
    // else if (gpio == pir3.pinNo) processPIR(pir3, state);
    // else if (gpio == pir4.pinNo) processPIR(pir4, state);
    if (gpio == pir5.pinNo) processPIR(pir5, state);
    else if (gpio == pir6.pinNo) processPIR(pir6, state);
    else if (gpio == pir7.pinNo) processPIR(pir7, state);
    else if (gpio == pir8.pinNo) processPIR(pir8, state);
    else if (gpio == pir9.pinNo) processPIR(pir9, state);
}

void rpm_callback(uint gpio, uint32_t events)
{
    if (gpio == rpm1.pinNo) rpm1.counter++;
    else if (gpio == rpm2.pinNo) rpm2.counter++;
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

/* MQTT */
void setMqttPwmTarget(PWMchannel& channel, uint8_t targetPwm)
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
    auto value = getValueFromMsg(msg_data) * 2.55;
    log("MQTT", value);
    setMqttPwmTarget(channel, value);
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
        if (channel.targetPWM > channel.currentPWM)
            channel.currentPWM++;
        else if (channel.targetPWM < channel.currentPWM)
            channel.currentPWM--;
    }

    return channel.currentPWM * channel.currentPWM;
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
    printf(" Conflict IP from DHCP\n");

    // halt or reset or any...
    while (1)
        ; // this example is halt.
}