/**
 * ----------------------------------------------------------------------------------------------------
 * Includes
 * ----------------------------------------------------------------------------------------------------
 */
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream> 

#include "defines.h"
#include "all-sense-config.h"

/* pico include */
#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
/* pico include end */

/* GPIO */
void inputInit(uint8_t pinNo, gpio_irq_callback_t callback);
void gpio_callback(uint gpio, uint32_t events);
void processInput(Input& input, bool state);
void reinitMacAndTopics();


int setup()
{
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
}


int loop_connected()
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