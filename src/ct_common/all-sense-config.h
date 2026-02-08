#pragma once

#include "defines.h"
#include <string>

using namespace std;

/* MQTT topics */
const string cmndSufix  = "/cmnd";

string statusTopic      = deviceName + "/status";
string uptimeTopic      = deviceName + "/uptime";
string willTopic        = deviceName + "/LWT";
const string willMessageOff   = "offline";
const string willMessageOn    = "online";

string In1A_Topic       = deviceName + "/in1a";
string In1B_Topic       = deviceName + "/in1b";
string In2A_Topic       = deviceName + "/in2a";
string In2B_Topic       = deviceName + "/in2b";
string In3A_Topic       = deviceName + "/in3a";
string In3B_Topic       = deviceName + "/in3b";
string In4A_Topic       = deviceName + "/in4a";
string In4B_Topic       = deviceName + "/in4b";
string In5A_Topic       = deviceName + "/in5a";
string In5B_Topic       = deviceName + "/in5b";
string In6A_Topic       = deviceName + "/in6a";
string In6B_Topic       = deviceName + "/in6b";
string In7A_Topic       = deviceName + "/in7a";
string In7B_Topic       = deviceName + "/in7b";
string In8A_Topic       = deviceName + "/in8a";
string In8B_Topic       = deviceName + "/in8b";
string In9A_Topic       = deviceName + "/in9a";
string In9B_Topic       = deviceName + "/in9b";


/* HW configs */
const int In1aPin = 2;
const int In1bPin = 3;
const int In2aPin = 4;
const int In2bPin = 5;
const int In3aPin = 6;
const int In3bPin = 7;
const int In4aPin = 8;
const int In4bPin = 9;
const int In5aPin = 10;
const int In5bPin = 11;
const int In6aPin = 12;
const int In6bPin = 13;
const int In7aPin = 28;
const int In7bPin = 14;
const int In8aPin = 27;
const int In8bPin = 15;
const int In9aPin = 26;
const int In9bPin = 22;


typedef struct Input
{
    bool initDone;
    bool measuredValue;
    bool lastReportedValue;
    const string& topicStat;
    const uint8_t pinNo;

    Input(const string& _topicStat, const uint8_t _pinNo)
     : initDone(false) 
     , measuredValue(false)
     , lastReportedValue(false)
     , topicStat(_topicStat)
     , pinNo(_pinNo)
    {}
} Input;


Input in1a = Input(In1A_Topic, In1aPin);
Input in1b = Input(In1B_Topic, In1bPin);
Input in2a = Input(In2A_Topic, In2aPin);
Input in2b = Input(In2B_Topic, In2bPin);
Input in3a = Input(In3A_Topic, In3aPin);
Input in3b = Input(In3B_Topic, In3bPin);
Input in4a = Input(In4A_Topic, In4aPin);
Input in4b = Input(In4B_Topic, In4bPin);
Input in5a = Input(In5A_Topic, In5aPin);
Input in5b = Input(In5B_Topic, In5bPin);
Input in6a = Input(In6A_Topic, In6aPin);
Input in6b = Input(In6B_Topic, In6bPin);
Input in7a = Input(In7A_Topic, In7aPin);
Input in7b = Input(In7B_Topic, In7bPin);
Input in8a = Input(In8A_Topic, In8aPin);
Input in8b = Input(In8B_Topic, In8bPin);
Input in9a = Input(In9A_Topic, In9aPin);
Input in9b = Input(In9B_Topic, In9bPin);
