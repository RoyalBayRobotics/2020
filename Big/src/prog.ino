#include <SparkFunESP8266WiFi.h>
#include "common.h"

void check(int16_t rsp) {
    Serial.print("#");
    Serial.println(rsp);
}

void setup() {
    Serial.begin(9600);
    esp8266.begin(9600, ESP8266_HARDWARE_SERIAL);

    sendCommand("+CIPCLOSE", "5");
    sendCommand("+CWMODE_CUR", "1");
    sendCommand("+CIPMUX", "1");
    sendCommand("+CIPSTA_CUR", "\"192.168.100.2\"");
    sendCommand("+CWJAP_CUR", "\"" SSID "\",\"" PASSWD "\"", WIFI_CONNECT_TIMEOUT);
    sendCommand("+CIPSTART", "0,\"UDP\",\"192.168.100.1\",8839,8838,0");
}

void loop() {
    // look for +IPD
    {
        char ipd[] = "+IPD";
        size_t i = 0;
        while(i < strlen(ipd)) {
            if(esp8266.available()) {
                byte val = esp8266.read();
                if(val == ipd[i]) i++;
                else i = 0;
            }
        }
    }

    // jump to first colon ... just gonna assume its size
    while(!(esp8266.available() && esp8266.read() == ':'));

    // read data
    XboxState xbox;
    byte *buff = (byte*) &xbox;
    for(size_t i = 0; i < sizeof(XboxState); i++) {
        while(!esp8266.available());
        buff[i] = esp8266.read();
    }

    Serial.print("#");
    Serial.println(xbox.leftX);
}
