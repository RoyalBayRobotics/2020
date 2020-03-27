#include <SparkFunESP8266WiFi.h>
#include <util/ESP8266_AT.h>

#define SSID "RBSSROBOT"
#define PASSWD "0x52425353524f424f54"

#define X_SHIFT 7
#define Y_SHIFT 6
#define A_SHIFT 5
#define B_SHIFT 4
#define LEFT_SHIFT 3
#define UP_SHIFT 2
#define RIGHT_SHIFT 1
#define DOWN_SHIFT 0

#define L1_SHIFT 3
#define R1_SHIFT 2
#define L3_SHIFT 1
#define R3_SHIFT 0

#define MASK(x, btn) (x<<btn ## _SHIFT)

#pragma pack(push, 1)
struct XboxState {
    int16_t leftX;
    int16_t leftY;
    int16_t rightX;
    int16_t rightY;
    uint8_t L2;
    uint8_t R2;
    uint8_t buttons; // MSB | X Y A B left up right down | LSB
    uint8_t analogButtons:4; // MSB | L1 R1 L3 R3 | LSB
};
#pragma pack(pop)

int16_t sendCommand(const char* cmd, const char* param, unsigned int timeout=COMMAND_RESPONSE_TIMEOUT) {
    esp8266.sendCommand(cmd, ESP8266_CMD_SETUP, param);
    return esp8266.readForResponses(RESPONSE_OK, RESPONSE_ERROR, timeout);
}

// tcpSend from ESP8266 library can't send null bytes
int16_t sendBuffer(uint8_t linkID, const uint8_t *buff, size_t size) {
    char params[8];
    sprintf(params, "%d,%d", linkID, size);

    int16_t rsp = sendCommand("+CIPSEND", params);
    if (rsp != ESP8266_RSP_FAIL) {
        Serial.write(buff, size);
        Serial.println();

        rsp = esp8266.readForResponse("SEND OK", COMMAND_RESPONSE_TIMEOUT);

        if (rsp > 0)
            return size;
    }

    return rsp;
}

// return 0 un success, -1 on timeout
int recieveXbox(XboxState *xbox, unsigned long timeout = 200) {
    unsigned long startTime = millis();

    // look for +IPD
    {
        const char ipd[] = "+IPD";
        size_t i = 0;
        while(i < strlen(ipd)) {
            if(millis() - timeout > startTime) return -1;
            if(esp8266.available()) {
                byte val = esp8266.read();
                if(val == ipd[i]) i++;
                else i = 0;
            }
        }
    }

    // jump to first colon ... just gonna assume its size
    while(!(esp8266.available() && esp8266.read() == ':')) {
        if(millis() - timeout > startTime) return -1;
    }

    // read data
    byte *buff = (byte*) xbox;
    for(size_t i = 0; i < sizeof(XboxState); i++) {
        while(!esp8266.available()) {
            if(millis() - timeout > startTime) return -1;
        }

        buff[i] = esp8266.read();
    }

    return 0;
}
