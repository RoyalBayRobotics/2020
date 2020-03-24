#include <SparkFunESP8266WiFi.h>
#include <util/ESP8266_AT.h>

#define SSID "RBSSROBOT"
#define PASSWD "0x52425353524f424f54"

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

uint16_t sendCommand(const char* cmd, const char* param, unsigned int timeout=COMMAND_RESPONSE_TIMEOUT) {
    esp8266.sendCommand(cmd, ESP8266_CMD_SETUP, param);
    return esp8266.readForResponses(RESPONSE_OK, RESPONSE_ERROR, timeout);
}

// tcpSend from ESP8266 library can't send null bytes
uint16_t sendBuffer(uint8_t linkID, const uint8_t *buff, size_t size) {
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
