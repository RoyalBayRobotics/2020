#include <SparkFunESP8266WiFi.h>
#include <util/ESP8266_AT.h>
#include <XBOXRECV.h>

#define SSID "RBSSROBOT"
#define PASSWD "0x52425353524f424f54"

/*
 * XBOXRECV.h:46
 * - #define XBOX_WIRELESS_RECEIVER_THIRD_PARTY_PID  0x0291  // Third party Wireless Gaming Receiver
 * + #define XBOX_WIRELESS_RECEIVER_THIRD_PARTY_PID  0x02A9  // Third party Wireless Gaming Receiver
 */

USB usb;
XBOXRECV xbox(&usb);

#pragma pack(push, 1)
struct XboxState {
    int16_t leftX;
    int16_t leftY;
    int16_t rightX;
    int16_t rightY;
    int8_t L2;
    int8_t R2;
    uint8_t buttons; // MSB | X Y A B left up right down | LSB
    uint8_t analogButtons:4; // MSB | L1 R1 L3 R3 | LSB
};
#pragma pack(pop)

uint16_t sendCommand(const char* cmd, const char* param) {
    esp8266.sendCommand(cmd, ESP8266_CMD_SETUP, param);
    return esp8266.readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);
}

// tcpSend from ESP8266 library can't send null bytes
uint16_t sendBuffer(uint8_t linkID, const uint8_t *buff, size_t size) {
    char params[8];
    sprintf(params, "%d,%d", linkID, size);
    esp8266.sendCommand(ESP8266_TCP_SEND, ESP8266_CMD_SETUP, params);

    int16_t rsp = esp8266.readForResponses(RESPONSE_OK, RESPONSE_ERROR, COMMAND_RESPONSE_TIMEOUT);
    if (rsp != ESP8266_RSP_FAIL) {
        Serial.write(buff, size);

        rsp = esp8266.readForResponse("SEND OK", COMMAND_RESPONSE_TIMEOUT);

        if (rsp > 0)
            return size;
    }

    return rsp;
}

uint8_t findLinkId(IPAddress ip) {
    esp8266.updateStatus();
    for(uint8_t i = 0; i < ESP8266_MAX_SOCK_NUM; i++) {
        esp8266_ipstatus *link = &esp8266._status.ipstatus[i];
        if(link->linkID == i && link->remoteIP == ip) {
            return i;
        }
    }
    return 255;
}

XboxState xboxState(int id) {
    return XboxState {
        .leftX = xbox.getAnalogHat(LeftHatX, id),
        .leftY = xbox.getAnalogHat(LeftHatY, id),
        .rightX = xbox.getAnalogHat(RightHatX, id),
        .rightY = xbox.getAnalogHat(RightHatY, id),
        .L2 = xbox.getButtonPress(L2),
        .R2 = xbox.getButtonPress(R2),
        .buttons =
            xbox.getButtonPress(X, id) << 7 | xbox.getButtonPress(Y, id) << 6 |
            xbox.getButtonPress(A, id) << 5 | xbox.getButtonPress(B, id) << 4 |
            xbox.getButtonPress(LEFT, id) << 3 | xbox.getButtonPress(UP, id) << 2 |
            xbox.getButtonPress(RIGHT, id) << 1 | xbox.getButtonPress(DOWN, id),
        .analogButtons =
            xbox.getButtonPress(L1, id) << 3 | xbox.getButtonPress(R1, id) << 2 |
            xbox.getButtonPress(L3, id) << 1 | xbox.getButtonPress(R3, id),
    };
}

void setup() {
    Serial.begin(9600);
    esp8266.begin(9600, ESP8266_HARDWARE_SERIAL);

    sendCommand("+CWMODE_CUR", "2");
    sendCommand("+CWSAP_CUR", "\"" SSID "\",\"" PASSWD "\",7,3");
    sendCommand("+CIPAP_CUR", "\"192.168.100.1\"");

    esp8266.configureTCPServer(8838, 1);

    usb.Init();
}

void loop() {
    usb.Task();
    if(xbox.XboxReceiverConnected) {
        for(int i = 0; i < 4; i++) {
            if(xbox.Xbox360Connected[i]) {
                uint8_t link = findLinkId(IPAddress(192, 168, 100, i+2));
                if(link < 255) {
                    XboxState state = xboxState(i);
                    uint8_t buff[sizeof(state)];
                    memcpy(buff, &state, sizeof(state));
                    sendBuffer(link, buff, sizeof(buff));
                }
            }
        }
    }
    delay(50);
}
