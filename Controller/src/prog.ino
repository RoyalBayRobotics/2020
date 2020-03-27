#include <SparkFunESP8266WiFi.h>
#include <XBOXRECV.h>
#include "common.h"

/*
 * XBOXRECV.h:46
 * - #define XBOX_WIRELESS_RECEIVER_THIRD_PARTY_PID  0x0291  // Third party Wireless Gaming Receiver
 * + #define XBOX_WIRELESS_RECEIVER_THIRD_PARTY_PID  0x02A9  // Third party Wireless Gaming Receiver
 */

USB usb;
XBOXRECV xbox(&usb);

struct {
    int16_t leftX, leftY, rightX, rightY, L2, R2
} neutral = {0};

XboxState xboxState(int id) {
    return XboxState {
        .leftX = xbox.getAnalogHat(LeftHatX, id),
        .leftY = xbox.getAnalogHat(LeftHatY, id),
        .rightX = xbox.getAnalogHat(RightHatX, id),
        .rightY = xbox.getAnalogHat(RightHatY, id),
        .L2 = xbox.getButtonPress(L2),
        .R2 = xbox.getButtonPress(R2),
        .buttons =
            MASK(xbox.getButtonPress(X, id), X) | MASK(xbox.getButtonPress(Y, id), Y) |
            MASK(xbox.getButtonPress(A, id), A) | MASK(xbox.getButtonPress(B, id), B) |
            MASK(xbox.getButtonPress(LEFT, id), LEFT) | MASK(xbox.getButtonPress(UP, id), UP) |
            MASK(xbox.getButtonPress(RIGHT, id), RIGHT) | xbox.getButtonPress(DOWN, id),
        .analogButtons =
            MASK(xbox.getButtonPress(L1, id), L1) | MASK(xbox.getButtonPress(R1, id), R1) |
            MASK(xbox.getButtonPress(L3, id), L3) | xbox.getButtonPress(R3, id),
    };
}

void setup() {
    Serial.begin(9600);
    esp8266.begin(9600, ESP8266_HARDWARE_SERIAL);

    sendCommand("+CIPCLOSE", "5");
    sendCommand("+CWMODE_CUR", "2");
    sendCommand("+CIPAP_CUR", "\"192.168.100.1\"");
    sendCommand("+CWSAP_CUR", "\"" SSID "\",\"" PASSWD "\",7,3");
    sendCommand("+CIPMUX", "1");

    for(int i = 0; i < 2; i++) {
        char param[35];
        sprintf(param, "%d,\"UDP\",\"192.168.100.%d\",8838,%d,0", i, i+2, 8839+i);
        sendCommand("+CIPSTART", param);
    }

    usb.Init();
}

void loop() {
    usb.Task();
    if(xbox.XboxReceiverConnected) {
        for(int i = 0; i < 4; i++) {
            if(xbox.Xbox360Connected[i]) {
                XboxState state = xboxState(i);
                uint8_t buff[sizeof(state)];
                memcpy(buff, &state, sizeof(state));
                sendBuffer(i, buff, sizeof(buff));
            }
        }
    }
}
