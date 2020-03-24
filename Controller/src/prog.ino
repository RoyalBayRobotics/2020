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
