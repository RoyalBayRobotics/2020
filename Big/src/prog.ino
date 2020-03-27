#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <SparkFunESP8266WiFi.h>

#include "common.h"

#define SPEED_THRESH 40

struct HBridgeMotor {
    int pins[2];

    HBridgeMotor(int pin0, int pin1) {
        pins[0] = pin0;
        pins[1] = pin1;

        pinMode(pin0, OUTPUT);
        pinMode(pin1, OUTPUT);
    }

    void run(int speed) {
        if(speed > 0) {
            analogWrite(pins[0], speed);
            analogWrite(pins[1], 0);
        } else if(speed < 0) {
            analogWrite(pins[0], 0);
            analogWrite(pins[1], speed);
        } else {
            analogWrite(pins[0], 0);
            analogWrite(pins[1], 0);
        }
    }
};

Adafruit_MotorShield afms;
HBridgeMotor *hbMotors[2];

void setup() {
    Serial.begin(9600);

    afms.begin();

    for(int i = 0; i < 4; i++) {
        auto *motor = afms.getMotor(i+1);
        motor->setSpeed(0);
        motor->run(RELEASE);
    }

    hbMotors[0] = new HBridgeMotor(4, 5);
    hbMotors[1] = new HBridgeMotor(6, 7);

    esp8266.begin(9600, ESP8266_HARDWARE_SERIAL);

    sendCommand("+CIPCLOSE", "5");
    sendCommand("+CWMODE_CUR", "1");
    sendCommand("+CIPMUX", "1");
    sendCommand("+CIPSTA_CUR", "\"192.168.100.2\"");
    while(sendCommand("+CWJAP_CUR", "\"" SSID "\",\"" PASSWD "\"", WIFI_CONNECT_TIMEOUT) < 0);
    while(sendCommand("+CIPSTART", "0,\"UDP\",\"192.168.100.1\",8839,8838,0") < 0);

    Serial.println("# Connected to controller");
}

void runMotor(int i, int speed) {
    if(i == 1) return; // using the 2nd port overheats the chip for some reason

    speed = constrain(speed, -255, 255);

    if(speed >= -SPEED_THRESH && speed <= SPEED_THRESH) {
        speed = 0;
    }

    if(i < 4) { // Adafruit Motors
        auto *motor = afms.getMotor(i+1); // afms uses 1 based index
        if(speed > 0) {
            motor->setSpeed(speed);
            motor->run(FORWARD);
        } else if(speed < 0) {
            motor->setSpeed(-speed);
            motor->run(BACKWARD);
        } else {
            motor->setSpeed(0);
            motor->run(RELEASE);
        }
    } else { // HBridge Motors
        hbMotors[i-4]->run(speed);
    }
}

int mapToByte(int16_t from) {
    return map(from, -32768, 32767, -256, 255);
}

void loop() {

    XboxState xbox = {0};
    int ret = recieveXbox(&xbox);

    int x = mapToByte(xbox.leftX), y = mapToByte(xbox.leftY), z = mapToByte(xbox.rightX);

    if(ret == 0) {
        runMotor(0, y + x + z);
        runMotor(4, y - x + z);
        runMotor(2, y + x - z);
        runMotor(3, y - x - z);
    } else {
        Serial.print("# Disconnected...");
        // didn't recieve anything, stop all motors
        for(int i = 0; i < 4; i++) {
            runMotor(i, 0);
        }
    }
}
