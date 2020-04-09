#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Adafruit_PWMServoDriver.h>
#include <SparkFunESP8266WiFi.h>

#include "common.h"

#define SPEED_THRESH 10
#define SPEED_SLOW 100

#define SERVO_MIN 150
#define SERVO_MAX 550

#define ARM_MAX 3000
#define ARM_MARG 300

#define ENCODER_A 2
#define ENCODER_B 4

#define CLAW0 6
#define CLAW1 5

#define LEFT 0
#define RIGHT 2
#define ARM 3

#define BTN 51

Adafruit_MotorShield afms;
Adafruit_PWMServoDriver afsd;
long encoderCount = 0;

void setup() {
    Serial.begin(9600);

    afms.begin();

    for(int i = 0; i < 4; i++) {
        auto *motor = afms.getMotor(i+1);
        motor->setSpeed(0);
        motor->run(RELEASE);
    }

    afsd.begin();
    afsd.setPWMFreq(50);

    // Rotary encoder
    pinMode(5, OUTPUT); // Used as power for encoder
    pinMode(3, OUTPUT); // Used as ground for encoder
    pinMode(ENCODER_A, INPUT);
    pinMode(ENCODER_B, INPUT);
    digitalWrite(5, HIGH);
    digitalWrite(3, LOW);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), &encoderIntr, RISING);

    // Safety button
    pinMode(BTN, INPUT_PULLUP);

    esp8266.begin(9600, ESP8266_SOFTWARE_SERIAL);

    sendCommand("+CIPCLOSE", "5");
    sendCommand("+CWMODE_CUR", "1");
    sendCommand("+CIPMUX", "1");
    sendCommand("+CIPSTA_CUR", "\"192.168.100.3\"");
    while(sendCommand("+CWJAP_CUR", "\"" SSID "\",\"" PASSWD "\"", WIFI_CONNECT_TIMEOUT) < 0);
    while(sendCommand("+CIPSTART", "0,\"UDP\",\"192.168.100.1\",8840,8838,0") < 0);

    Serial.println("# Connected to controller");
}

void encoderIntr() {
    if(digitalRead(ENCODER_B))
        encoderCount--;
    else
        encoderCount++;
}

void runMotor(int i, int speed) {

    speed = constrain(speed, -255, 255);
    if(speed >= -SPEED_THRESH && speed <= SPEED_THRESH) {
        speed = 0;
    }

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
}

int mapToByte(int16_t from) {
    return map(from, -32768, 32767, -256, 255);
}

void loop() {

    XboxState xbox = {0};
    int ret = recieveXbox(&xbox);

    if(ret != 0) {
        Serial.println("# No data...");
        // didn't recieve anything, stop all motors
        for(int i = 0; i < 6; i++) {
            runMotor(i, 0);
        }
        return;
    }

    // wheels
    int x = mapToByte(xbox.leftX),
        y = mapToByte(xbox.leftY);
    runMotor(LEFT, y + x);
    runMotor(RIGHT, y - x);

    // servo claws and wrist
    afsd.setPWM(CLAW0, 0, map(xbox.L2, -32768, 32767, SERVO_MIN, SERVO_MAX));
    afsd.setPWM(CLAW1, 0, map(xbox.R2, -32768, 32767, SERVO_MIN, SERVO_MAX));

    // arm safety button
    bool hit = false;
    if(digitalRead(BTN) == LOW) { // button pressed
        encoderCount = 0;
        hit = true;
    }

    // arm
    int movement = mapToByte(xbox.rightY);
    bool force = xbox.analogButtons & MASK(1, R3) > 0;
    if(movement < 0) {
        if(hit || (encoderCount <= 0 && !force))
            runMotor(ARM, 0);
        else if(encoderCount < ARM_MARG)
            runMotor(ARM, max(movement, -SPEED_SLOW));
        else
            runMotor(ARM, movement);
    } else {
        if(encoderCount >= ARM_MAX && !force)
            runMotor(ARM, 0);
        else if(encoderCount > ARM_MAX - ARM_MARG)
            runMotor(ARM, min(movement, SPEED_SLOW));
        else
            runMotor(ARM, movement);
    }
}
