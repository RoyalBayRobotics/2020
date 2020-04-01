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

#define CLAW 6
#define WRIST0 5
#define WRIST1 4

#define FL 0
#define RL 2
#define FR 3
#define RR 4
#define ARM 5

#define HB0 8, 9
#define HB1 10, 11

#define BTN 51

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
            analogWrite(pins[1], -speed);
        } else {
            analogWrite(pins[0], 0);
            analogWrite(pins[1], 0);
        }
    }
};

HBridgeMotor *hbMotors[2];
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

    hbMotors[0] = new HBridgeMotor(HB0);
    hbMotors[1] = new HBridgeMotor(HB1);

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

    esp8266.begin(9600, ESP8266_HARDWARE_SERIAL);

    sendCommand("+CIPCLOSE", "5");
    sendCommand("+CWMODE_CUR", "1");
    sendCommand("+CIPMUX", "1");
    sendCommand("+CIPSTA_CUR", "\"192.168.100.2\"");
    while(sendCommand("+CWJAP_CUR", "\"" SSID "\",\"" PASSWD "\"", WIFI_CONNECT_TIMEOUT) < 0);
    while(sendCommand("+CIPSTART", "0,\"UDP\",\"192.168.100.1\",8839,8838,0") < 0);

    Serial.println("# Connected to controller");
}

void encoderIntr() {
    if(digitalRead(ENCODER_B))
        encoderCount--;
    else
        encoderCount++;
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
        y = mapToByte(xbox.leftY),
        z = mapToByte(xbox.rightX);
    runMotor(FL, y + x + z);
    runMotor(FR, y - x + z);
    runMotor(RL, y + x - z);
    runMotor(RR, y - x - z);

    // servo claws and wrist
    afsd.setPWM(CLAW, 0, map(xbox.L2, -32768, 32767, SERVO_MIN, SERVO_MAX));
    afsd.setPWM(WRIST0, 0, map(xbox.R2, -32768, 32767, SERVO_MIN, SERVO_MAX));

    // cr servo wrist
    if(xbox.analogButtons & (MASK(1, L1) | MASK(1, R1))) {
        if(xbox.analogButtons & MASK(1, L1)) {
            afsd.setPWM(WRIST1, 0, SERVO_MAX);
        } else {
            afsd.setPWM(WRIST1, 0, SERVO_MIN);
        }
    } else {
        afsd.setPWM(WRIST1, 0, 0);
    }

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
