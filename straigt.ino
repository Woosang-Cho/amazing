#include <SMCController.h>

#define TRIG 3
#define ECHO 2

#define LEFT_PWM 5
#define RIGHT_PWM 6

// SMC(k, phi, c1, c2, dt)
SMCController smc(8.0, 0.05, 1.0, 0.015, 0.01);

float ref_distance = 20.0; // cm
unsigned long lastSensorRead = 0;
float lastDistance = 0.0;

float readDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 20000);
    if (duration == 0) return lastDistance;

    float dist = duration * 0.0343 / 2.0;
    lastDistance = dist;
    return dist;
}

void setup() {
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);

    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
}

void loop() {
    unsigned long now = micros();

    float measured;
    if (now - lastSensorRead > 20000) { // 20ms마다 측정
        measured = readDistance();
        lastSensorRead = now;
    } else {
        measured = lastDistance;
    }

    float control = smc.update(ref_distance, measured);

    int left_pwm = 120 + (int)control;
    int right_pwm = 120 + (int)control;

    if (left_pwm < 0) left_pwm = 0;
    if (left_pwm > 255) left_pwm = 255;
    if (right_pwm < 0) right_pwm = 0;
    if (right_pwm > 255) right_pwm = 255;

    analogWrite(LEFT_PWM, left_pwm);
    analogWrite(RIGHT_PWM, right_pwm);
}
