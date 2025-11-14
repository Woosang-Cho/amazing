#include <SMCController.h>

SMCController smc(3.0f, 2.0f, 0.05f, 0.01f);

const int motorPin = 9;

unsigned long last_t = 0;

void setup() {
    pinMode(motorPin, OUTPUT);
}

void loop() {
    unsigned long now = micros();
    if (now - last_t >= 10000) {
        last_t = now;

        float error = 0.0f;

        float u = smc.update(error);

        int pwm = constrain((int)(u * 255.0f), -255, 255);

        if (pwm >= 0) analogWrite(motorPin, pwm);
        else analogWrite(motorPin, -pwm);
    }
}
