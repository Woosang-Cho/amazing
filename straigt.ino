#include <EEPROM.h>
#include "SMCController.h"

#define TRIG 9
#define ECHO 8

#define LEFT_PWM 5
#define RIGHT_PWM 6

#define IN1 3
#define IN2 4
#define IN3 11
#define IN4 12

/*
#define VR_L  A0
#define VR_K  A1
*/


#define START_BTN 2
bool started = false;

// ====== 최소 PWM 임계값 (실험으로 80~140 조정) ======
const int MIN_EFFECTIVE_PWM = 120; // 80~140 사이 여러 값 실험 후 맞춤

float ref_distance = 6.0;
uint32_t lastSensorRead = 0;
float lastDistance = 20.0;

// 튜닝 값
float lambda;
float K;
const float phi = 0.5;
const float c1  = 1.0;
float c2;
const float dt = 0.01;

SMCController smc(K, phi, c1, c2, dt);

void setup() {
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    Serial.begin(9600);
    pinMode(START_BTN, INPUT_PULLUP);

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);

    loadTuning();
    c2 = lambda;
    smc.setParameters(lambda, K, phi, dt);
}

void loop() {
    if(!started) {
        if(digitalRead(START_BTN) == LOW) {
            delay(50);
            if(digitalRead(START_BTN) == LOW) {
                started = true;
            }
        }
        return;
    }
/*
    // 실시간 튜닝(가변저항)
    lambda = analogRead(VR_L)/1023.0 * 2.0;
    K = analogRead(VR_K)/1023.0 * 10.0;
    c2 = lambda;
    smc.setParameters(lambda, K, phi, dt);
*/


    // 초음파 거리 측정 (30ms마다)
    uint32_t now = micros();
    float measured;
    if (now - lastSensorRead > 30000) {
        float raw = readDistance();
        measured = 0.6 * lastDistance + 0.4 * raw;
        lastDistance = measured;
        lastSensorRead = now;
    } else {
        measured = lastDistance;
    }

    // SMC 제어
    float control_pwm = smc.update(ref_distance, measured);

    // ---- 최소 PWM 임계값을 적용한 감속 구간 코드 ----
    int base_pwm;
    if (measured > 20.0) {
        base_pwm = 180;
    } else if (measured > ref_distance) {
        base_pwm = MIN_EFFECTIVE_PWM + (measured - ref_distance)/(12.0) * (180 - MIN_EFFECTIVE_PWM);
        if (base_pwm < MIN_EFFECTIVE_PWM) base_pwm = 0; // 최소 이하 바로 정지
    } else {
        base_pwm = 0;
    }

    int pwm = base_pwm + (int)control_pwm;
    if (pwm < MIN_EFFECTIVE_PWM) pwm = 0; // 소리날 구간 즉시 정지
    pwm = constrain(pwm, 0, 255);

    // 모터 제어
    analogWrite(LEFT_PWM, pwm);
    analogWrite(RIGHT_PWM, pwm);

if(measured < 6.0) {
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);
    delay(100);

    // 오른쪽 90도 회전
    digitalWrite(IN1, HIGH);    // 왼쪽 앞으로
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);     // 오른쪽 뒤로
    digitalWrite(IN4, HIGH);
    int turn_pwm = 150;
    analogWrite(LEFT_PWM, turn_pwm);
    analogWrite(RIGHT_PWM, turn_pwm);
    delay(650); 
    analogWrite(LEFT_PWM, 0);
    analogWrite(RIGHT_PWM, 0);

    while(1){} // 회전 끝나고 정지
}

}

float readDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 30000);
    if(duration == 0) return lastDistance;
    return duration * 0.0343 / 2.0;
}

/*
void saveTuning(){
    EEPROM.put(0, lambda);
    EEPROM.put(sizeof(lambda), K);
}

void loadTuning(){
    EEPROM.get(0, lambda);
    EEPROM.get(sizeof(lambda), K);
}
*/

