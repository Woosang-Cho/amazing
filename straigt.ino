#include <EEPROM.h>
#include "SMCController.h"

#define TRIG 9
#define ECHO 8

#define LEFT_PWM 5
#define RIGHT_PWM 6

#define VR_L  A0
#define VR_K  A1

#define MAX_RECORDS 96  // (더 줄일 수 있음)

#define START_BTN 2
bool started = false;

struct Record {
    uint32_t t;             // 시간(ms)
    uint16_t distance;      // 0.1cm 단위
};

Record records[MAX_RECORDS];
uint8_t recordIndex = 0;    // 0~255면 충분 (uint8_t)

float ref_distance = 20.0;
uint32_t lastSensorRead = 0;
float lastDistance = 30.0;

// 튜닝 값
float lambda;    // EEPROM에서 불러옴
float K;         // EEPROM에서 불러옴
const float phi = 0.5;
const float c1  = 1.0;
float c2;        // lambda랑 같아서 나중에 할당
const float dt = 0.01;

SMCController smc(K, phi, c1, c2, dt);

void setup() {
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    Serial.begin(9600);

    pinMode(START_BTN, INPUT_PULLUP);

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
                Serial.println(F("=== 주행 시작 ==="));
            }
        }
        return;
    }

    // 실시간 튜닝
    lambda = analogRead(VR_L)/1023.0 * 2.0;
    K = analogRead(VR_K)/1023.0 * 10.0;
    c2 = lambda;
    smc.setParameters(lambda, K, phi, dt);

    // 초음파 거리 측정 (30ms 간격)
    uint32_t now = micros();
    float measured;
    if (now - lastSensorRead > 30000) {
        float raw = readDistance();
        measured = 0.7*lastDistance + 0.3*raw;
        lastDistance = measured;
        lastSensorRead = now;

        // 기록
        if(recordIndex < MAX_RECORDS){
            records[recordIndex].t = millis();
            records[recordIndex].distance = (uint16_t)(measured * 10.0);  // 0.1cm 단위로 저장
            recordIndex++;
        }
    } else {
        measured = lastDistance;
    }

    // SMC 제어
    float control_pwm = smc.update(ref_distance, measured);

    int base_pwm = 145;
    int pwm = base_pwm + (int)control_pwm;
    pwm = constrain(pwm, 0, 255);

    analogWrite(LEFT_PWM, pwm);
    analogWrite(RIGHT_PWM, pwm);

    Serial.print(F("dist = ")); Serial.print(measured);
    Serial.print(F("  pwm = ")); Serial.print(pwm);
    Serial.print(F("  lambda = ")); Serial.print(lambda);
    Serial.print(F("  K = ")); Serial.println(K);

    if(recordIndex >= MAX_RECORDS){
        Serial.println(F("=== 주행 종료: 기록 완료 ==="));
        printRecords();
        while(1){}
    }

    delay(50);
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

void saveTuning(){
    EEPROM.put(0, lambda);
    EEPROM.put(sizeof(lambda), K);
}

void loadTuning(){
    EEPROM.get(0, lambda);
    EEPROM.get(sizeof(lambda), K);
}

void printRecords(){
    Serial.println(F("time_ms,distance_cm"));
    for(uint8_t i=0; i<recordIndex; i++){
        Serial.print(records[i].t);
        Serial.print(",");
        Serial.println(records[i].distance / 10.0, 1);  // float 형태로 출력, 소수 1자리
    }
}
