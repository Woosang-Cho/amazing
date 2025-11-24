#include <EEPROM.h>
#include "SMCController.h"

#define TRIG 9
#define ECHO 8

#define LEFT_PWM 5
#define RIGHT_PWM 6

#define VR_L  A0  // λ 조절 가변저항
#define VR_K  A1  // k 조절 가변저항

#define MAX_RECORDS 500

struct Record {
    unsigned long t;
    float distance;
};

Record records[MAX_RECORDS];
int recordIndex = 0;

float ref_distance = 20.0;
unsigned long lastSensorRead = 0;
float lastDistance = 30.0;

float lambda = 0.7;
float K = 6.0;
float phi = 0.1;
float dt = 0.01;

SMCController smc(lambda, K, phi, dt);

void setup() {
    pinMode(TRIG, OUTPUT);
    pinMode(ECHO, INPUT);
    pinMode(LEFT_PWM, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    Serial.begin(9600);

    loadTuning();               // EEPROM에서 마지막 λ, K 불러오기
    smc.setParameters(lambda, K, phi, dt);
}

void loop() {
    // 1. 실시간 튜닝
    lambda = analogRead(VR_L)/1023.0 * 2.0;   // 0~2 범위 예시
    K = analogRead(VR_K)/1023.0 * 10.0;       // 0~10 범위 예시
    smc.setParameters(lambda, K, phi, dt);

    // 2. 초음파 거리 측정 (30ms 간격)
    unsigned long now = micros();
    float measured;
    if (now - lastSensorRead > 30000) { // 30ms
        float raw = readDistance();
        measured = 0.7*lastDistance + 0.3*raw;  // 간단 필터
        lastDistance = measured;
        lastSensorRead = now;

        // 3. 거리-시간 데이터 기록
        if(recordIndex < MAX_RECORDS){
            records[recordIndex].t = millis();
            records[recordIndex].distance = measured;
            recordIndex++;
        }
    } else {
        measured = lastDistance;
    }

    // 4. SMC 제어
    float control_pwm = smc.update(ref_distance, measured);

    int base_pwm = 145; 
    int pwm = base_pwm + (int)control_pwm;
    pwm = constrain(pwm, 0, 255);

    analogWrite(LEFT_PWM, pwm);
    analogWrite(RIGHT_PWM, pwm);

    // 5. 시리얼 출력
    Serial.print("dist = "); Serial.print(measured);
    Serial.print("  pwm = "); Serial.print(pwm);
    Serial.print("  lambda = "); Serial.print(lambda);
    Serial.print("  K = "); Serial.println(K);

    // 6. 주행 종료 조건 (기록 배열 가득 찬 경우)
    if(recordIndex >= MAX_RECORDS){
        Serial.println("=== 주행 종료: 기록 완료 ===");
        printRecords();  // CSV 형태 출력
        while(1){}        // 루프 멈춤
    }

    delay(50);
}

// ================== 함수 정의 ==================

float readDistance() {
    digitalWrite(TRIG, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    long duration = pulseIn(ECHO, HIGH, 30000);
    if(duration == 0) return lastDistance;
    return duration * 0.0343 / 2.0; // cm
}

void saveTuning(){
    EEPROM.put(0, lambda);
    EEPROM.put(sizeof(lambda), K);
}

void loadTuning(){
    EEPROM.get(0, lambda);
    EEPROM.get(sizeof(lambda), K);
}

// 시리얼로 기록 데이터 확인
void printRecords(){
    Serial.println("time_ms,distance_cm");
    for(int i=0; i<recordIndex; i++){
        Serial.print(records[i].t);
        Serial.print(",");
        Serial.println(records[i].distance);
    }
}
