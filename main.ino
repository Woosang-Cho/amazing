#include "SMCController.h"

// ==========================================
// 1. 핀 설정
// ==========================================
// 모터 드라이버 핀
#define LEFT_PWM  5
#define RIGHT_PWM 6
#define IN1 3
#define IN2 4
#define IN3 11
#define IN4 12

// 초음파 센서 핀 (A0-A5를 디지털 I/O로 사용)
#define TRIG_FRONT A0
#define ECHO_FRONT A1
#define TRIG_LEFT  A2
#define ECHO_LEFT  A3
#define TRIG_RIGHT A4
#define ECHO_RIGHT A5

// 버튼
#define START_BTN 2

// ==========================================
// 2. 파라미터 설정 (튜닝 필요)
// ==========================================
// 기본 속도 및 제어 주기
const int BASE_PWM = 120;        // 최소 주행 속도
const float CELL_DISTANCE = 6.0; // 정지/감속 목표 거리 (cm)
const float LOOP_TIME = 0.05;    // 제어 루프 주기 (50ms)

// 종방향 SMC 파라미터
float K_front = 5.0;
float lambda_front = 1.0;
float phi_front = 0.5;
const float TAU_FRONT = 0.02; // 종방향 LPF 시정수

// 횡방향 SMC 파라미터
float K_lat = 15.0;
float lambda_lat = 1.5;
float phi_lat = 1.0;
const float TAU_LAT = 0.02;   // 횡방향 LPF 시정수

// SMC 컨트롤러 인스턴스 생성
SMCController smcFront(K_front, lambda_front, phi_front, LOOP_TIME, TAU_FRONT);
SMCController smcLat(K_lat, lambda_lat, phi_lat, LOOP_TIME, TAU_LAT);

// ==========================================
// 3. 전역 변수
// ==========================================
bool started = false;
float lastFront = 20.0;
float lastLeft = 20.0;
float lastRight = 20.0;
unsigned long previousMillis = 0;

// 함수 프로토타입 선언
float readDistance(int trigPin, int echoPin);
void driveMotors(int leftSpeed, int rightSpeed);
void stopMotors();
void turnRight();

void setup() 
{
    // 시리얼 통신 시작
    Serial.begin(115200);

    // 모터 핀 초기화
    pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

    // 초음파 핀 초기화
    pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
    pinMode(TRIG_LEFT, OUTPUT); pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT); pinMode(ECHO_RIGHT, INPUT);

    // 버튼 핀 초기화
    pinMode(START_BTN, INPUT_PULLUP);

    // 시작 버튼 대기
    while(digitalRead(START_BTN) == HIGH) {
        // 대기 중
    }
    delay(500);
    started = true;
}

void loop() {
    if (!started) return;

    unsigned long currentMillis = millis();

    // 제어 루프 주기를 일정하게 유지
    if (currentMillis - previousMillis >= (LOOP_TIME * 1000)) {
        previousMillis = currentMillis;

        // ----------------------------------
        // 1. 센서 값 읽기 및 필터링
        // ----------------------------------
        float newFront = readDistance(TRIG_FRONT, ECHO_FRONT);
        float newLeft  = readDistance(TRIG_LEFT, ECHO_LEFT);
        float newRight = readDistance(TRIG_RIGHT, ECHO_RIGHT);

        lastFront = 0.6 * lastFront + 0.4 * newFront;
        lastLeft  = 0.6 * lastLeft  + 0.4 * newLeft;
        lastRight = 0.6 * lastRight + 0.4 * newRight;

        // ----------------------------------
        // 2. SMC 제어 계산
        // ----------------------------------
        // uFront: 속도 조절 (부호 반전: 멀면 속도 UP)
        float uFront = -smcFront.update(CELL_DISTANCE, lastFront);
        
        // uLat: 조향 조절 (Positive: 오른쪽 턴 필요)
        float errorLat = lastLeft - lastRight; 
        float uLat = smcLat.update(0, errorLat); 

        // ----------------------------------
        // 3. 모터 믹싱
        // ----------------------------------
        int corrected_base_pwm = constrain(BASE_PWM + (int)uFront, 0, 255);

        // uLat 양수 -> 오른쪽 턴 -> Left UP, Right DOWN
        int pwmLeft  = constrain(corrected_base_pwm + (int)uLat, 0, 255);
        int pwmRight = constrain(corrected_base_pwm - (int)uLat, 0, 255);

        // ----------------------------------
        // 4. 전방 벽 감지 및 주행
        // ----------------------------------
        if (lastFront < 3.0) { 
            stopMotors();
            delay(200);     
            turnRight();    
            previousMillis = millis(); 
        } else {
            driveMotors(pwmLeft, pwmRight);
        }

      // 기존 8개 변수 출력
    Serial.print(lastLeft); Serial.print(",");
    Serial.print(lastRight); Serial.print(",");
    Serial.print(lastFront); Serial.print(",");
    Serial.print(errorLat); Serial.print(","); 
    Serial.print(uFront); Serial.print(",");
    Serial.print(uLat); Serial.print(",");
    Serial.print(pwmLeft); Serial.print(",");
    Serial.print(pwmRight); Serial.print(",");

// 새로 추가된 4개 SMC 핵심 변수 출력 (Getter 함수 사용)
    Serial.print(smcLat.get_e_dot()); Serial.print(",");  // 횡방향 e_dot (LPF 적용)
    Serial.print(smcLat.get_s()); Serial.print(",");       // 횡방향 s
    Serial.print(smcFront.get_e_dot()); Serial.print(","); // 종방향 e_dot (LPF 적용)
    Serial.println(smcFront.get_s());                       // 종방향 s (마지막 변수는 println)
    }
}

// ==========================================
// 보조 함수 구현 (기존과 동일)
// ==========================================

float readDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 2500); 

    if (duration == 0) return 30.0;
    return duration * 0.0343 / 2.0;
}

void driveMotors(int leftSpeed, int rightSpeed) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(LEFT_PWM, leftSpeed);
    analogWrite(RIGHT_PWM, rightSpeed);
}

void stopMotors() {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
    analogWrite(LEFT_PWM, 0); analogWrite(RIGHT_PWM, 0);
}

void turnRight() {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); 
    int turnSpeed = 160; 
    analogWrite(LEFT_PWM, turnSpeed);
    analogWrite(RIGHT_PWM, turnSpeed);
    delay(550); 
    stopMotors();
    delay(200); 
}