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
const float CELL_DISTANCE = 6.0; // 정지 목표 거리 (cm)
const float LOOP_TIME = 0.05;    // 제어 루프 주기 (50ms)
const float PWM_MIXING_LIMIT = 50.0; // uLat이 적용될 최대 PWM 제한

// 미로 탐색 종료 임계값 (전방/좌/우 모두 이 거리 이상이면 종료)
const float EXIT_DISTANCE_THRESHOLD = 60.0; 

// 종방향 SMC 파라미터 (전방 튜닝)
float K_front = 5.0;
float lambda_front = 1.0;
float phi_front = 1.0;
const float TAU_FRONT = 0.02; // LPF 시정수

// 횡방향 SMC 파라미터 (측면 튜닝)
float K_lat = 15.0;
float lambda_lat = 1.5;
float phi_lat = 1.5;
const float TAU_LAT = 0.02;   // LPF 시정수

// SMC 컨트롤러 인스턴스 생성 (tau 파라미터 추가)
SMCController smcFront(K_front, lambda_front, phi_front, LOOP_TIME, TAU_FRONT);
SMCController smcLat(K_lat, lambda_lat, phi_lat, LOOP_TIME, TAU_LAT);

// ==========================================
// 3. 전역 변수
// ==========================================
// 상태 머신을 위한 상태 정의
enum RobotState { STRAIGHT, TURNING };
RobotState currentState = STRAIGHT;
unsigned long turnStartTime = 0; // 턴 시작 시간 기록

bool started = false;
float lastFront = 20.0;
float lastLeft = 20.0;
float lastRight = 20.0;
unsigned long previousMillis = 0;

// 함수 프로토타입 선언
float readDistance(int trigPin, int echoPin);
void driveMotors(int leftSpeed, int rightSpeed);
void stopMotors();
bool updateTurnRight(); // Non-blocking 턴 함수

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
    delay(500); // 디바운싱 및 안정화
    started = true;
}

void loop() {
    if (!started) return;

    unsigned long currentMillis = millis();

    // ----------------------------------
    // 0. Non-Blocking 턴 동작 업데이트 (상태 머신)
    // ----------------------------------
    if (currentState == TURNING) {
        if (updateTurnRight()) {
            currentState = STRAIGHT; // 턴 완료 후 직진 상태로 복귀
        }
        return; // 턴 중에는 제어 루프 건너뛰기
    }

    // ----------------------------------
    // 제어 루프 주기를 일정하게 유지 (50ms)
    // ----------------------------------
    if (currentMillis - previousMillis >= (LOOP_TIME * 1000)) {
        previousMillis = currentMillis;

        // ----------------------------------
        // 1. 센서 값 읽기 (LPF 제거)
        // ----------------------------------
        float newFront = readDistance(TRIG_FRONT, ECHO_FRONT);
        float newLeft  = readDistance(TRIG_LEFT, ECHO_LEFT);
        float newRight = readDistance(TRIG_RIGHT, ECHO_RIGHT);

        lastFront = newFront; 
        lastLeft  = newLeft;  
        lastRight = newRight; 

        // ----------------------------------
        // 2. SMC 제어 계산
        // ----------------------------------
        float uFront = smcFront.update(CELL_DISTANCE, lastFront);
        
        float errorLat = lastLeft - lastRight; 
        float uLat = smcLat.update(0, errorLat); 

        // ----------------------------------
        // 3. 모터 믹싱
        // ----------------------------------
        uLat = constrain(uLat, -PWM_MIXING_LIMIT, PWM_MIXING_LIMIT); 

        // uFront는 BASE_PWM에서 감속/가속 역할
        int corrected_base_pwm = constrain(BASE_PWM + (int)uFront, 0, 255);

        // uLat 양수 -> 오른쪽 턴 -> Left UP, Right DOWN
        int pwmLeft  = constrain(corrected_base_pwm + (int)uLat, 0, 255);
        int pwmRight = constrain(corrected_base_pwm - (int)uLat, 0, 255);

        // ----------------------------------
        // 4. 전방 벽 감지 및 주행 
        // ----------------------------------
        if (lastFront <= CELL_DISTANCE) { // 6.0cm 이하로 진입하면 멈추고 회전
            stopMotors();
            currentState = TURNING; // 턴 상태로 진입
            turnStartTime = millis(); 
        } else {
            driveMotors(pwmLeft, pwmRight); // SMC 제어값으로 주행
        }
        
        // ----------------------------------
        // 5. 미로 탐색 종료 조건 체크 (종료 메시지 제거)
        // ----------------------------------
        if (lastFront > EXIT_DISTANCE_THRESHOLD && 
            lastLeft > EXIT_DISTANCE_THRESHOLD && 
            lastRight > EXIT_DISTANCE_THRESHOLD) 
        {
            stopMotors();
            while(1) {
                // 영구 정지: 시리얼 통신을 포함한 모든 동작 중단
            } 
        }


        // ----------------------------------
        // 6. 시리얼 로깅 (12개 숫자 변수만 출력, 프레임 동기화 문자 추가)
        // ----------------------------------
        Serial.print("<"); // 시작 문자
        
        // 순서: L_Dist, R_Dist, F_Dist, ErrorLat, uFront, uLat, PWM_L, PWM_R, ErrorLat_dot, s_lat, ErrorFront_dot, s_front
        Serial.print(lastLeft); Serial.print(",");
        Serial.print(lastRight); Serial.print(",");
        Serial.print(lastFront); Serial.print(",");
        Serial.print(errorLat); Serial.print(","); 
        Serial.print(uFront); Serial.print(",");
        Serial.print(uLat); Serial.print(",");
        Serial.print(pwmLeft); Serial.print(",");
        Serial.print(pwmRight); Serial.print(",");
        
        Serial.print(smcLat.get_e_dot()); Serial.print(",");  
        Serial.print(smcLat.get_s()); Serial.print(",");       
        Serial.print(smcFront.get_e_dot()); Serial.print(","); 
        Serial.print(smcFront.get_s()); // 마지막 값은 쉼표 없이

        Serial.println(">"); // 종료 문자 (줄바꿈 포함)
    }
}

// ==========================================
// 보조 함수 구현 (생략)
// ==========================================
// 초음파 센서 거리 측정 함수 (타임아웃 20000us로 늘림)
float readDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    
    long duration = pulseIn(echoPin, HIGH, 20000); 

    if (duration == 0) return 80.0; 
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

// Non-Blocking 턴 동작 업데이트 함수
bool updateTurnRight() {
    // 턴 동작 시간과 속도
    const unsigned long TURN_DURATION = 550; // 튜닝된 값으로 설정
    const int TURN_SPEED = 160; 

    if (millis() - turnStartTime < TURN_DURATION) {
        // 턴 동작 수행
        digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); // 왼쪽 전진
        digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // 오른쪽 후진
        analogWrite(LEFT_PWM, TURN_SPEED);
        analogWrite(RIGHT_PWM, TURN_SPEED);
        return false; // 턴 미완료
    } else {
        // 턴 완료
        stopMotors();
        return true; // 턴 완료
    }
}