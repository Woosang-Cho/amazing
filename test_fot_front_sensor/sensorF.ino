#include <Arduino.h>
#include "SMCController.h" 

// ==========================================
// 1. 핀 설정
// ==========================================
// 모터 드라이버 핀
#define LEFT_PWM 5
#define RIGHT_PWM 6
#define IN1 3
#define IN2 4
#define IN3 11
#define IN4 12

// 전면 초음파 센서 핀
#define TRIG_FRONT A0 
#define ECHO_FRONT A1 
#define START_BTN 2
bool started = false;

// ==========================================
// 2. SMC 및 제어 상수 설정
// ==========================================
const float LOOP_TIME = 0.050;     // 제어 루프 주기 (dt), 현재 50ms 
const int MAX_SPEED = 160;         // 직진 시 최대 PWM
const int MIN_EFFECTIVE_PWM = 120; // 모터 구동 최소 임계값

const float MAZE_CELL_SIZE = 20.0; // 미로 한 칸의 실제 크기 (20cm)

// SMC 목표 정지 거리: 좁은 미로에 맞춰 5.0cm (정확도 향상)
const float REF_DISTANCE = 5.0; 
// SMC 감속 시작 거리: 5cm 목표를 위해 15cm에서 감속 시작
const float DECEL_DIST = 15.0; 

// SMC 파라미터 (튜닝된 값 유지)
const float SMC_K = 4.0;      
const float SMC_LAMBDA = 0.2; 
const float SMC_PHI = 0.5;    
const float LPF_TAU = 0.040;  

SMCController smc(SMC_K, SMC_LAMBDA, SMC_PHI, LOOP_TIME, LPF_TAU);

// LPF 및 SMC 상태 변수 (extern 유지)
extern float last_dist_raw; 
extern float last_dist_lpf;
extern float last_error; 
extern float last_e_dot_lpf;
extern float last_s_value;

// ==========================================
// 3. 상태 머신 및 유틸리티
// ==========================================
enum RobotState {
    WAITING_TO_START,
    GO_STRAIGHT,
    DECELERATE_AND_STOP,
    TURN_RIGHT,
    STOPPED_FINAL
};
RobotState currentState = WAITING_TO_START;
unsigned long last_loop_time = 0;
unsigned long stop_timer = 0; 

// 이동 칸 수 카운터 (총 2칸 이동 목표)
int path_count = 0; 
const int TARGET_CELLS = 2; 

// 센서 클리핑 로직 통합: 30cm 이상 비정상 값 차단
float readDistance() {
    digitalWrite(TRIG_FRONT, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_FRONT, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_FRONT, LOW);
    delayMicroseconds(10); 
    long duration = pulseIn(ECHO_FRONT, HIGH, 80000); 
    
    if(duration == 0) return last_dist_raw; 
    
    float measured_dist = duration * 0.0343 / 2.0;

    // 센서 값 클리핑: 30cm 이상은 이전 값 유지 (오작동 방지)
    if (measured_dist > 30.0) {
        return last_dist_raw; 
    }
    
    return measured_dist;
}

void setMotorSpeed(int left_pwm, int right_pwm) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); 
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); 
    analogWrite(LEFT_PWM, constrain(left_pwm, 0, 255));
    analogWrite(RIGHT_PWM, constrain(right_pwm, 0, 255));
}

// ==========================================
// 4. Setup & Loop
// ==========================================
void setup() {
    // 핀 설정
    pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT); 
    pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(START_BTN, INPUT_PULLUP);
    
    Serial.begin(115200); 
    
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    
    float initial_dist = readDistance();
    last_dist_lpf = initial_dist; 
    last_dist_raw = initial_dist; 
    
    last_loop_time = millis();
    
    Serial.println("time_sec,raw_dist_cm,lpf_dist_cm,error_cm,e_dot_lpf_cms,s_value,state"); 
}

void loop() {
    // 시작 버튼 대기
    if (currentState == WAITING_TO_START) {
        if (digitalRead(START_BTN) == LOW) {
            delay(50);
            if (digitalRead(START_BTN) == LOW) {
                currentState = GO_STRAIGHT;
                path_count = 0; // 0번째 칸부터 시작
                last_loop_time = millis();
            }
        }
        return;
    }

    // LOOP_TIME (50ms) 주기로 SMC 제어 및 시리얼 출력 실행
    if (millis() - last_loop_time >= (unsigned long)(LOOP_TIME * 1000)) {
        last_loop_time = millis();
        
        // 1. 센서 값 측정 및 SMC/LPF 업데이트
        float measured_raw = readDistance(); 
        float control_U = smc.update(REF_DISTANCE, measured_raw); 
        
        // 2. 시리얼 출력 
        Serial.print(millis() / 1000.0); Serial.print(",");
        Serial.print(last_dist_raw); Serial.print(",");
        Serial.print(last_dist_lpf); Serial.print(",");
        Serial.print(last_error); Serial.print(",");
        Serial.print(last_e_dot_lpf); Serial.print(",");
        Serial.print(last_s_value); Serial.print(",");

        // 상태 출력
        switch(currentState) {
            case GO_STRAIGHT: Serial.println("GO"); break;
            case DECELERATE_AND_STOP: Serial.println("SMC"); break;
            case TURN_RIGHT: Serial.println("TURN"); break;
            case STOPPED_FINAL: Serial.println("FINAL"); break;
            default: Serial.println("WAIT"); break;
        }

        // 3. 상태 머신 실행
        switch (currentState) {
            
            case GO_STRAIGHT:
                setMotorSpeed(MAX_SPEED, MAX_SPEED); 
                
                // 15cm 앞에서 감속 시작
                if (last_dist_lpf <= DECEL_DIST) { 
                    currentState = DECELERATE_AND_STOP;
                }
                break;

            case DECELERATE_AND_STOP:
                
                // SMC 제어 신호를 PWM으로 변환
                int target_pwm = (int)constrain(MAX_SPEED * (1.0 + control_U) / 2.0, 0, MAX_SPEED);
                
                // 최소 PWM 임계값 적용
                if (target_pwm > 0 && target_pwm < MIN_EFFECTIVE_PWM) {
                    target_pwm = 0; 
                }

                setMotorSpeed(target_pwm, target_pwm);
                
                // 천이 조건: 목표 거리에 도달(5cm)하고 정지 상태를 150ms 유지
                if (last_dist_lpf <= REF_DISTANCE + 0.5 && target_pwm == 0) {
                    if (stop_timer == 0) {
                        stop_timer = millis(); 
                    } else if (millis() - stop_timer >= 150) { // 150ms 동안 정지 유지
                        setMotorSpeed(0, 0); 
                        
                        path_count++; // 1칸 이동 완료
                        
                        // 2칸을 모두 이동했는지 확인
                        if (path_count >= TARGET_CELLS) {
                            currentState = STOPPED_FINAL; // 최종 정지
                        } else {
                            currentState = TURN_RIGHT; // 다음 칸을 위해 우회전
                        }
                        stop_timer = 0; // 타이머 리셋
                    }
                } else {
                    stop_timer = 0; // 조건 불만족 시 타이머 리셋
                }
                break;
                
            case TURN_RIGHT:
                // 우회전 로직
                setMotorSpeed(0, 0); delay(50); 

                digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  // 왼쪽 앞으로
                digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); // 오른쪽 뒤로
                int turn_pwm = 160;
                analogWrite(LEFT_PWM, turn_pwm);
                analogWrite(RIGHT_PWM, turn_pwm);
                
                delay(650); // 90도 회전 튜닝 값
                
                analogWrite(LEFT_PWM, 0); analogWrite(RIGHT_PWM, 0); // 모터 정지
                delay(50); 

                // 우회전 후 다음 칸으로 재시작
                currentState = GO_STRAIGHT; 
                break;
                
            case STOPPED_FINAL:
                setMotorSpeed(0, 0); // 모터 정지 유지
                break;
        }
    }
}