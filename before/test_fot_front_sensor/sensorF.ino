#include <Arduino.h>
#include "SMCController.h"

// ========================
// 1. 핀 설정
// ========================
#define LEFT_PWM 5
#define RIGHT_PWM 6
#define IN1 3
#define IN2 4
#define IN3 11
#define IN4 12

#define TRIG_FRONT A0
#define ECHO_FRONT A1
#define START_BTN 2

// ========================
// 2. SMC & 제어 상수
// ========================
const float LOOP_TIME = 0.05;       // 50ms
const int MAX_SPEED = 160;
const int MIN_EFFECTIVE_PWM = 50;   // 최소 유효 PWM

const float MAZE_CELL_SIZE = 20.0;  
const float REF_DISTANCE = 7.0;     
const float DECEL_DIST = 15.0;      

const float SMC_K = 4.0;
const float SMC_LAMBDA = 0.2;
const float SMC_PHI = 0.5;
const float LPF_TAU = 0.04;

SMCController smc(SMC_K, SMC_LAMBDA, SMC_PHI, LOOP_TIME, LPF_TAU);

// ========================
// 3. 상태 및 변수
// ========================
enum RobotState { WAITING_TO_START, GO_STRAIGHT, DECELERATE_AND_STOP, TURN_RIGHT, STOPPED_FINAL };
RobotState currentState = WAITING_TO_START;

unsigned long last_loop_time = 0;
unsigned long stop_timer = 0;

int path_count = 0;
int target_cells = 0;

// ========================
// 4. 센서 읽기
// ========================
extern float last_dist_raw;
extern float last_dist_lpf;
extern float last_error;
extern float last_e_dot_lpf;
extern float last_s_value;

float readDistance() {
    digitalWrite(TRIG_FRONT, LOW); delayMicroseconds(2);
    digitalWrite(TRIG_FRONT, HIGH); delayMicroseconds(10);
    digitalWrite(TRIG_FRONT, LOW);
    delayMicroseconds(10);

    long duration = pulseIn(ECHO_FRONT, HIGH, 80000);
    if (duration == 0) return last_dist_raw;

    float dist = duration * 0.0343 / 2.0;
    if (dist > 120.0) dist = 120.0;
    return dist;
}

// ========================
// 5. 모터 제어
// ========================
void setMotorSpeed(int left_pwm, int right_pwm) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(LEFT_PWM, constrain(left_pwm, 0, 255));
    analogWrite(RIGHT_PWM, constrain(right_pwm, 0, 255));
}

// ========================
// 6. Setup
// ========================
void setup() {
    pinMode(TRIG_FRONT, OUTPUT); pinMode(ECHO_FRONT, INPUT);
    pinMode(LEFT_PWM, OUTPUT); pinMode(RIGHT_PWM, OUTPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(START_BTN, INPUT_PULLUP);

    Serial.begin(115200);
    float init_dist = readDistance();
    last_dist_raw = last_dist_lpf = init_dist;
    last_loop_time = millis();

    Serial.println("time_sec,raw_dist_cm,lpf_dist_cm,error_cm,e_dot_lpf_cms,s_value,target_distance_cm,state");
}

// ========================
// 7. Loop
// ========================
void loop() {
    if (currentState == WAITING_TO_START) {
        if (digitalRead(START_BTN) == LOW) {
            delay(50);
            if (digitalRead(START_BTN) == LOW) {
                currentState = GO_STRAIGHT;
                path_count = 0;
                target_cells = 2;  // 시작 예시
                last_loop_time = millis();
            }
        }
        return;
    }

    if (millis() - last_loop_time >= (unsigned long)(LOOP_TIME * 1000)) {
        last_loop_time = millis();
        float measured = readDistance();
        float target_distance_cm = MAZE_CELL_SIZE;  // 한 칸씩 이동
        float u = smc.update(target_distance_cm, measured);

        // 시리얼 출력
        Serial.print(millis() / 1000.0); Serial.print(",");
        Serial.print(last_dist_raw); Serial.print(",");
        Serial.print(last_dist_lpf); Serial.print(",");
        Serial.print(last_error); Serial.print(",");
        Serial.print(last_e_dot_lpf); Serial.print(",");
        Serial.print(last_s_value); Serial.print(",");
        Serial.print(target_distance_cm); Serial.print(",");
        switch(currentState){
            case GO_STRAIGHT: Serial.println("GO"); break;
            case DECELERATE_AND_STOP: Serial.println("SMC"); break;
            case TURN_RIGHT: Serial.println("TURN"); break;
            case STOPPED_FINAL: Serial.println("FINAL"); break;
            default: Serial.println("WAIT"); break;
        }

        // 상태 머신
        switch(currentState){
            case GO_STRAIGHT:
                setMotorSpeed(MAX_SPEED, MAX_SPEED);
                if(last_dist_lpf <= DECEL_DIST) currentState = DECELERATE_AND_STOP;
                break;

            case DECELERATE_AND_STOP:
            {
                int pwm = (int)constrain(MAX_SPEED*(1.0+u)/2.0, 0, MAX_SPEED);
                if(pwm < MIN_EFFECTIVE_PWM && pwm > 0) pwm = MIN_EFFECTIVE_PWM;
                setMotorSpeed(pwm, pwm);

                if(last_dist_lpf <= target_distance_cm + REF_DISTANCE && pwm <= MIN_EFFECTIVE_PWM){
                    if(stop_timer==0) stop_timer=millis();
                    else if(millis()-stop_timer>=150){
                        setMotorSpeed(0,0);
                        path_count++;
                        target_cells--;
                        if(target_cells <= 0) currentState=STOPPED_FINAL;
                        else currentState=TURN_RIGHT;
                        stop_timer=0;
                    }
                } else stop_timer=0;
                break;
            }

            case TURN_RIGHT:
                setMotorSpeed(0,0); delay(50);
                digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);
                digitalWrite(IN3,LOW); digitalWrite(IN4,HIGH);
                analogWrite(LEFT_PWM, MAX_SPEED); analogWrite(RIGHT_PWM, MAX_SPEED);
                delay(650);
                analogWrite(LEFT_PWM,0); analogWrite(RIGHT_PWM,0);
                delay(50);
                currentState=GO_STRAIGHT;
                break;

            case STOPPED_FINAL:
                setMotorSpeed(0,0);
                break;
        }
    }
}
