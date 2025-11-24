#include <Arduino.h>
#line 1 "/Users/woosang/Downloads/ek1123/ek1124/ek1124.ino"
#include "SMCController.h"

// 핀 설정
#define LEFT_PWM 5
#define RIGHT_PWM 6
#define IN1 3
#define IN2 4
#define IN3 11
#define IN4 12
#define TRIG_FRONT A0
#define ECHO_FRONT A1
#define START_BTN 2

// 파라미터
#define MAX_SPEED 140
#define MIN_EFFECTIVE_PWM 120
#define MAZE_CELL_SIZE 20.0
#define REF_DISTANCE 7.0
#define DECEL_MARGIN 4.0
#define TOTAL_CELLS 2
const float LOOP_TIME = 0.05;

const float SMC_K = 4.0;
const float SMC_LAMBDA = 0.2;
const float SMC_PHI = 0.5;
const float LPF_TAU = 0.04;
const float SENSOR_SPIKE_THRESHOLD = 5.0;
const float FILTER_TAU = 0.035;
const float MIN_VALID_DISTANCE = 2.0;
const float MAX_VALID_DISTANCE = 120.0;

// 센서 필터 변수
float med_buf[3] = {100.0, 100.0, 100.0};
float prev_med = 100.0;
float prev_lpf = 100.0;

// 상태 변수 및 SMC 객체
enum RobotState { WAITING_TO_START, GO_STRAIGHT, DECELERATE_AND_STOP, TURN_RIGHT_AND_STOP, STOPPED_FINAL };
RobotState currentState = WAITING_TO_START;

unsigned long last_loop_time = 0;
unsigned long stop_timer = 0;
int completed_cells = 0;

float movement_start_distance = 0;
float target_distance_cm = 0;
float decel_start_distance = 0;

SMCController smc(SMC_K, SMC_LAMBDA, SMC_PHI, LOOP_TIME, LPF_TAU);

// 함수 선언
void setMotorSpeed(int left, int right);
float readDistance();
float readDistanceRaw();
float median3(float x);
float lowpass_simple(float raw, float prev, float dt, float tau);

#line 58 "/Users/woosang/Downloads/ek1123/ek1124/ek1124.ino"
void setup();
#line 73 "/Users/woosang/Downloads/ek1123/ek1124/ek1124.ino"
void loop();
#line 58 "/Users/woosang/Downloads/ek1123/ek1124/ek1124.ino"
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(TRIG_FRONT, OUTPUT);
  pinMode(ECHO_FRONT, INPUT);
  pinMode(START_BTN, INPUT_PULLUP);
  setMotorSpeed(0, 0);
  delay(500);
}

void loop() {
  switch(currentState) {
    case WAITING_TO_START:
      if (digitalRead(START_BTN) == LOW) {
        delay(50);
        if (digitalRead(START_BTN) == LOW) {
          // 센서값 실시간 체크
          movement_start_distance = readDistance();
          Serial.print("Start Dist: "); Serial.println(movement_start_distance, 2);

          // 목표 거리 계산 (무조건 시작점-20cm 또는 최소 REF_DISTANCE)
          target_distance_cm = movement_start_distance - MAZE_CELL_SIZE;
          if (target_distance_cm < REF_DISTANCE) target_distance_cm = REF_DISTANCE;

          decel_start_distance = target_distance_cm + DECEL_MARGIN;
          Serial.print("Target: "); Serial.println(target_distance_cm, 2);
          Serial.print("Decel Start: "); Serial.println(decel_start_distance, 2);

          currentState = GO_STRAIGHT;
        }
      }
      break;

    case GO_STRAIGHT:
      if (millis() - last_loop_time >= (unsigned long)(LOOP_TIME * 1000)) {
        last_loop_time = millis();
        float dist = readDistance();
        Serial.print("GO STR - Dist: "); Serial.println(dist, 2);

        // decel_start_distance 이하로 떨어지면 감속 전환
        if (dist <= decel_start_distance) {
          currentState = DECELERATE_AND_STOP;
        }
      }
      setMotorSpeed(MAX_SPEED, MAX_SPEED);
      break;

    case DECELERATE_AND_STOP:
      if (millis() - last_loop_time >= (unsigned long)(LOOP_TIME * 1000)) {
        last_loop_time = millis();
        float dist = readDistance();
        float u = smc.update(target_distance_cm, dist);

        float filtered_dist = smc.getFilteredDistance();
        float error = smc.getError();
        float error_rate = smc.getFilteredErrorRate();
        float s_value = smc.getSlidingSurface();

        float normalized_u = constrain(u / SMC_K, -1.0, 1.0);
        int pwm = (int)constrain(MAX_SPEED * (1.0 + normalized_u) / 2.0, 0, MAX_SPEED);
        if (pwm < MIN_EFFECTIVE_PWM && pwm > 0) pwm = MIN_EFFECTIVE_PWM;
        setMotorSpeed(pwm, pwm);

        // 실시간 상태 전체 출력
        Serial.print("DECEL - FiltDist: "); Serial.print(filtered_dist, 2);
        Serial.print(", Target: "); Serial.print(target_distance_cm, 2);
        Serial.print(", Error: "); Serial.print(error, 2);
        Serial.print(", PWM: "); Serial.print(pwm);
        Serial.print(", Cells: "); Serial.println(completed_cells);

        // 정지판단 (오차 2cm로 완화, 실물에 맞게 조정)
        if (abs(filtered_dist - target_distance_cm) <= 2.0 && pwm <= MIN_EFFECTIVE_PWM) {
          if (stop_timer == 0) {
            stop_timer = millis();
          } else if (millis() - stop_timer >= 200) {
            setMotorSpeed(0, 0);
            completed_cells++;
            stop_timer = 0;
            if (completed_cells >= TOTAL_CELLS) {
              currentState = TURN_RIGHT_AND_STOP;
            } else {
              movement_start_distance = readDistance();
              target_distance_cm = movement_start_distance - MAZE_CELL_SIZE;
              if (target_distance_cm < REF_DISTANCE) target_distance_cm = REF_DISTANCE;
              decel_start_distance = target_distance_cm + DECEL_MARGIN;
              Serial.print("Next Start Dist: "); Serial.println(movement_start_distance, 2);
              Serial.print("Next Target: "); Serial.println(target_distance_cm, 2);
              Serial.print("Next Decel Start: "); Serial.println(decel_start_distance, 2);
              currentState = GO_STRAIGHT;
            }
          }
        } else {
          stop_timer = 0;
        }
      }
      break;

    case TURN_RIGHT_AND_STOP:
      setMotorSpeed(-100, 100);
      delay(650);
      setMotorSpeed(0, 0);
      currentState = STOPPED_FINAL;
      break;

    case STOPPED_FINAL:
      setMotorSpeed(0, 0);
      delay(5000);
      break;
  }
}

// ======== 유틸 함수들 ========
void setMotorSpeed(int left, int right) {
  if (left > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(LEFT_PWM, left); }
  else if (left < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(LEFT_PWM, -left); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(LEFT_PWM, 0); }
  if (right > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(RIGHT_PWM, right); }
  else if (right < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(RIGHT_PWM, -right); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(RIGHT_PWM, 0); }
}

float readDistanceRaw() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);

  long duration = pulseIn(ECHO_FRONT, HIGH, 30000);
  if (duration == 0) return MAX_VALID_DISTANCE;
  float dist = duration * 0.0343 / 2.0;
  if (dist < MIN_VALID_DISTANCE) dist = MIN_VALID_DISTANCE;
  if (dist > MAX_VALID_DISTANCE) dist = MAX_VALID_DISTANCE;
  return dist;
}

float median3(float x) {
  med_buf[0] = med_buf[1]; med_buf[1] = med_buf[2]; med_buf[2] = x;
  float a = med_buf[0], b = med_buf[1], c = med_buf[2];
  if ((a <= b && b <= c) || (c <= b && b <= a)) return b;
  if ((b <= a && a <= c) || (c <= a && a <= b)) return a;
  return c;
}

float lowpass_simple(float raw, float prev, float dt, float tau) {
  float alpha = dt / (tau + dt);
  return alpha * raw + (1.0 - alpha) * prev;
}

float readDistance() {
  float raw = readDistanceRaw();
  float med = median3(raw);
  if (fabs(med - prev_med) > SENSOR_SPIKE_THRESHOLD) med = prev_med;
  prev_med = med;
  float filtered = lowpass_simple(med, prev_lpf, LOOP_TIME, FILTER_TAU);
  prev_lpf = filtered;
  return filtered;
}

