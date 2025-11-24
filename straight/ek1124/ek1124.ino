#include "SMCController.h"

// 핀/파라미터 설정
#define LEFT_PWM 5
#define RIGHT_PWM 6
#define IN1 3
#define IN2 4
#define IN3 11
#define IN4 12
#define TRIG_FRONT A0
#define ECHO_FRONT A1
#define START_BTN 2

#define MAX_SPEED 140
#define MIN_EFFECTIVE_PWM 120
#define MAZE_CELL_SIZE 20.0
#define MIN_STOP_DIST 7.0
#define DECEL_MARGIN 4.0
#define TOTAL_CELLS 2

const float LOOP_TIME = 0.05;
const float SMC_K = 4.0;
const float SMC_LAMBDA = 0.2;
const float SMC_PHI = 0.5;
const float FILTER_TAU = 0.035;
const float SENSOR_SPIKE_THRESHOLD = 5.0;
const float MIN_VALID_DISTANCE = 2.0;
const float MAX_VALID_DISTANCE = 120.0;

// 센서 필터 변수 (loop 배치, 실시간 갱신)
float med_buf[3];
float prev_med = 80.0;
float prev_lpf = 80.0;

enum RobotState { WAITING_TO_START, GO_STRAIGHT, DECELERATE_AND_STOP, TURN_RIGHT_AND_STOP, STOPPED_FINAL };
RobotState currentState = WAITING_TO_START;
unsigned long last_loop_time = 0;
unsigned long stop_timer = 0;
int completed_cells = 0;

float movement_start_distance = 0;
float target_distance_cm = 0;
float decel_start_distance = 0;

SMCController smc(SMC_K, SMC_LAMBDA, SMC_PHI, LOOP_TIME, FILTER_TAU);

// 데이터 송신 함수
void sendLog(float time_sec, float raw_dist_cm, float lpf_dist_cm,
             float error_cm, float e_dot_lpf_cms, float s_value,
             float target_distance_cm, int state)
{
  Serial.print(time_sec); Serial.print(",");
  Serial.print(raw_dist_cm); Serial.print(",");
  Serial.print(lpf_dist_cm); Serial.print(",");
  Serial.print(error_cm); Serial.print(",");
  Serial.print(e_dot_lpf_cms); Serial.print(",");
  Serial.print(s_value); Serial.print(",");
  Serial.print(target_distance_cm); Serial.print(",");
  Serial.println(state);
}

// 센서 raw 함수
float readDistanceRaw() {
  digitalWrite(TRIG_FRONT, LOW); delayMicroseconds(5);
  digitalWrite(TRIG_FRONT, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW); delayMicroseconds(10);
  long duration = pulseIn(ECHO_FRONT, HIGH, 8000);
  float dist = (duration == 0) ? 80.0 : duration * 0.0343 / 2.0;
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

void setMotorSpeed(int left, int right) {
  if (left > 0) { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(LEFT_PWM, left); }
  else if (left < 0) { digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH); analogWrite(LEFT_PWM, -left); }
  else { digitalWrite(IN1, LOW); digitalWrite(IN2, LOW); analogWrite(LEFT_PWM, 0); }
  if (right > 0) { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(RIGHT_PWM, right); }
  else if (right < 0) { digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH); analogWrite(RIGHT_PWM, -right); }
  else { digitalWrite(IN3, LOW); digitalWrite(IN4, LOW); analogWrite(RIGHT_PWM, 0); }
}

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
  float init_dist = readDistanceRaw();
  med_buf[0] = med_buf[1] = med_buf[2] = prev_med = prev_lpf = init_dist;
  delay(500);
}

void loop() {
  float raw_dist = readDistanceRaw();
  float med = median3(raw_dist);
  if (fabs(med - prev_med) > SENSOR_SPIKE_THRESHOLD) med = prev_med;
  prev_med = med;
  float lpf_dist = lowpass_simple(med, prev_lpf, LOOP_TIME, FILTER_TAU);
  prev_lpf = lpf_dist;

  switch(currentState) {
    case WAITING_TO_START:
      if (digitalRead(START_BTN) == LOW) {
        delay(50);
        if (digitalRead(START_BTN) == LOW) {
          movement_start_distance = lpf_dist;
          target_distance_cm = movement_start_distance - MAZE_CELL_SIZE;
          if (target_distance_cm < MIN_STOP_DIST) target_distance_cm = MIN_STOP_DIST;
          decel_start_distance = target_distance_cm + DECEL_MARGIN;
          currentState = GO_STRAIGHT;
        }
      }
      break;

    case GO_STRAIGHT:
      if (millis() - last_loop_time >= (unsigned long)(LOOP_TIME * 1000)) {
        last_loop_time = millis();
        sendLog(millis()/1000.0, raw_dist, lpf_dist, 0.0, 0.0, 0.0, target_distance_cm, 0);
        if (lpf_dist <= decel_start_distance) {
          currentState = DECELERATE_AND_STOP;
        }
      }
      setMotorSpeed(MAX_SPEED, MAX_SPEED);
      break;

    case DECELERATE_AND_STOP:
      if (millis() - last_loop_time >= (unsigned long)(LOOP_TIME * 1000)) {
        last_loop_time = millis();
        float u = smc.update(target_distance_cm, lpf_dist);

        float error = smc.getError();
        float error_rate = smc.getFilteredErrorRate();
        float s_value = smc.getSlidingSurface();

        float normalized_u = constrain(u / SMC_K, -1.0, 1.0);
        int pwm = (int)constrain(MAX_SPEED * (1.0 + normalized_u) / 2.0, 0, MAX_SPEED);
        if (pwm < MIN_EFFECTIVE_PWM && pwm > 0) pwm = MIN_EFFECTIVE_PWM;
        setMotorSpeed(pwm, pwm);

        sendLog(millis()/1000.0, raw_dist, lpf_dist, error, error_rate, s_value, target_distance_cm, 1);

        if (abs(lpf_dist - target_distance_cm) <= 2.0 && pwm <= MIN_EFFECTIVE_PWM) {
          if (stop_timer == 0) {
            stop_timer = millis();
          } else if (millis() - stop_timer >= 200) {
            setMotorSpeed(0, 0);
            completed_cells++;
            stop_timer = 0;
            if (completed_cells >= TOTAL_CELLS) {
              currentState = TURN_RIGHT_AND_STOP;
            } else {
              movement_start_distance = lpf_dist;
              target_distance_cm = movement_start_distance - MAZE_CELL_SIZE;
              if (target_distance_cm < MIN_STOP_DIST) target_distance_cm = MIN_STOP_DIST;
              decel_start_distance = target_distance_cm + DECEL_MARGIN;
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
