// sensorF.ino - Modified for relative distance control
#include "SMCController.h"

// ========================
// 1. Pin Configuration
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
// 2. SMC & Control Constants
// ========================
const float LOOP_TIME = 0.05; // 50ms
const int MAX_SPEED = 160;
const int MIN_EFFECTIVE_PWM = 50;
const float MAZE_CELL_SIZE = 20.0; // cm to advance per move
const float REF_DISTANCE = 7.0; // minimum safe distance from wall
const float DECEL_DIST = 15.0; // distance to start deceleration

const float SMC_K = 4.0;
const float SMC_LAMBDA = 0.2;
const float SMC_PHI = 0.5;
const float LPF_TAU = 0.04;

SMCController smc(SMC_K, SMC_LAMBDA, SMC_PHI, LOOP_TIME, LPF_TAU);

// ========================
// 3. State Machine & Variables
// ========================
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

int path_count = 0;
int target_cells = 3;

// Relative distance control variables
float movement_start_distance = 0;
bool distance_initialized = false;

// ========================
// 4. Function Declarations
// ========================
void setMotorSpeed(int left, int right);
float readDistance();

// ========================
// 5. Setup
// ========================
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

// ========================
// 6. Main Loop
// ========================
void loop() {
  switch(currentState) {
    
    case WAITING_TO_START:
      if (digitalRead(START_BTN) == LOW) {
        delay(50);
        if (digitalRead(START_BTN) == LOW) {
          movement_start_distance = readDistance();
          distance_initialized = true;
          currentState = GO_STRAIGHT;
        }
      }
      break;
      
    case GO_STRAIGHT:
      setMotorSpeed(MAX_SPEED, MAX_SPEED);
      if (last_dist_lpf <= DECEL_DIST) {
        currentState = DECELERATE_AND_STOP;
      }
      break;
      
    case DECELERATE_AND_STOP: {
      float target_distance_cm = movement_start_distance - MAZE_CELL_SIZE;
      
      if (target_distance_cm < REF_DISTANCE) {
        target_distance_cm = REF_DISTANCE;
      }
      
      if (millis() - last_loop_time >= (unsigned long)(LOOP_TIME * 1000)) {
        last_loop_time = millis();
        
        last_dist_raw = readDistance();
        
        // FIXED: Pass single value directly (not array)
        float u = smc.update(target_distance_cm, last_dist_raw);
        
        // Get internal states from SMC using getter functions
        last_dist_lpf = smc.getFilteredDistance();
        last_error = smc.getError();
        last_e_dot_lpf = smc.getFilteredErrorRate();
        last_s_value = smc.getSlidingSurface();
        
        int pwm = (int)constrain(MAX_SPEED * (1.0 + u) / 2.0, 0, MAX_SPEED);
        
        if (pwm < MIN_EFFECTIVE_PWM && pwm > 0) {
          pwm = MIN_EFFECTIVE_PWM;
        }
        
        setMotorSpeed(pwm, pwm);
        
        Serial.print(millis() / 1000.0, 3); Serial.print(",");
        Serial.print(last_dist_raw, 2); Serial.print(",");
        Serial.print(last_dist_lpf, 2); Serial.print(",");
        Serial.print(last_error, 2); Serial.print(",");
        Serial.print(last_e_dot_lpf, 2); Serial.print(",");
        Serial.print(last_s_value, 3); Serial.print(",");
        Serial.print(target_distance_cm, 2); Serial.print(",");
        Serial.println("DECEL");
        
        if (abs(last_dist_lpf - target_distance_cm) <= 1.0 && 
            abs(last_e_dot_lpf) < 0.5 && 
            pwm <= MIN_EFFECTIVE_PWM) {
          
          if (stop_timer == 0) {
            stop_timer = millis();
          } else if (millis() - stop_timer >= 200) {
            setMotorSpeed(0, 0);
            
            path_count++;
            target_cells--;
            distance_initialized = false;
            
            if (target_cells <= 0) {
              currentState = STOPPED_FINAL;
            } else {
              currentState = TURN_RIGHT;
            }
            stop_timer = 0;
          }
        } else {
          stop_timer = 0;
        }
      }
      break;
    }
      
    case TURN_RIGHT:
      setMotorSpeed(-100, 100);
      delay(650);
      setMotorSpeed(0, 0);
      delay(100);
      
      movement_start_distance = readDistance();
      distance_initialized = true;
      currentState = GO_STRAIGHT;
      break;
      
    case STOPPED_FINAL:
      setMotorSpeed(0, 0);
      delay(5000);
      break;
  }
  
  if (currentState == GO_STRAIGHT) {
    if (millis() - last_loop_time >= (unsigned long)(LOOP_TIME * 1000)) {
      last_loop_time = millis();
      
      last_dist_raw = readDistance();
      
      float alpha = LOOP_TIME / (LPF_TAU + LOOP_TIME);
      last_dist_lpf = alpha * last_dist_raw + (1 - alpha) * last_dist_lpf;
      
      Serial.print(millis() / 1000.0, 3); Serial.print(",");
      Serial.print(last_dist_raw, 2); Serial.print(",");
      Serial.print(last_dist_lpf, 2); Serial.print(",");
      Serial.print(0.0); Serial.print(",");
      Serial.print(0.0); Serial.print(",");
      Serial.print(0.0); Serial.print(",");
      Serial.print(0.0); Serial.print(",");
      Serial.println("STRAIGHT");
    }
  }
}

// ========================
// 7. Motor Control Function
// ========================
void setMotorSpeed(int left, int right) {
  if (left > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(LEFT_PWM, left);
  } else if (left < 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(LEFT_PWM, -left);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    analogWrite(LEFT_PWM, 0);
  }
  
  if (right > 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(RIGHT_PWM, right);
  } else if (right < 0) {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(RIGHT_PWM, -right);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    analogWrite(RIGHT_PWM, 0);
  }
}

// ========================
// 8. Ultrasonic Sensor Function
// ========================
float readDistance() {
  digitalWrite(TRIG_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_FRONT, LOW);
  
  long duration = pulseIn(ECHO_FRONT, HIGH, 30000);
  
  if (duration == 0) {
    return last_dist_raw;
  }
  
  float dist = duration * 0.0343 / 2.0;
  
  if (dist < 2.0) dist = 2.0;
  if (dist > 120.0) dist = 120.0;
  
  return dist;
}
