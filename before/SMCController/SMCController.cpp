#include "SMCController.h"
#include <math.h> // tanh 함수 사용을 위해 포함

// ==========================================
// SMC/LPF 상태 변수 정의 (링커 오류 방지)
// ==========================================
float last_dist_lpf = 0.0;
float last_error = 0.0;
float last_e_dot_lpf = 0.0;
float last_dist_raw = 0.0; 
float last_s_value = 0.0;

// ==========================================
// 유틸리티 함수
// ==========================================

// LPF 적용 함수
float applyLPF(float raw_value, float last_lpf_value, float dt, float tau) {
    // ALPHA = DT / (TAU + DT)
    const float ALPHA = dt / (tau + dt); 
    return ALPHA * raw_value + (1.0 - ALPHA) * last_lpf_value;
}

// Chattering 감소를 위한 tanh 함수 기반 포화 함수 (Saturation function)
float sgn_sat(float val, float phi) {
    return tanh(val / phi); 
}

// ==========================================
// SMCController 구현
// ==========================================

SMCController::SMCController(float K_, float lambda_, float phi_, float dt_, float tau_) 
    : K(K_), lambda(lambda_), phi(phi_), dt(dt_), tau(tau_) {
    
    // 초기화
    prev_error = 0.0; 
    current_e_dot = 0.0; 
    current_s = 0.0;
}

float SMCController::update(float reference, float measured_raw) {
    // 0. Raw 거리 저장
    last_dist_raw = measured_raw; 

    // 1. Distance LPF 적용 (tau=0.040s 사용)
    float dist_lpf = applyLPF(measured_raw, last_dist_lpf, dt, tau);
    last_dist_lpf = dist_lpf;
    
    // 2. 오차 계산: e = 측정값 - 목표값
    float error = dist_lpf - reference;
    last_error = error;
    
    // 3. 오차 미분값 (e_dot) 계산 및 LPF 적용
    float raw_e_dot = (error - prev_error) / dt;
    float e_dot_lpf = applyLPF(raw_e_dot, last_e_dot_lpf, dt, tau);
    last_e_dot_lpf = e_dot_lpf;
    current_e_dot = e_dot_lpf;
    
    // 4. 슬라이딩 표면 계산: s = e_dot + lambda * e
    float s = e_dot_lpf + lambda * error;
    current_s = s;
    last_s_value = s;

    // 5. SMC 제어 신호 U (감속 정지 목표)
    // U = -K * tanh(s / phi) : 
    //   - e > 0 (벽에서 멀리 있음), e_dot < 0 (벽으로 접근 중)
    //   - s < 0 일 때 (슬라이딩 표면 아래), U > 0 (가속해야 함)
    //   - s > 0 일 때 (슬라이딩 표면 위), U < 0 (감속해야 함)
    // 이 부호는 감속 정지 목표에 맞으며, main.ino에서 속도 계산에 사용됩니다.
    float control_U = -K * sgn_sat(s, phi); 

    // 6. 다음 루프를 위한 값 저장
    prev_error = error;

    return control_U;
}

// Getter 함수 구현
float SMCController::get_e_dot() const {
    return current_e_dot; 
}
float SMCController::get_s() const {
    return current_s;     
}