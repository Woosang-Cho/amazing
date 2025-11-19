#include "SMCController.h"
#include <math.h>  // tanh 함수를 위해 추가 (챠터링 완화)

// LPF tau 설명: 값이 작을수록 alpha = dt/(tau+dt) 크고 필터링 약해지지만 노이즈 통과 증가.
// 응답 속도 vs 노이즈 trade-off: 0.01~0.05s 튜닝, Serial로 런타임 변경 

SMCController::SMCController(float K_, float lambda_, float phi_, float dt_, float tau_)
  : K(K_), lambda(lambda_), phi(phi_), dt(dt_), tau(tau_),
    prev_error(0.0), current_e_dot(0.0), current_s(0.0) {
    // Optional: 초기 prev_error를 실제 첫 measured로 set하려면 main.ino에서 update 전 초기화 호출
}

float SMCController::update(float reference, float measured) {
    float e = reference - measured;  // e <0: 목표 초과 (종방향: 벽 멀음)
    
    // 1. raw 오차 미분값 계산
    float raw_e_dot = (e - prev_error) / dt; 

    // 2. LPF 계수 계산
    const float ALPHA = dt / (tau + dt); 
    
    // 3. LPF 적용: current_e_dot 업데이트
    current_e_dot = ALPHA * raw_e_dot + (1.0 - ALPHA) * current_e_dot; 

    // 4. 슬라이딩 변수 s 계산
    current_s = e + lambda * current_e_dot; 

    // 5. sat_s 계산: tanh로 smoother (챠터링 완화)
    float sat_s = tanh(current_s / phi);  // -1 ~ 1 범위
    
    // Optional: 동적 phi (e_dot 클 때 phi 증가, 과도 응답 완화)
    // float dynamic_phi = phi + 0.1 * fabs(current_e_dot);
    // float sat_s = tanh(current_s / dynamic_phi);

    // 6. 제어 입력 u: -K * sat_s로 부호 맞춤 (e<0 → s<0 → sat_s<0 → u>0: 속도 증가/오른쪽 턴)
    float u = -K * sat_s;

    // 이전 오차 업데이트
    prev_error = e;

    return u;
}

float SMCController::get_e_dot() const {
    return current_e_dot;
}

float SMCController::get_s() const {
    return current_s;
}