#include "SMCController.h"

// LPF tau
// 값이 작을수록 필터링이 강해져 노이즈가 줄지만, 응답 속도가 느려짐
// 0.01s ~ 0.05s 사이에서 튜닝 고민
const float TAU = 0.01; 

SMCController::SMCController(float K_, float lambda_, float phi_, float dt_)
  : K(K_), lambda(lambda_), phi(phi_), dt(dt_), prev_error(0.0), 
    current_e_dot(0.0), current_s(0.0) {}

float SMCController::update(float reference, float measured) {
    float e = reference - measured;
    
    // 1. 오차 미분값 (e_dot) 계산
    float raw_e_dot = (e - prev_error) / dt; 

    // 2. LPF 계수 (alpha) 계산
    const float ALPHA = dt / (TAU + dt); 
    
    // 3. LPF 적용 (filtered_e_dot = alpha * raw_e_dot + (1 - alpha) * current_e_dot)
    // current_e_dot은 여기서 이전 필터링 결과(y[k-1])의 역할
    current_e_dot = ALPHA * raw_e_dot + (1.0 - ALPHA) * current_e_dot; 

    current_s = e + lambda * current_e_dot; 

    float sat_s;
    if (current_s > phi) sat_s = 1.0;
    else if (current_s < -phi) sat_s = -1.0;
    else sat_s = current_s / phi;

    float u = -K * sat_s;

    prev_error = e;

    return u;
}

float SMCController::get_e_dot() const {
    return current_e_dot;
}

float SMCController::get_s() const {
    return current_s;
}