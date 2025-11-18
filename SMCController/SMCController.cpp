#include "SMCController.h"

SMCController::SMCController(float k_, float phi_, float c1_, float c2_, float dt_)
    : k(k_),
      phi(phi_),
      c1(c1_),
      c2(c2_),
      dt(dt_),
      lambda(0.0),     
      prev_error(0.0) {}

void SMCController::setParameters(float lambda_, float K_, float phi_, float dt_) {
    lambda = lambda_;
    k      = K_;
    phi    = phi_;
    dt     = dt_;
    // 필요하면 c1, c2도 이 함수에서 바꾸도록 인터페이스를 확장할 수 있음
    // 예: c1 = c1_; c2 = c2_;
}

// 제어 입력 계산 함수
float SMCController::update(float reference, float measured) {
    // 오차 및 오차 미분
    float e     = reference - measured;
    float e_dot = (e - prev_error) / dt;

    // 슬라이딩 면 s (현재는 c1, c2 사용)
    float s = c1 * e + c2 * e_dot;
    // 만약 lambda를 쓰고 싶다면 예를 들어:
    // float s = e + lambda * e_dot;

    // saturation(s/phi)
    float sat_s;
    if (s > phi)      sat_s =  1.0;
    else if (s < -phi) sat_s = -1.0;
    else               sat_s =  s / phi;

    // 제어 입력 u
    float u = -k * sat_s;

    // 상태 업데이트
    prev_error = e;

    return u;
}
