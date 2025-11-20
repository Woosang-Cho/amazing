#ifndef SMCController_h
#define SMCController_h

#include <Arduino.h>

// SMC/LPF 상태 변수 선언 (main.ino에서 시리얼 출력을 위해 사용)
// SMCController.cpp에 실제 메모리 할당(정의)이 되어야 링커 오류가 발생하지 않습니다.
extern float last_dist_lpf;
extern float last_error;
extern float last_e_dot_lpf;
extern float last_dist_raw; 
extern float last_s_value;

class SMCController {
  public:
    // 생성자: SMC 파라미터 및 LPF 시정수 초기화
    // K: 제어 gain
    // lambda: sliding surface 기울기 (C 역할)
    // phi: boundary layer 너비
    // dt: 샘플링 시간 (e.g., 0.05s)
    // tau: LPF 시정수 (e.g., 0.040s)
    SMCController(float K_, float lambda_, float phi_, float dt_, float tau_);

    // 업데이트 함수: reference(목표)와 measured(측정 Raw 값) 입력으로 제어 u 반환
    float update(float reference, float measured);
    
    // 현재 오차 미분값(e_dot) getter (LPF 적용 후 값)
    float get_e_dot() const; 
    
    // 현재 슬라이딩 변수(s) getter
    float get_s() const;     

  private:
    float K;
    float lambda;
    float phi;
    float dt;
    float tau;
    
    // 이전 오차 (e_dot 계산용)
    float prev_error; 
    
    // 현재 LPF 적용 e_dot 및 s 저장 (getter 반환용)
    float current_e_dot; 
    float current_s;     
};

#endif