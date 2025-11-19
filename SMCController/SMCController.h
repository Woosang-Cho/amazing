#ifndef SMCController_h
#define SMCController_h

#include <Arduino.h>

class SMCController {
  public:
    SMCController(float K_, float lambda_, float phi_, float dt_);

    float update(float reference, float measured);
    
    // 현재 오차 미분값(e_dot)을 반환
    float get_e_dot() const; 
    
    // 현재 슬라이딩 변수(s)를 반환
    float get_s() const;     

  private:
    float K;
    float lambda;
    float phi;
    float dt;
    
    // 이전 오차 (미분값 계산용)
    float prev_error; 
    
    // 현재 계산된 오차 미분값 저장
    float current_e_dot; 
    
    // 현재 계산된 슬라이딩 변수 값 저장
    float current_s;     
};

#endif