#ifndef SMCController_h
#define SMCController_h

#include <Arduino.h>

class SMCController {
  public:
    // 생성자: SMC 파라미터 초기화
    // K: 제어 gain (크면 강한 제어, but 챠터링 증가)
    // lambda: sliding surface 기울기 (미분 항 강조, 0.5~2.0 튜닝)
    // phi: boundary layer 너비 (tanh에서 steepness 조절, 챠터링 완화; 1.0~3.0 추천)
    // dt: 샘플링 시간 (제어 루프 주기, e.g., 0.05s)
    // tau: LPF 시정수 (작을수록 alpha 크고 필터링 약함, 노이즈 통과 but 응답 빠름; 0.01~0.05s)
    SMCController(float K_, float lambda_, float phi_, float dt_, float tau_);

    // 업데이트 함수: reference(목표)와 measured(측정값) 입력으로 제어 u 반환
    // 주의: e = reference - measured로 정의 (종방향 예: ref=6cm, measured=10cm(멀음) 시 e=-4 <0 → s <0 → u <0 but 의도(멀면 속도 증가 u>0)와 mismatch 가능
    //       .cpp에서 u = -K * tanh(s/phi)로 부호 조정 추천 (분석 결과 참조)
    // 종방향 예: 벽 멀 때 (measured > ref, e <0) u >0 목표로 구현 조정 필요
    // 횡방향 예: errorLat = left - right >0 (왼쪽 멀음) 시 u >0 (오른쪽 턴)
    float update(float reference, float measured);
    
    // 현재 오차 미분값(e_dot) getter (LPF 적용 후 값, 노이즈 완화 확인용)
    float get_e_dot() const; 
    
    // 현재 슬라이딩 변수(s) getter (s→0 수렴 모니터링용)
    float get_s() const;     

  private:
    float K;
    float lambda;
    float phi;
    float dt;
    float tau;
    
    // 이전 오차 (e_dot 계산용, 초기화 취약성 방지: constructor에서 set 추천)
    float prev_error; 
    
    // 현재 LPF 적용 e_dot 저장
    float current_e_dot; 
    
    // 현재 s 저장
    float current_s;     
};

#endif