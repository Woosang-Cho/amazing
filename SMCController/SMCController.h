#ifndef SMC_CONTROLLER_H
#define SMC_CONTROLLER_H

class SMCController {
public:
    // 생성자: 초기 파라미터 설정
    SMCController(float k_, float phi_, float c1_, float c2_, float dt_);

    // 파라미터를 런타임에 다시 설정하고 싶을 때 사용하는 함수
    void setParameters(float lambda_, float K_, float phi_, float dt_);

    // 제어입력 계산
    float update(float reference, float measured);

private:
    // 제어 파라미터
    float k;        // 게인 K
    float phi;      // 경계층 두께 (saturation 경계)
    float c1;       // 슬라이딩면 계수 1
    float c2;       // 슬라이딩면 계수 2
    float dt;       // 샘플링 시간
    float lambda;   // 필요하다면 사용하는 파라미터 (예: s = e + lambda * e_dot 등)

    float prev_error;   // 이전 오차 (e_{k-1})
};

#endif
