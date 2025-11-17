#ifndef SMC_CONTROLLER_H
#define SMC_CONTROLLER_H

#include <Arduino.h>

class SMCController {
private:
    float lambda;
    float k;
    float epsilon;
    float Ts;
    float prev_error;

    float sat(float s);

public:
    SMCController(float lambda_, float k_, float epsilon_, float Ts_);

    float update(float error);
};

#endif
