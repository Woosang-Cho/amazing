#ifndef SMCController_h
#define SMCController_h

#include <Arduino.h>

class SMCController {
public:
    SMCController(float k, float phi, float c1, float c2, float dt = 0.01);

    float update(float reference, float measured);

private:
    float k;       // switching gain
    float phi;     // boundary layer thickness
    float c1, c2;  // sliding surface coefficients
    float dt;

    float prev_error;
};

#endif
