#ifndef SMCController_h
#define SMCController_h

#include <Arduino.h>

class SMCController {
  public:
    SMCController(float K_, float lambda_, float phi_, float dt_);
    float update(float reference, float measured);

  private:
    float K;
    float lambda;
    float phi;
    float dt;
    float prev_error;
};

#endif