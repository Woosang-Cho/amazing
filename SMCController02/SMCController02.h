// SMCController02.h
#ifndef SMCController_h
#define SMCController_h

#include <Arduino.h>

// SMC/LPF state variables for serial output in main.ino
extern float last_dist_lpf;
extern float last_error;
extern float last_e_dot_lpf;
extern float last_dist_raw; 
extern float last_s_value;

class SMCController {
  public:
    // Constructor: Initialize SMC parameters and LPF time constant
    // K: control gain
    // lambda: sliding surface slope
    // phi: boundary layer width
    // dt: sampling time (e.g., 0.05s)
    // tau: LPF time constant (e.g., 0.040s)
    SMCController(float K_, float lambda_, float phi_, float dt_, float tau_);

    // Update function: returns control signal u
    float update(float reference, float measured);
    
    // Getter functions for internal states
    float getFilteredDistance() const;
    float getError() const;
    float getFilteredErrorRate() const;
    float getSlidingSurface() const;

  private:
    float K;
    float lambda;
    float phi;
    float dt;
    float tau;
    
    float prev_error; 
    float current_e_dot; 
    float current_s;     
};

#endif
