// SMCController02.cpp
#include "SMCController.h"
#include <math.h>

// Global variable definitions
float last_dist_lpf;
float last_error;
float last_e_dot_lpf;
float last_dist_raw; 
float last_s_value;

// Utility function: Low-pass filter
float applyLPF(float raw_value, float last_lpf_value, float dt, float tau) {
    const float ALPHA = dt / (tau + dt); 
    return ALPHA * raw_value + (1.0 - ALPHA) * last_lpf_value;
}

// Utility function: Saturation function using tanh to reduce chattering
float sgn_sat(float val, float phi) {
    return tanh(val / phi); 
}

// Constructor
SMCController::SMCController(float K_, float lambda_, float phi_, float dt_, float tau_) 
    : K(K_), lambda(lambda_), phi(phi_), dt(dt_), tau(tau_) {
    prev_error = 0.0; 
    current_e_dot = 0.0; 
    current_s = 0.0;
}

// Main update function
float SMCController::update(float reference, float measured_raw) {
    last_dist_raw = measured_raw; 

    // 1. Apply LPF to distance measurement
    float dist_lpf = applyLPF(measured_raw, last_dist_lpf, dt, tau);
    last_dist_lpf = dist_lpf;
    
    // 2. Calculate error: e = target - measured
    float error = reference - dist_lpf;
    last_error = error;
    
    // 3. Calculate error derivative and apply LPF
    float raw_e_dot = (error - prev_error) / dt;
    float e_dot_lpf = applyLPF(raw_e_dot, last_e_dot_lpf, dt, tau);
    last_e_dot_lpf = e_dot_lpf;
    current_e_dot = e_dot_lpf;
    
    // 4. Calculate sliding surface: s = e_dot + lambda * e
    float s = e_dot_lpf + lambda * error;
    current_s = s;
    last_s_value = s;

    // 5. SMC control signal: U = -K * tanh(s / phi)
    float control_U = -K * sgn_sat(s, phi); 

    // 6. Save for next iteration
    prev_error = error;

    return control_U;
}

// Getter implementations
float SMCController::getFilteredDistance() const {
    return last_dist_lpf;
}

float SMCController::getError() const {
    return last_error;
}

float SMCController::getFilteredErrorRate() const {
    return current_e_dot;
}

float SMCController::getSlidingSurface() const {
    return current_s;
}
