#include "SMCController.h"

SMCController::SMCController(float K_, float lambda_, float phi_, float dt_)
  : K(K_), lambda(lambda_), phi(phi_), dt(dt_), prev_error(0.0) {}

float SMCController::update(float reference, float measured) {
    float e = reference - measured;
    float e_dot = (e - prev_error) / dt;

    float s = e + lambda * e_dot;

    float sat_s;
    if (s > phi) sat_s = 1.0;
    else if (s < -phi) sat_s = -1.0;
    else sat_s = s / phi;

    float u = -K * sat_s;

    prev_error = e;

    return u;
}