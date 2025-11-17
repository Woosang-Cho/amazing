#include "SMCController.h"

SMCController::SMCController(float k_, float phi_, float c1_, float c2_, float dt_)
    : k(k_), phi(phi_), c1(c1_), c2(c2_), dt(dt_), prev_error(0.0) {}

float SMCController::update(float reference, float measured) {
    float e = reference - measured;
    float e_dot = (e - prev_error) / dt;

    float s = c1 * e + c2 * e_dot;

    float sat_s;
    if (s > phi) sat_s = 1.0;
    else if (s < -phi) sat_s = -1.0;
    else sat_s = s / phi;

    float u = -k * sat_s;

    prev_error = e;

    return u;
}
