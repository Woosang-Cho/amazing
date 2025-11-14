#include "SMCController.h"

SMCController::SMCController(float lambda_, float k_, float epsilon_, float Ts_)
{
    lambda = lambda_;
    k = k_;
    epsilon = epsilon_;
    Ts = Ts_;
    prev_error = 0.0f;
}

float SMCController::sat(float s)
{
    if (s > epsilon) return 1.0f;
    if (s < -epsilon) return -1.0f;
    return s / epsilon;
}

float SMCController::update(float error)
{
    float de = (error - prev_error) / Ts;
    prev_error = error;

    float s = lambda * error + de;

    float u_eq = -lambda * de;
    float u = u_eq - k * sat(s);

    return u;
}
