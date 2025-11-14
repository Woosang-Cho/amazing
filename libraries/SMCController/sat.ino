#include <math.h>

float sat(float value)
{
  // speed up some math; 20 is modifiable
  if (value < -20) return -1;
  if (value > 20) return 1;
  // workhorse
  float f = 10.0 * value;     // 10 is a parameter to adjust the steepness
  float rv = atan(f) * 0.636619772;     // 0.63.. == 2/pi  as atan goes form -pi/2 .. pi/2 one needs to multiply with the reciproke;

  return rv;
}