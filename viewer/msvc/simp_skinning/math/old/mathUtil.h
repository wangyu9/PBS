#ifndef MATHUTIL_H
#define MATHUTIL_H

#include <cmath>

const float kPi = 3.14159265f;
const float k2Pi = kPi * 2.0f;
const float kPiOver2 = kPi / 2.0f;
const float k1OverPi = 1.0f / kPi;
const float k1Over2Pi = 1.0f / k2Pi;

float wrapPi(float theta);
float safeAcos(float x);
inline void sinCos(float * returnSin, float *returnCos, float theta)
{
	*returnSin = sin(theta);
	*returnCos = cos(theta);
}

#endif //MATHUTIL_H
