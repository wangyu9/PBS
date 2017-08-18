#ifndef MY_GLMAT_H
#define MY_GLMAT_H

#include "vector3.h"
#include "quaternion.h"

// --                --
// | e0  e4   e8  e12 |
// | e1  e5   e9  e13 |
// | e2  e6  e10  e14 |
// | e3  e7  e11  e15 |
// --               --

struct MyGLMat
{
	float e[16];

	void setIdentity()
	{
		e[1] = e[2] = e[3] = e[4] = e[6] = e[7] = e[8] = e[9] = e[11] = e[12] = e[13] = e[14] = 0.0f;
		e[0] = e[5] = e[10] = e[15] = 1.0f;
	}

	void setRotbyQuaternion(const Quaternion & q)
	{
        float ww = 2.0f * q.w;
        float xx = 2.0f * q.x;
        float yy = 2.0f * q.y;
        float zz = 2.0f * q.z;

        e[0] = 1.0f - yy*q.y - zz*q.z;
        e[1] = xx*q.y + ww*q.z;
        e[2] = xx*q.z - ww*q.y;

        e[4] = xx*q.y - ww*q.z;
        e[5] = 1.0f - xx*q.x - zz*q.z;
        e[6] = yy*q.z + ww*q.x;

        e[8] = xx*q.z + ww*q.y;
        e[9] = yy*q.z - ww*q.x;
        e[10] = 1.0f - xx*q.x - yy*q.y;

        e[12] = e[13] = e[14] = e[3] = e[7] = e[11] = 0.0f;
        e[15] = 1.0f;
    }

	void setTranslate(const Vector3& v)
	{
		e[12] = v.x;
		e[13] = v.y;
		e[14] = v.z;

		e[1] = e[2] = e[3] = e[4] = e[6] = e[7] = e[8] = e[9] = e[11] = 0.0f;
		e[0] = e[5] = e[10] = e[15] = 1.0f;
	}
};

#endif
