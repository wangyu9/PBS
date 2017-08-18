#ifndef MATRIX3X4_H
#define MATRIX3X4_H

#include "vector3.h"
#include <cmath>
#define MY_PI 3.14159265358979323846
//--                  --
//| e0  e1  e2     e3  |
//| e4  e5  e6     e7  |
//| e8  e9  e10    e11 |
//--                  --

class Matrix3x4
{
public:
	Matrix3x4()	{}

	Matrix3x4(const Matrix3x4& rhs)
	{
		for (int i=0; i<12; i++)
			e[i] = rhs.e[i];
	}

	void setZeroMat()
	{
		for (int i=0; i<12; i++)
			e[i] = 0.0f;
	}

	void setIdentityMat()
	{
		e[0] = e[5] = e[10] = 1.0f;
		e[1] = e[2] = e[3] = e[4] = e[6] = e[7] = e[8] = e[9] = e[11] = 0.0f;
	}

	void setRotateX(float rx)
	{
		rx = rx * MY_PI /180.0f;
		e[5] = cos(rx); e[10] =  e[5];
		e[9] = sin(rx); e[6]  = -e[9];
		e[0] = 1.0f;
		e[1] = e[2] = e[3] = e[4] = e[7] = e[8] = e[11] = 0.0f;
	}

	void setRotateY(float ry)
	{
		ry = ry * MY_PI /180.0f;
		e[0] = cos(ry); e[10] =  e[0];
		e[2] = sin(ry); e[8]  = -e[2];
		e[5] = 1.0f;
		e[1] = e[3] = e[4] = e[6] = e[7] = e[9] = e[11] = 0.0f;
	}

	void setRotateZ(float rz)
	{
		rz = rz * MY_PI /180.0f;
		e[0] = cos(rz); e[5] =  e[0];
		e[4] = sin(rz); e[1] = -e[4];
		e[10] = 1.0f;
		e[2] = e[3] = e[6] = e[7] = e[8] = e[9] = e[11] = 0.0f;
	}

	void setTranslate(Vector3 trans)
	{
		e[3] = trans.x; e[7] = trans.y; e[11] = trans.z;
		e[0] = e[5] = e[10] = 1.0f;
		e[1] = e[2] = e[4] = e[6] = e[8] = e[9] = 0.0f;
	}

	Matrix3x4 & operator =(const Matrix3x4& rhs)
	{
		for (int i=0; i<12; i++)
			e[i] = rhs.e[i];
		return *this;
	}

	Vector3 operator *(const Vector3& v) const
	{
		return Vector3( e[0]*v.x + e[1]*v.y + e[2]*v.z + e[3],
						e[4]*v.x + e[5]*v.y + e[6]*v.z + e[7],
						e[8]*v.x + e[9]*v.y + e[10]*v.z + e[11]	);
	}

public:
	float e[12];
};

// non member function
Matrix3x4 operator*(const Matrix3x4& lhs, const Matrix3x4& rhs)
{
	Matrix3x4 r;

	r.e[0]  = lhs.e[0]*rhs.e[0] + lhs.e[1]*rhs.e[4] + lhs.e[2]*rhs.e[8];
	r.e[1]  = lhs.e[0]*rhs.e[1] + lhs.e[1]*rhs.e[5] + lhs.e[2]*rhs.e[9];
	r.e[2]  = lhs.e[0]*rhs.e[2] + lhs.e[1]*rhs.e[6] + lhs.e[2]*rhs.e[10];
	r.e[3]  = lhs.e[0]*rhs.e[3] + lhs.e[1]*rhs.e[7] + lhs.e[2]*rhs.e[11] + lhs.e[3];

	r.e[4]  = lhs.e[4]*rhs.e[0] + lhs.e[5]*rhs.e[4] + lhs.e[6]*rhs.e[8];
	r.e[5]  = lhs.e[4]*rhs.e[1] + lhs.e[5]*rhs.e[5] + lhs.e[6]*rhs.e[9];
	r.e[6]  = lhs.e[4]*rhs.e[2] + lhs.e[5]*rhs.e[6] + lhs.e[6]*rhs.e[10];
	r.e[7]  = lhs.e[4]*rhs.e[3] + lhs.e[5]*rhs.e[7] + lhs.e[6]*rhs.e[11] + lhs.e[7];

	r.e[8]  = lhs.e[8]*rhs.e[0] + lhs.e[9]*rhs.e[4] + lhs.e[10]*rhs.e[8];
	r.e[9]  = lhs.e[8]*rhs.e[1] + lhs.e[9]*rhs.e[5] + lhs.e[10]*rhs.e[9];
	r.e[10] = lhs.e[8]*rhs.e[2] + lhs.e[9]*rhs.e[6] + lhs.e[10]*rhs.e[10];
	r.e[11] = lhs.e[8]*rhs.e[3] + lhs.e[9]*rhs.e[7] + lhs.e[10]*rhs.e[11] +lhs.e[11];

	return r;
}

Matrix3x4 & operator *=(Matrix3x4 & lhs, const Matrix3x4& rhs)
{
	lhs = lhs*rhs;
	return lhs;
}

#endif // MATRIX3X4_H
