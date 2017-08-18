//////////////////////////////////////////////////////////////////////////
// Quaternion:  a simple 3d vector class
// from “3D数学基础：图形与游戏开发”
//////////////////////////////////////////////////////////////////////////
#include <cassert>
#include <cmath>

#include "quaternion.h"
#include "vector3.h"
#include "mathUtil.h"

const Quaternion kQuaternionIdentity = {1.0f, 0.0f, 0.0f, 0.0f };

void Quaternion::setToRotateAboutX(float theta)
{
    float half_theta = theta *  0.5f;
    w = cos(half_theta);
    x = sin(half_theta);
    y = 0.0f;
    z = 0.0f;
}

void Quaternion::setToRotateAboutY(float theta)
{
    float half_theta = theta * 0.5f;
    w = cos(half_theta);
    x = 0.0f;
    y = sin(half_theta);
    z = 0.0f;
}

void Quaternion::setToRotateAboutZ(float theta)
{
    float half_theta = theta * 0.5f;
    w = cos(half_theta);
    x = 0.0f;
    y = 0.0f;
    z = sin(half_theta);
}

void Quaternion::setToRotateAboutAxis(const Vector3 &axis, float theta)
{
    assert(fabs(vectorMag(axis)-1.0f) < .01f);

    float half_theta = theta * 0.5f;
    float sin_half_theta = sin(half_theta);
    w = cos(half_theta);
    x = axis.x * sin_half_theta;
    y = axis.y * sin_half_theta;
    z = axis.z * sin_half_theta;
}

// from Eular angle
void Quaternion::setToRotateEuler(const Vector3 &orientation)
{
    Quaternion qx,qy,qz;

    qx.setToRotateAboutX(orientation.x);
    qy.setToRotateAboutY(orientation.y);
    qz.setToRotateAboutZ(orientation.z);

    *this = qx*qy*qz;
}

Vector3 Quaternion::getRotateEuler()
{
    float pitch, bank, heading;
    float sp = -2.0f*(y*z+w*x);
    if (fabs(sp) > 0.999f)
    {
        pitch = kPiOver2 * sp;
        heading = atan2(-x*z-w*y, 0.5f-y*y-z*z);
        bank = 0.0f;
    }
    else
    {
        pitch   = asin(sp);
        heading = atan2(x*z-w*y, 0.5f-x*x-y*y);
        bank    = atan2(x*y-w*z, 0.5f-x*x-z*z);
    }
    return Vector3(pitch, heading, bank);
}

Quaternion Quaternion::operator * (const Quaternion &a) const
{
    Quaternion result;

    result.w = w*a.w - x*a.x - y*a.y - z*a.z;
    result.x = w*a.x + x*a.w + z*a.y - y*a.z;
    result.y = w*a.y + y*a.w + x*a.z - z*a.x;
    result.z = w*a.z + z*a.w + y*a.x - x*a.y;

    return result;
}

Quaternion & Quaternion::operator *=(const Quaternion &a)
{
    *this = *this * a;
    return *this;
}

void Quaternion::normalize()
{
    float mag = (float) sqrt(w*w + x*x + y*y + z*z);
    if (mag>0.0f)
    {
        float oneOverMag = 1.0f / mag;
        w *= oneOverMag;
        x *= oneOverMag;
        y *= oneOverMag;
        z *= oneOverMag;
    }
    else
    {
        assert(false);
        identity();
    }
}

float Quaternion::getRotationAngle() const
{
    float half_theta = safeAcos(w);
    return half_theta*2.0f;
}

Vector3 Quaternion::getRotationAxis() const
{
    float sq_sin_half_theta = 1.0f - w*w;
    if (sq_sin_half_theta <= 0.0f)
    {
        return Vector3(1.0f, 0.0f, 0.0f);
    }

    float oneOverSinThetaOver2 = 1.0f / sqrt(sq_sin_half_theta);

    return Vector3( x * oneOverSinThetaOver2, y * oneOverSinThetaOver2, z * oneOverSinThetaOver2);
}

float dotProduct(const Quaternion &a, const Quaternion &b)
{
    return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z;
}

//slerp
Quaternion slerp(const Quaternion& p, const Quaternion& q, float t)
{
    if (t<=0.0f) return p;
    if (t>=1.0f) return q;

    float cosOmega = dotProduct(p, q);
    float qw = q.w;
    float qx = q.x;
    float qy = q.y;
    float qz = q.z;

    if (cosOmega < 0.0f)
    {
        qw = -qw;
        qx = -qx;
        qy = -qy;
        qz = -qz;
        cosOmega = -cosOmega;
    }

    assert(cosOmega < 1.1f);

    float kp, kq;
    if (cosOmega>0.9999f)
    {
        kp = 1.0f -t;
        kq = t;
    }
    else
    {
        float sinOmega = sqrt(1.0f - cosOmega*cosOmega);

        float omega = atan2(sinOmega, cosOmega);

        float oneOverSinOmega = 1.0f / sinOmega;

        kp = sin((1.0f -t)*omega) * oneOverSinOmega;
        kq = sin(t*omega) * oneOverSinOmega;
    }

    Quaternion result;
    result.w = kp*p.w + kq*qw;
    result.x = kp*p.x + kq*qx;
    result.y = kp*p.y + kq*qy;
    result.z = kp*p.z + kq*qz;

    return result;
}

Quaternion conjugate(const Quaternion& q)
{
    Quaternion result;

    result.w =  q.w;

    result.x = -q.x;
    result.y = -q.y;
    result.z = -q.z;

    return result;
}

Quaternion pow(const Quaternion&q, float exponent)
{
    if (fabs(q.w) > 0.9999f)
        return q;
    float alpha = acos(q.w);
    float newAlpha = alpha * exponent;

    Quaternion result;
    result.w = cos(newAlpha);

    float mult = sin(newAlpha) / sin(alpha);
    result.x = q.x * mult;
    result.y = q.y * mult;
    result.z = q.z * mult;

    return result;
}
