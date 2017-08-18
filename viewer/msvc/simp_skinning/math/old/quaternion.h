//////////////////////////////////////////////////////////////////////////
// Quaternion:  a simple 3d vector class
// from “3D数学基础：图形与游戏开发”
//////////////////////////////////////////////////////////////////////////

#ifndef QUATERNION_H
#define QUATERNION_H

class Vector3;

class Quaternion
{
public:
    float w,x,y,z;

    void identity() { w=1.0f; x=y=z=0.0f; }

    void setToRotateAboutX(float theta);
    void setToRotateAboutY(float theta);
    void setToRotateAboutZ(float theta);
    void setToRotateAboutAxis(const Vector3 &axis, float theta);

    //void setToRotateObjectToInertail(const EulerAngles &orientatian);
    void setToRotateEuler(const Vector3 &orientation);
    Vector3 getRotateEuler();
    Quaternion  operator* (const Quaternion &a) const; // cross product
    Quaternion& operator*=(const Quaternion &a);

    void normalize();

    float getRotationAngle() const;
    Vector3 getRotationAxis() const;
};

extern const Quaternion kQuaternionIdentity;

//dot product
float dotProduct(const Quaternion &a, const Quaternion &b);
Quaternion slerp(const Quaternion &p, const Quaternion &q, float t);
Quaternion conjugate(const Quaternion &q);
Quaternion pow(const Quaternion &q, float exponent);

#endif  // quaternion.h
