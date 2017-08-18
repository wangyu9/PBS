#ifndef DUAL_QUAT_H
#define DUAL_QUAT_H
#include  <Eigen/Core>
#include <Eigen/Geometry>

template <typename Real>
class DualQuaternion
{
public:
	// Construction.
	DualQuaternion ();  // uninitialized
	DualQuaternion (const DualQuaternion& q);
	DualQuaternion (const Eigen::Quaternion<Real, Eigen::DontAlign>& primal, const Eigen::Quaternion<Real, Eigen::DontAlign>& dual);
	DualQuaternion (const Eigen::Matrix<Real, 3, 1>& translate, const Eigen::Quaternion<Real, Eigen::DontAlign>& rotate);
	void setIdentity();
	void setZero();

	// Coordinate access as an array:  0 = w, 1 = x, 2 = y, 3 = z.
	inline Eigen::Quaternion<Real, Eigen::DontAlign> PrimalQuat () const;
	inline Eigen::Quaternion<Real, Eigen::DontAlign>& PrimalQuat ();
	inline Eigen::Quaternion<Real, Eigen::DontAlign> DualQuat () const;
	inline Eigen::Quaternion<Real, Eigen::DontAlign>& DualQuat ();

	// Assignment.
	inline DualQuaternion& operator= (const DualQuaternion& q);

	// Comparison (for use by STL containers).
	inline bool operator== (const DualQuaternion& q) const;
	inline bool operator!= (const DualQuaternion& q) const;
	//inline bool operator<  (const Quaternion& q) const;
	//inline bool operator<= (const Quaternion& q) const;
	//inline bool operator>  (const Quaternion& q) const;
	//inline bool operator>= (const Quaternion& q) const;

	//// Arithmetic operations.
	inline DualQuaternion operator+ (const DualQuaternion& q) const;
	inline DualQuaternion operator- (const DualQuaternion& q) const;
	inline DualQuaternion operator* (const DualQuaternion& q) const;
	inline DualQuaternion operator/ (const DualQuaternion& q) const;
	inline DualQuaternion operator* (Real scalar) const;
	inline DualQuaternion operator* (Eigen::Matrix<Real, 2, 1> dualNumber) const;
	inline DualQuaternion operator/ (Real scalar) const;
	inline DualQuaternion operator- () const;
	DualQuaternion Inverse(Real epsilon = 0) const;
	DualQuaternion Conjugate() const; //conjugate for the quaternion
	DualQuaternion ConjugateDual() const; //conjugate for the dual number
	DualQuaternion ConjDual() const; //conjugate and conj dual in all
	DualQuaternion applyXform(const DualQuaternion& q) const; //return q*this*ConjDual(q)

	friend DualQuaternion<Real> operator* (Real scalar,
		const DualQuaternion<Real>& q)
	{
		return q*scalar;
	}

	// Arithmetic updates.
	inline DualQuaternion& operator+= (const DualQuaternion& q);
	inline DualQuaternion& operator-= (const DualQuaternion& q);
	inline DualQuaternion& operator*= (Real scalar);
	inline DualQuaternion& operator/= (Real scalar);

	// Conversion between quaternions, matrices, and axis-angle.
	void FromTransRot (const Eigen::Matrix<Real, 3, 1>& trans, const Eigen::Quaternion<Real, Eigen::DontAlign>& rot);
	void ToTransRot (Eigen::Matrix<Real, 3, 1>& trans, Eigen::Quaternion<Real, Eigen::DontAlign>& rot) const;
	//if this dq is not unit
	void ApproximateToTransRot (Eigen::Matrix<Real, 3, 1>& trans, Eigen::Quaternion<Real, Eigen::DontAlign>& rot)const; 

	// Functions of a quaternion.
	Eigen::Matrix<Real, 2, 1> LengthDQ() const; //the norm of a dual quaternion
	inline Real Length () const;  // length of 8-tuple
	Eigen::Matrix<Real, 2, 1> SquaredLengthDQ() const; //the squared norm of a dual quaternion
	inline Real SquaredLength () const;  // squared length of 8-tuple
	//inline Real Dot (const Quaternion& q) const;  // dot product of 4-tuples
	Real NormalizeDQ (Real epsilon = 1e-6); //the dual quaternion's unit, only return its primal part's length to indicated whether it can be unified
	inline Real Normalize (Real epsilon = 1e-6);
	DualQuaternion Exp () const;  // apply to quaternion with w = 0
	DualQuaternion Log (Real epsilon = 0.0001) const;  // apply to unit-length quaternion
	
	void toScrew(Real& rotation, Real& pitch, Eigen::Matrix<Real, 3, 1>& dir, Eigen::Matrix<Real, 3, 1>& moment)const;
	void fromScrew(Real rotation, Real pitch, Eigen::Matrix<Real, 3, 1> dir, Eigen::Matrix<Real, 3, 1> moment);


	inline bool checkPlucker()const;
	inline bool isUnit()const;
	inline bool hasRotation() const;

	// Do screw linear interpolation (the "slerp" for dual quaternions) for two unit dual quaternions
	static DualQuaternion sScLERP(Real inT, const DualQuaternion& inFrom, const DualQuaternion& inTo);

	//// Special quaternions.
	//WM5_MATHEMATICS_ITEM static const DualQuaternion ZERO;
	//WM5_MATHEMATICS_ITEM static const DualQuaternion IDENTITY;
private:
	// Order of storage is (w,x,y,z).
	Eigen::Quaternion<Real, Eigen::DontAlign> mPrimeQuat;
	Eigen::Quaternion<Real, Eigen::DontAlign> mDualQuat;
};

#include "dualQuat.inl"

typedef DualQuaternion<float> DualQuaternionf;
typedef DualQuaternion<double> DualQuaterniond;



#endif