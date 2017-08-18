//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real>::DualQuaternion ()
{
	// Uninitialized for performance in array construction.
}

template <typename Real>
void DualQuaternion<Real>::setIdentity()
{
	mPrimeQuat.setIdentity();
	mDualQuat.w() = 0.0;
	mDualQuat.x() = 0.0;
	mDualQuat.y() = 0.0;
	mDualQuat.z() = 0.0;
}

template <typename Real>
void DualQuaternion<Real>::setZero()
{
	mPrimeQuat.w() = 0.0;
	mPrimeQuat.x() = 0.0;
	mPrimeQuat.y() = 0.0;
	mPrimeQuat.z() = 0.0;
	mDualQuat.w() = 0.0;
	mDualQuat.x() = 0.0;
	mDualQuat.y() = 0.0;
	mDualQuat.z() = 0.0;
}

//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real>::DualQuaternion (const DualQuaternion& q)
{
	mPrimeQuat = q.mPrimeQuat;
	mDualQuat = q.mDualQuat;
}
//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real>::DualQuaternion (const Eigen::Quaternion<Real, Eigen::DontAlign>& primal, const Eigen::Quaternion<Real, Eigen::DontAlign>& dual)
{
	mPrimeQuat = primal;
	mDualQuat = dual;
}
//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real>::DualQuaternion (const Eigen::Matrix<Real, 3, 1>& translate, const Eigen::Quaternion<Real, Eigen::DontAlign>& rotate)
{
	FromTransRot(translate, rotate);
}
//----------------------------------------------------------------------------
template <typename Real>
inline Eigen::Quaternion<Real, Eigen::DontAlign> DualQuaternion<Real>::PrimalQuat () const
{
	return mPrimeQuat;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Eigen::Quaternion<Real, Eigen::DontAlign>& DualQuaternion<Real>::PrimalQuat ()
{
	return mPrimeQuat;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Eigen::Quaternion<Real, Eigen::DontAlign> DualQuaternion<Real>::DualQuat () const
{
	return mDualQuat;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Eigen::Quaternion<Real, Eigen::DontAlign>& DualQuaternion<Real>::DualQuat ()
{
	return mDualQuat;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real>& DualQuaternion<Real>::operator= (const DualQuaternion& q)
{
	mPrimeQuat = q.mPrimeQuat;
	mDualQuat = q.mDualQuat;
	return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline bool DualQuaternion<Real>::checkPlucker()const
{
	return fabs(mPrimeQuat.dot(mDualQuat))<1e-5;
}
//----------------------------------------------------------------------------
template <typename Real>
inline bool DualQuaternion<Real>::isUnit()const
{
	return (fabs(mPrimeQuat.dot(mPrimeQuat)-1.0)<1e-5) && checkPlucker();
}
//----------------------------------------------------------------------------
template <typename Real>
inline bool DualQuaternion<Real>::hasRotation() const
{
	ASSERT(isUnit());
	return fabs(mPrimeQuat.w())<0.999999f;
}
//----------------------------------------------------------------------------
template <typename Real>
inline bool DualQuaternion<Real>::operator== (const DualQuaternion& q) const
{
	return (mPrimeQuat == q.mPrimeQuat) && (mDualQuat == q.mDualQuat) ;
}
//----------------------------------------------------------------------------
template <typename Real>
inline bool DualQuaternion<Real>::operator!= (const DualQuaternion& q) const
{
	return (mPrimeQuat != q.mPrimeQuat) || (mDualQuat != q.mDualQuat);
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::operator+ (const DualQuaternion& q)
const
{
	DualQuaternion result;
	//result.mPrimeQuat = mPrimeQuat + q.mPrimeQuat;
	//result.mDualQuat = mDualQuat + q.mDualQuat;

	result.mPrimeQuat.w() = mPrimeQuat.w() + q.mPrimeQuat.w();
	result.mPrimeQuat.x() = mPrimeQuat.x() + q.mPrimeQuat.x();
	result.mPrimeQuat.y() = mPrimeQuat.y() + q.mPrimeQuat.y();
	result.mPrimeQuat.z() = mPrimeQuat.z() + q.mPrimeQuat.z();

	result.mDualQuat.w() = mDualQuat.w() + q.mDualQuat.w();
	result.mDualQuat.x() = mDualQuat.x() + q.mDualQuat.x();
	result.mDualQuat.y() = mDualQuat.y() + q.mDualQuat.y();
	result.mDualQuat.z() = mDualQuat.z() + q.mDualQuat.z();
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::operator- (const DualQuaternion& q)
const
{
	DualQuaternion result;
	result.mPrimeQuat.w() = mPrimeQuat.w() - q.mPrimeQuat.w();
	result.mPrimeQuat.x() = mPrimeQuat.x() - q.mPrimeQuat.x();
	result.mPrimeQuat.y() = mPrimeQuat.y() - q.mPrimeQuat.y();
	result.mPrimeQuat.z() = mPrimeQuat.z() - q.mPrimeQuat.z();
	result.mDualQuat.w() = mDualQuat.w() - q.mDualQuat.w();
	result.mDualQuat.x() = mDualQuat.x() - q.mDualQuat.x();
	result.mDualQuat.y() = mDualQuat.y() - q.mDualQuat.y();
	result.mDualQuat.z() = mDualQuat.z() - q.mDualQuat.z();
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::operator* (const DualQuaternion& q)
const
{
	DualQuaternion result;
	result.mPrimeQuat = mPrimeQuat * q.mPrimeQuat;
	//result.mDualQuat = mPrimeQuat * q.mDualQuat + mDualQuat * q.mPrimeQuat;
	Eigen::Quaternion<Real> t1 =  mPrimeQuat * q.mDualQuat;
	Eigen::Quaternion<Real> t2 =  mDualQuat * q.mPrimeQuat;
	result.mDualQuat = Eigen::Quaternion<Real>(t1.w() + t2.w(), t1.x() + t2.x(), t1.y() + t2.y(), t1.z() + t2.z());
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::operator/ (const DualQuaternion& q)
const
{
	return this * q.Inverse();
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::operator* (Eigen::Matrix<Real, 2, 1> dualNumber) const
{
	DualQuaternion result;
	result.mPrimeQuat = mPrimeQuat*dualNumber[0];
	result.mDualQuat = mDualQuat*dualNumber[0] + mPrimeQuat*dualNumber[1];
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::operator* (Real scalar) const
{
	DualQuaternion result;
	result.mPrimeQuat.x() = mPrimeQuat.x() * scalar;
	result.mPrimeQuat.y() = mPrimeQuat.y() * scalar;
	result.mPrimeQuat.z() = mPrimeQuat.z() * scalar;
	result.mPrimeQuat.w() = mPrimeQuat.w() * scalar;

	result.mDualQuat.x() = mDualQuat.x() * scalar;
	result.mDualQuat.y() = mDualQuat.y() * scalar;
	result.mDualQuat.z() = mDualQuat.z() * scalar;
	result.mDualQuat.w() = mDualQuat.w() * scalar;
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::operator/ (Real scalar) const
{
	DualQuaternion result;
	result.mPrimeQuat.x() = mPrimeQuat.x() / scalar;
	result.mPrimeQuat.y() = mPrimeQuat.y() / scalar;
	result.mPrimeQuat.z() = mPrimeQuat.z() / scalar;
	result.mPrimeQuat.w() = mPrimeQuat.w() / scalar;

	result.mDualQuat.x() = mDualQuat.x() / scalar;
	result.mDualQuat.y() = mDualQuat.y() / scalar;
	result.mDualQuat.z() = mDualQuat.z() / scalar;
	result.mDualQuat.w() = mDualQuat.w() / scalar;
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::operator- () const
{
	DualQuaternion result;
	result.mPrimeQuat = -mPrimeQuat;
	result.mDualQuat = -mDualQuat;
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real>& DualQuaternion<Real>::operator+= (const DualQuaternion& q)
{
	mPrimeQuat.x() += q.mPrimeQuat.x();
	mPrimeQuat.y() += q.mPrimeQuat.y();
	mPrimeQuat.z() += q.mPrimeQuat.z();
	mPrimeQuat.w() += q.mPrimeQuat.w();

	mDualQuat.x() += q.mDualQuat.x();
	mDualQuat.y() += q.mDualQuat.y();
	mDualQuat.z() += q.mDualQuat.z();
	mDualQuat.w() += q.mDualQuat.w();
	return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real>& DualQuaternion<Real>::operator-= (const DualQuaternion& q)
{
	mPrimeQuat.x() -= q.mPrimeQuat.x();
	mPrimeQuat.y() -= q.mPrimeQuat.y();
	mPrimeQuat.z() -= q.mPrimeQuat.z();
	mPrimeQuat.w() -= q.mPrimeQuat.w();

	mDualQuat.x() -= q.mDualQuat.x();
	mDualQuat.y() -= q.mDualQuat.y();
	mDualQuat.z() -= q.mDualQuat.z();
	mDualQuat.w() -= q.mDualQuat.w();
	return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real>& DualQuaternion<Real>::operator*= (Real scalar)
{
	mPrimeQuat.x() *= scalar;
	mPrimeQuat.y() *= scalar;
	mPrimeQuat.z() *= scalar;
	mPrimeQuat.w() *= scalar;

	mDualQuat.x() *= scalar;
	mDualQuat.y() *= scalar;
	mDualQuat.z() *= scalar;
	mDualQuat.w() *= scalar;
	return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real>& DualQuaternion<Real>::operator/= (Real scalar)
{
	mPrimeQuat.x() /= scalar;
	mPrimeQuat.y() /= scalar;
	mPrimeQuat.z() /= scalar;
	mPrimeQuat.w() /= scalar;

	mDualQuat.x() /= scalar;
	mDualQuat.y() /= scalar;
	mDualQuat.z() /= scalar;
	mDualQuat.w() /= scalar;
	return *this;
}
//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real> DualQuaternion<Real>::Inverse(Real epsilon) const
{
	DualQuaternion<Real> result(Eigen::Quaternion<Real, Eigen::DontAlign>::Zero(), Eigen::Quaternion<Real, Eigen::DontAlign>::Zero());
	Real primeSqrLength = mPrimeQuat.SquaredLength();
	if ( primeSqrLength > epsilon * epsilon )
	{
		Real invSqrLength = ((Real)1)/primeSqrLength;
		Eigen::Matrix<Real, 2, 1> invSqrNormDQ(invSqrLength,
			2.0 * mPrimeQuat.Dot(mDualQuat)*invSqrLength*invSqrLength);

		//result = this->Conjugate()/this->SquaredLengthDQ();
		result.mDualQuat = invSqrNormDQ[1] * mPrimeQuat.Conjugate() + invSqrNormDQ[0] * mDualQuat.Conjugate();
		result.mPrimeQuat = invSqrNormDQ[0] * mPrimeQuat;//note the bug if update mPrimeQuat first
	}

	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real> DualQuaternion<Real>::Conjugate() const
{
	DualQuaternion<Real> result( mPrimeQuat.conjugate(), mDualQuat.conjugate() );
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::ConjDual() const
{
	DualQuaternion<Real> result( mPrimeQuat.conjugate(), mDualQuat.conjugate() );
	result.mDualQuat.w() = - result.mDualQuat.w();
	result.mDualQuat.x() = - result.mDualQuat.x();
	result.mDualQuat.y() = - result.mDualQuat.y();
	result.mDualQuat.z() = - result.mDualQuat.z();
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::applyXform(const DualQuaternion& q) const
{
	return q * (*this) * q.ConjDual();
}
//----------------------------------------------------------------------------
template <typename Real>
inline DualQuaternion<Real> DualQuaternion<Real>::ConjugateDual() const
{
	DualQuaternion<Real> result( mPrimeQuat, -mDualQuat );
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
Eigen::Matrix<Real, 2, 1> DualQuaternion<Real>::LengthDQ () const
{
	Real primeNorm = mPrimeQuat.norm();
	if (primeNorm == 0)
		return Eigen::Matrix<Real, 2, 1>::Zero();

	Eigen::Matrix<Real, 2, 1> result( primeNorm ,
		2.0 * mPrimeQuat.Dot(mDualQuat) / primeNorm );
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real DualQuaternion<Real>::Length () const
{
	return sqrt(mPrimeQuat.squaredNorm() + mDualQuat.squaredNorm());
}
//----------------------------------------------------------------------------
template <typename Real>
Eigen::Matrix<Real, 2, 1> DualQuaternion<Real>::SquaredLengthDQ () const
{
	Eigen::Matrix<Real, 2, 1> result( mPrimeQuat.squaredNorm() ,
		2.0 * mPrimeQuat.Dot(mDualQuat));
	return result;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real DualQuaternion<Real>::SquaredLength () const
{
	return mPrimeQuat.squaredNorm() + mDualQuat.squaredNorm();
}
//----------------------------------------------------------------------------
template <typename Real>
Real DualQuaternion<Real>::NormalizeDQ (Real epsilon)
{
	Real primeLength = mPrimeQuat.norm();

	if (primeLength > epsilon)
	{
		Real invLength = ((Real)1)/primeLength;
		Eigen::Matrix<Real, 2, 1> invNormDQ(invLength,
			mPrimeQuat.Dot(mDualQuat)*invLength*invLength*invLength );
		//mPrimeQuat = ;
		mDualQuat = mPrimeQuat * invNormDQ[1] + mDualQuat * invNormDQ[0];
		mPrimeQuat = mPrimeQuat * invNormDQ[0];//note the bug if update mPrimeQuat first
	}
	else
	{
		primeLength = 0;
		mPrimeQuat = Eigen::Quaternion<Real, Eigen::DontAlign>::Zero();
		mDualQuat = Eigen::Quaternion<Real, Eigen::DontAlign>::Zero();
	}

	return primeLength;
}
//----------------------------------------------------------------------------
template <typename Real>
inline Real DualQuaternion<Real>::Normalize (Real epsilon)
{
	Real length = Length();

	if (length > epsilon)
	{
		Real invLength = ((Real)1)/length;
		mPrimeQuat *= invLength;
		mDualQuat *= invLength;
	}
	else
	{
		length = (Real)0;
		mPrimeQuat.setZero();
		mDualQuat.setZero();
	}

	return length;
}
//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real> DualQuaternion<Real>::Log (Real epsilon) const
{
	/*DualQuaternion<Real> unit = *this;
	unit.NormalizeDQ();

	DualQuaternion<Real> s;
	Eigen::Matrix<Real, 2, 1> theta;
	Eigen::Matrix<Real, 3, 1> translate;
	Eigen::Quaternion<Real, Eigen::DontAlign> dummy;
	unit.ToTransRot(translate, dummy);



	if ( mPrimeQuat[0] >  1 - epsilon )
	{
		theta[0] = 0;
		Real transLen = translate.Normalize();
		s.mPrimeQuat.w() = 0;
		s.mPrimeQuat.x() = translate.x();
		s.mPrimeQuat.y() = translate.y();
		s.mPrimeQuat.z() = translate.z();
		theta[1] = transLen;
		s.mDualQuat = Eigen::Quaternion<Real, Eigen::DontAlign>::ZERO;
	}
	else if ( mPrimeQuat[0] < epsilon - 1 )
	{
		theta[0] = M_TWO_PI;
		Real transLen = translate.Normalize();
		s.mPrimeQuat.w() = 0;
		s.mPrimeQuat.x() = translate.x();
		s.mPrimeQuat.y() = translate.y();
		s.mPrimeQuat.z() = translate.z();
		theta[1] = transLen;
		s.mDualQuat = Eigen::Quaternion<Real, Eigen::DontAlign>::ZERO;
	}
	else
	{
		Eigen::Matrix<Real, 3, 1> s0;
		mPrimeQuat.ToAxisAngle(s0, theta[0]);
		s.mPrimeQuat.w() = 0;
		s.mPrimeQuat.x() = s0.x();
		s.mPrimeQuat.y() = s0.y();
		s.mPrimeQuat.z() = s0.z();

		theta[1] = translate.Dot(s0);
		Eigen::Matrix<Real, 3, 1> se;
		se = 0.5 * ( Wm5::Math<Real>::FastInvTan0(0.5*theta[0]) *
			(s0.Cross(translate)) + translate ).Cross(s0);
		s.mDualQuat.w() = 0;
		s.mDualQuat.x() = se.x();
		s.mDualQuat.y() = se.y();
		s.mDualQuat.z() = se.z();
	}

	return s * theta * 0.5;*/

	//DualQuaternion<Real> result;

	////Eigen::Matrix<Real, 3, 1> translate;
	////Eigen::Quaternion<Real, Eigen::DontAlign> rotate;
	////ToTransRot(translate, rotate);
	//Eigen::Quaternion<Real, Eigen::DontAlign> gamma = mDualQuat * mPrimeQuat.Inverse();

	//result.mPrimeQuat = mPrimeQuat.Log();
	//result.mDualQuat = gamma;

	//return result;

	Real angle, pitch;
	Eigen::Matrix<Real, 3, 1> direction, moment;
	toScrew(angle, pitch, direction, moment);
	angle *= 0.5;
	pitch *= 0.5;

	DualQuaternion<Real> result;
	result.mPrimeQuat.x() = direction[0]*angle;
	result.mPrimeQuat.y() = direction[1]*angle;
	result.mPrimeQuat.z() = direction[2]*angle;
	result.mPrimeQuat.w() = 0.0;

	result.mDualQuat.x() = moment[0]*angle + direction[0]*pitch;
	result.mDualQuat.y() = moment[1]*angle + direction[1]*pitch;
	result.mDualQuat.z() = moment[2]*angle + direction[2]*pitch;
	result.mDualQuat.w() = 0.0;

	return result;

}
//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real> DualQuaternion<Real>::Exp () const
{
	//DualQuaternion<Real> result;
	//
	//Eigen::Matrix<Real, 2, 1> halfTheta = LengthDQ();

	//Eigen::Matrix<Real, 2, 1> sinTheta = Eigen::Matrix<Real, 2, 1>( Wm5::Math<Real>::Sin(halfTheta[0]),
	//	halfTheta[1] * Wm5::Math<Real>::Cos(halfTheta[0]) );
	//Eigen::Matrix<Real, 2, 1> cosTheta = Eigen::Matrix<Real, 2, 1>( Wm5::Math<Real>::Cos(halfTheta[0]),
	//	-halfTheta[1] * Wm5::Math<Real>::Sin(halfTheta[0]) );

	//DualQuaternion<Real> s = *this;
	//s.NormalizeDQ();


	//result = s * sinTheta;
	//result.mPrimeQuat.w() += cosTheta[0];
	//result.mDualQuat.w()  += cosTheta[1];

	//return result;

	//DualQuaternion<Real> result;
	//
	//result.mPrimeQuat = mPrimeQuat.Exp();
	//result.mDualQuat = result.mPrimeQuat * mDualQuat.Conjugate();

	//return result;

	DualQuaternion<Real> res;
	Eigen::Matrix<Real, 3, 1> n(mPrimeQuat.x(), mPrimeQuat.y(), mPrimeQuat.z());

	double half_angle = n.norm();

	// Pure translation?
	if (half_angle<1e-5)
		return DualQuaternion<Real>(Eigen::Quaternion<Real, Eigen::DontAlign>::Identity(), mDualQuat);

	// Get normalized dir
	Eigen::Matrix<Real, 3, 1> dir = (1.0/half_angle) * n;

	Eigen::Matrix<Real, 3, 1> d(mDualQuat.x(), mDualQuat.y(), mDualQuat.z());
	double half_pitch = d.Dot(dir);
	Eigen::Matrix<Real, 3, 1> mom = (d - dir*half_pitch) / half_angle;
	res.fromScrew(half_angle*2.0, half_pitch*2.0, dir, mom);

	return res;
}
//----------------------------------------------------------------------------
template <typename Real>
void DualQuaternion<Real>::FromTransRot (const Eigen::Matrix<Real, 3, 1>& t, const Eigen::Quaternion<Real, Eigen::DontAlign>& q0)
{
	//// non-dual part (just copy q0):
	//for (int i=0; i<4; i++) dq[0][i] = q0[i];
	//// dual part:
	//dq[1][0] = -0.5*(t[0]*q0[1] + t[1]*q0[2] + t[2]*q0[3]);
	//dq[1][1] = 0.5*( t[0]*q0[0] + t[1]*q0[3] - t[2]*q0[2]);s
	//dq[1][2] = 0.5*(-t[0]*q0[3] + t[1]*q0[0] + t[2]*q0[1]);
	//dq[1][3] = 0.5*( t[0]*q0[2] - t[1]*q0[1] + t[2]*q0[0]);

	mPrimeQuat = q0;

	mDualQuat.w() = -0.5*(t[0]*q0.x() + t[1]*q0.y() + t[2]*q0.z());
	mDualQuat.x() = 0.5*( t[0]*q0.w() + t[1]*q0.z() - t[2]*q0.y());
	mDualQuat.y() = 0.5*(-t[0]*q0.z() + t[1]*q0.w() + t[2]*q0.x());
	mDualQuat.z() = 0.5*( t[0]*q0.y() - t[1]*q0.x() + t[2]*q0.w());
}
//----------------------------------------------------------------------------
template <typename Real>
void DualQuaternion<Real>::ToTransRot (Eigen::Matrix<Real, 3, 1>& t, Eigen::Quaternion<Real, Eigen::DontAlign>& q0)const
{
	//// regular quaternion (just copy the non-dual part):
	//for (int i=0; i<4; i++) q0[i] = dq[0][i];
	//// translation vector:
	//t[0] = 2.0*(-dq[1][0]*dq[0][1] + dq[1][1]*dq[0][0] - dq[1][2]*dq[0][3] + dq[1][3]*dq[0][2]);
	//t[1] = 2.0*(-dq[1][0]*dq[0][2] + dq[1][1]*dq[0][3] + dq[1][2]*dq[0][0] - dq[1][3]*dq[0][1]);
	//t[2] = 2.0*(-dq[1][0]*dq[0][3] - dq[1][1]*dq[0][2] + dq[1][2]*dq[0][1] + dq[1][3]*dq[0][0]);

	q0 = mPrimeQuat;

	t[0] = 2.0*(-mDualQuat.w()*mPrimeQuat.x() + mDualQuat.x()*mPrimeQuat.w() - mDualQuat.y()*mPrimeQuat.z() + mDualQuat.z()*mPrimeQuat.y());
	t[1] = 2.0*(-mDualQuat.w()*mPrimeQuat.y() + mDualQuat.x()*mPrimeQuat.z() + mDualQuat.y()*mPrimeQuat.w() - mDualQuat.z()*mPrimeQuat.x());
	t[2] = 2.0*(-mDualQuat.w()*mPrimeQuat.z() - mDualQuat.x()*mPrimeQuat.y() + mDualQuat.y()*mPrimeQuat.x() + mDualQuat.z()*mPrimeQuat.w());
}
//----------------------------------------------------------------------------
template <typename Real>
void DualQuaternion<Real>::ApproximateToTransRot (Eigen::Matrix<Real, 3, 1>& t, Eigen::Quaternion<Real, Eigen::DontAlign>& q0)const
{
	//float len = 0.0;
	//for (int i=0; i<4; i++) len += dq[0][i] * dq[0][i];
	//len = sqrt(len);
	//for (int i=0; i<4; i++) q0[i] = dq[0][i] / len;
	//t[0] = 2.0*(-dq[1][0]*dq[0][1] + dq[1][1]*dq[0][0] - dq[1][2]*dq[0][3] + dq[1][3]*dq[0][2]) / len;
	//t[1] = 2.0*(-dq[1][0]*dq[0][2] + dq[1][1]*dq[0][3] + dq[1][2]*dq[0][0] - dq[1][3]*dq[0][1]) / len;
	//t[2] = 2.0*(-dq[1][0]*dq[0][3] - dq[1][1]*dq[0][2] + dq[1][2]*dq[0][1] + dq[1][3]*dq[0][0]) / len;

	q0 = mPrimeQuat;
	Real len = q0.squaredNorm();
	MYRELEASEASSERT(len>1e-8);
	Real sqrtlen = sqrt(len);
	q0.x() /= sqrtlen;
	q0.y() /= sqrtlen;
	q0.z() /= sqrtlen;
	q0.w() /= sqrtlen;

	t[0] = 2.0*(-mDualQuat.w()*mPrimeQuat.x() + mDualQuat.x()*mPrimeQuat.w() - mDualQuat.y()*mPrimeQuat.z() + mDualQuat.z()*mPrimeQuat.y()) / len;
	t[1] = 2.0*(-mDualQuat.w()*mPrimeQuat.y() + mDualQuat.x()*mPrimeQuat.z() + mDualQuat.y()*mPrimeQuat.w() - mDualQuat.z()*mPrimeQuat.x()) / len;
	t[2] = 2.0*(-mDualQuat.w()*mPrimeQuat.z() - mDualQuat.x()*mPrimeQuat.y() + mDualQuat.y()*mPrimeQuat.x() + mDualQuat.z()*mPrimeQuat.w()) / len;
}
//----------------------------------------------------------------------------
template <typename Real>
void DualQuaternion<Real>::toScrew (Real& rotation, Real& pitch, Eigen::Matrix<Real, 3, 1>& dir, Eigen::Matrix<Real, 3, 1>& moment)const
{
	ASSERT(isUnit());

	// See if it's a pure translation:
	if (!hasRotation())
	{
		rotation		= 0.0;
		dir[0]			= mDualQuat.x();
		dir[1]			= mDualQuat.y();
		dir[2]			= mDualQuat.z();

		double dir_sq_len = dir.SquaredLength();

		// If a translation is nonzero, normalize is
		// else leave <outDir> zero vector (no motion at all)
		if (dir_sq_len>1e-6)
		{
			double dir_len	= sqrt(dir_sq_len);
			pitch		= 2.0*dir_len;
			dir			/= dir_len;
		}
		else
			pitch		= 0.0;

		// Moment can be arbitrary
		moment			= Eigen::Matrix<Real, 3, 1>::Zero();
	}
	else
	{
		// Rigid transformation with a nonzero rotation
		rotation = 2.0*acos(mPrimeQuat.w());

		double s = mPrimeQuat.x()*mPrimeQuat.x() +
			mPrimeQuat.y()*mPrimeQuat.y() + mPrimeQuat.z()*mPrimeQuat.z();
		if (s<1e-6)
		{
			dir			= Eigen::Matrix<Real, 3, 1>::Zero();
			pitch		= 0.0;
			moment		= Eigen::Matrix<Real, 3, 1>::Zero();
		}
		else
		{
			double oos		= 1.0/sqrt(s);
			dir[0]		= mPrimeQuat.x() * oos;
			dir[1]		= mPrimeQuat.y() * oos;
			dir[2]		= mPrimeQuat.z() * oos;

			pitch		= -2.0*mDualQuat.w()*oos;

			moment[0] 	= mDualQuat.x();
			moment[1] 	= mDualQuat.y();
			moment[2] 	= mDualQuat.z();

			moment		= (moment - dir*pitch*mPrimeQuat.w()*0.5) * oos;
		}
	}

}
//----------------------------------------------------------------------------
template <typename Real>
void DualQuaternion<Real>::fromScrew (Real rotation, Real pitch, Eigen::Matrix<Real, 3, 1> dir, Eigen::Matrix<Real, 3, 1> moments)
{
	double sin_half_angle = sin(rotation*0.5);
	double cos_half_angle = cos(rotation*0.5);

	mPrimeQuat.w() = cos_half_angle;
	mPrimeQuat.x() = sin_half_angle*dir[0];
	mPrimeQuat.y() = sin_half_angle*dir[1];
	mPrimeQuat.z() = sin_half_angle*dir[2];

	pitch *= 0.5;
	mDualQuat.w() = -pitch*sin_half_angle;
	mDualQuat.x() = sin_half_angle*moments[0] + pitch*cos_half_angle*dir[0];
	mDualQuat.y() = sin_half_angle*moments[1] + pitch*cos_half_angle*dir[1];
	mDualQuat.z() = sin_half_angle*moments[2] + pitch*cos_half_angle*dir[2];
}
//----------------------------------------------------------------------------
template <typename Real>
DualQuaternion<Real> DualQuaternion<Real>::sScLERP(Real inT, const DualQuaternion<Real>& inFrom, const DualQuaternion<Real>& inTo)
{
	//DualQuaternion<Real> result;

	ASSERT(0.0<=inT && inT<=1.0);
	ASSERT(inFrom.isUnit() && inTo.isUnit());

	//// Make sure dot product is >= 0
	//Real quat_dot = inFrom.PrimalQuat().Dot(inTo.PrimalQuat());
	//quat_dot = 0;
	//DualQuaternion to_sign_corrected = (quat_dot>=0.0) ? inTo : -inTo;

	DualQuaternion dif_dq = inFrom.Inverse() * inTo;// * to_sign_corrected;

	Real  angle, pitch;
	Eigen::Matrix<Real, 3, 1> direction, moment;
	dif_dq.toScrew(angle, pitch, direction, moment);

	angle *= inT;
	pitch *= inT;
	dif_dq.fromScrew(angle, pitch, direction, moment);

	return inFrom * dif_dq;
}
