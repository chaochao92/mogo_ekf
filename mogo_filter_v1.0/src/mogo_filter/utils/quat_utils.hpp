#ifndef QUAT_UTILS_HPP_
#define QUAT_UTILS_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#ifndef TO_RAD
	#define TO_RAD M_PI/180
#endif 
#ifndef TO_DEG
	#define TO_DEG 180/M_PI
#endif

using namespace Eigen;


/*****************************************************************************
 *                        	Quaternion operation
 *****************************************************************************/

class QuatUtils
{
public:	
	// Body 3-2-1 rotation along the z-y-x axis of the input vector
	template<typename T>
	static Eigen::Quaternion<T> QuatFromYpr(const Matrix<T,3,1>& eular_angle)
	{
		AngleAxis<T> yawQ(eular_angle.z(), Matrix<T,3,1>::UnitZ());
		AngleAxis<T> pitQ(eular_angle.y(), Matrix<T,3,1>::UnitY());
		AngleAxis<T> rolQ(eular_angle.x(), Matrix<T,3,1>::UnitX());
		Quaternion<T> q = yawQ * pitQ * rolQ;
		if(q.w() <0)
		{
			q = QuatFlip(q);
		}
		return q;
	}


	// Return a 3D Vector(in radian) defining the Body 3-2-1 rotation along the z-y-x body axis
	template<typename T>
	static Eigen::Matrix<T,3,1> QuatToYpr(const Quaternion<T>& q)
	{
		Matrix<T,3,1> vec;
		T qw = q.w();
		T qx = q.x();
		T qy = q.y();
		T qz = q.z();
		vec[0] = atan2(2*(qy*qz+qw*qx),  qw*qw - qx*qx - qy*qy + qz*qz);
		vec[1] = -asin(2*(qx*qz-qw*qy));
		vec[2] = atan2(2*(qx*qy+qw*qz),  qw*qw + qx*qx - qy*qy - qz*qz);
		return vec;
	}

	
	// Return a 3D Vector(in radian) defining the Body 3-1-2 rotation along the z-x-y body axis
	template<typename T>
	static Eigen::Matrix<T,3,1> QuatToYrp(const Quaternion<T>& q)
	{
		Matrix<T,3,1> vec;
		T qw = q.w();
		T qx = q.x();
		T qy = q.y();
		T qz = q.z();
		vec[0] = atan2(-2*(qx*qz-qw*qy), qw*qw-qx*qx-qy*qy+qz*qz);
		vec[1] = asin(2*(qy*qz+qw*qx));
		vec[2] = atan2(-2*(qx*qy-qw*qz),  qw*qw - qx*qx + qy*qy - qz*qz);
		return vec;
	}


	// Convert a quaternion to rotation matrix defining the transformation from body to earth
	// Vec_in_earth = Tbn * Vec_in_body
	template<typename T>
	static Eigen::Matrix<T,3,3> QuatToTbn(const Quaternion<T>& q)
	{
		return q.matrix();
	}

	
	// Convert a quaternion to rotation matrix defining the transformation from earth to body
	// Vec_in_body = Tnb * Vec_in_earth
	template<typename T>
	static Eigen::Matrix<T,3,3> QuatToTnb(const Quaternion<T>& q)
	{
		return q.matrix().transpose();
	}


	// Find a Quaternion that rotates VecBody to VecWorld. 
	template<typename T>
	static Eigen::Quaternion<T> QuatFromTwoVec(const Matrix<T,3,1>& VecBody, const Matrix<T,3,1>& VecWorld)
	{
		Quaternion<T> q(0,0,0,0);
		// Check small value 
		if( ( VecBody.norm() < 1e-6 )|| (VecWorld.norm() < 1e-6 ) )
		{
			return q;
		}
		q.setFromTwoVectors(VecBody, VecWorld);
		return q;
	}
	
	
	// Convert a rotation matrix to a quaternion
	template<typename T>
	static Eigen::Quaternion<T> QuatFromRot(const Matrix<T,3,1>& Vec)
	{
		Quaternion<T> q;
		T vecLength = Vec.norm();
		if( vecLength < NumTraits<T>::dummy_precision() )
		{
			q.setIdentity();
		}else {
			Matrix<T,3,1> qxyz(Vec/vecLength*sin(0.5*vecLength));
			q = Quaternion<T>(cos(0.5*vecLength), qxyz.x(),qxyz.y(),qxyz.z());
		}
		return q;
	}


	// Flip a Quaternion
	template<typename T>
	static Eigen::Quaternion<T> QuatFlip(const Eigen::Quaternion<T> &q_in)
	{
		Quaternion<T> q_out;
		q_out.w() = -q_in.w();
		q_out.x() = -q_in.x();
		q_out.y() = -q_in.y();
		q_out.z() = -q_in.z();
		return q_out;
	}
};

#endif
