#ifndef VEC_UTILS_HPP_
#define VEC_UTILS_HPP_

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
 *                        	Vector operation
 *****************************************************************************/

class VecUtils
{
public:
    
	// convert a 
	template<typename T>
	static Matrix<T,3,3> FormSkew(const Matrix<T,3,1>& x)
	{
		Matrix<T,3,3> m;		
		m <<    0, -x(2),  x(1),
		     x(2),     0, -x(0),
			-x(1),  x(0),     0;
		return m;
	}
	
	// Check if all elements is greater than a value
	template<typename T>
	static bool AllGreaterThan(const Matrix<T,3,1>& vec, const T& threshold)
	{
		bool res = true;
		for(int i=0;i<3;i++)
		{
			res &= (vec(i)>threshold);
		}
		return res;
	}
	
	// Check if all elements is less than a value
	template<typename T, int N>
	static bool AllLessThan(const Matrix<T,N,1>& vec, const T& threshold)
	{
		bool res = true;
		for(int i=0;i<N;i++)
		{
			res &= (vec(i)<threshold);
		}
		return res;
	}
	
	// convert yaw to a 2D Tbn 
	template<typename T>
	static Matrix<T,2,2> YawToTbn2d(const T & yaw)
	{
		Matrix<T,2,2> m;
		m << cos(yaw), -sin(yaw),
		     sin(yaw),  cos(yaw);
		return m;
	}
	
	// convert yaw to a 2D Tnb 
	template<typename T>
	static Matrix<T,2,2> YawToTnb2d(const T & yaw)
	{
		Matrix<T,2,2> m;
		m <<  cos(yaw), sin(yaw),
		     -sin(yaw), cos(yaw);
		return m;
	}
};

#endif
