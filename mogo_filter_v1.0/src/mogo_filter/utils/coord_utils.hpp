#ifndef COORD_UTILS_HPP_
#define COORD_UTILS_HPP_

/*****************************************************************************
 *                        	Coordinate operation
 *****************************************************************************/

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <iostream>

#ifndef TO_RAD
	#define TO_RAD M_PI/180
#endif 
#ifndef TO_DEG
	#define TO_DEG 180/M_PI
#endif

#define WGS84_RAD 6378137				// earth radius
#define WGS84_FLAT (1/298.257223563) 	// earth flattening
#define GRAVITY_EQ 9.7803253359 		// gravity at equator
#define K_GRAV 0.00193185265241			// gravity formula constant = (b*gp - a*ge) / a*ge

using namespace Eigen;

class CoordUtils
{
public:
	/****************************************************************
	*                          LLH2NED()
	* Calculate local surface tangent cartisian coordinate given a 
	* latitude, longitude, Height w.r.t a reference earth LLH
	* Input 
	*   [llh]     (rad,rad,m) Target latitude, longitude, altitude
	*   [ref_llh] (rad,rad,m) Reference latitude, longitude, altitude
	* Output  
	*   [ned]     (m) North, East, Down position
	* Return
	*   [true]    Successful conversion
	*   [false]   Incorrect llh cordinates
	* Note
	*   For coordinates, the function uses WGS84 model. The model 
	*   includes a ellipsoidal earth, that the parameters are 
	*   defined above.
	*   The local tangent plane is defined assuming the Earth is a 
	*   Perfect sphere. The effect is, height will grow higher than 
	*   actual if going north bound. But the error is super small.
	*****************************************************************/
	template<typename T>
	static bool LLH2NED(const Matrix<T,3,1>& llh, 
						const Matrix<T,3,1> &ref_llh, 
						Matrix<T,3,1>* ned )
	{
		if( (abs(llh(0))>M_PI/2.0f) || (abs(ref_llh(0))>M_PI/2.0f) ||
			(abs(llh(1))>M_PI) ||(abs(ref_llh(1))>M_PI) )
		{
			//Error: out of range
			(*ned).setZero();
			return false;
		}
		
		//T e = sqrt(WGS84_FLAT*(2-WGS84_FLAT));
		/*T e2 = WGS84_FLAT * (2.0f-WGS84_FLAT);
		T Rm = WGS84_RAD * (1.0f-e2) / sqrt( pow( (1.0f-e2*pow(sin(ref_llh(0)),2)) ,3) );
		T Rn = WGS84_RAD / sqrt( 1.0f-e2*pow(sin(ref_llh(0)),2) );
		std::cout << "e2: " <<e2 << "Rm: " << Rm << "Rn: " << Rn << std::endl;
		(*ned)(0) = (llh(0)-ref_llh(0)) * (Rm+llh(2));
		(*ned)(1) = (llh(1)-ref_llh(1)) * (Rn+llh(2)) * cos(ref_llh(0));
		(*ned)(2) = -(llh(2)-ref_llh(2));*/
		
		T sinla = sin(ref_llh(0));
		T cosla = cos(ref_llh(0));
		T sinlo = sin(ref_llh(1)); 
		T coslo = cos(ref_llh(1));
		Matrix<T,3,3> M_ecef2ned;
		M_ecef2ned << -coslo*sinla, -sinlo*sinla,  cosla,
						    -sinlo,        coslo,      0,
				      -coslo*cosla, -sinlo*cosla, -sinla;
					 
		Matrix<T,3,1> tgt_ecef;
		Matrix<T,3,1> ref_ecef;
		LLH2ECEF(llh, &tgt_ecef);
		LLH2ECEF(ref_llh, &ref_ecef);
		(*ned) = M_ecef2ned * (tgt_ecef - ref_ecef);
		
		return true;
	}
	
	/****************************************************************
	*                          NED2LLH()
	* Calculate latitude, longitude, and height from a local surface 
	* tangent cartisian coordinates w.r.t a reference earth LLH
	* Input 
	*   [ned]     (m) North, East, Down position
	*   [ref_llh] (rad,rad,m) Reference latitude, longitude, altitude
	* Output  
	*   [llh]     (rad,rad,m) Target latitude, longitude, altitude
	* Return
	*   [true]    Successful conversion
	*   [false]   Incorrect llh cordinates
	* Note
	*   For coordinates, the function uses WGS84 model. The model 
	*   includes a ellipsoidal earth, that the parameters are 
	*   defined above.
	*   The local tangent plane is defined assuming the Earth is a 
	*   Perfect sphere. The effect is, height will grow higher than 
	*   actual if going north bound. But the error is super small.
	*****************************************************************/
	template<typename T>
	static bool NED2LLH(const Matrix<T,3,1>& ned, 
						const Matrix<T,3,1> &ref_llh, 
						Matrix<T,3,1>* llh )
	{
		// LLH: latitude, longitude, Hieght
		if ( (abs(ref_llh(0))>M_PI/2.0f)||(abs(ref_llh(1))>M_PI )  )
		{
			// Latitude is limited to [pi/2, -pi/2], longitude is limited to [pi, -pi]");  Error: out of range
				llh->setZero();
				return false;
		}

		T sinla = sin(ref_llh(0));
		T cosla = cos(ref_llh(0));
		T sinlo = sin(ref_llh(1)); 
		T coslo = cos(ref_llh(1));
		/*Matrix<T,3,3> M_ecef2enu <<  -sinlo     ,        coslo,   0.0,
									-sinla*coslo, -sinla*sinlo, cosla,
									cosla*coslo,  cosla*sinlo, sinla    ;
		M_enu2ecef = transpose(M_ecef2enu);*/
		Matrix<T,3,3> M_ned2ecef;
		M_ned2ecef << -coslo*sinla, -sinlo, -coslo*cosla,
					  -sinlo*sinla,  coslo, -sinlo*cosla,
					         cosla,      0,       -sinla;

		Matrix<T,3,1> ref_ecef;
		if(!LLH2ECEF(ref_llh, &ref_ecef)) {
			return false;
		}
		Matrix<T,3,1> V_ecef;
		V_ecef = M_ned2ecef * ned + ref_ecef;
		T r2 = (V_ecef.template block<2,1>(0,0)).dot(V_ecef.template block<2,1>(0,0));

		// WGS84 model config
		//T e = sqrt(WGS84_FLAT*(2-WGS84_FLAT));
		//T e2 = pow(e,2);
		T e2 = WGS84_FLAT*(2-WGS84_FLAT);

		T z = V_ecef.z();
		T z_last = 0;
		T v = 0;
		T dist_ratio;
		while ( abs(z-z_last) > 1e-9 )
		{
			z_last = z;    
			dist_ratio = z/sqrt(r2+pow(z,2));
			v = WGS84_RAD / sqrt(1.0f-e2*pow(dist_ratio,2));
			z = V_ecef.z() + v*e2*dist_ratio;
		}

		T Lat, Lon, Alt;
		if(r2 > 1e-12)
		{
			Lat = atan(z/sqrt(r2));
			Lon = atan2(V_ecef.y(), V_ecef.x());
		} else
		{
			if( V_ecef.z() > 0) {
				Lat = M_PI/2.0f;
			}else{
				Lat = -M_PI/2.0f;
			}
			Lon = 0;
		}
		Alt = sqrt(r2 + pow(z,2))-v;
		(*llh) << Lat, Lon, Alt;
		return true;
	}
	
	/****************************************************************
	*                          LLH2ECEF()
	* Calculate an ECEF coordinate base on latitude, longitude, and 
	* height
	* Input 
	*   [llh]  (rad,rad,m) Reference latitude, longitude, altitude
	* Output  
	*   [ecef] (m) ECEF coordinates
	* Return
	*   [true]    Successful conversion
	*   [false]   Incorrect llh cordinates
	* Note
	*   For coordinates, the function uses WGS84 model. The model 
	*   includes a ellipsoidal earth, that the parameters are 
	*   defined above.
	*   ECEF frame has X-axis point toward 0-deg latitude at prime 
	*   meridian; Z-axis point toward the Geographic North Pole; 
	*   right handed coordinates. 
	*****************************************************************/
	template<typename T>
	static bool LLH2ECEF(const Matrix<T,3,1>& llh,  Matrix<T,3,1>* ecef )
	{
		if(abs(llh(0))>M_PI/2.0f || abs(llh(1))>M_PI)
		{
			//error("Latitude is limited to [pi/2, -pi/2], longitude is limited to [pi, -pi]"); 
			ecef->setZero();
			return false;
		}

		//llh: latitude, longitude, Hieght
		T Lat = llh(0);
		T Lon = llh(1);
		T Alt = llh(2);

		// WGS84 model config
		//e = sqrt(WGS84_FLAT*(2-WGS84_FLAT));
		//e2 = e^2;
		T e2 = WGS84_FLAT*(2-WGS84_FLAT);
		
		T v = WGS84_RAD / sqrt( 1-e2*pow(sin(Lat),2) );

		(*ecef)(0) = (Alt + v)* cos(Lat) * cos(Lon);
		(*ecef)(1) = (Alt + v)* cos(Lat) * sin(Lon);
		(*ecef)(2) = (Alt + v*(1.0f-e2))* sin(Lat);

		return true;
	}
	
	/****************************************************************
	*                           GravityFromLLH()
	* Calculate the gravity accroding to latitude
	* Input: llh Latitude(rad), Longitude(rad) Altitude(m)
	* Output:  
	* Return: (none)
	* Note: https://en.wikipedia.org/wiki/Theoretical_gravity
	* input. The two-step process of prediction(propagation) and 
	* measurement(update) is (selectively) executed automatically 
	* within this function.
	*****************************************************************/
	template<typename T>
	static bool GravityFromLLH(const Matrix<T,3,1>&llh, T *gravity)
	{
		if(abs(llh(0))>M_PI/2.0f || abs(llh(1))>M_PI)
		{
			(*gravity) = 0;
			return false;
		}
		T e_sq = WGS84_FLAT*(2-WGS84_FLAT);
		T sin_lat_sq = pow(sin(llh(0)),2);
		(*gravity) = GRAVITY_EQ *((1 + K_GRAV * sin_lat_sq) / sqrt( 1-e_sq*sin_lat_sq));
		return true;
	}
	
};

#endif
