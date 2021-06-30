
/*****************************************************************************
*  Sensor data sturcture and filter related data sturcture                   *
*  @file     ekf_struct.hpp                                                  *
*  @brief    define ekf structure                                            *
*  Details.                                                                  *
*                                                                            *
*  @author   Yan li                                                          *
*  @email    liyan1@mogoauto.com                                             *
*  @version  1.0.0.1                                                         *
*  @date     2021.06.21                                                      *
*                                                                            *
*----------------------------------------------------------------------------*
*  Remark         : Description                                              *
*----------------------------------------------------------------------------*
*  Change History :                                                          *
*  <Date>     | <Version> | <Author>       | <Description>                   *
*----------------------------------------------------------------------------*
*  2021.06.21 | 1.0.0.1   | Yan Li         | Create file                     *
*----------------------------------------------------------------------------*
*                                                                            *
*****************************************************************************/

#ifndef EKF_STRUCT_HPP_
#define EKF_STRUCT_HPP_

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
namespace EKFStruct
{

//-------------------------------------------------------------------
// Filter States
//-------------------------------------------------------------------
struct EKFStates
{
	Eigen::Quaternion<double> att;      // attitude
	Eigen::Vector3d vel;                // velocity
	Eigen::Vector3d pos;                // position
	Eigen::Vector3d gyro_b;             // gyroscope bias
	Eigen::Vector3d acc_b;              // accelerometer  bias
	Eigen::Vector3d gps_b;              // gps bias
	public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

typedef Eigen::Matrix<double,18,18> CovMatrix;
	
struct ImuData
{
	double dt_ms; 	                    // time in ms
	Eigen::Vector3d gyro;  	            // gyroscope reading (rad/s)
	Eigen::Vector3d acc; 	            // accelerometer reading  (m/s^2)
	public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct GpsData
{
    double dt_ms; 					    // time in ms
	Eigen::Vector3d pos_ned; 	        // position (m)
	Eigen::Vector3d vel_ned;  	        // velocity (m/s)
	double yaw;					        // yaw (rad)
	Eigen::Vector3d pos_sd; 	        // (m,1SD) for position
	Eigen::Vector3d vel_sd; 	        // (m/s,1SD) for velocity
	double yaw_sd;				        // (rad,1SD) for yaw
	unsigned int sat_cnt; 		        // # of satellite
	bool valid;					        // if current data is valid
	public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

enum EKFStatus
{
	UNINIT, 		                    // Un-initialized
	NO_YAW, 		                    // Aligned imu with gravity
	GPS_ON,			                    // Valid GPS signal in
	DRMD,			                    // Dead reckoning
	DMPMD,		                        // Damping mode
	ATTMD,		                        // Attitude mode
	STATUSCNT
};

enum GPSStatus
{
    NOGPS,
	GPS3D,			                    // Point positioning
	DIFGPS,		                        // Differencial 
	RTKFLT,		                        // RTK float
	RTKFIX,		                        // RTK fixed
	POSFIXCNT	
};

}
#endif
