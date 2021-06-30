
/*****************************************************************************
*  Sensor parameters and filter related parameters  	                     *
*  @file     ekf_param.hpp                                                   *
*  @brief    define ekf parameters                                           *
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

#ifndef EKF_PARAM_HPP_
#define EKF_PARAM_HPP_

#include <Eigen/Geometry>
#include <math.h>
#include "ekf_struct.hpp"

#ifndef TO_RAD
	#define TO_RAD M_PI/180.0
#endif

using namespace Eigen;
using namespace EKFStruct;

namespace EKFStruct
{
	
//-------------------------------------------------------------------
// Vehicle parameters
//-------------------------------------------------------------------
struct VehicleParam
{
	Vector3d Imu2GPS = Vector3d(0, 0, 0);		// Extrinsic parameters of loc and GPS
	public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


//-------------------------------------------------------------------
// Filter states transition parameters
//-------------------------------------------------------------------
struct ControlParam
{
	double covPredPrd_ms = 0.02;							// accumulated time to predict convariance

	// GPS control 
	unsigned int gpsAosCntThr 		= 3; 					// # of valid gps signal to switch to GPS_ON Status
	unsigned int gpsAosMinSatCnt 	= 10; 					// # of satellite to define a valid GPS signal
	double gpsAosVelSdThr 			= 2.0; 					// Thre to define a valid velocity (m/s,1sd)
	double gpsAosPosSdThr 			= 1.8; 					// Thre to define a valid position (m,1sd) 
	double gpsYawInitSdThr 			= 8.0*TO_RAD;			// Thre to define a valid yaw (rad)

	// Time control 
	double gps_timeout 			= 1.5;						// no valid GPS time (s)
	double dr_timeout 			= 30; 						// DR time (s)
	double damping_len 			= 8;  						// velocity damping time (s)
	double damping_timeout 		= 0.2;						// damping mode timeout (s)
	double atmd_timeout 		= 1;						// attitude mode timeout (s)

	// Multipath control 
	bool noelevator = true;     							// multipath happen?
	double elevatorZToleranceVel = 0.01;					// movement tolerance in z direction for elevator detection,
															// (m/(m/s),1sd)
	double elevatorZGate = 1.5;       						// GPS jump in z direction (reject thre) (m,1sd)
	
	// Gps yaw fusion ON/OFF
	bool enableGpsYaw = true;								// Enable GPS yaw update
};

//-------------------------------------------------------------------
//  Observation update parameters
//-------------------------------------------------------------------
struct UpdateParam
{
	// GPS 
	unsigned int gpsMinSatCnt 	= 8; 	      				// Minimum # of satellite to update a GPS observation
	double gpsPosErrLimH 		= 1.5;    					// Update GPS horizontal position if err below this limit (1SD, m)
	double gpsPosErrLimV 		= 2.0;   					// Update GPS vertical position if err below this limit (1SD, m)
	double gpsYawErrLim 		= 3.0*TO_RAD; 				// Update GPS yaw if err below this limit (1sd, rad) 
	double gpsVelGate 			= 5.0; 						// sd ratio of GPS velocity innovation outlier check
	double gpsPosGate 			= 4.0; 						// sd ratio of GPS position innovation outlier check
	double gpsYawGate 			= 4.0;						// sd ratio of GPS yaw innovation outlier check
	
	double gpshSdMin 			= 1.0*TO_RAD; 	
	double gpshVelLow 			= 8;						// (m/s) yaw upadte started at this speed, with expanded SD
	double gpshVelHigh 			= 15;						// (m/s) yaw upadte using original sd above this speed
	double gpshVelSdGain 		= 3;						// (sd)  yaw sd scaling factor at lowest speed
	double gpshAngSdGain 		= 1.5/(10*TO_RAD);			// (sd/(rad/s)) yaw sd scaling factor by yaw rate
	double gpshXYSdGain 		= 3;						// sd ratio of x and y axis of yaw measurement
	double gpsDriftSdGain 		= 1.02;						// sd ration of GPS pos err because of drift	
	
	// Attitude mode 
	Vector3d atmdPosSd 			= Vector3d(30, 30, 30); 	// virtual position measurement sd (m,1sd)
	double atmdPosGate 			= 8.0;						// SD gate size of the fake position innovation outlier check

	public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

//-------------------------------------------------------------------
//  Predict parameters
//-------------------------------------------------------------------
struct PredictParam
{
	double gyroNoise 			= 0.00175; 					// (rad/s) IMU gyroscope white noise
	double accNoiseX 			= 0.075;					// (m/s^2) IMU accelerometer X white noise
	double accNoiseY 			= 0.075;					// (m/s^2) IMU accelerometer Y white noise
	double accNoiseZ 			= 0.15;						// (m/s^2) IMU accelerometer Z white noise
	double gyroBiasNoise 		= 0.0000035;				// ((rad/s)/s) IMU gyro random walk 
	double accBiasNoise 		= 0.0003;					// ((m/s^2)/s) IMU acc random walk  
	double limGyroBias 			= 1.0*TO_RAD;				// (rad/s) IMU gyro bias limit
	double limAccBias 			= 0.4;						// (m/s^2) IMU acc bias limit
	double gpsBiasNoise[GPSStatus::POSFIXCNT];				// (m/s) GPS bias 
	double limGpsBias[GPSStatus::POSFIXCNT]; 				// (m) GPS bias limit
	double gpsBiasSdInj[GPSStatus::POSFIXCNT]; 				// (m) GPS bias limit

	PredictParam()
	{
		// GPS bias
		gpsBiasNoise[GPSStatus::NOGPS]  = 0;
		gpsBiasNoise[GPSStatus::GPS3D]  = 0.8/50;
		gpsBiasNoise[GPSStatus::DIFGPS] = 0.5/50;
		gpsBiasNoise[GPSStatus::RTKFLT] = 0.3/50;
		gpsBiasNoise[GPSStatus::RTKFIX] = 0.1/50;
		
		// GPS bias limit
		limGpsBias[GPSStatus::NOGPS]  = 20;
		limGpsBias[GPSStatus::GPS3D]  = 10;
		limGpsBias[GPSStatus::DIFGPS] = 15;
		limGpsBias[GPSStatus::RTKFLT] = 4.5;
		limGpsBias[GPSStatus::RTKFIX] = 2.5;
		
		gpsBiasSdInj[GPSStatus::NOGPS]  = 0;
		gpsBiasSdInj[GPSStatus::GPS3D]  = 1.5;
		gpsBiasSdInj[GPSStatus::DIFGPS] = 1.5;
		gpsBiasSdInj[GPSStatus::RTKFLT] = 1.0;
		gpsBiasSdInj[GPSStatus::RTKFIX] = 0.75;
	}
};

//-------------------------------------------------------------------
// Filter initial parameters
//-------------------------------------------------------------------
struct InitParam
{
	Vector3d angErr 		= Vector3d(0.1, 0.1, 0.523);		// Initial uncertainty in quaternion.
	Vector3d velErr 		= Vector3d(5.0, 5.0, 5.0);			// Initial velocity error when aligning without GPS. (m/sec)
	Vector3d posErr 		= Vector3d(10.0, 10.0, 10.0);		// Initial position error when aligning without GPS. (m/sec)
	
	double gyroBiasErr 		= 0.1*TO_RAD; 						// Initial gyro bias uncertainty (rad/s)
	double accBiasErr 		= 0.1;								// Initial accelerometer bias uncertainty (m/s^2)
	double gpsBiasErr 		= 0.075;							// Initial GPS bias uncertainty (m)
	double initDuration 	= 3;								// Initialization time (s)
	public:
  	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}
#endif
