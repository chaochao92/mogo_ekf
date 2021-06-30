/*****************************************************************************
*  Filter prediction step       							                 *
*  @file     ekf_predict.cpp                                                 *
*  @brief    predict states and covariance separately                        *
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

#include "ekf.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

//-------------------------------------------------------------------
// Predict States
//-------------------------------------------------------------------
void EKF::PredictStates(const double& dt, 
					    const Vector3d& gyro, 
                        const Vector3d& acc,
					    EKFStates* states,
					    const double& gravity, 
                        const double& latitude,
				        Vector3d* correctedAng, 
                        Vector3d* correctedVel )
{
    // Remove IMU white noise
	Vector3d delAng = ((gyro+_prev_gyro)/2 - states->gyro_b)*(double)dt;
	Vector3d delVel = ((acc+_prev_acc)/2 - states->acc_b)*(double)dt;
	
	(*correctedVel) = delVel;

    // Apply corrections for coning errors and earth spin rate
    // REF: "A new strapdown attitude algorithm", 
	// R. B. MILLER, 
	// Journal of Guidance, Control, and Dynamics
	// July, Vol. 6, No. 4, pp. 287-291, Eqn 11 
	Vector3d delAngEarth_NED(0.000072921 * cos(latitude) * dt,
							 0.0,
							 -0.000072921 * sin(latitude) * dt);
	
	(*correctedAng) = delAng - _prev_tbn.transpose()*delAngEarth_NED;
	
	_prev_gyro = gyro;
	_prev_acc = acc;

    // convert to quaternion
	Quaterniond deltaQuat = QuatUtils::QuatFromRot((*correctedAng));

    // **att
	states->att = states->att * deltaQuat;
	states->att.normalize();	
	
	// Calculate the body to nav cosine matrix
	Matrix3d Tbn = QuatUtils::QuatToTbn(states->att);
	_prev_tbn= Tbn;

	// transform body delta velocities to delta velocities in the nav frame
	Vector3d delVelNav = Tbn* (*correctedVel) + Vector3d(0,0,(double)gravity*dt);

	// take a copy of the previous velocity
	Vector3d prevVel = states->vel;

    // **vel
	states->vel += delVelNav;
    // **pos
	states->pos += 0.5 * dt * (prevVel + states->vel);
}


//-------------------------------------------------------------------
// Predict covariance
//-------------------------------------------------------------------
CovMatrix EKF::PredictCovariance(   const double& dt, 
						            const Vector3d& gyro, 
                                    const Vector3d& acc,
                                    const EKFStates& states,
						            const CovMatrix& convariance,
						            const EKFStatus &flag,
						            const GPSStatus &gpsFlag,
						            const PredictParam& pdt_param )
{
	double quat_arr[4], gyro_arr[3], acc_arr[3];
	quat_arr[0] = states.att.w();
	quat_arr[1] = states.att.x();
	quat_arr[2] = states.att.y();
	quat_arr[3] = states.att.z();

	for(int i=0;i<3;i++)
	{
		gyro_arr[i] = gyro(i);
		acc_arr[i]  = acc(i);
	}

	// fd
	double fd_arr[324];
	CalcFd(dt, quat_arr, gyro_arr, acc_arr, fd_arr);

	Vector3d rotVar = pow(pdt_param.gyroNoise,2) *Vector3d(1,1,1);
	Vector3d accVar = Vector3d(pdt_param.accNoiseX,pdt_param.accNoiseY,pdt_param.accNoiseZ).array().pow(2);
	Vector3d gyroBiasVar;
	Vector3d accBiasVar;
	Vector3d gpsBiasVar;
	if( flag == EKFStatus::GPS_ON)
	{
		gyroBiasVar = pow(pdt_param.gyroBiasNoise,2) *Vector3d(1,1,1);
		accBiasVar = pow(pdt_param.accBiasNoise,2) *Vector3d(1,1,1);
	}
	else
	{
		gyroBiasVar.setZero();
		accBiasVar.setZero();
	}	

	double gyroVar_arr[3], accVar_arr[3],gyroBiasVar_arr[3],accBiasVar_arr[3], gpsBiasVar_arr[3];
	for(int i=0;i<3;i++)
	{
		gyroVar_arr[i] = rotVar(i);
		accVar_arr[i] = accVar(i);
		gyroBiasVar_arr[i] = gyroBiasVar(i);
		accBiasVar_arr[i] = accBiasVar(i);
		gpsBiasVar_arr[i] = gpsBiasVar(i);
	}

    // qd
	double qd_arr[324];
	CalcQd( dt, quat_arr, gyro_arr, acc_arr, gyroVar_arr, accVar_arr, 
            gyroBiasVar_arr, accBiasVar_arr, gpsBiasVar_arr, qd_arr);
	

	CovMatrix Fd = Map<Matrix<double,18,18>>(fd_arr);
	CovMatrix Qd = Map<Matrix<double,18,18>>(qd_arr);

    // calculate and validate covariance matrix
	CovMatrix conv_pred;
	conv_pred = Fd*convariance*Fd.transpose() + Qd;
	conv_pred = 0.5*(conv_pred + conv_pred.transpose());
	for(int i=0;i<18;i++)
	{
		if (conv_pred(i,i) < 0)
		{
			conv_pred(i,i) = 0;
		}
	}
	return conv_pred;
}


