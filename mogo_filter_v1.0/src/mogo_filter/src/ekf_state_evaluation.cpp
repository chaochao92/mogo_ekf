/*****************************************************************************
*  Detect GPS multi-path          		                					 *
*  @file     ekf_state_evaluation.cpp                                        *
*  @brief    Calcualte vertical movement, and compare with a threshold       *
*  true:     elevator detected                                               *
*  false:    no elevator detected                                            *
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
using namespace Eigen;

bool EKF::ElevatorDetector( const EKFStates& states, 
						    const CovMatrix& covariance,
						    const Vector3d& pos,
						    const double& z_tolorance_vel,
						    const Vector3d& Imu2gps,
						    const double& Gate          )
{
    // Estimate heading velocity
	Vector3d ypr = QuatUtils::QuatToYpr(states.att);
	
	// Vx in body "yaw" frame
	double vx_hat = (VecUtils::YawToTnb2d(ypr.z()) * states.vel.head<2>()).x();
	double sig_z = sqrt(covariance(8,8)); 
	double margin = Gate * (sig_z + z_tolorance_vel*abs(vx_hat));
	
	Vector3d antenna_pos = QuatUtils::QuatToTbn(states.att)*Imu2gps;
	if( abs( pos.z()-(states.pos.z()+antenna_pos.z()) ) > margin)
	{
		return true;  	//detected
	} else
	{
		return false;
	}
}
