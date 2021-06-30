/*****************************************************************************
*  Initialize filter state/covariance/flags	         						 *
*  @file     ekf_initialization.cpp                                          *
*  @brief    Initialize filter state/covariance/flags	                     *
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


#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#include "ekf.h"

#define GRAVITY 9.80665

using namespace Eigen;

//-------------------------------------------------------------------
// Init states
//-------------------------------------------------------------------
bool EKF::InitStates(const ImuData& imu_data, EKFStates *states)
{
    // states
    states->att.setIdentity();
	states->vel.setZero();
	states->pos.setZero();
	states->gyro_b.setZero();
	states->acc_b.setZero();
	states->gps_b.setZero();

    // align with gravity
	if( (imu_data.acc.norm() > 1.2*GRAVITY) || (imu_data.acc.norm() < 0.8*GRAVITY))
	{
		;
	}else{
		states->att = states->att * (QuatUtils::QuatFromTwoVec(imu_data.acc, Vector3d(0,0,-GRAVITY)));
	}
	
	_prev_gyro = imu_data.gyro;
	_prev_acc = imu_data.acc;
	_prev_tbn = QuatUtils::QuatToTbn(states->att);
	_curr_llh.setZero();
	
	_del_ang.setZero(); 
	_del_vel.setZero();  
	_last_avg_gyro.setZero();
	_last_avg_acc.setZero();	

	_last_fsmd_pos = states->pos;
	
	return true;
}

//-------------------------------------------------------------------
// Init covariance
//-------------------------------------------------------------------
bool EKF::InitCovariance(const InitParam &ini_param, CovMatrix *covariance)
{
    for(int row = 0; row<18; row++){
		for(int col = 0; col<18; col++){
			(*covariance)(row,col) = 0;
		}
	}	

	for(int i=0;i<3;i++)
	{
		(*covariance)(i,i)          = pow(ini_param.angErr(i),2);
		(*covariance)(i+3,i+3)      = pow(ini_param.velErr(i),2);
		(*covariance)(i+6,i+6)      = pow(ini_param.posErr(i),2);
		(*covariance)(i+9,i+9)      = pow(ini_param.gyroBiasErr,2);
		(*covariance)(i+12,i+12)    = pow(ini_param.accBiasErr,2);
		(*covariance)(i+15,i+15)    = pow(ini_param.gpsBiasErr*0.1,2);
	}
	
	return true;	
}

//-------------------------------------------------------------------
// Init flags
//-------------------------------------------------------------------
bool EKF::InitCtrlFlags()
{
    // flag
    _f_dead_gps         = true;
	_f_last_dead_gps    = true;
	_f_gps_aos          = false;
	_f_last_gps_fused   = false;	

	_f_atmd_status      = 2;
	_f_last_atmd_status = 2;	

	_f_reinit_yaw       = true;

	_f_is_elevator      = false;
	_f_last_is_elevator = false;
	
    // data
	_new_imu_meas       = false;
	_new_gps_meas       = false;

    // counter
	_gps_aos_cntr       = 0;

    // time
    _init_ms            = 0; 			
	_local_ms           = 0; 			
	_local_cov_ms       = 0; 	
	_sum_for_cov_ms     = 0;	
	_last_cov_dt_ms     = 0;
	_last_fused_gps_ms  = 0;
	_last_fused_atmd_ms = 0;
	_atmd1_start_ms     = 0;
	_atmd1_last_ms      = 0;

    return true;
}
