/*****************************************************************************
*  Filter RUN                      							                 *
*  @file     ekf_run.cpp                                                     *
*  @brief    how filter works when receive sensor data                       *
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

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
using namespace Eigen;
using namespace std;
void EKF::RunOnce(vector<double>& result)
{
    if(!_new_imu_meas)
	{
		return;
	}
	_new_imu_meas = false;
		
	//===== Predict and constrain states with IMU input =====
	_local_ms       +=  _imu_data.dt_ms;
	double dt_us    =   _imu_data.dt_ms;
	double dt_imu   =   dt_us;
	
	CoordUtils::NED2LLH( _states.pos, _refllh, &_curr_llh) ;
	double latitude = _curr_llh(0);
	double gravity;
	CoordUtils::GravityFromLLH(_curr_llh, &gravity);
	Vector3d delAngCorrected, delVelCorrected;
	
	// Predict and constrain states
	PredictStates( dt_imu, _imu_data.gyro, _imu_data.acc, &_states, gravity, latitude,
				   &delAngCorrected, &delVelCorrected);

	_sum_for_cov_ms += dt_us;		
	_del_ang += delAngCorrected;	
	_del_vel += delVelCorrected;	
	
    // Predict covariance
	if (_sum_for_cov_ms >= _ctr_param.covPredPrd_ms)
	{
		double dt = _sum_for_cov_ms;
		Vector3d avg_omega = _del_ang / dt;
		Vector3d avg_accel = _del_vel / dt;
		
		_covariance = PredictCovariance( dt, avg_omega, avg_accel,_states, _covariance, 
										 _ekf_status, _gps_status, _pdt_param);

		_del_ang.setZero();
		_del_vel.setZero();
		_sum_for_cov_ms = 0;
		
		_f_last_gps_fused = false;
		// ========== GPS ==========
		if (_new_gps_meas)
		{
			_new_gps_meas = false;
			if(_ekf_status == ATTMD )
			{				
				if( _gps_data.valid)
				{
					_gps_aos_cntr += 1;
					if(_gps_aos_cntr >= _ctr_param.gpsAosCntThr)
					{
						_states.gps_b.setZero();   // reset GPS bias
						Matrix3d Tbn = QuatUtils::QuatToTbn(_states.att);
						_states.pos = _gps_data.pos_ned - Tbn*_veh_param.Imu2GPS - _states.gps_b;
						_f_reinit_yaw = true;
						_f_gps_aos = true;
						_ekf_status = NO_YAW;
					}
				}
			}
			
			// Fuse GPS data when available if GPS use has started
			if( _f_gps_aos)
			{			
                // no multi-path
				if(!_f_is_elevator)
				{		             
					if( (_f_reinit_yaw )) 
					{
						Matrix3d Tbn = QuatUtils::QuatToTbn(_states.att);
						_states.pos = _gps_data.pos_ned - Tbn*_veh_param.Imu2GPS - _states.gps_b;
						_f_reinit_yaw = false;
					}	
					
					// GPS position update
					bool sucp = FuseGPSPos( &_states, 
                                            &_covariance,
										    _gps_data.pos_ned,
										    _gps_data.pos_sd.array().square().matrix(), 
										    _upd_param.gpsDriftSdGain,
										    _upd_param.gpsPosGate,
										    _veh_param.Imu2GPS,
										    &_gpsp_innov, &_gpsp_inno_var   );
					// status flag
					if(sucp)
					{
						_f_last_gps_fused = true;
					}
				} 
				_f_is_elevator = false;
			}

			// save result
            CoordUtils::NED2LLH(_states.pos, _refllh, &_curr_llh);
            //std::cout << _states.pos[0] << ", " << _states.pos[1] << ", " << _states.pos[2] << std::endl;
            //std::cout << _curr_llh[0] << ", " << _curr_llh[1] << ", " << _curr_llh[2] << std::endl;
            //std::cout << "STATUS: " << _ekf_status << std::endl;

			result.clear();
			result.push_back(_imu_data.dt_ms);
    		result.push_back(_states.pos[0]);
			result.push_back(_states.pos[1]);
			result.push_back(_states.pos[2]);            
		}					

		// ========== Status detector ==========
		if(_f_last_gps_fused)
		{
			_f_dead_gps = false;
			_f_last_dead_gps = _f_dead_gps;
			_gps_aos_cntr = 0;
			if( (!_f_reinit_yaw) && ((_ekf_status == NO_YAW)) )
			{
				_ekf_status = GPS_ON;
                std::cout << "GPS ON" << std::endl; 
			}
		}
	} 
} 

