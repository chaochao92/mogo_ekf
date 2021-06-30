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

    // TODO: add constrain operation after prediction
	// ConstrainStates(&_states, _covariance);
	
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
		_local_cov_ms += _sum_for_cov_ms;
		_sum_for_cov_ms = 0;
		
		_f_last_gps_fused = false;
		// ========== GPS ==========
		if (_new_gps_meas)
		{
			_new_gps_meas = false;
			// Re-Initialization, If we haven't started using GPS, check that 
			// the quality is sufficient before aligning the position and velocity states to GPS
			if( _ekf_status == DRMD )
			{
				_f_gps_aos = true;
			}
			else if( _ekf_status == DMPMD || _ekf_status == ATTMD )
			{				
				if( _gps_data.valid && 
					 VecUtils::AllLessThan(_gps_data.vel_sd, _ctr_param.gpsAosVelSdThr) &&
					 VecUtils::AllLessThan(_gps_data.pos_sd, _ctr_param.gpsAosPosSdThr) &&
					 (_gps_data.sat_cnt >= _ctr_param.gpsAosMinSatCnt))
				{
					_gps_aos_cntr += 1;
					if(_gps_aos_cntr >= _ctr_param.gpsAosCntThr)
					{
						_states.gps_b.setZero();   // reset GPS bias
						Matrix3d Tbn = QuatUtils::QuatToTbn(_states.att);
						_states.vel = _gps_data.vel_ned - Tbn*avg_omega.cross(_veh_param.Imu2GPS);
						_states.pos = _gps_data.pos_ned - Tbn*_veh_param.Imu2GPS - _states.gps_b;
						_last_fsmd_pos = _states.pos;
						_f_reinit_yaw = true;
						_f_gps_aos = true;
						_ekf_status = NO_YAW;
						_f_atmd_status = 0;
					}
				}
			}
			
			// Fuse GPS data when available if GPS use has started
			if( _f_gps_aos &&      
                _gps_data.valid &&
				(_gps_data.sat_cnt >= _upd_param.gpsMinSatCnt) &&
				(_gps_data.pos_sd.x() <= _upd_param.gpsPosErrLimH) &&
				(_gps_data.pos_sd.y() <= _upd_param.gpsPosErrLimH) &&
                (_gps_data.pos_sd.z() <= _upd_param.gpsPosErrLimV) )
			{
				if( _ctr_param.noelevator && _ekf_status != DMPMD &&
                    _ekf_status != ATTMD &&  _ekf_status != NO_YAW)
				{
					_f_is_elevator = ElevatorDetector(  _states, _covariance, 
													    _gps_data.pos_ned,
													    _ctr_param.elevatorZToleranceVel,
													    _veh_param.Imu2GPS,
													    _ctr_param.elevatorZGate);                               
				}
				
                // no multi-path
				if(!_f_is_elevator)
				{		             
					if( (_f_reinit_yaw ) ) //&& 
						//(_gps_data.vel_ned.head<2>().norm() >= 1) &&
						//(_gps_data.yaw_sd < _ctr_param.gpsYawInitSdThr) )
					{
						double new_y = _gps_data.yaw;
						Vector3d ypr = QuatUtils::QuatToYpr(_states.att);
						_states.att = QuatUtils::QuatFromYpr(Vector3d(ypr.x(), ypr.y(), new_y));
						
						// update position according to new yaw
						Matrix3d Tbn = QuatUtils::QuatToTbn(_states.att);
						_states.vel = _gps_data.vel_ned - Tbn*avg_omega.cross(_veh_param.Imu2GPS);
						_states.pos = _gps_data.pos_ned - Tbn*_veh_param.Imu2GPS - _states.gps_b;
						_last_fsmd_pos = _states.pos;
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
					
					// NED GPS velocity update
					bool sucv = FuseGPSVel( &_states, 
                                            &_covariance, 
											_gps_data.vel_ned, 
											_gps_data.vel_sd.array().square().matrix(),
											_upd_param.gpsVelGate,
											avg_omega, 
											pow(_pdt_param.gyroNoise,2), 
											_veh_param.Imu2GPS,
											&_gpsv_innov,&_gpsv_inno_var);
					
					// status flag
					if(sucp && sucv)
					{
                        _last_fused_gps_ms = _local_ms;
						_f_last_gps_fused = true;
						
						// GPS yaw update
						double h_spd = _states.vel.head<2>().norm();
                        if( _ctr_param.enableGpsYaw &&
                            h_spd > _upd_param.gpshVelLow &&
                            _gps_data.yaw_sd < _upd_param.gpsYawErrLim )
						{
							double hdg_sd = std::max(_gps_data.yaw_sd*4, _upd_param.gpshSdMin);
							double hdg_sd_scale = (_upd_param.gpshVelHigh-h_spd)*(_upd_param.gpshVelSdGain-1)/(_upd_param.gpshVelHigh-_upd_param.gpshVelLow)+1;
							hdg_sd = hdg_sd*std::max(1.0, hdg_sd_scale);  
							hdg_sd = hdg_sd*(1+ _upd_param.gpshAngSdGain*abs(avg_omega.z()) );
							double hvarg = pow(_upd_param.gpshXYSdGain,2);
							Vector3d yaw_var = Vector3d(hvarg, hvarg, 1)*pow(hdg_sd,2);
							bool such = FuseGPSYaw( &_states, 
                                                    &_covariance,
													_gps_data.yaw,
													yaw_var,
													_upd_param.gpsYawGate, 
													&_gpsh_innov,
													&_gpsh_inno_var );
						}
					}
				} 
				_f_is_elevator = false;
			}

            // TODO: add other observation (camera, lidar...)




			// save result
            CoordUtils::NED2LLH(_states.pos, _refllh, &_curr_llh);
            std::cout << _states.pos[0] << ", " << _states.pos[1] << ", " << _states.pos[2] << std::endl;
            std::cout << _curr_llh[0] << ", " << _curr_llh[1] << ", " << _curr_llh[2] << std::endl;
            std::cout << "STATUS: " << _ekf_status << std::endl;

			result.clear();
			result.push_back(_imu_data.dt_ms);
    		result.push_back(_states.pos[0]);
			result.push_back(_states.pos[1]);
			result.push_back(_states.pos[2]);

            // result.push_back(_imu_data.dt_ms);
    		// result.push_back(_states.pos[0]);
			// result.push_back(_states.pos[1]);
			// result.push_back(_states.pos[2]);
            
		}					

		// ========== Status detector ==========
		// State transition to GPS_ON
		if(_f_last_gps_fused)
		{
			_f_dead_gps = false;
			_f_last_dead_gps = _f_dead_gps;
			_gps_aos_cntr = 0;
			if( (!_f_reinit_yaw) && ((_ekf_status == NO_YAW) || (_ekf_status == DRMD)) )
			{
				// only move to next state if yaw init success
                // this also happeneds in DRMD
				_ekf_status = GPS_ON;
			}
		}
		
		double t_since_last_gps = (double)(_local_ms - _last_fused_gps_ms)*1e-6;
		// Dead gps detector
		if (t_since_last_gps > _ctr_param.gps_timeout*1000)
		{
			_f_dead_gps = true;
			_f_gps_aos = false;
		}		
		
		// State transition for dead GPS
		if(_f_dead_gps && !_f_last_dead_gps)
		{
			if(_ekf_status == GPS_ON) 
            {
				_ekf_status = DRMD;
			}
			else if(_ekf_status == NO_YAW) 
            {
				_ekf_status = DRMD;
			}
			_f_last_dead_gps = _f_dead_gps; 
		}
		
		// State transition to Dead Reckoning and Attitude mode
		double t_dr_lapsed = t_since_last_gps;
		if( _ekf_status == DRMD && t_dr_lapsed >= _ctr_param.dr_timeout*1000)
		{
			_f_atmd_status = 1;
			_ekf_status = DMPMD;
			_atmd1_start_ms = _local_ms;
			_atmd1_last_ms = _local_ms;
		}

		if ( t_dr_lapsed >= (_ctr_param.dr_timeout+_ctr_param.damping_len) *1000) 
		{
			_f_atmd_status = 2;
			_ekf_status = ATTMD;
		}				
		
		// ========== Dead DR ==========
		unsigned int  us_since_last_atmd(_local_ms-_last_fused_atmd_ms);
		if(_f_atmd_status > 0)  
		{
			if( (_f_atmd_status == 1)&&(us_since_last_atmd >= _ctr_param.damping_timeout*1000) )
			{
				// Damping mode
				double t_last_atmd1_left = (_atmd1_last_ms - _atmd1_start_ms)*1e-6;
				double t_since_last_atmd1 = (_local_ms-_atmd1_last_ms)*1e-6;
				Vector3d new_vel = ( 1-t_since_last_atmd1/((double)_ctr_param.damping_timeout*1000-t_last_atmd1_left) )*_states.vel;
				_states.vel = new_vel;
				_atmd1_last_ms = _local_ms;
				_last_fused_atmd_ms = _local_ms;
			}
			else if( (_f_atmd_status == 2)&&(us_since_last_atmd >= _ctr_param.atmd_timeout*1000) )
			{
				bool suc = FuseGPSPos( &_states, &_covariance,
										_last_fsmd_pos,
										_upd_param.atmdPosSd.array().square().matrix(),
										_upd_param.gpsDriftSdGain,
										_upd_param.atmdPosGate,
										Vector3d(0,0,0),
										&_atmd2p_innov, &_atmd2p_inno_var );
				_last_fused_atmd_ms = _local_ms;
			}
		}

		if (_f_atmd_status < 2 )
		{
			_last_fsmd_pos = _states.pos; 
		}
	} 
} 

