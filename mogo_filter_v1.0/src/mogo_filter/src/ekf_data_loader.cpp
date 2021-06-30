/*****************************************************************************
*  Initialize filter and read sensor data         		 					 *
*  @file     ekf_data_loader.cpp                                             *
*  @brief    Initialize filter and read sensor data                          *
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

//-------------------------------------------------------------------
// Filter Constructor
//-------------------------------------------------------------------
EKF::EKF(   const VehicleParam& veh_param, 
			const ControlParam& ctr_param, 
			const UpdateParam&  upd_param,
			const PredictParam& pdt_param, 
			const InitParam&    ini_param ) : EKF()
{
    _veh_param = veh_param;
	_ctr_param = ctr_param;
	_upd_param = upd_param;
	_pdt_param = pdt_param;
	_ini_param = ini_param;
}

EKF::EKF() 
{
	_ekf_status = UNINIT;
}

//-------------------------------------------------------------------
// Filter init: state/covariance/flags
//-------------------------------------------------------------------
bool EKF::Init( const ImuData& imu, 
			    const unsigned long& tinit_ms, 
			    const Eigen::Vector3d& refllh) 
{ 
    if( abs(refllh(0)) > M_PI / 2 || abs(refllh(1)) > M_PI)
	{
		return false;
	}
	
	if( InitStates(imu, &_states) && 
		InitCovariance(_ini_param, &_covariance) && 
		InitCtrlFlags() )
	{		
		_init_ms = tinit_ms;
		_ekf_status = ATTMD;
		_gps_status = NOGPS;
        std::cout << "STATUS: " <<  _ekf_status << std::endl;
		_refllh = refllh;
		return true;
	}else {
		return false;
	}
};


//-------------------------------------------------------------------
// New IMU data in 
//-------------------------------------------------------------------
bool EKF::ImuMeas(const ImuData& input)
{
    if(input.acc.norm() != 0)
	{
		_imu_data = input;
		_new_imu_meas = true;
		return true;
	}
	return false;
}

//-------------------------------------------------------------------
// New GPS data in 
//-------------------------------------------------------------------
bool EKF::GpsMeas(const GpsData& input)
{
	if( (input.pos_ned.norm() != 0) && (input.valid) )
	{
		_gps_data = input;
		_new_gps_meas = true;
		return true;
	}
	return false;
}

//-------------------------------------------------------------------
// Check GPS Status
//-------------------------------------------------------------------
EKFStruct::GPSStatus EKF::GetGpsMode()
{
	return GPSStatus::RTKFIX;
}