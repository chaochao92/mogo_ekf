/*****************************************************************************
*  Filter header  			    		         							 *
*  @file     ekf.c                 			                                 *
*  @brief    define filter class		                                     *
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

#ifndef EKF_HPP_
#define EKF_HPP_

#include "quat_utils.hpp"
#include "vec_utils.hpp"
#include "print_utils.hpp"
#include "coord_utils.hpp"


#include "ekf_base.hpp"
#include "ekf_param.hpp"
#include "ekf_struct.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;
using namespace EKFStruct;

class EKF : public EKFBase
{
	// friend class EKFTester;
	
public:

	//-------------------------------------------------------------------
	// ** ekf_base_impl.cpp
	//-------------------------------------------------------------------
	EKF();
	EKF(const VehicleParam& veh_param, 
		const ControlParam& ctr_param, 
		const UpdateParam&  upd_param,
		const PredictParam& pdt_param, 
		const InitParam&    ini_param );
	
	//-------------------------------------------------------------------
    // ** ekf_data_loader.cpp
	//-------------------------------------------------------------------
	bool Init(const ImuData& imu, const unsigned long& init_ms, const Eigen::Vector3d& refllh);
	bool ImuMeas(const ImuData& input);
	bool GpsMeas(const GpsData& input);
	
	//-------------------------------------------------------------------
	// ** ekf_run.cpp
	//-------------------------------------------------------------------
	void RunOnce(vector<double>& result);
	

private:
	// **************************** variables ***************************//
	// === ekf parameters
	VehicleParam 	_veh_param;
	ControlParam 	_ctr_param;
	UpdateParam 	_upd_param;
	PredictParam 	_pdt_param; 
	InitParam 		_ini_param;

	// === ekf status
	EKFStatus _ekf_status;

	// === imu
	bool _new_imu_meas;
	ImuData _imu_data;

	// === gps
	bool _new_gps_meas;
	GpsData _gps_data;
	GPSStatus _gps_status;

	// === init
	Vector3d _refllh;	
	double _init_ms; 
	EKFStates _states;
	CovMatrix _covariance;

	// === init state
	Vector3d _prev_gyro, _prev_acc;
	Matrix3d _prev_tbn;   	
	Vector3d _curr_llh;		

	Vector3d _last_fsmd_pos;
	Vector3d _del_ang; 	
	Vector3d _del_vel; 	
	Vector3d _last_avg_gyro; 
	Vector3d _last_avg_acc;	

	// === flag
	bool _f_dead_gps;
	bool _f_gps_aos;
	bool _f_last_dead_gps;
	bool _f_last_gps_fused;
	uint8_t _f_atmd_status;
	uint8_t _f_last_atmd_status;
	bool _f_reinit_yaw;
	bool _f_is_elevator;
	bool _f_last_is_elevator;

	// === aos control
	unsigned int _gps_aos_cntr;	

	// === time
	double _local_ms; 		
	double _local_cov_ms; 
	double _sum_for_cov_ms;	
	double _last_cov_dt_ms;	
	double _last_fused_gps_ms;
	double _last_fused_atmd_ms;
	double _atmd1_start_ms;
	double _atmd1_last_ms;

	// === update
	Vector3d _gpsv_innov, _gpsp_innov, _gpsh_innov ,_atmd2p_innov;
	Matrix3d _gpsv_inno_var, _gpsp_inno_var, _gpsh_inno_var,_atmd2p_inno_var;

	// **************************** functions ***************************//

	//-------------------------------------------------------------------
	// ** ekf_data_loader.cpp
	//-------------------------------------------------------------------
	GPSStatus GetGpsMode();

	//-------------------------------------------------------------------
	// ** ekf_initialization.cpp
	//-------------------------------------------------------------------
	bool InitStates(const ImuData& imu_data, EKFStates *states);
	bool InitCovariance(const InitParam &ini_param, CovMatrix *covariance);
	bool InitCtrlFlags();
	
	//-------------------------------------------------------------------
	// ** ekf_predict.cpp
	//-------------------------------------------------------------------
	void PredictStates(	const double& dt, 
					    const Vector3d& gyro, 
                        const Vector3d& acc,
					    EKFStates* states,
					    const double& gravity, 
                        const double& latitude,
				        Vector3d* correctedAng, 
                        Vector3d* correctedVel );

	CovMatrix PredictCovariance(const double& deltat, 
						        const Vector3d& gyro, 
                                const Vector3d& acc,
                                const EKFStates& states,
						        const CovMatrix& convariance,
						        const EKFStatus &flag,
						        const GPSStatus &gpsFlag,
						        const PredictParam& pdt_param);		

	//-------------------------------------------------------------------
	// ** ekf_cal_fd.cpp
	//-------------------------------------------------------------------
	void CalcFd(double dt, 
				const double (&quat)[4],	
				const double (&gyro)[3], 
				const double (&acc)[3], 
				double (&Fd)[324]);

	//-------------------------------------------------------------------
	// ** ekf_cal_qd.cpp
	//-------------------------------------------------------------------
	void CalcQd(double dt, 
			  	const double (&quat)[4],
			  	const double (&gyro)[3], 
			  	const double (&acc)[3], 
			  	const double (&gyroVar)[3], 
			  	const double (&accVar)[3],
			  	const double (&gyroBiasVar)[3], 
			  	const double (&accBiasVar)[3], 
			  	const double (&gpsBiasVar)[3], 
			  	double Qd[324]);
	
	//-------------------------------------------------------------------
	// ** ekf_update.cpp
	//-------------------------------------------------------------------
	bool FuseGPSVel(	EKFStates* states, 
                        CovMatrix* covariance, 
					    const Vector3d& measVel, 
                        const Vector3d& vel_var,
						const double &gateSize,
						const Vector3d& gyro, 
                        const double& gyro_var,
						const Vector3d& Imu2gps,
						Vector3d *innovation, 
                        Matrix3d *varInnov);	
	bool FuseGPSPos(	EKFStates* states, 
                        CovMatrix* covariance,
					    const Vector3d& measPos, 
                        const Vector3d& pos_var,
				        const double drift_gain,
					    const double &gateSize,
					    const Vector3d& Imu2gps,
					    Vector3d *innovation, 
                        Matrix3d *varInnov);	
	bool FuseGPSYaw(	EKFStates* states, 
                        CovMatrix* covariance,
					    const double& measYaw, 
                        const Vector3d& yaw_var,
					    const double &gateSize,
					    Vector3d *innovation, 
                        Matrix3d *varInnov);	

	template <int ROWS>
	bool MeasurementUpdate(	EKFStates* states, 
							CovMatrix* covariance, 
						    const Matrix<double,ROWS,18> &H,
						    const Matrix<double,ROWS,1> &innovation,
						    const Matrix<double,ROWS,ROWS> &varInnov,
						    const Matrix<double,ROWS,ROWS> &R      );

	//-------------------------------------------------------------------
	// ** ekf_state_evaluation.cpp
	//-------------------------------------------------------------------
	bool ElevatorDetector(	const EKFStates& states, 
						    const CovMatrix& covariance,
						    const Vector3d& pos,
						    const double& z_tolorance_vel,
						    const Vector3d& Imu2gps,
						    const double& Gate         );

};

#endif
