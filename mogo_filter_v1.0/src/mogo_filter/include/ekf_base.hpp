/*****************************************************************************
*  Filter base functions  	                    							 *		
*  @file     ekf_base.hpp                                                    *
*  @brief    define ekf virtual functions                                    *
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


#ifndef EKF_BASE_HPP_
#define EKF_BASE_HPP_

#include <memory>
#include "ekf_struct.hpp"
#include "ekf_param.hpp"
#include <vector>

using namespace std;
using namespace EKFStruct;

class EKFBase
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	typedef std::shared_ptr<EKFBase> FilterPtr;
	
	static FilterPtr CreateFilter();
	static FilterPtr CreateFilter(	const VehicleParam& veh_param, 
									const ControlParam& ctr_param, 
									const UpdateParam& 	upd_param, 
									const PredictParam& pdt_param, 
									const InitParam&    ini_param    );	

	virtual bool Init(const ImuData& imu, 
					  const unsigned long& init_ms, 
					  const Eigen::Vector3d& refllh) = 0;

	virtual bool ImuMeas(const ImuData& input) = 0;
	virtual bool GpsMeas(const GpsData& input) = 0;
	// TODO: add lane matching result

	virtual void RunOnce(vector<double>& result) = 0;
};

#endif
