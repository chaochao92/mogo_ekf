/*****************************************************************************
*  Create Filter	            		         							 *
*  @file     ekf_base_impl.cpp                                               *
*  @brief    Create Filter				                                     *
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


#include "ekf_base.hpp"
#include "ekf.h"

EKFBase::FilterPtr EKFBase::CreateFilter()
{
	std::shared_ptr<EKF> FilterPtr = std::allocate_shared<EKF>( Eigen::aligned_allocator<EKF>());
	return FilterPtr;
};

EKFBase::FilterPtr 
EKFBase::CreateFilter(	const VehicleParam& veh_param, 
						const ControlParam& ctr_param, 
						const UpdateParam& 	upd_param, 
						const PredictParam& pdt_param, 
						const InitParam&    ini_param   )
{
	std::shared_ptr<EKF> FilterPtr = 
		std::allocate_shared<EKF>(Eigen::aligned_allocator<EKF>(), 
			veh_param, ctr_param, upd_param, pdt_param, ini_param);

	return FilterPtr;
};