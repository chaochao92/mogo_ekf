/*****************************************************************************
*  Filter update                      							             *
*  @file     ekf_update.cpp                                                  *
*  @brief    Filter updates when receive GPS data                            *
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
using namespace Eigen;

//-------------------------------------------------------------------
// Update GPS velocity
//-------------------------------------------------------------------
bool EKF::FuseGPSVel(   EKFStates* states, 
                        CovMatrix* covariance, 
					    const Vector3d& measVel, 
                        const Vector3d& vel_var,
						const double &gateSize,
						const Vector3d& gyro, 
                        const double& gyro_var,
						const Vector3d& Imu2gps,
						Vector3d *innovation, 
                        Matrix3d *varInnov)
{
    Matrix3d I3 = Matrix3d::Identity();
	Matrix3d O3 = Matrix3d::Zero();
	Matrix3d Tbn = QuatUtils::QuatToTbn(states->att);
	Matrix3d gyro_r = gyro_var * I3;

	(*innovation) = measVel - (states->vel + Tbn*gyro.cross(Imu2gps));
		
	Matrix<double,3,18> H;
	H.block<3,3>(0,0) = -Tbn*VecUtils::FormSkew(gyro)*VecUtils::FormSkew(Imu2gps);
	H.block<3,3>(0,3) = I3;
	H.block<3,3>(0,6) = O3;
	H.block<3,3>(0,9) = Tbn*VecUtils::FormSkew(Imu2gps);
	H.block<3,3>(0,12) = O3;
	H.block<3,3>(0,15) = O3;
	Matrix3d R = Matrix3d(vel_var.asDiagonal()) +
		Tbn*VecUtils::FormSkew(Imu2gps)*gyro_r*(VecUtils::FormSkew(Imu2gps).transpose())*(Tbn.transpose());

	(*varInnov) = H * (*covariance) * H.transpose() + R;

	Vector3d correction_ratio;
	for(int i=0; i<3; i++) {
		correction_ratio(i) = pow((*innovation)(i),2) / (pow(gateSize,2)*(*varInnov)(i,i));
	}
	bool check = VecUtils::AllLessThan(correction_ratio, (double)1.0);
	if (!check)
	{
		//std::cout << "Fuse GPS velocity Failed!" << std::endl;
		//return false;
	}
	
	MeasurementUpdate(states, covariance, H, (*innovation), (*varInnov), R);
	return true;
}

//-------------------------------------------------------------------
// Update GPS position
//-------------------------------------------------------------------
bool EKF::FuseGPSPos(   EKFStates* states, 
                        CovMatrix* covariance,
					    const Vector3d& measPos, 
                        const Vector3d& pos_var,
				        const double drift_gain,
					    const double &gateSize,
					    const Vector3d& Imu2gps,
					    Vector3d *innovation, 
                        Matrix3d *varInnov)
{
    Matrix3d I3 = Matrix3d::Identity();
	Matrix3d O3 = Matrix3d::Zero();
	Matrix3d Tbn = QuatUtils::QuatToTbn(states->att);

	(*innovation) = measPos - (states->pos + Tbn*Imu2gps + states->gps_b);	
	
	Matrix<double,3,18> H;
	H.block<3,3>(0,0) = -Tbn*VecUtils::FormSkew(Imu2gps);
	H.block<3,3>(0,3) = O3;
	H.block<3,3>(0,6) = I3;
	H.block<3,3>(0,9) = O3;
	H.block<3,3>(0,12) = O3;
	H.block<3,3>(0,15) = I3;
	
	Matrix3d R = Matrix3d(pos_var.asDiagonal()*drift_gain);

	(*varInnov) = H * (*covariance) * (H.transpose()) + R;

	Vector3d correction_ratio;
	for(int i=0; i<3; i++) {
		correction_ratio(i) = pow((*innovation)(i),2) / (pow(gateSize,2)*(*varInnov)(i,i));
	}
	bool innov_pass = VecUtils::AllLessThan(correction_ratio, (double)1.0);
	if (!innov_pass)
	{		
		//std::cout << "Fuse GPS position Failed!" << std::endl;
		return false;
	}

	MeasurementUpdate(states, covariance, H, (*innovation), (*varInnov), R);
	return true;
}

//-------------------------------------------------------------------
// Update GPS yaw
//-------------------------------------------------------------------
bool EKF::FuseGPSYaw(   EKFStates* states, 
                        CovMatrix* covariance,
					    const double& measYaw, 
                        const Vector3d& yaw_var,
					    const double &gateSize,
					    Vector3d *innovation, 
                        Matrix3d *varInnov)
{
	Matrix3d I3 = Matrix3d::Identity();
	Matrix3d O3 = Matrix3d::Zero();
	Matrix3d Tnb = QuatUtils::QuatToTnb(states->att);
	Vector3d ypr = QuatUtils::QuatToYpr(states->att);
	
	Quaterniond qhead = QuatUtils::QuatFromYpr( Vector3d(ypr.x(), ypr.y(), measYaw) );
	Quaterniond qerr = states->att.conjugate() * qhead;
	if( qerr.w() <0 )
	{
		qerr = QuatUtils::QuatFlip(qerr); 
	}
	double th_err = 2 * acos( qerr.w() );
	if(th_err < 1e-14) {
		(*innovation) = Vector3d(0,0,0);
	}
	else 
	{
		Vector3d ax_err = qerr.vec() / sin( acos( qerr.w() ) );
		(*innovation) = th_err * ax_err;
	}
		
	Matrix<double,3,18> H;
	H.block<3,3>(0,0) = I3;
	H.block<3,3>(0,3) = O3;
	H.block<3,3>(0,6) = O3;
	H.block<3,3>(0,9) = O3;
	H.block<3,3>(0,12) = O3;
	H.block<3,3>(0,15) = O3;
	
	Matrix3d R = Tnb * Matrix3d(yaw_var.asDiagonal()) * Tnb.transpose();

	(*varInnov) = H * (*covariance) * (H.transpose()) + R;

	Vector3d correction_ratio;
	for(int i=0; i<3; i++) {
		correction_ratio(i) = pow((*innovation)(i),2) / (pow(gateSize,2)*(*varInnov)(i,i));
	}
	bool innov_pass = VecUtils::AllLessThan(correction_ratio, (double)1.0);
	if (!innov_pass)
	{
		//std::cout << "Fuse GPS heading Failed!" << std::endl;
		return false;
	}
	
	MeasurementUpdate(states, covariance, H, (*innovation), (*varInnov), R);
	return true;	
}

//-------------------------------------------------------------------
// Observation update 
//-------------------------------------------------------------------
template <int ROWS>
bool EKF::MeasurementUpdate(EKFStates* states, 
                            CovMatrix* covariance, 
						    const Matrix<double,ROWS,18> &H,
						    const Matrix<double,ROWS,1> &innovation,
						    const Matrix<double,ROWS,ROWS> &varInnov,
						    const Matrix<double,ROWS,ROWS> &R      )
{
    bool res = true;
	LDLT<Matrix<double,ROWS,ROWS>> VT( varInnov.transpose() );
	Matrix<double,18,ROWS> K = (VT.solve(((*covariance)*H.transpose()).transpose())).transpose();
	
	bool appr = (K*varInnov).isApprox(((*covariance)*H.transpose()),1e-10);
	if(!appr) {
		res = false;
	}
	
	Matrix<double,18,1> xth = K*innovation;
	
	for(int i=0;i<3;i++) {
		if( xth(i) > 10*TO_RAD ) {
			res = false;
		}
	}

	Quaterniond deltaQuat = QuatUtils::QuatFromRot(Vector3d(xth.segment<3>(0)));
	states->att = states->att * deltaQuat; 					
	states->vel = states->vel + xth.segment<3>(3); 			
	states->pos = states->pos + xth.segment<3>(6); 			
	states->gyro_b = states->gyro_b + xth.segment<3>(9);	
	states->acc_b = states->acc_b + xth.segment<3>(12);	
	states->gps_b = states->gps_b + xth.segment<3>(15);	
	
	CovMatrix innovRes = (CovMatrix::Identity()-K*H);
	(*covariance) = innovRes * (*covariance) * innovRes.transpose() + K*R*K.transpose();
	(*covariance) = 0.5*((*covariance) + covariance->transpose());

	for (int i=0;i<18;i++) {
		if ((*covariance)(i,i) < 0)
		{
			(*covariance)(i,i) = 0;
			res = false;
		}
	}

	return res;
}
