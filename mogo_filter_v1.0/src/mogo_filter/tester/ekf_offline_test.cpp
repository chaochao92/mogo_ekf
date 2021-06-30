#include <iostream>
#include <Eigen/Dense>
#include "ekf_base.hpp"
#include "ekf.h"
#include <fstream>
#include <vector>
#include <iomanip>
#include <ros/ros.h>

using namespace std;
using namespace Eigen;

// TODO: change to ROS 


int main(int argc, char* argv[])
{
	EKF ekf;	

	ImuData imu;	
	GpsData gps;	
  
   	// STEP 1:  Initilize filter using the 1st line of data
	imu.gyro   = Vector3d(0, 0,	0);
	imu.acc  = Vector3d(0, 0, -10);
	imu.dt_ms = 0.0012;                            // Imu HZ 
    
	// HELLO: 1. change reference llh position
	Vector3d refllh(0.699721, 2.028966, 34.930000);
	
	
	std::cout << "Init filter..." << std::endl;
	bool init_ = ekf.Init(imu, 0, refllh);
    std::cout << init_ << std::endl;
	// read input
	ifstream idata("../../tester/test_data.txt", ios::in);

	//save result 
    ofstream odata("../../tester/result.txt"); 

	if(!idata.is_open())
	{
		cout<<"Error opening file" << endl;
	}

    vector<double> result;     

	int count = 0;
    // STEP2: processing data
    while(!idata.eof())  {
        string s;
        getline(idata,s);

		// std::cout << count++ << std::endl;

        if(!s.empty()) {
            stringstream ss;
            ss << s;
            
			// imu_data
			double imu_t, imu_gyro_x, imu_gyro_y, imu_gyro_z, imu_acce_x, imu_acce_y, imu_acce_z;
			// gps_data
			double gps_t, gps_flag, gps_fix_type, gps_valid, gps_sat_cnt, gps_p_dop, gps_yaw, gps_yaw_err;
			// llh
			double gps_llh_rad_x, gps_llh_rad_y, gps_llh_rad_z;
			// pos_ned
			double gps_pos_ned_x, gps_pos_ned_y, gps_pos_ned_z;
			// pos_ned_er
			double gps_pos_ned_err_x, gps_pos_ned_err_y, gps_pos_ned_err_z;
			// vel_ned
			double gps_vel_ned_x, gps_vel_ned_y, gps_vel_ned_z;
			// vel_ned_err
			double gps_vel_ned_err_x, gps_vel_ned_err_y, gps_vel_ned_err_z;


			// STEP2.1: read IMU and Gps data
            ss >> imu_t >> imu_acce_x >> imu_acce_y >> imu_acce_z >> imu_gyro_x >> imu_gyro_y >> imu_gyro_z 
			    >> gps_t >> gps_flag >> gps_fix_type >> gps_valid >> gps_sat_cnt >> gps_p_dop 
				>> gps_yaw >> gps_yaw_err
				>> gps_llh_rad_x >> gps_llh_rad_y >> gps_llh_rad_z
				>> gps_pos_ned_x >> gps_pos_ned_y >> gps_pos_ned_z
				>> gps_pos_ned_err_x >> gps_pos_ned_err_y >> gps_pos_ned_err_z
				>> gps_vel_ned_x >> gps_vel_ned_y >> gps_vel_ned_z
				>> gps_vel_ned_err_x >> gps_vel_ned_err_y >> gps_vel_ned_err_z;

            // std::cout << setprecision(20) << imu_t << std:: endl;

             // STEP2.2: get IMU
            imu.gyro = Vector3d(imu_gyro_x, imu_gyro_y, imu_gyro_z);
			imu.acc = Vector3d(imu_acce_x, imu_acce_y, imu_acce_z);
            imu.dt_ms = 0.01;

            ekf.ImuMeas(imu);
           
            // STEP2.3: get Gps
            
            	// HELLO: 2. change llh position to ned, call utils/coord_utils.hpp(fun: LLH2NED)
			gps.pos_ned = Vector3d(gps_pos_ned_x, gps_pos_ned_y, gps_pos_ned_z);
			
		// HELLO: 3. change gps.pos_sd to (0.2,0.2,0.2)

    		gps.pos_sd = Vector3d(gps_pos_ned_err_x, gps_pos_ned_err_y, gps_pos_ned_err_z);   
			gps.yaw = gps_yaw;
			gps.yaw_sd = gps_yaw_err;   
			gps.vel_ned = Vector3d(gps_vel_ned_x, gps_vel_ned_y, gps_vel_ned_z);
    		gps.vel_sd = Vector3d(gps_vel_ned_err_x, gps_vel_ned_err_y, gps_vel_ned_err_z); 
			gps.sat_cnt = gps_sat_cnt;
			gps.valid = gps_valid;  

    		ekf.GpsMeas(gps);

            // STEP2.4: process and save
            ekf.RunOnce(result);
						//std::cout << count++ << std::endl;
            for(vector<double>::iterator it=result.begin();it!=result.end();it++)
			{
        		odata << *it << "    " ;
				//std::cout << *it << "    " ;
			}
        	odata << endl;
			//std::cout << endl;
        }
    }	
	idata.close();
    odata.close();
	return 0;
}
