MOGOEKF: a loosely coupled framework to fuse multiple sensors (GPS/RTK, IMU, Wheel-Encoder/Yaw-Rate, Camera, Lidar ...)
 
BUilding EKF
prerequisites:
	Eigen (recommend 3.3.8)
 
Steps:
	mkdir build/
	cd build/
	cmake ..
	make

Testing:
	cd build/tester
	./ekf_offline_test 
	
V1.0: fuse GPS/RTK and IMU data
0. Test data titles are as follows, required are marked using *; the more data you provided, the better performance you can get

imu_t*, 		imu_acc_x*, 		imu_acc_y*, 		imu_acce_z*, 		imu_gyro_x*, 		imu_gyro_y*, 		imu_gyro_z*, 
gps_t*, 		gps_flag, 		gps_fix_type, 		gps_valid, 		gps_sat_cnt, 		gps_p_dop, 
gps_yaw*, 		gps_yaw_err, 		gps_llh_rad_x*, 	gps_llh_rad_y*, 	gps_llh_rad_z*, 
gps_pos_ned_x, 	gps_pos_ned_y, 	gps_pos_ned_z, 	gps_pos_ned_err_x, 	gps_pos_ned_err_y, 	gps_pos_ned_err_z, 
gps_vel_ned_x, 	gps_vel_ned_y, 	gps_vel_ned_z, 	gps_vel_ned_err_x, 	gps_vel_ned_err_y, 	gps_vel_ned_err_z



1. The result is recorded in /tester/result/result.txt, title: timestamp, pos_ned_x, pos_ned_y, pos_ned_z
   (if you want to modify the pos from ned to llh, you can simply comment lines 207-210 in /src/ekf_run.cpp, and open lines 212-215)
  
2. The reference llh is defined in 27 at /tester/ekf_offline_test.cpp, you can change when you are not in BeiJing.
                      
Future Work:
0. Modify the framework to adapt different vehicles and different data structs 
1. Add Wheel-Encoder/Yaw-Rate (if availble)
2. Add map-matching results, (prerequisites: map-matching functions)
3. More tests
4. Add ROS version
5. Transfer parameters in a common file

