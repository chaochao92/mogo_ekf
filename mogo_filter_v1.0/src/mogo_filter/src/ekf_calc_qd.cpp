/*****************************************************************************
*  Calaulate noise matrix Qd      							                 *
*  @file     ekf_cal_qd.cpp                                                  *
*  @brief    E.Q2: P^k = fd*(P^k-1)*fd^T +Qd                                 *
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

void EKF::CalcQd(   double dt, 
			        const double (&quat)[4],
			        const double (&gyro)[3], 
			        const double (&acc)[3], 
			        const double (&gyroVar)[3], 
			        const double (&accVar)[3],
			        const double (&gyroBiasVar)[3], 
			        const double (&accBiasVar)[3], 
			        const double (&gpsBiasVar)[3], 
			        double Qd[324]  )
{
  double t2;
  double t3;
  double t4;
  double t5;
  double t6;
  double t7;
  double t22;
  double t23;
  double t8;
  double t11;
  double t14;
  double t16;
  double t17;
  double t15;
  double t19;
  double t21;
  double t24;
  double t25;
  double t27;
  double t30;
  double t32;
  double t34;
  double t35;
  double t36;
  double t39;
  double t40_tmp;
  double t40;
  double t41_tmp;
  double t41;
  double t42;
  double t44_tmp;
  double t44;
  double t53_tmp;
  double t53;
  double t45;
  double t47_tmp;
  double b_t47_tmp;
  double t47;
  double t48;
  double t49;
  double t50;
  double t51;
  double t52;
  double t54_tmp;
  double t57;
  double t59_tmp;
  double t58_tmp;
  double t60_tmp;
  double t62_tmp;
  double t62;
  double t63_tmp;
  double t120;
  double t67;
  double t70;
  double t74;
  double t75;
  double t89;
  double t77;
  double t78;
  double t79;
  double t82;
  double t83;
  double t84_tmp;
  double t86;
  double t87;
  double t88;
  double t92;
  double t93;
  double t98;
  double t99;
  double t101;
  double t102;
  double t103_tmp;
  double t109_tmp;
  double t104;
  double t107;
  double t108;
  double t110_tmp;
  double t116_tmp;
  double t111_tmp;
  double t114;
  double t115_tmp;
  double t118;
  double t136_tmp;
  double t155;
  double t125_tmp;
  double t128;
  double t131;
  double t135;
  double t137;
  double t138;
  double t139;
  double t141;
  double t142;
  double t144;
  double t146_tmp;
  double t147;
  double t149;
  double t154;
  double t157;
  double t158;
  double t159;
  double t162;
  double t163;
  double t166;
  double t167_tmp;
  double t172_tmp;
  double t168_tmp;
  double t173_tmp;
  double t175_tmp;
  double t177_tmp;
  double t177;
  double t178;
  double t179;
  double t180;
  double t181;
  double t183;
  double t184;
  double t185;
  double t187_tmp;
  double t190;
  double t191;
  double t193;
  double t194;
  double t197;
  double t201;
  double t204;
  double t207;
  double t210;
  double t215_tmp;
  double t215;
  double t217;
  double t216_tmp;
  double t220;
  double t223;
  double t252_tmp;
  double b_t252_tmp;
  double c_t252_tmp;
  double d_t252_tmp;
  double e_t252_tmp;
  double t252_tmp_tmp;
  double f_t252_tmp;
  double g_t252_tmp;
  double t717;
  double h_t252_tmp;
  double i_t252_tmp;
  double j_t252_tmp;
  double k_t252_tmp;
  double l_t252_tmp;
  double m_t252_tmp;
  double n_t252_tmp;
  double o_t252_tmp;
  double p_t252_tmp;
  double q_t252_tmp;
  double r_t252_tmp;
  double s_t252_tmp;
  double t_t252_tmp;
  double t252;
  double t253;
  double t254_tmp;
  double t254;
  double t255;
  double t258;
  double t260;
  double t263_tmp;
  double t264;
  double t266;
  double t268;
  double t269_tmp;
  double t269;
  double t300_tmp;
  double b_t300_tmp;
  double c_t300_tmp;
  double d_t300_tmp;
  double e_t300_tmp;
  double f_t300_tmp;
  double g_t300_tmp;
  double h_t300_tmp;
  double i_t300_tmp;
  double j_t300_tmp;
  double t300;
  double t328_tmp;
  double b_t328_tmp;
  double c_t328_tmp;
  double d_t328_tmp;
  double e_t328_tmp;
  double f_t328_tmp;
  double g_t328_tmp;
  double t328;
  double t329;
  double t330;
  double t331_tmp;
  double t331;
  double t332_tmp;
  double t332;
  double t333;
  double t334;
  double t338;
  double t339;
  double t341;
  double t342;
  double t343;
  double t346_tmp;
  double b_t346_tmp;
  double t347_tmp;
  double t350_tmp;
  double b_t350_tmp;
  double c_t350_tmp;
  double d_t350_tmp;
  double t351_tmp;
  double t352_tmp;
  double t357_tmp;
  double b_t357_tmp;
  double t359_tmp;
  double t360_tmp;
  double t361;
  double t364_tmp;
  double t365_tmp;
  double t366;
  double t367;
  double t368_tmp;
  double t368;
  double t369;
  double t370_tmp;
  double t370;
  double t371_tmp;
  double t371;
  double t374_tmp;
  double t381_tmp;
  double b_t381_tmp;
  double c_t381_tmp;
  double d_t381_tmp;
  double e_t381_tmp;
  double t381;
  double t408_tmp;
  double b_t408_tmp;
  double c_t408_tmp;
  double d_t408_tmp;
  double e_t408_tmp;
  double f_t408_tmp;
  double t408;
  double t409;
  double t411;
  double t419;
  double t420_tmp;
  double t424;
  double t426;
  double t430;
  double t431;
  double t434_tmp;
  double t434;
  double t435_tmp;
  double t437_tmp;
  double b_t437_tmp;
  double t439_tmp;
  double t441_tmp;
  double t444_tmp;
  double t444;
  double t445_tmp;
  double t447_tmp;
  double b_t447_tmp;
  double t475_tmp;
  double b_t475_tmp;
  double c_t475_tmp;
  double d_t475_tmp;
  double e_t475_tmp;
  double f_t475_tmp;
  double g_t475_tmp;
  double h_t475_tmp;
  double i_t475_tmp;
  double j_t475_tmp;
  double k_t475_tmp;
  double l_t475_tmp;
  double m_t475_tmp;
  double n_t475_tmp;
  double o_t475_tmp;
  double p_t475_tmp;
  double q_t475_tmp;
  double t475;
  double t485_tmp_tmp;
  double t485_tmp;
  double t501_tmp;
  double b_t501_tmp;
  double t507_tmp;
  double t516_tmp;
  double b_t516_tmp;
  double c_t516_tmp;
  double d_t516_tmp;
  double e_t516_tmp;
  double t523_tmp;
  double b_t523_tmp;
  double c_t523_tmp;
  double d_t523_tmp;
  double t527_tmp;
  double b_t527_tmp;
  double t530_tmp;
  double b_t530_tmp;
  double c_t530_tmp;
  double t531;
  double t532;
  double t536;
  double t540;
  double t543;
  double t545;
  double t550;
  double t553;
  double t557;
  double t563_tmp;
  double t564_tmp;
  double b_t564_tmp;
  double t565_tmp;
  double t566_tmp;
  double t568_tmp;
  double t569_tmp;
  double b_t569_tmp;
  double t572_tmp;
  double b_t572_tmp;
  double t573_tmp;
  double t574_tmp;
  double t604_tmp;
  double b_t604_tmp;
  double c_t604_tmp;
  double d_t604_tmp;
  double e_t604_tmp;
  double f_t604_tmp;
  double g_t604_tmp;
  double h_t604_tmp;
  double i_t604_tmp;
  double t604;
  double t634_tmp;
  double b_t634_tmp;
  double c_t634_tmp;
  double t634;
  double t668_tmp;
  double b_t668_tmp;
  double t668;
  double t702;
  double t703;
  double t705;
  double t710;
  double t714;
  double t718;
  double t720;
  double t723;
  double t724;
  double t734;
  double t735;
  double t739_tmp;
  double t740_tmp;
  double t741_tmp;
  double b_t741_tmp;
  double t743_tmp;
  double t744_tmp;
  double t744;
  double t747_tmp;
  double t748;
  double t749_tmp;
  double t749;
  double t750;
  double t756_tmp;
  double b_t756_tmp;
  double t763;
  double t764;
  double t765_tmp;
  double t796_tmp;
  double b_t796_tmp;
  double c_t796_tmp;
  double d_t796_tmp;
  double e_t796_tmp;
  double f_t796_tmp;
  double t796;
  double t889_tmp;
  double t882;
  double t819_tmp;
  double b_t819_tmp;
  double c_t819_tmp;
  double d_t819_tmp;
  double e_t819_tmp;
  double f_t819_tmp;
  double g_t819_tmp;
  double h_t819_tmp;
  double i_t819_tmp;
  double j_t819_tmp;
  double k_t819_tmp;
  double l_t819_tmp;
  double t819;
  double t1011_tmp;
  double t1005_tmp;
  double t842_tmp;
  double b_t842_tmp;
  double c_t842_tmp;
  double d_t842_tmp;
  double e_t842_tmp;
  double f_t842_tmp;
  double g_t842_tmp;
  double h_t842_tmp;
  double i_t842_tmp;
  double t842;
  double t843;
  double t844;
  double t845;
  double t846;
  double t847;
  double t848;
  double t850;
  double t851;
  double t853;
  double t856_tmp;
  double b_t856_tmp;
  double t857_tmp;
  double t858_tmp;
  double b_t858_tmp;
  double t859_tmp;
  double t860;
  double t861_tmp;
  double t861;
  double t862_tmp;
  double t863_tmp;
  double t866;
  double t867_tmp;
  double t867;
  double t870_tmp;
  double t871;
  double t872;
  double t873_tmp;
  double t873;
  double t879_tmp;
  double t880;
  double t881;
  double t890;
  double t919_tmp;
  double b_t919_tmp;
  double c_t919_tmp;
  double d_t919_tmp;
  double e_t919_tmp;
  double f_t919_tmp;
  double t919;
  double t938_tmp;
  double t940_tmp;
  double t1020;
  double t944_tmp;
  double b_t944_tmp;
  double c_t944_tmp;
  double d_t944_tmp;
  double e_t944_tmp;
  double f_t944_tmp;
  double g_t944_tmp;
  double h_t944_tmp;
  double i_t944_tmp;
  double t944;
  double t963_tmp;
  double b_t963_tmp;
  double t963;
  double t964;
  double t965;
  double t966;
  double t967;
  double t968;
  double t969;
  double t970;
  double t972;
  double t973;
  double t975_tmp;
  double t975;
  double t976_tmp;
  double b_t976_tmp;
  double t976;
  double t977_tmp;
  double t982_tmp;
  double t988_tmp;
  double t989;
  double t990_tmp;
  double t990;
  double t995_tmp;
  double t998;
  double t1002;
  double t1003;
  double t1004;
  double t1012;
  double t1013;
  double t1014;
  double t1021;
  double t1025;
  double t1026;
  double t1055_tmp;
  double b_t1055_tmp;
  double c_t1055_tmp;
  double d_t1055_tmp;
  double e_t1055_tmp;
  double f_t1055_tmp;
  double t1055;
  double t1074;
  double t1093;
  double t1094;
  double t1095;
  double t1096;
  double t1097;
  double t1098;
  double t1099;
  double t1101;
  double t1102;
  double t1104;
  double t1106;
  double t1108;
  double t1109;
  double t1112;
  double t1115;
  double t1117;
  double t1118;
  double t1121;
  double t1123;
  double t1125;
  double t1127;
  double t1130;
  double t1131;
  double t1134;
  double t1137;
  double t1139;
  double t1141;
  double t1142;
  double t1143;
  double t1144;
  double t1146;
  double t1148_tmp;
  double t1148;
  double Qd_tmp;
  double b_Qd_tmp;
  double c_Qd_tmp;

  // This function was generated by the Symbolic Math Toolbox version 8.2.
  t2 = dt * dt;
  t3 = t2 * t2;
  t4 = gyro[0] * gyro[0];
  t5 = gyro[1] * gyro[1];
  t6 = gyro[2] * gyro[2];
  t7 = t5 + t6;
  t22 = t6 / 2.0;
  t23 = t5 / 2.0;
  t8 = t22 + t23;
  t11 = t4 * gyro[1] / 6.0 + t7 * gyro[1] / 6.0;
  t14 = t4 * gyro[2] / 6.0 + t7 * gyro[2] / 6.0;
  t16 = t6 / 6.0;
  t17 = t5 / 6.0;
  t15 = t16 + t17;
  t19 = t4 + t6;
  t21 = t5 * gyro[0] / 6.0 + t19 * gyro[0] / 6.0;
  t24 = t4 / 2.0;
  t25 = t22 + t24;
  t22 = t4 / 6.0;
  t27 = t16 + t22;
  t30 = t5 * gyro[2] / 6.0 + t19 * gyro[2] / 6.0;
  t32 = t4 + t5;
  t34 = t6 * gyro[0] / 6.0 + t32 * gyro[0] / 6.0;
  t35 = t23 + t24;
  t36 = t17 + t22;
  t39 = t6 * gyro[1] / 6.0 + t32 * gyro[1] / 6.0;
  t40_tmp = quat[0] * quat[2];
  t40 = t40_tmp * 2.0;
  t41_tmp = quat[1] * quat[3];
  t41 = t41_tmp * 2.0;
  t42 = t40 + t41;
  t44_tmp = quat[0] * quat[3];
  t44 = t44_tmp * 2.0;
  t53_tmp = quat[1] * quat[2];
  t53 = t53_tmp * 2.0;
  t45 = t44 - t53;
  t47_tmp = acc[1] * t42;
  b_t47_tmp = acc[2] * t45;
  t47 = t47_tmp + b_t47_tmp;
  t48 = quat[0] * quat[0];
  t49 = quat[1] * quat[1];
  t50 = quat[2] * quat[2];
  t51 = quat[3] * quat[3];
  t52 = ((t48 + t49) - t50) - t51;
  t54_tmp = acc[0] * t42;
  t22 = acc[1] * t52;
  t16 = acc[0] * t45;
  t57 = t22 + t16;
  t59_tmp = acc[2] * t52;
  t58_tmp = t54_tmp - t59_tmp;
  t60_tmp = t47 * gyro[1];
  t62_tmp = t58_tmp * gyro[0];
  t62 = t60_tmp + t62_tmp;
  t63_tmp = t57 * gyro[1];
  t24 = t58_tmp * gyro[2];
  t23 = t63_tmp - t24;
  t17 = t47 * gyro[2];
  t120 = t57 * gyro[0];
  t67 = t17 + t120;
  t70 = t22 / 2.0 + t16 / 2.0;
  t74 = t67 * gyro[0] / 6.0;
  t75 = t23 * gyro[1] / 6.0 + t74;
  t89 = t23 * gyro[2] / 6.0;
  t77 = t62 * gyro[0] / 6.0 - t89;
  t78 = t63_tmp / 2.0;
  t79 = t78 - t24 / 2.0;
  t82 = t17 / 2.0 + t120 / 2.0;
  t83 = t60_tmp / 2.0;
  t84_tmp = t54_tmp / 2.0 - t59_tmp / 2.0;
  t86 = t83 + t62_tmp / 2.0;
  t87 = t63_tmp / 6.0;
  t88 = t87 - t24 / 6.0;
  t92 = t17 / 6.0 + t120 / 6.0;
  t93 = t60_tmp / 6.0;
  t98 = t47_tmp / 2.0 + b_t47_tmp / 2.0;
  t99 = t67 * gyro[2] / 6.0;
  t101 = t99 + t62 * gyro[1] / 6.0;
  t102 = t44 + t53;
  t103_tmp = quat[0] * quat[1];
  t62 = t103_tmp * 2.0;
  t109_tmp = quat[2] * quat[3];
  t44 = t109_tmp * 2.0;
  t104 = t62 - t44;
  b_t47_tmp = acc[0] * t104;
  t67 = acc[2] * t102;
  t107 = b_t47_tmp + t67;
  t53 = t48 - t49;
  t108 = (t53 + t50) - t51;
  t110_tmp = acc[1] * t102;
  t116_tmp = acc[0] * t108;
  t111_tmp = t110_tmp - t116_tmp;
  t23 = acc[2] * t108;
  t24 = acc[1] * t104;
  t114 = t23 + t24;
  t115_tmp = t107 * gyro[2];
  t47_tmp = t111_tmp * gyro[1];
  t118 = t115_tmp + t47_tmp;
  t136_tmp = t114 * gyro[2];
  t49 = t111_tmp * gyro[0];
  t120 = t49 - t136_tmp;
  t22 = t107 * gyro[0];
  t16 = t114 * gyro[1];
  t17 = t22 + t16;
  t155 = t136_tmp / 2.0;
  t125_tmp = t49 / 2.0 - t155;
  t128 = t22 / 6.0 + t16 / 6.0;
  t131 = t22 / 2.0 + t16 / 2.0;
  t135 = t23 / 2.0 + t24 / 2.0;
  t137 = t17 * gyro[1] / 6.0;
  t138 = t137 - t120 * gyro[2] / 6.0;
  t139 = t17 * gyro[0] / 6.0;
  t141 = t139 + t118 * gyro[2] / 6.0;
  t142 = t120 * gyro[0] / 6.0;
  t144 = t115_tmp / 6.0;
  t146_tmp = t110_tmp / 2.0 - t116_tmp / 2.0;
  t147 = t115_tmp / 2.0;
  t48 = gyro[1] * t111_tmp;
  t149 = t147 + t48 / 2.0;
  t120 = t115_tmp + t48;
  t154 = b_t47_tmp / 2.0 + t67 / 2.0;
  t157 = t144 + t47_tmp / 6.0;
  t158 = t40 - t41;
  t159 = t62 + t44;
  t67 = acc[0] * t159;
  t62 = acc[1] * t158;
  t162 = t67 + t62;
  t163 = (t53 - t50) + t51;
  t24 = acc[0] * t163;
  t17 = acc[2] * t158;
  t166 = t24 + t17;
  t167_tmp = acc[2] * t159;
  t172_tmp = acc[1] * t163;
  t168_tmp = t167_tmp - t172_tmp;
  t47_tmp = t162 * gyro[1];
  b_t47_tmp = t166 * gyro[2];
  t16 = t47_tmp + b_t47_tmp;
  t173_tmp = t166 * gyro[0];
  t22 = t168_tmp * gyro[1];
  t23 = t173_tmp - t22;
  t175_tmp = t162 * gyro[0];
  t177_tmp = t168_tmp * gyro[2];
  t177 = t175_tmp + t177_tmp;
  t178 = t173_tmp / 2.0;
  t179 = t178 - t22 / 2.0;
  t180 = t175_tmp / 6.0;
  t181 = t175_tmp / 2.0;
  t183 = t181 + t177_tmp / 2.0;
  t184 = t173_tmp / 6.0;
  t185 = t184 - t22 / 6.0;
  t187_tmp = t167_tmp / 2.0 - t172_tmp / 2.0;
  t44 = t23 * gyro[1] / 6.0;
  t190 = t44 - t177 * gyro[2] / 6.0;
  t191 = t16 * gyro[1] / 6.0;
  t193 = t16 * gyro[2] / 6.0;
  t194 = t23 * gyro[0] / 6.0 + t193;
  t197 = t47_tmp / 6.0 + b_t47_tmp / 6.0;
  t23 = gyro[2] * t168_tmp;
  t16 = t175_tmp + t23;
  t201 = t191 + t16 * gyro[0] / 6.0;
  t204 = t24 / 2.0 + t17 / 2.0;
  t207 = t47_tmp / 2.0 + b_t47_tmp / 2.0;
  t210 = t67 / 2.0 + t62 / 2.0;
  t215_tmp = gyro[0] * t58_tmp;
  t215 = t93 + t215_tmp / 6.0;
  t217 = t136_tmp / 6.0;
  t216_tmp = t49 / 6.0 - t217;
  t220 = t144 + t48 / 6.0;
  t223 = t180 + t23 / 6.0;
  t252_tmp = t25 * gyroVar[1];
  b_t252_tmp = t8 * gyroVar[0];
  c_t252_tmp = t21 * gyroVar[2];
  d_t252_tmp = t11 * gyroVar[2];
  t49 = t27 * gyroBiasVar[1];
  t24 = t15 * gyroBiasVar[0];
  e_t252_tmp = t2 * t3;
  t252_tmp_tmp = gyroBiasVar[2] * gyro[0];
  t22 = t252_tmp_tmp * gyro[1];
  f_t252_tmp = t14 * gyroVar[1];
  t17 = gyroBiasVar[0] * gyro[0];
  g_t252_tmp = gyroBiasVar[1] * gyro[0];
  t47_tmp = b_t252_tmp * gyro[0];
  b_t47_tmp = gyroVar[0] * gyro[0];
  t40 = gyroVar[2] * gyro[0];
  t41 = gyroVar[1] * gyro[0];
  t67 = t24 * gyro[0];
  t50 = t30 * gyroVar[0];
  t51 = t49 * gyro[0];
  t717 = t252_tmp * gyro[0];
  h_t252_tmp = b_t47_tmp * gyro[1];
  i_t252_tmp = t41 * gyro[1];
  j_t252_tmp = t40 * gyro[1];
  k_t252_tmp = t5 * gyroVar[2];
  l_t252_tmp = d_t252_tmp * gyro[0];
  m_t252_tmp = g_t252_tmp * gyro[1];
  n_t252_tmp = t5 * gyroBiasVar[2];
  o_t252_tmp = gyroVar[1] * gyro[2];
  p_t252_tmp = t4 * gyroVar[2];
  q_t252_tmp = c_t252_tmp * gyro[1];
  r_t252_tmp = t17 * gyro[1];
  s_t252_tmp = t4 * gyroBiasVar[2];
  t_t252_tmp = gyroBiasVar[0] * gyro[2];
  t252 = ((((t3 * (((((((q_t252_tmp / 5.0 + l_t252_tmp / 5.0) + r_t252_tmp /
                        30.0) + m_t252_tmp / 30.0) + t6 * gyroVar[2] * gyro[0] *
                      gyro[1] / 20.0) - t22 / 20.0) - t47_tmp * gyro[1] / 10.0)
                   - t717 * gyro[1] / 10.0) * dt + t2 * ((h_t252_tmp / 6.0
    + i_t252_tmp / 6.0) - j_t252_tmp / 3.0) * dt) - t2 * (gyroVar[0] * gyro
             [2] / 2.0 - o_t252_tmp / 2.0)) - t3 * (((((((t_t252_tmp / 8.0 +
    f_t252_tmp / 4.0) + t252_tmp * gyro[2] / 4.0) + k_t252_tmp * gyro[2] / 8.0)
    - t50 / 4.0) - gyroBiasVar[1] * gyro[2] / 8.0) - p_t252_tmp * gyro[2] / 8.0)
            - b_t252_tmp * gyro[2] / 4.0)) - e_t252_tmp * (((((((t49 * gyro[2]
    / 12.0 + n_t252_tmp * gyro[2] / 72.0) + t8 * t30 * gyroVar[0] / 6.0) +
    c_t252_tmp * gyro[0] * gyro[2] / 12.0) - t14 * t25 * gyroVar[1] / 6.0) -
             s_t252_tmp * gyro[2] / 72.0) - t24 * gyro[2] / 12.0) - d_t252_tmp
           * gyro[1] * gyro[2] / 12.0)) - e_t252_tmp * (((t11 * t21 * gyroVar[2]
    / 7.0 + t51 * gyro[1] / 42.0) + t67 * gyro[1] / 42.0) - t6 * gyroBiasVar[2]
    * gyro[0] * gyro[1] / 252.0) * dt;
  t253 = t22 * gyro[2] / 36.0;
  t254_tmp = t4 * t5;
  t254 = t254_tmp / 4.0;
  t255 = t6 / 3.0;
  t258 = t83 + t215_tmp / 2.0;
  t260 = t139 + t120 * gyro[2] / 6.0;
  t263_tmp = t142 + t120 * gyro[1] / 6.0;
  t264 = t44 - t16 * gyro[2] / 6.0;
  t266 = t181 + t23 / 2.0;
  t268 = gyro[2] * 4.0;
  t269_tmp = dt * gyro[0];
  t269 = t269_tmp * gyro[1];
  t300_tmp = t35 * gyroVar[2];
  b_t300_tmp = t34 * gyroVar[1];
  t22 = t36 * gyroBiasVar[2];
  c_t300_tmp = t39 * gyroVar[0];
  d_t300_tmp = gyroBiasVar[2] * gyro[1];
  t16 = gyroBiasVar[0] * gyro[1];
  t23 = t300_tmp * gyro[1];
  t62 = t22 * gyro[0];
  t44 = gyroVar[2] * gyro[1];
  t53 = gyroVar[0] * gyro[1];
  t48 = t300_tmp * gyro[0];
  t22 *= gyro[1];
  e_t300_tmp = t6 * gyroVar[1];
  f_t300_tmp = f_t252_tmp * gyro[0];
  g_t300_tmp = t6 * gyroBiasVar[1];
  h_t300_tmp = t4 * gyroVar[1];
  i_t300_tmp = t4 * gyroBiasVar[1];
  j_t300_tmp = t40 * gyro[2];
  t300 = ((((t3 * (((((((b_t300_tmp * gyro[2] / 5.0 + f_t300_tmp / 5.0) + t17 *
                        gyro[2] / 30.0) + t252_tmp_tmp * gyro[2] / 30.0) + t5 *
                      gyroVar[1] * gyro[0] * gyro[2] / 20.0) - g_t252_tmp *
                     gyro[2] / 20.0) - t47_tmp * gyro[2] / 10.0) - t48 *
                   gyro[2] / 10.0) * dt + t2 * (t53 / 2.0 - t44 / 2.0)) +
            t3 * (((((((t16 / 8.0 + d_t252_tmp / 4.0) + t23 / 4.0) + e_t300_tmp *
                      gyro[1] / 8.0) - c_t300_tmp / 4.0) - d_t300_tmp / 8.0) -
                   h_t300_tmp * gyro[1] / 8.0) - b_t252_tmp * gyro[1] / 4.0))
           + e_t252_tmp * (((((((t22 / 12.0 + g_t300_tmp * gyro[1] / 72.0) + t8
    * t39 * gyroVar[0] / 6.0) + b_t300_tmp * gyro[0] * gyro[1] / 12.0) - t11 *
    t35 * gyroVar[2] / 6.0) - i_t300_tmp * gyro[1] / 72.0) - t24 * gyro[1] /
             12.0) - f_t252_tmp * gyro[1] * gyro[2] / 12.0)) + t2 *
          ((b_t47_tmp * gyro[2] / 6.0 + j_t300_tmp / 6.0) - t41 * gyro[2] /
           3.0) * dt) - e_t252_tmp * (((t14 * t34 * gyroVar[1] / 7.0 + t62 *
    gyro[2] / 42.0) + t67 * gyro[2] / 42.0) - t5 * gyroBiasVar[1] * gyro[0] *
    gyro[2] / 252.0) * dt;
  t328_tmp = gyroVar[1] * gyro[1] * gyro[2];
  b_t328_tmp = t6 * gyroVar[0];
  c_t328_tmp = t50 * gyro[0];
  d_t328_tmp = t6 * gyroBiasVar[0];
  e_t328_tmp = t5 * gyroVar[0];
  f_t328_tmp = c_t300_tmp * gyro[0];
  g_t328_tmp = t5 * gyroBiasVar[0];
  t328 = ((((t3 * (((((((c_t300_tmp * gyro[2] / 5.0 + t50 * gyro[1] / 5.0) +
                        gyroBiasVar[1] * gyro[1] * gyro[2] / 30.0) + d_t300_tmp
                       * gyro[2] / 30.0) + t4 * gyroVar[0] * gyro[1] * gyro[2]
                      / 20.0) - t16 * gyro[2] / 20.0) - t252_tmp * gyro[1] *
                    gyro[2] / 10.0) - t23 * gyro[2] / 10.0) * dt +
             e_t252_tmp * (((((((t51 / 12.0 + g_t328_tmp * gyro[0] / 72.0) +
    t21 * t35 * gyroVar[2] / 6.0) + c_t328_tmp * gyro[2] / 12.0) - t25 * t34 *
    gyroVar[1] / 6.0) - d_t328_tmp * gyro[0] / 72.0) - t62 / 12.0) - f_t328_tmp *
              gyro[1] / 12.0)) + t2 * ((t328_tmp / 6.0 + t44 * gyro[2] / 6.0)
             - t53 * gyro[2] / 3.0) * dt) - t2 * (t41 / 2.0 - t40 / 2.0)) -
          t3 * (((((((g_t252_tmp / 8.0 + c_t252_tmp / 4.0) + t48 / 4.0) +
                    b_t328_tmp * gyro[0] / 8.0) - b_t300_tmp / 4.0) -
                  t252_tmp_tmp / 8.0) - e_t328_tmp * gyro[0] / 8.0) - t717 /
                4.0)) - e_t252_tmp * (((t30 * t39 * gyroVar[0] / 7.0 + t22 *
    gyro[2] / 42.0) + t49 * gyro[1] * gyro[2] / 42.0) - t4 * gyroBiasVar[0] *
    gyro[1] * gyro[2] / 252.0) * dt;
  t329 = h_t252_tmp * gyro[2] / 4.0;
  t330 = i_t252_tmp * gyro[2] / 4.0;
  t331_tmp = t5 * t6;
  t331 = t331_tmp / 4.0;
  t332_tmp = t4 * t6;
  t332 = t332_tmp / 4.0;
  t333 = t4 / 3.0;
  t334 = t5 / 3.0;
  t22 = t60_tmp + t215_tmp;
  t338 = t99 + t22 * gyro[1] / 6.0;
  t339 = t89 - t22 * gyro[0] / 6.0;
  t341 = gyro[1] * 4.0;
  t342 = gyro[0] * 4.0;
  t343 = dt * gyro[1] * gyro[2];
  t346_tmp = t84_tmp * gyroBiasVar[1];
  b_t346_tmp = t346_tmp * gyro[0];
  t22 = t70 * gyroBiasVar[2];
  t347_tmp = t22 * gyro[0];
  t350_tmp = t58_tmp * gyroVar[1];
  b_t350_tmp = t57 * gyroVar[2];
  c_t350_tmp = t79 * gyroVar[0];
  d_t350_tmp = b_t350_tmp * gyro[1];
  t351_tmp = t88 * gyroBiasVar[0];
  t16 = t75 * gyroVar[2];
  t352_tmp = t16 * gyro[1];
  t357_tmp = t92 * gyroBiasVar[1];
  b_t357_tmp = t357_tmp * gyro[0];
  t359_tmp = t350_tmp * gyro[0];
  t360_tmp = b_t350_tmp * gyro[0];
  t361 = t2 * t47 * gyroVar[0] / 2.0;
  t364_tmp = t351_tmp * gyro[0];
  t23 = t215 * gyroBiasVar[2];
  t365_tmp = t23 * gyro[1];
  t366 = c_t350_tmp * gyro[2] / 4.0;
  t367 = t25 * t58_tmp * gyroVar[1] / 4.0;
  t368_tmp = t47 * gyroVar[0];
  t24 = t368_tmp * gyro[0];
  t368 = t24 * gyro[1] / 8.0;
  t369 = t30 * t79 * gyroVar[0] / 6.0;
  t370_tmp = t23 * gyro[0];
  t370 = t370_tmp / 12.0;
  t371_tmp = t22 * gyro[1];
  t371 = t371_tmp * gyro[2] / 36.0;
  t374_tmp = t82 * gyroVar[1];
  t381_tmp = t258 * gyroVar[2];
  b_t381_tmp = t338 * gyroVar[0];
  c_t381_tmp = t98 * gyroBiasVar[0];
  t17 = c_t350_tmp * gyro[0];
  d_t381_tmp = t381_tmp * gyro[1];
  e_t381_tmp = t16 * gyro[0];
  t381 = t3 * (((((((((t357_tmp / 5.0 + b_t381_tmp * gyro[2] / 5.0) + t30 * t47
                      * gyroVar[0] / 5.0) + e_t381_tmp / 5.0) + t21 * t57 *
                    gyroVar[2] / 5.0) - t25 * t82 * gyroVar[1] / 5.0) - t347_tmp /
                  10.0) - c_t381_tmp * gyro[2] / 10.0) - t17 * gyro[1] / 10.0)
               - d_t381_tmp * gyro[2] / 10.0) * dt;
  t408_tmp = t339 * gyroVar[1];
  b_t408_tmp = t2 * dt;
  c_t408_tmp = t408_tmp * gyro[0];
  d_t408_tmp = t374_tmp * gyro[0];
  e_t408_tmp = c_t381_tmp * gyro[0];
  f_t408_tmp = b_t381_tmp * gyro[0];
  t408 = ((((e_t252_tmp * ((((t36 * t215 * gyroBiasVar[2] / 7.0 + t39 * t338 *
    gyroVar[0] / 7.0) + t357_tmp * gyro[1] * gyro[2] / 42.0) - t34 * t339 *
    gyroVar[1] / 7.0) - t364_tmp * gyro[2] / 42.0) * dt + b_t408_tmp *
             ((t359_tmp / 3.0 + t368_tmp * gyro[1] / 3.0) - t381_tmp / 3.0)) -
            t3 * ((((((t22 / 4.0 + d_t408_tmp / 4.0) + c_t350_tmp * gyro[1] /
                      4.0) + t350_tmp * gyro[1] * gyro[2] / 8.0) - t16 / 4.0)
                   - t35 * t57 * gyroVar[2] / 4.0) - t24 * gyro[2] / 8.0)) -
           e_t252_tmp * (((((((((b_t357_tmp / 12.0 + t351_tmp * gyro[1] / 12.0)
    + t35 * t75 * gyroVar[2] / 6.0) + f_t408_tmp * gyro[2] / 12.0) + t346_tmp *
    gyro[1] * gyro[2] / 36.0) + t408_tmp * gyro[1] * gyro[2] / 12.0) - t34 *
    t82 * gyroVar[1] / 6.0) - t39 * t79 * gyroVar[0] / 6.0) - t36 * t70 *
             gyroBiasVar[2] / 6.0) - e_t408_tmp * gyro[2] / 36.0)) - t3 *
          (((((((((t23 / 5.0 + b_t381_tmp * gyro[1] / 5.0) + t39 * t47 *
                  gyroVar[0] / 5.0) + t17 * gyro[2] / 10.0) + t34 * t58_tmp *
                gyroVar[1] / 5.0) - t35 * t258 * gyroVar[2] / 5.0) - b_t346_tmp /
              10.0) - c_t408_tmp / 5.0) - c_t381_tmp * gyro[1] / 10.0) -
           t374_tmp * gyro[1] * gyro[2] / 10.0) * dt) - t2 * t57 * gyroVar
    [2] / 2.0;
  t409 = -t54_tmp + t59_tmp;
  t23 = t409 * gyro[2];
  t411 = t78 + t23 / 2.0;
  t24 = t409 * gyro[0];
  t22 = t60_tmp - t24;
  t16 = t63_tmp + t23;
  t419 = t99 + t22 * gyro[1] / 6.0;
  t420_tmp = t22 * gyro[0] / 6.0 - t16 * gyro[2] / 6.0;
  t48 = t45 * t45;
  t120 = t42 * t42;
  t49 = t52 * t52;
  t424 = t93 - t24 / 6.0;
  t426 = t74 + t16 * gyro[1] / 6.0;
  t430 = t87 + t23 / 6.0;
  t431 = t83 - t24 / 2.0;
  t434_tmp = t128 * gyroBiasVar[2];
  t16 = t434_tmp * gyro[0];
  t434 = t16 * gyro[2] / 42.0;
  t435_tmp = t138 * gyroVar[0];
  t437_tmp = t107 * gyroVar[1];
  b_t437_tmp = t437_tmp * gyro[0];
  t439_tmp = t434_tmp * gyro[1];
  t441_tmp = t146_tmp * gyroBiasVar[2];
  t22 = t111_tmp * gyroVar[2];
  t444_tmp = t149 * gyroVar[0];
  t23 = t22 * gyro[1];
  t444 = b_t408_tmp * ((t23 / 3.0 + t437_tmp * gyro[2] / 3.0) - t444_tmp / 3.0);
  t445_tmp = t260 * gyroVar[1];
  t447_tmp = t263_tmp * gyroVar[2];
  b_t447_tmp = t447_tmp * gyro[1];
  t475_tmp = t114 * gyroVar[0];
  b_t475_tmp = t131 * gyroVar[2];
  c_t475_tmp = t135 * gyroBiasVar[0];
  d_t475_tmp = t220 * gyroBiasVar[0];
  e_t475_tmp = d_t475_tmp * gyro[0];
  f_t475_tmp = t216_tmp * gyroBiasVar[1];
  g_t475_tmp = t475_tmp * gyro[0];
  h_t475_tmp = t154 * gyroBiasVar[1];
  i_t475_tmp = t435_tmp * gyro[0];
  j_t475_tmp = t444_tmp * gyro[0];
  k_t475_tmp = t3 * dt;
  l_t475_tmp = b_t475_tmp * gyro[1];
  m_t475_tmp = t22 * gyro[0];
  n_t475_tmp = t447_tmp * gyro[0];
  o_t475_tmp = t441_tmp * gyro[1];
  p_t475_tmp = b_t475_tmp * gyro[0];
  q_t475_tmp = c_t475_tmp * gyro[0];
  t475 = ((((t2 * ((gyroVar[1] * t125_tmp / 3.0 + t475_tmp * gyro[2] / 3.0) -
                   m_t475_tmp / 3.0) * dt + k_t475_tmp * (((((((((f_t475_tmp
    / 5.0 + c_t475_tmp * gyro[2] / 10.0) + n_t475_tmp / 5.0) + t21 * t111_tmp *
    gyroVar[2] / 5.0) + l_t475_tmp * gyro[2] / 10.0) - t30 * t114 * gyroVar[0] /
    5.0) - t435_tmp * gyro[2] / 5.0) - t252_tmp * t125_tmp / 5.0) -
    t252_tmp_tmp * t146_tmp / 10.0) - j_t475_tmp * gyro[1] / 10.0)) + t2 * t107
            * gyroVar[1] / 2.0) - t3 * ((((((t445_tmp / 4.0 + t25 * t107 *
    gyroVar[1] / 4.0) + g_t475_tmp * gyro[1] / 8.0) + t23 * gyro[2] / 8.0) -
              h_t475_tmp / 4.0) - p_t475_tmp / 4.0) - t444_tmp * gyro[2] / 4.0))
          - e_t252_tmp * (((((((((t27 * t154 * gyroBiasVar[1] / 6.0 + t21 * t131 *
    gyroVar[2] / 6.0) + t30 * t149 * gyroVar[0] / 6.0) + q_t475_tmp * gyro[1] /
    36.0) + o_t475_tmp * gyro[2] / 36.0) - t25 * t260 * gyroVar[1] / 6.0) - t16 /
              12.0) - d_t475_tmp * gyro[2] / 12.0) - i_t475_tmp * gyro[1] /
            12.0) - b_t447_tmp * gyro[2] / 12.0)) - e_t252_tmp * ((((t27 *
    t216_tmp * gyroBiasVar[1] / 7.0 + t21 * t263_tmp * gyroVar[2] / 7.0) +
    e_t475_tmp * gyro[1] / 42.0) - t30 * t138 * gyroVar[0] / 7.0) - t439_tmp *
    gyro[2] / 42.0) * dt;
  t485_tmp_tmp = t125_tmp * gyroVar[1];
  t485_tmp = t485_tmp_tmp * gyro[0];
  t501_tmp = h_t475_tmp * gyro[0];
  b_t501_tmp = t445_tmp * gyro[0];
  t507_tmp = t70 * t146_tmp;
  t22 = t45 * t108;
  t16 = t42 * t104;
  t23 = t52 * t102;
  t516_tmp = t22 * accBiasVar[1];
  b_t516_tmp = t16 * accBiasVar[2];
  c_t516_tmp = t23 * accBiasVar[0];
  d_t516_tmp = t47 * t114;
  e_t516_tmp = t107 * t409;
  t523_tmp = t135 * t430;
  b_t523_tmp = t92 * t154;
  c_t523_tmp = t146_tmp * t424;
  d_t523_tmp = t70 * t128;
  t527_tmp = t128 * t424;
  b_t527_tmp = e_t252_tmp * dt;
  t530_tmp = t22 * accVar[1];
  b_t530_tmp = t16 * accVar[2];
  c_t530_tmp = t23 * accVar[0];
  t531 = -t110_tmp + t116_tmp;
  t23 = t531 * gyro[1];
  t532 = t147 - t23 / 2.0;
  t24 = t531 * gyro[0];
  t16 = t136_tmp + t24;
  t536 = t155 + t24 / 2.0;
  t22 = t115_tmp - t23;
  t540 = t16 * gyro[0] / 6.0 - t22 * gyro[1] / 6.0;
  t543 = t139 + t22 * gyro[2] / 6.0;
  t545 = t137 + t16 * gyro[2] / 6.0;
  t40 = t102 * t102;
  t41 = t104 * t104;
  t50 = t108 * t108;
  t550 = t144 - t23 / 6.0;
  t553 = t217 + t24 / 6.0;
  t557 = (t530_tmp / 2.0 + b_t530_tmp / 2.0) - c_t530_tmp / 2.0;
  t16 = t185 * gyroBiasVar[2];
  t563_tmp = t16 * gyro[0];
  t564_tmp = t166 * gyroVar[1];
  b_t564_tmp = t564_tmp * gyro[0];
  t565_tmp = t187_tmp * gyroBiasVar[0];
  t566_tmp = t16 * gyro[1];
  t23 = t210 * gyroBiasVar[2];
  t568_tmp = t23 * gyro[0];
  t569_tmp = t194 * gyroVar[1];
  b_t569_tmp = t569_tmp * gyro[0];
  t22 = t162 * gyroVar[2];
  t572_tmp = t207 * gyroVar[0];
  b_t572_tmp = t22 * gyro[1];
  t573_tmp = t197 * gyroBiasVar[0];
  t24 = t201 * gyroVar[2];
  t574_tmp = t24 * gyro[1];
  t17 = t168_tmp * gyroVar[0];
  t47_tmp = t179 * gyroVar[2];
  t604_tmp = t264 * gyroVar[0];
  b_t47_tmp = t573_tmp * gyro[0];
  b_t604_tmp = t223 * gyroBiasVar[1];
  t67 = t17 * gyro[0];
  c_t604_tmp = t266 * gyroVar[1];
  t62 = t565_tmp * gyro[0];
  d_t604_tmp = t204 * gyroBiasVar[1];
  t44 = t604_tmp * gyro[0];
  t53 = t572_tmp * gyro[0];
  e_t604_tmp = t47_tmp * gyro[1];
  f_t604_tmp = t22 * gyro[0];
  g_t604_tmp = t23 * gyro[1];
  h_t604_tmp = t47_tmp * gyro[0];
  i_t604_tmp = t24 * gyro[0];
  t604 = ((((e_t252_tmp * ((((t27 * t223 * gyroBiasVar[1] / 7.0 + t21 * t201 *
    gyroVar[2] / 7.0) + b_t47_tmp * gyro[1] / 42.0) - t30 * t264 * gyroVar[0] /
    7.0) - t566_tmp * gyro[2] / 42.0) * dt + b_t408_tmp * ((t17 * gyro[2] /
    3.0 + f_t604_tmp / 3.0) - c_t604_tmp / 3.0)) - t3 * ((((((d_t604_tmp / 4.0 +
    t572_tmp * gyro[2] / 4.0) + h_t604_tmp / 4.0) + t67 * gyro[1] / 8.0) -
    t569_tmp / 4.0) - t25 * t166 * gyroVar[1] / 4.0) - b_t572_tmp * gyro[2] /
             8.0)) - e_t252_tmp * (((((((((t573_tmp * gyro[2] / 12.0 + t563_tmp
    / 12.0) + t25 * t194 * gyroVar[1] / 6.0) + t574_tmp * gyro[2] / 12.0) + t62 *
    gyro[1] / 36.0) + t44 * gyro[1] / 12.0) - t21 * t179 * gyroVar[2] / 6.0) -
              t30 * t207 * gyroVar[0] / 6.0) - t27 * t204 * gyroBiasVar[1] / 6.0)
            - g_t604_tmp * gyro[2] / 36.0)) - t3 * (((((((((b_t604_tmp / 5.0 +
    i_t604_tmp / 5.0) + t21 * t162 * gyroVar[2] / 5.0) + e_t604_tmp * gyro[2] /
    10.0) + t30 * t168_tmp * gyroVar[0] / 5.0) - t25 * t266 * gyroVar[1] / 5.0) -
              t568_tmp / 10.0) - t604_tmp * gyro[2] / 5.0) - t565_tmp * gyro[2]
            / 10.0) - t53 * gyro[1] / 10.0) * dt) - t2 * t166 * gyroVar[1] /
    2.0;
  t634_tmp = c_t604_tmp * gyro[0];
  b_t634_tmp = d_t604_tmp * gyro[0];
  c_t634_tmp = b_t604_tmp * gyro[0];
  t634 = ((((e_t252_tmp * ((((t36 * t185 * gyroBiasVar[2] / 7.0 + t34 * t194 *
    gyroVar[1] / 7.0) + t39 * t264 * gyroVar[0] / 7.0) + b_t47_tmp * gyro[2] /
    42.0) - b_t604_tmp * gyro[1] * gyro[2] / 42.0) * dt + t2 * t162 *
             gyroVar[2] / 2.0) - t3 * ((((((t24 / 4.0 + t35 * t162 * gyroVar[2] /
    4.0) + t67 * gyro[2] / 8.0) + t564_tmp * gyro[1] * gyro[2] / 8.0) - t23 /
    4.0) - t634_tmp / 4.0) - t572_tmp * gyro[1] / 4.0)) - e_t252_tmp *
           (((((((((t36 * t210 * gyroBiasVar[2] / 6.0 + t34 * t266 * gyroVar[1] /
                    6.0) + t39 * t207 * gyroVar[0] / 6.0) + t62 * gyro[2] / 36.0)
                 + d_t604_tmp * gyro[1] * gyro[2] / 36.0) + t44 * gyro[2] /
                12.0) - t35 * t201 * gyroVar[2] / 6.0) - c_t634_tmp / 12.0) -
             t573_tmp * gyro[1] / 12.0) - t569_tmp * gyro[1] * gyro[2] / 12.0))
          - t2 * ((t47_tmp / 3.0 + t17 * gyro[1] / 3.0) - b_t564_tmp / 3.0) *
          dt) - t3 * (((((((((t16 / 5.0 + b_t569_tmp / 5.0) + t604_tmp *
    gyro[1] / 5.0) + t34 * t166 * gyroVar[1] / 5.0) + c_t604_tmp * gyro[1] *
    gyro[2] / 10.0) + t565_tmp * gyro[1] / 10.0) - t39 * t168_tmp * gyroVar[0] /
    5.0) - t35 * t179 * gyroVar[2] / 5.0) - b_t634_tmp / 10.0) - t53 * gyro[2] /
    10.0) * dt;
  t668_tmp = t52 * t158;
  b_t668_tmp = t45 * t159;
  t217 = t42 * t163;
  t147 = t668_tmp * accVar[0];
  t110_tmp = b_t668_tmp * accVar[1];
  t116_tmp = t217 * accVar[2];
  t668_tmp *= accBiasVar[0];
  b_t668_tmp *= accBiasVar[1];
  t217 *= accBiasVar[2];
  t139 = t98 * t197;
  t136_tmp = t92 * t204;
  t155 = t210 * t424;
  t137 = t70 * t210;
  t115_tmp = t84_tmp * t204;
  t144 = t166 * t409;
  t45 = t57 * t162;
  t42 = t197 * t430;
  t668 = ((((t3 * (((((t57 * t179 * gyroVar[2] / 4.0 + t168_tmp * t411 * gyroVar[0]
                       / 4.0) + t47 * t207 * gyroVar[0] / 4.0) - t82 * t166 *
                     gyroVar[1] / 4.0) - t162 * t431 * gyroVar[2] / 4.0) - t266 *
                   t409 * gyroVar[1] / 4.0) + k_t475_tmp * (((((((((((t115_tmp *
    gyroBiasVar[1] / 5.0 + t194 * t409 * gyroVar[1] / 5.0) + t162 * t426 * gyroVar
    [2] / 5.0) + t57 * t201 * gyroVar[2] / 5.0) + t168_tmp * t419 * gyroVar[0] /
    5.0) + t179 * t431 * gyroVar[2] / 5.0) - t47 * t264 * gyroVar[0] / 5.0) - t82 *
    t266 * gyroVar[1] / 5.0) - t137 * gyroBiasVar[2] / 5.0) - t207 * t411 *
    gyroVar[0] / 5.0) - c_t381_tmp * t187_tmp / 5.0) - t564_tmp * t420_tmp / 5.0))
            + e_t252_tmp * (((((((((((t70 * t185 * gyroBiasVar[2] / 6.0 + t430 *
    gyroBiasVar[0] * t187_tmp / 6.0) + t139 * gyroBiasVar[0] / 6.0) + t264 * t411 *
    gyroVar[0] / 6.0) + t201 * t431 * gyroVar[2] / 6.0) + b_t604_tmp * t84_tmp /
    6.0) + t82 * t194 * gyroVar[1] / 6.0) - t136_tmp * gyroBiasVar[1] / 6.0) -
    t179 * t426 * gyroVar[2] / 6.0) - t207 * t419 * gyroVar[0] / 6.0) - t266 *
              t420_tmp * gyroVar[1] / 6.0) - t155 * gyroBiasVar[2] / 6.0)) -
           ((t147 + t110_tmp) - t116_tmp) * dt) - t2 * (((((t668_tmp / 3.0 +
    t144 * gyroVar[1] / 3.0) + b_t668_tmp / 3.0) + t45 * gyroVar[2] / 3.0) + t47 *
            t168_tmp * gyroVar[0] / 3.0) - t217 / 3.0) * dt) - e_t252_tmp *
    (((((t92 * t223 * gyroBiasVar[1] / 7.0 + t201 * t426 * gyroVar[2] / 7.0) + t42
        * gyroBiasVar[0] / 7.0) - t194 * t420_tmp * gyroVar[1] / 7.0) - t264 *
      t419 * gyroVar[0] / 7.0) - t185 * t424 * gyroBiasVar[2] / 7.0) * dt;
  t104 *= t163;
  t215_tmp = t102 * t158;
  t89 = t108 * t159;
  t6 = t104 * accVar[2];
  t4 = t215_tmp * accVar[0];
  t5 = t89 * accVar[1];
  t89 *= accBiasVar[1];
  t104 *= accBiasVar[2];
  t215_tmp *= accBiasVar[0];
  t99 = t204 * t553;
  t78 = t128 * t210;
  t54_tmp = t187_tmp * t550;
  t59_tmp = t135 * t197;
  t60_tmp = t154 * t204;
  t63_tmp = t146_tmp * t210;
  t252_tmp_tmp = t135 * t187_tmp;
  t74 = t197 * t550;
  t87 = t107 * t166;
  t83 = t162 * t531;
  t702 = ((((t3 * (((((t166 * t536 * gyroVar[1] / 4.0 + t131 * t162 * gyroVar[2] /
                       4.0) + t168_tmp * t532 * gyroVar[0] / 4.0) - t114 * t207 *
                     gyroVar[0] / 4.0) - t107 * t266 * gyroVar[1] / 4.0) - t179 *
                   t531 * gyroVar[2] / 4.0) + e_t252_tmp * (((((((((((t146_tmp *
    t185 * gyroBiasVar[2] / 6.0 + t99 * gyroBiasVar[1] / 6.0) + t78 * gyroBiasVar[2]
    / 6.0) + t179 * t540 * gyroVar[2] / 6.0) + t266 * t543 * gyroVar[1] / 6.0) +
    t54_tmp * gyroBiasVar[0] / 6.0) + t207 * t545 * gyroVar[0] / 6.0) + t264 *
    t532 * gyroVar[0] / 6.0) - t131 * t201 * gyroVar[2] / 6.0) - t59_tmp *
    gyroBiasVar[0] / 6.0) - t154 * t223 * gyroBiasVar[1] / 6.0) - t194 * t536 *
              gyroVar[1] / 6.0)) - ((t6 + t4) - t5) * dt) - t3 *
           (((((((((((t207 * t532 * gyroVar[0] / 5.0 + t60_tmp * gyroBiasVar[1] /
                      5.0) + t201 * t531 * gyroVar[2] / 5.0) + t162 * t540 *
                    gyroVar[2] / 5.0) + t131 * t179 * gyroVar[2] / 5.0) + t63_tmp *
                  gyroBiasVar[2] / 5.0) + t168_tmp * t545 * gyroVar[0] / 5.0) -
                t107 * t194 * gyroVar[1] / 5.0) - t114 * t264 * gyroVar[0] / 5.0)
              - t252_tmp_tmp * gyroBiasVar[0] / 5.0) - t166 * t543 * gyroVar[1] /
             5.0) - t266 * t536 * gyroVar[1] / 5.0) * dt) - t2 * (((((t104 /
    3.0 + t215_tmp / 3.0) + t87 * gyroVar[1] / 3.0) - t114 * t168_tmp * gyroVar[0]
             / 3.0) - t89 / 3.0) - t83 * gyroVar[2] / 3.0) * dt) - e_t252_tmp
    * (((((t264 * t545 * gyroVar[0] / 7.0 + t74 * gyroBiasVar[0] / 7.0) + t194 *
          t543 * gyroVar[1] / 7.0) + t128 * t185 * gyroBiasVar[2] / 7.0) - t201 *
        t540 * gyroVar[2] / 7.0) - t223 * t553 * gyroBiasVar[1] / 7.0) * dt;
  t703 = -t167_tmp + t172_tmp;
  t23 = t703 * gyro[1];
  t705 = t178 + t23 / 2.0;
  t24 = t703 * gyro[2];
  t22 = t175_tmp - t24;
  t710 = t191 + t22 * gyro[0] / 6.0;
  t16 = t173_tmp + t23;
  t714 = t16 * gyro[1] / 6.0 - t22 * gyro[2] / 6.0;
  t53 = t158 * t158;
  t51 = t159 * t159;
  t717 = t163 * t163;
  t718 = t180 - t24 / 6.0;
  t720 = t193 + t16 * gyro[0] / 6.0;
  t723 = t184 + t23 / 6.0;
  t724 = t181 - t24 / 2.0;
  t734 = (t6 / 2.0 + t4 / 2.0) - t5 / 2.0;
  t735 = t89 / 8.0;
  t739_tmp = t47 * gyroBiasVar[0];
  t740_tmp = t92 * gyroVar[1];
  t741_tmp = t57 * gyroBiasVar[2];
  b_t741_tmp = t741_tmp * gyro[1];
  t743_tmp = t740_tmp * gyro[0];
  t744_tmp = t84_tmp * gyroVar[1];
  t744 = t744_tmp * gyro[2] / 4.0;
  t747_tmp = t741_tmp * gyro[0];
  h_t252_tmp = t2 * t98 * dt;
  t748 = h_t252_tmp * gyroVar[0] / 3.0;
  t749_tmp = t739_tmp * gyro[0];
  t749 = t749_tmp * gyro[1] / 252.0;
  t22 = t70 * gyroVar[2];
  t167_tmp = t22 * gyro[1];
  t750 = t167_tmp * gyro[2] / 10.0;
  t756_tmp = t98 * gyroVar[0];
  b_t756_tmp = t22 * gyro[0];
  t763 = t739_tmp * gyro[1] / 72.0;
  t764 = t740_tmp * gyro[1] * gyro[2] / 12.0;
  t765_tmp = t744_tmp * gyro[0];
  t22 = t57 * t70;
  t16 = t47 * t98;
  t23 = t84_tmp * t409;
  t24 = t47 * t430;
  t17 = t57 * t424;
  t47_tmp = t92 * t409;
  t796_tmp = t48 * accVar[1];
  b_t796_tmp = t49 * accVar[0];
  c_t796_tmp = t120 * accVar[2];
  d_t796_tmp = t48 * accBiasVar[1];
  e_t796_tmp = t49 * accBiasVar[0];
  f_t796_tmp = t120 * accBiasVar[2];
  t796 = (((e_t252_tmp * ((((((((t22 * gyroBiasVar[2] / 36.0 + t16 * gyroBiasVar[0]
    / 36.0) + t411 * t430 * gyroVar[0] / 6.0) + t424 * t431 * gyroVar[2] / 6.0) +
    t82 * t92 * gyroVar[1] / 6.0) - t70 * t426 * gyroVar[2] / 6.0) - t84_tmp *
    t420_tmp * gyroVar[1] / 6.0) - t98 * t419 * gyroVar[0] / 6.0) - t23 *
             gyroBiasVar[1] / 36.0) + t2 * ((t796_tmp / 2.0 + c_t796_tmp / 2.0) +
             b_t796_tmp / 2.0)) + e_t252_tmp * (((((t17 * gyroBiasVar[2] / 42.0 +
    t47_tmp * gyroBiasVar[1] / 42.0) + t740_tmp * t420_tmp / 7.0) + t419 * t430 *
              gyroVar[0] / 7.0) - t24 * gyroBiasVar[0] / 42.0) - t424 * t426 *
            gyroVar[2] / 7.0) * dt) + t3 * (((((d_t796_tmp / 8.0 + f_t796_tmp
    / 8.0) + e_t796_tmp / 8.0) + t22 * gyroVar[2] / 4.0) + t16 * gyroVar[0] / 4.0)
           - t23 * gyroVar[1] / 4.0)) - t3 * (((((t82 * t84_tmp * gyroVar[1] / 5.0
    + t98 * t411 * gyroVar[0] / 5.0) + t24 * gyroVar[0] / 5.0) - t17 * gyroVar[2] /
    5.0) - t47_tmp * gyroVar[1] / 5.0) - t70 * t431 * gyroVar[2] / 5.0) * dt;
  t889_tmp = c_t516_tmp / 8.0;
  t882 = t2 * t557;
  t819_tmp = t114 * t430;
  b_t819_tmp = t92 * t107;
  t16 = t47 * t135;
  c_t819_tmp = t516_tmp / 8.0 + b_t516_tmp / 8.0;
  d_t819_tmp = t57 * t146_tmp;
  e_t819_tmp = t154 * t409;
  f_t819_tmp = t57 * t128;
  g_t819_tmp = t47 * t550;
  h_t819_tmp = t409 * t553;
  i_t819_tmp = t84_tmp * t107;
  j_t819_tmp = t98 * t114;
  k_t819_tmp = t70 * t531;
  l_t819_tmp = t424 * t531;
  t819 = (((e_t252_tmp * ((((((((d_t819_tmp * gyroBiasVar[2] / 36.0 + t70 * t540 *
    gyroVar[2] / 6.0) + e_t819_tmp * gyroBiasVar[1] / 36.0) + t98 * t545 * gyroVar
    [0] / 6.0) + t84_tmp * t543 * gyroVar[1] / 6.0) + t430 * t532 * gyroVar[0] /
    6.0) - t16 * gyroBiasVar[0] / 36.0) - t131 * t424 * gyroVar[2] / 6.0) - t92 *
             t536 * gyroVar[1] / 6.0) + k_t475_tmp * (((((t84_tmp * t536 *
    gyroVar[1] / 5.0 + t819_tmp * gyroVar[0] / 5.0) + b_t819_tmp * gyroVar[1] / 5.0)
    - t70 * t131 * gyroVar[2] / 5.0) - t98 * t532 * gyroVar[0] / 5.0) - l_t819_tmp
             * gyroVar[2] / 5.0)) - t882) - t3 * ((((c_t819_tmp + k_t819_tmp *
              gyroVar[2] / 4.0) + j_t819_tmp * gyroVar[0] / 4.0) + i_t819_tmp *
            gyroVar[1] / 4.0) - t889_tmp)) - e_t252_tmp * (((((h_t819_tmp *
    gyroBiasVar[1] / 42.0 + g_t819_tmp * gyroBiasVar[0] / 42.0) + t430 * t545 *
    gyroVar[0] / 7.0) + t92 * t543 * gyroVar[1] / 7.0) + f_t819_tmp * gyroBiasVar[2]
    / 42.0) - t424 * t540 * gyroVar[2] / 7.0) * dt;
  t1011_tmp = t217 / 8.0;
  t1005_tmp = t2 * ((t147 / 2.0 + t110_tmp / 2.0) - t116_tmp / 2.0);
  b_t47_tmp = t70 * t162;
  t842_tmp = t47 * t197;
  b_t842_tmp = t162 * t424;
  c_t842_tmp = t92 * t166;
  t67 = t204 * t409;
  t62 = t57 * t210;
  t44 = t47 * t187_tmp;
  d_t842_tmp = t668_tmp / 8.0 + b_t668_tmp / 8.0;
  e_t842_tmp = t57 * t723;
  f_t842_tmp = t409 * t718;
  g_t842_tmp = t84_tmp * t166;
  h_t842_tmp = t98 * t703;
  i_t842_tmp = t430 * t703;
  t842 = (((e_t252_tmp * (((((e_t842_tmp * gyroBiasVar[2] / 42.0 + t842_tmp *
    gyroBiasVar[0] / 42.0) + t430 * t714 * gyroVar[0] / 7.0) + t424 * t710 *
    gyroVar[2] / 7.0) + t92 * t720 * gyroVar[1] / 7.0) - f_t842_tmp * gyroBiasVar[1]
             / 42.0) * dt - t1005_tmp) - t3 * ((((d_t842_tmp + b_t47_tmp *
    gyroVar[2] / 4.0) - t1011_tmp) - g_t842_tmp * gyroVar[1] / 4.0) - h_t842_tmp *
            gyroVar[0] / 4.0)) - e_t252_tmp * ((((((((t67 * gyroBiasVar[1] / 36.0
    + t92 * t724 * gyroVar[1] / 6.0) + t62 * gyroBiasVar[2] / 36.0) + t98 * t714 *
    gyroVar[0] / 6.0) + t207 * t430 * gyroVar[0] / 6.0) + t44 * gyroBiasVar[0] /
              36.0) + t84_tmp * t720 * gyroVar[1] / 6.0) - t70 * t710 * gyroVar[2]
            / 6.0) - t424 * t705 * gyroVar[2] / 6.0)) - t3 * (((((i_t842_tmp *
    gyroVar[0] / 5.0 + b_t842_tmp * gyroVar[2] / 5.0) + c_t842_tmp * gyroVar[1] /
    5.0) - t98 * t207 * gyroVar[0] / 5.0) - t70 * t705 * gyroVar[2] / 5.0) -
    t84_tmp * t724 * gyroVar[1] / 5.0) * dt;
  t843 = t424 * t424;
  t844 = t92 * t92;
  t845 = t57 * t57;
  t846 = t409 * t409;
  t847 = t430 * t430;
  t848 = t47 * t47;
  t850 = t70 * t70;
  t851 = t98 * t98;
  t853 = t40_tmp + t41_tmp;
  t856_tmp = t107 * gyroBiasVar[1];
  b_t856_tmp = t856_tmp * gyro[0];
  t857_tmp = t114 * gyroBiasVar[0];
  t858_tmp = t128 * gyroVar[2];
  b_t858_tmp = t858_tmp * gyro[1];
  t22 = t146_tmp * gyroVar[2];
  t859_tmp = t22 * gyro[0];
  t860 = t856_tmp * gyro[2] / 72.0;
  t861_tmp = t858_tmp * gyro[0];
  t861 = t861_tmp * gyro[2] / 12.0;
  t862_tmp = t22 * gyro[1];
  t863_tmp = t154 * gyroVar[1];
  t866 = t30 * t135 * gyroVar[0] / 6.0;
  t867_tmp = t135 * gyroVar[0];
  t867 = t867_tmp * gyro[2] / 4.0;
  t870_tmp = t857_tmp * gyro[0];
  t49 = t2 * t154 * dt;
  t871 = t49 * gyroVar[1] / 3.0;
  t872 = t856_tmp * gyro[1] * gyro[2] / 252.0;
  t873_tmp = t867_tmp * gyro[0];
  t873 = t873_tmp * gyro[2] / 10.0;
  t879_tmp = t863_tmp * gyro[0];
  t880 = t135 * t419 * gyroVar[0] / 6.0;
  t881 = t154 * t420_tmp * gyroVar[1] / 6.0;
  t890 = t16 * gyroVar[0] / 4.0;
  t22 = t114 * t135;
  t16 = t107 * t154;
  t23 = t146_tmp * t531;
  t24 = t107 * t553;
  t17 = t128 * t531;
  t47_tmp = t114 * t550;
  t919_tmp = t40 * accVar[0];
  b_t919_tmp = t41 * accVar[2];
  c_t919_tmp = t50 * accVar[1];
  d_t919_tmp = t40 * accBiasVar[0];
  e_t919_tmp = t41 * accBiasVar[2];
  f_t919_tmp = t50 * accBiasVar[1];
  t919 = (((e_t252_tmp * ((((((((t22 * gyroBiasVar[0] / 36.0 + t16 * gyroBiasVar[1]
    / 36.0) + t540 * gyroVar[2] * t146_tmp / 6.0) + t536 * t553 * gyroVar[1] / 6.0)
    + t128 * t131 * gyroVar[2] / 6.0) + t532 * t550 * gyroVar[0] / 6.0) - t135 *
    t545 * gyroVar[0] / 6.0) - t154 * t543 * gyroVar[1] / 6.0) - t23 * gyroBiasVar
             [2] / 36.0) + t2 * ((t919_tmp / 2.0 + b_t919_tmp / 2.0) +
             c_t919_tmp / 2.0)) + t3 * (((((d_t919_tmp / 8.0 + e_t919_tmp / 8.0)
    + f_t919_tmp / 8.0) + t22 * gyroVar[0] / 4.0) + t16 * gyroVar[1] / 4.0) - t23 *
            gyroVar[2] / 4.0)) - t3 * (((((t131 * t146_tmp * gyroVar[2] / 5.0 +
    t154 * t536 * gyroVar[1] / 5.0) + t24 * gyroVar[1] / 5.0) - t17 * gyroVar[2] /
             5.0) - t47_tmp * gyroVar[0] / 5.0) - t135 * t532 * gyroVar[0] / 5.0) *
          dt) - e_t252_tmp * (((((t24 * gyroBiasVar[1] / 42.0 + t128 * t540 *
    gyroVar[2] / 7.0) + t545 * t550 * gyroVar[0] / 7.0) - t17 * gyroBiasVar[2] /
    42.0) - t47_tmp * gyroBiasVar[0] / 42.0) - t543 * t553 * gyroVar[1] / 7.0) *
    dt;
  t938_tmp = t104 / 8.0;
  t940_tmp = t215_tmp / 8.0;
  t1020 = t2 * t734;
  t944_tmp = t154 * t166;
  b_t944_tmp = t146_tmp * t162;
  t23 = t114 * t197;
  c_t944_tmp = t166 * t553;
  d_t944_tmp = t128 * t162;
  t24 = t114 * t187_tmp;
  t17 = t210 * t531;
  e_t944_tmp = t107 * t204;
  f_t944_tmp = t107 * t718;
  g_t944_tmp = t531 * t723;
  h_t944_tmp = t135 * t703;
  i_t944_tmp = t550 * t703;
  t944 = (((e_t252_tmp * ((((((((t24 * gyroBiasVar[0] / 36.0 + t135 * t714 *
    gyroVar[0] / 6.0) + t17 * gyroBiasVar[2] / 36.0) + t154 * t720 * gyroVar[1] /
    6.0) + t146_tmp * t710 * gyroVar[2] / 6.0) + t553 * t724 * gyroVar[1] / 6.0) -
    e_t944_tmp * gyroBiasVar[1] / 36.0) - t207 * t550 * gyroVar[0] / 6.0) - t128 *
             t705 * gyroVar[2] / 6.0) + k_t475_tmp * (((((t146_tmp * t705 *
    gyroVar[2] / 5.0 + c_t944_tmp * gyroVar[1] / 5.0) + d_t944_tmp * gyroVar[2] /
    5.0) - t135 * t207 * gyroVar[0] / 5.0) - t154 * t724 * gyroVar[1] / 5.0) -
             i_t944_tmp * gyroVar[0] / 5.0)) - t1020) - t3 * (((((-t735 +
    t938_tmp) + h_t944_tmp * gyroVar[0] / 4.0) + t940_tmp) + t944_tmp * gyroVar[1]
            / 4.0) + b_t944_tmp * gyroVar[2] / 4.0)) - e_t252_tmp *
    (((((g_t944_tmp * gyroBiasVar[2] / 42.0 + f_t944_tmp * gyroBiasVar[1] / 42.0)
        + t553 * t720 * gyroVar[1] / 7.0) + t128 * t710 * gyroVar[2] / 7.0) + t23 *
      gyroBiasVar[0] / 42.0) - t550 * t714 * gyroVar[0] / 7.0) * dt;
  t963_tmp = t98 * t135;
  b_t963_tmp = t84_tmp * t154;
  t963 = ((e_t252_tmp * (((((t84_tmp * t553 * gyroVar[1] / 6.0 + t523_tmp *
    gyroVar[0] / 6.0) + b_t523_tmp * gyroVar[1] / 6.0) + c_t523_tmp * gyroVar[2] /
              6.0) - d_t523_tmp * gyroVar[2] / 6.0) - t98 * t550 * gyroVar[0] /
            6.0) - t2 * ((t530_tmp / 3.0 + b_t530_tmp / 3.0) - c_t530_tmp / 3.0)
           * dt) - t3 * (((((t516_tmp / 20.0 + b_t516_tmp / 20.0) + t963_tmp
              * gyroVar[0] / 5.0) + b_t963_tmp * gyroVar[1] / 5.0) - t507_tmp *
            gyroVar[2] / 5.0) - c_t516_tmp / 20.0) * dt) - e_t252_tmp *
    (((((t57 * t531 * gyroBiasVar[2] / 252.0 + t527_tmp * gyroVar[2] / 7.0) +
        d_t516_tmp * gyroBiasVar[0] / 252.0) + t92 * t553 * gyroVar[1] / 7.0) -
      e_t516_tmp * gyroBiasVar[1] / 252.0) - t430 * t550 * gyroVar[0] / 7.0) *
    dt;
  t964 = t128 * t128;
  t965 = t550 * t550;
  t966 = t114 * t114;
  t967 = t531 * t531;
  t968 = t553 * t553;
  t969 = t107 * t107;
  t970 = t135 * t135;
  t972 = t154 * t154;
  t973 = t44_tmp + t53_tmp;
  t975_tmp = t162 * gyroBiasVar[2];
  t22 = t975_tmp * gyro[0];
  t975 = t22 * gyro[2] / 252.0;
  t976_tmp = t204 * gyroVar[1];
  b_t976_tmp = t976_tmp * gyro[0];
  t976 = b_t976_tmp * gyro[1] / 10.0;
  t977_tmp = t166 * gyroBiasVar[1];
  t50 = t975_tmp * gyro[1];
  t982_tmp = t197 * gyroVar[0];
  t16 = t210 * gyroVar[2];
  t41 = t16 * gyro[1];
  t40 = t187_tmp * gyroVar[0];
  t988_tmp = t40 * gyro[0];
  t989 = t22 / 72.0;
  t990_tmp = t982_tmp * gyro[0];
  t990 = t990_tmp * gyro[1] / 12.0;
  t120 = t16 * gyro[0];
  t995_tmp = t977_tmp * gyro[0];
  t998 = t40 * gyro[1] / 4.0;
  t48 = t2 * t210 * dt;
  t1002 = t48 * gyroVar[2] / 3.0;
  t1003 = b_t47_tmp * gyroBiasVar[2] / 36.0;
  t1004 = t197 * t411 * gyroVar[0] / 6.0;
  t1012 = t67 * gyroVar[1] / 4.0;
  t1013 = t62 * gyroVar[2] / 4.0;
  t1014 = t44 * gyroVar[0] / 4.0;
  t1021 = t23 * gyroVar[0] / 5.0;
  t1025 = t24 * gyroVar[0] / 4.0;
  t1026 = t17 * gyroVar[2] / 4.0;
  t22 = t166 * t204;
  t16 = t162 * t210;
  t23 = t187_tmp * t703;
  t24 = t162 * t723;
  t17 = t166 * t718;
  t47_tmp = t197 * t703;
  t1055_tmp = t53 * accVar[0];
  b_t1055_tmp = t51 * accVar[1];
  c_t1055_tmp = t717 * accVar[2];
  d_t1055_tmp = t53 * accBiasVar[0];
  e_t1055_tmp = t51 * accBiasVar[1];
  f_t1055_tmp = t717 * accBiasVar[2];
  t1055 = (((e_t252_tmp * ((((((((t22 * gyroBiasVar[1] / 36.0 + t16 * gyroBiasVar
    [2] / 36.0) + t714 * gyroVar[0] * t187_tmp / 6.0) + t705 * t723 * gyroVar[2] /
    6.0) + t718 * t724 * gyroVar[1] / 6.0) + t197 * t207 * gyroVar[0] / 6.0) -
    t210 * t710 * gyroVar[2] / 6.0) - t204 * t720 * gyroVar[1] / 6.0) - t23 *
              gyroBiasVar[0] / 36.0) + t2 * ((t1055_tmp / 2.0 + b_t1055_tmp / 2.0)
              + c_t1055_tmp / 2.0)) + t3 * (((((d_t1055_tmp / 8.0 + e_t1055_tmp /
    8.0) + f_t1055_tmp / 8.0) + t22 * gyroVar[1] / 4.0) + t16 * gyroVar[2] / 4.0)
             - t23 * gyroVar[0] / 4.0)) - t3 * (((((t210 * t705 * gyroVar[2] / 5.0
    + t24 * gyroVar[2] / 5.0) + t187_tmp * t207 * gyroVar[0] / 5.0) - t17 *
              gyroVar[1] / 5.0) - t47_tmp * gyroVar[0] / 5.0) - t204 * t724 *
            gyroVar[1] / 5.0) * dt) - e_t252_tmp * (((((t24 * gyroBiasVar[2] /
    42.0 + t718 * t720 * gyroVar[1] / 7.0) + t197 * t714 * gyroVar[0] / 7.0) - t17
    * gyroBiasVar[1] / 42.0) - t47_tmp * gyroBiasVar[0] / 42.0) - t710 * t723 *
    gyroVar[2] / 7.0) * dt;
  t1074 = ((e_t252_tmp * (((((t70 * t723 * gyroVar[2] / 6.0 + t187_tmp * t430 *
    gyroVar[0] / 6.0) + t139 * gyroVar[0] / 6.0) + t84_tmp * t718 * gyroVar[1] /
    6.0) - t136_tmp * gyroVar[1] / 6.0) - t155 * gyroVar[2] / 6.0) - t2 * ((t147 /
              3.0 + t110_tmp / 3.0) - t116_tmp / 3.0) * dt) - t3 *
           (((((t668_tmp / 20.0 + b_t668_tmp / 20.0) + t137 * gyroVar[2] / 5.0) +
              t98 * t187_tmp * gyroVar[0] / 5.0) - t217 / 20.0) - t115_tmp *
            gyroVar[1] / 5.0) * dt) - e_t252_tmp * (((((t144 * gyroBiasVar[1] /
    252.0 + t92 * t718 * gyroVar[1] / 7.0) + t45 * gyroBiasVar[2] / 252.0) + t42 *
    gyroVar[0] / 7.0) - t47 * t703 * gyroBiasVar[0] / 252.0) - t424 * t723 *
    gyroVar[2] / 7.0) * dt;
  t1093 = ((e_t252_tmp * (((((t146_tmp * t723 * gyroVar[2] / 6.0 + t99 * gyroVar[1]
    / 6.0) + t78 * gyroVar[2] / 6.0) + t54_tmp * gyroVar[0] / 6.0) - t59_tmp *
              gyroVar[0] / 6.0) - t154 * t718 * gyroVar[1] / 6.0) - t2 * ((t6 /
              3.0 + t4 / 3.0) - t5 / 3.0) * dt) - t3 * (((((t104 / 20.0 +
    t215_tmp / 20.0) + t60_tmp * gyroVar[1] / 5.0) + t63_tmp * gyroVar[2] / 5.0) -
             t252_tmp_tmp * gyroVar[0] / 5.0) - t89 / 20.0) * dt) -
    e_t252_tmp * (((((t114 * t703 * gyroBiasVar[0] / 252.0 + t74 * gyroVar[0] /
                      7.0) + t87 * gyroBiasVar[1] / 252.0) + t128 * t723 *
                    gyroVar[2] / 7.0) - t83 * gyroBiasVar[2] / 252.0) - t553 *
                  t718 * gyroVar[1] / 7.0) * dt;
  t1094 = t718 * t718;
  t1095 = t197 * t197;
  t1096 = t166 * t166;
  t1097 = t703 * t703;
  t1098 = t723 * t723;
  t1099 = t162 * t162;
  t1101 = t204 * t204;
  t1102 = t210 * t210;
  t1104 = t103_tmp + t109_tmp;
  t1106 = t3 * t15 * gyroBiasVar[0] / 4.0 - t2 * gyroBiasVar[0] / 2.0;
  t1108 = t2 * (t268 - t269) * dt * gyroBiasVar[0] / 24.0;
  t1109 = t269_tmp * gyro[2];
  t1112 = t3 * t430 * gyroBiasVar[0] / 4.0 - h_t252_tmp * gyroBiasVar[0] / 3.0;
  t16 = t2 * t135 * dt;
  t1115 = t3 * t550 * gyroBiasVar[0] / 4.0 + t16 * gyroBiasVar[0] / 3.0;
  t1117 = b_t408_tmp * gyroBiasVar[0] * t187_tmp / 3.0 - t3 * t197 * gyroBiasVar[0]
    / 4.0;
  t1118 = t3 * t114 * gyroBiasVar[0] / 24.0;
  t1121 = t3 * t27 * gyroBiasVar[1] / 4.0 - t2 * gyroBiasVar[1] / 2.0;
  t1123 = t2 * (t342 - t343) * dt * gyroBiasVar[1] / 24.0;
  t1125 = b_t408_tmp * gyroBiasVar[1] * t84_tmp / 3.0 - t3 * t92 * gyroBiasVar[1] /
    4.0;
  t1127 = t3 * t553 * gyroBiasVar[1] / 4.0 - t49 * gyroBiasVar[1] / 3.0;
  t24 = t2 * t204 * dt;
  t1130 = t3 * t718 * gyroBiasVar[1] / 4.0 + t24 * gyroBiasVar[1] / 3.0;
  t1131 = t3 * t166 * gyroBiasVar[1] / 24.0;
  t1134 = t3 * t36 * gyroBiasVar[2] / 4.0 - t2 * gyroBiasVar[2] / 2.0;
  t17 = t2 * t70 * dt;
  t1137 = t3 * t424 * gyroBiasVar[2] / 4.0 + t17 * gyroBiasVar[2] / 3.0;
  t23 = b_t408_tmp * gyroBiasVar[2];
  t1139 = t23 * t146_tmp / 3.0 - t3 * t128 * gyroBiasVar[2] / 4.0;
  t1141 = t3 * t723 * gyroBiasVar[2] / 4.0 - t48 * gyroBiasVar[2] / 3.0;
  t1142 = t3 * t57 * gyroBiasVar[2] / 24.0;
  t22 = t2 * (t40_tmp - t41_tmp);
  t1143 = t22 * accBiasVar[0];
  t1144 = t22 * dt * accBiasVar[0] / 3.0;
  t22 = t2 * (t44_tmp - t53_tmp);
  t1146 = t22 * dt * accBiasVar[1] / 3.0;
  t1148_tmp = t2 * (t103_tmp - t109_tmp);
  t1148 = t1148_tmp * dt * accBiasVar[2] / 3.0;
  Qd_tmp = j_t252_tmp * gyro[2] / 4.0;
  b_Qd_tmp = m_t252_tmp * gyro[2] / 36.0;
  Qd[0] = ((((dt * gyroVar[0] + t3 * (t330 - Qd_tmp)) + b_t408_tmp *
             (((gyroBiasVar[0] / 3.0 + k_t252_tmp / 3.0) + e_t300_tmp / 3.0) - t7
              * gyroVar[0] / 3.0)) - e_t252_tmp * (((t253 + f_t300_tmp * gyro[1]
    / 6.0) - l_t252_tmp * gyro[2] / 6.0) - b_Qd_tmp)) + k_t475_tmp *
           (((((gyroBiasVar[0] * (t255 + t334) * -0.2 + n_t252_tmp / 20.0) +
               g_t300_tmp / 20.0) + t8 * t8 * gyroVar[0] / 5.0) + gyroVar[2] *
             (t332 - t11 * gyro[1] * 2.0) / 5.0) + gyroVar[1] * (t254 - t14 *
             gyro[2] * 2.0) / 5.0)) + b_t527_tmp * ((((t11 * t11 * gyroVar[2] /
    7.0 + t14 * t14 * gyroVar[1] / 7.0) + t15 * t15 * gyroBiasVar[0] / 7.0) +
    t254_tmp * gyroBiasVar[1] / 252.0) + t332_tmp * gyroBiasVar[2] / 252.0);
  Qd[1] = t252;
  Qd[2] = t300;
  c_Qd_tmp = t381_tmp * gyro[0];
  t7 = (t8 * t47 * gyroVar[0] / 4.0 + t359_tmp * gyro[1] / 8.0) + t360_tmp *
    gyro[2] / 8.0;
  t41_tmp = c_t381_tmp / 4.0;
  t53_tmp = t374_tmp * gyro[2] / 4.0;
  t40_tmp = t2 * ((c_t350_tmp / 3.0 + t350_tmp * gyro[2] / 3.0) - d_t350_tmp /
                  3.0) * dt;
  t109_tmp = ((t15 * t98 * gyroBiasVar[0] / 6.0 + t14 * t82 * gyroVar[1] / 6.0) +
              b_t346_tmp * gyro[1] / 36.0) + t347_tmp * gyro[2] / 36.0;
  n_t252_tmp = t357_tmp * gyro[2] / 12.0;
  l_t252_tmp = e_t381_tmp * gyro[2] / 12.0;
  k_t252_tmp = (((((t351_tmp / 5.0 + t352_tmp / 5.0) + t11 * t57 * gyroVar[2] /
                   5.0) + t346_tmp * gyro[2] / 10.0) - t14 * t58_tmp * gyroVar[1]
                 / 5.0) - t8 * t79 * gyroVar[0] / 5.0) - t371_tmp / 10.0;
  j_t252_tmp = d_t408_tmp * gyro[1] / 10.0;
  g_t300_tmp = (t15 * t88 * gyroBiasVar[0] / 7.0 + t11 * t75 * gyroVar[2] / 7.0) +
    b_t357_tmp * gyro[1] / 42.0;
  Qd[3] = ((((t361 - t3 * ((((t7 - t41_tmp) + b_t381_tmp / 4.0) - d_t381_tmp /
    4.0) - t53_tmp)) - t40_tmp) - e_t252_tmp * ((((((t109_tmp + t11 * t258 *
    gyroVar[2] / 6.0) - t8 * t338 * gyroVar[0] / 6.0) - t365_tmp / 12.0) -
    n_t252_tmp) - l_t252_tmp) + c_t408_tmp * gyro[1] / 12.0)) - k_t475_tmp *
           (((k_t252_tmp + t408_tmp * gyro[2] / 5.0) - j_t252_tmp) + c_Qd_tmp *
            gyro[2] / 10.0)) + b_t527_tmp * ((g_t300_tmp + t14 * t339 * gyroVar
    [1] / 7.0) - t370_tmp * gyro[2] / 42.0);
  f_t300_tmp = (((t435_tmp / 4.0 + t8 * t114 * gyroVar[0] / 4.0) + b_t437_tmp *
                 gyro[1] / 8.0) - c_t475_tmp / 4.0) - l_t475_tmp / 4.0;
  e_t300_tmp = m_t475_tmp * gyro[2] / 8.0;
  d_t381_tmp = (((((t14 * t125_tmp * gyroVar[1] / 6.0 + t439_tmp / 12.0) + t8 *
                   t138 * gyroVar[0] / 6.0) + t441_tmp * gyro[0] * gyro[2] /
                  36.0) - t11 * t131 * gyroVar[2] / 6.0) - t15 * t135 *
                gyroBiasVar[0] / 6.0) - f_t475_tmp * gyro[2] / 12.0;
  c_t408_tmp = t501_tmp * gyro[1] / 36.0;
  t370_tmp = ((t445_tmp * gyro[2] / 5.0 + t14 * t107 * gyroVar[1] / 5.0) +
              b_t447_tmp / 5.0) + t11 * t111_tmp * gyroVar[2] / 5.0;
  b_t381_tmp = t8 * t149 * gyroVar[0] / 5.0;
  t371_tmp = o_t475_tmp / 10.0;
  d_t408_tmp = h_t475_tmp * gyro[2] / 10.0;
  b_t357_tmp = t485_tmp * gyro[1] / 10.0;
  t357_tmp = p_t475_tmp * gyro[2] / 10.0;
  e_t381_tmp = t2 * t114 * gyroVar[0] / 2.0;
  Qd[4] = ((((t444 + t3 * ((f_t300_tmp + o_t252_tmp * t125_tmp / 4.0) -
    e_t300_tmp)) - e_t252_tmp * (((d_t381_tmp - c_t408_tmp) + b_t501_tmp *
    gyro[1] / 12.0) - n_t475_tmp * gyro[2] / 12.0)) - k_t475_tmp *
            ((((((t370_tmp + d_t475_tmp / 5.0) - b_t381_tmp) - t371_tmp) -
               d_t408_tmp) - b_t357_tmp) - t357_tmp)) - e_t381_tmp) + b_t527_tmp
    * ((((t434 + d_t252_tmp * t263_tmp / 7.0) + t14 * t260 * gyroVar[1] / 7.0) +
        t15 * t220 * gyroBiasVar[0] / 7.0) + m_t252_tmp * t216_tmp / 42.0);
  b_t346_tmp = b_t564_tmp * gyro[1] / 8.0 + t565_tmp / 4.0;
  t347_tmp = t8 * t168_tmp * gyroVar[0] / 4.0;
  c_t350_tmp = e_t604_tmp / 4.0;
  t350_tmp = f_t604_tmp * gyro[2] / 8.0;
  n_t475_tmp = ((((((t573_tmp / 5.0 + t574_tmp / 5.0) + t11 * t162 * gyroVar[2] /
                    5.0) + t569_tmp * gyro[2] / 5.0) + t14 * t166 * gyroVar[1] /
                  5.0) - t8 * t207 * gyroVar[0] / 5.0) - g_t604_tmp / 10.0) -
    d_t604_tmp * gyro[2] / 10.0;
  c_t381_tmp = h_t604_tmp * gyro[2] / 10.0;
  t359_tmp = t2 * ((t564_tmp * gyro[2] / 3.0 + b_t572_tmp / 3.0) - t572_tmp /
                   3.0) * dt;
  p_t475_tmp = t2 * t168_tmp * gyroVar[0] / 2.0;
  t381_tmp = (((t566_tmp / 12.0 + t15 * t187_tmp * gyroBiasVar[0] / 6.0) +
               t568_tmp * gyro[2] / 36.0) + b_t569_tmp * gyro[1] / 12.0) - t11
    * t179 * gyroVar[2] / 6.0;
  o_t475_tmp = b_t604_tmp * gyro[2] / 12.0;
  b_t447_tmp = b_t634_tmp * gyro[1] / 36.0;
  t269_tmp = i_t604_tmp * gyro[2] / 12.0;
  t563_tmp = (t15 * t197 * gyroBiasVar[0] / 7.0 + t14 * t194 * gyroVar[1] / 7.0) +
    t563_tmp * gyro[2] / 42.0;
  Qd[5] = ((((-t3 * (((((b_t346_tmp + t604_tmp / 4.0) - t347_tmp) - c_t350_tmp)
                      + c_t604_tmp * gyro[2] / 4.0) - t350_tmp) + k_t475_tmp *
              ((n_t475_tmp - t634_tmp * gyro[1] / 10.0) - c_t381_tmp)) -
             t359_tmp) - p_t475_tmp) + e_t252_tmp * (((((t381_tmp + t8 * t264 *
    gyroVar[0] / 6.0) + t14 * t266 * gyroVar[1] / 6.0) - o_t475_tmp) - b_t447_tmp)
            - t269_tmp)) - b_t527_tmp * ((t563_tmp + t11 * t201 * gyroVar[2] /
    7.0) + c_t634_tmp * gyro[1] / 42.0);
  b_t604_tmp = t424 * gyroVar[2];
  b_t634_tmp = t409 * gyroBiasVar[1];
  l_t475_tmp = b_t604_tmp * gyro[0];
  m_t475_tmp = t430 * gyroVar[0];
  i_t604_tmp = b_t604_tmp * gyro[1];
  t439_tmp = b_t634_tmp * gyro[0];
  g_t604_tmp = t167_tmp / 4.0;
  t172_tmp = ((b_t741_tmp / 72.0 + f_t252_tmp * t84_tmp / 6.0) + t743_tmp *
              gyro[1] / 12.0) - t11 * t70 * gyroVar[2] / 6.0;
  t104 = (t739_tmp / 30.0 + t740_tmp * gyro[2] / 5.0) - t8 * t98 * gyroVar[0] /
    5.0;
  t102 = t765_tmp * gyro[1] / 10.0;
  t178 = b_t756_tmp * gyro[2] / 10.0;
  t175_tmp = (t15 * t47 * gyroBiasVar[0] / 42.0 + t14 * t92 * gyroVar[1] / 7.0) +
    t747_tmp * gyro[2] / 252.0;
  Qd[6] = (((t748 - t3 * ((t744 + m_t475_tmp / 4.0) - g_t604_tmp)) + e_t252_tmp *
            (((t172_tmp + t8 * t430 * gyroVar[0] / 6.0) + b_t634_tmp * gyro[2] /
              72.0) - l_t475_tmp * gyro[2] / 12.0)) + k_t475_tmp * (((t104 +
              i_t604_tmp / 5.0) - t102) - t178)) - b_t527_tmp * ((t175_tmp + t11
    * t424 * gyroVar[2] / 7.0) - t439_tmp * gyro[1] / 252.0);
  t569_tmp = t553 * gyroVar[1];
  t573_tmp = t531 * gyroBiasVar[2];
  t574_tmp = t573_tmp * gyro[0];
  d_t604_tmp = t550 * gyroVar[0];
  h_t604_tmp = t573_tmp * gyro[1];
  t564_tmp = t569_tmp * gyro[0];
  t572_tmp = t862_tmp / 4.0 + t863_tmp * gyro[2] / 4.0;
  b_t572_tmp = t11 * t146_tmp * gyroVar[2] / 6.0;
  t566_tmp = t14 * t154 * gyroVar[1] / 6.0;
  t568_tmp = ((t857_tmp / 30.0 + b_t858_tmp / 5.0) + t859_tmp * gyro[2] / 10.0)
    - t8 * t135 * gyroVar[0] / 5.0;
  b_t569_tmp = t879_tmp * gyro[1] / 10.0;
  t173_tmp = t16 * gyroVar[0] / 3.0;
  t193 = (t11 * t128 * gyroVar[2] / 7.0 + t15 * t114 * gyroBiasVar[0] / 42.0) +
    b_t856_tmp * gyro[1] / 252.0;
  Qd[7] = (((t3 * (t572_tmp - d_t604_tmp / 4.0) - e_t252_tmp * ((((((-t860 -
    t861) + b_t572_tmp) + t566_tmp) - t8 * t550 * gyroVar[0] / 6.0) + h_t604_tmp /
    72.0) + t564_tmp * gyro[1] / 12.0)) - k_t475_tmp * ((t568_tmp + t569_tmp *
              gyro[2] / 5.0) - b_t569_tmp)) - t173_tmp) + b_t527_tmp * ((t193 +
    t14 * t553 * gyroVar[1] / 7.0) + t574_tmp * gyro[2] / 252.0);
  t159 = t718 * gyroVar[1];
  t158 = t723 * gyroVar[2];
  t184 = t703 * gyroBiasVar[0];
  t181 = t158 * gyro[1];
  t167_tmp = t158 * gyro[0];
  b_t668_tmp = t159 * gyro[0];
  t668_tmp = t120 * gyro[2] / 10.0;
  b_t564_tmp = -t3 * ((t976_tmp * gyro[2] / 4.0 + t41 / 4.0) - t982_tmp / 4.0);
  t565_tmp = (((t977_tmp * gyro[2] / 72.0 + t8 * t197 * gyroVar[0] / 6.0) + t50 /
               72.0) - t14 * t204 * gyroVar[1] / 6.0) - t11 * t210 * gyroVar[2] /
    6.0;
  e_t604_tmp = t995_tmp * gyro[1] / 252.0;
  f_t604_tmp = t2 * t187_tmp * dt * gyroVar[0] / 3.0;
  Qd[8] = (((b_t564_tmp + k_t475_tmp * (((((-t976 + t184 / 30.0) + t181 / 5.0) -
    t159 * gyro[2] / 5.0) + b_t252_tmp * t187_tmp / 5.0) + t668_tmp)) -
            e_t252_tmp * ((t565_tmp + b_t668_tmp * gyro[1] / 12.0) + t167_tmp *
             gyro[2] / 12.0)) - f_t604_tmp) - b_t527_tmp * ((((-t975 - t14 *
    t718 * gyroVar[1] / 7.0) + t11 * t723 * gyroVar[2] / 7.0) + t15 * t703 *
    gyroBiasVar[0] / 42.0) + e_t604_tmp);
  Qd[9] = t1106;
  t42 = t2 * (t268 + t269) * dt * gyroBiasVar[1] * -0.041666666666666664;
  Qd[10] = t42;
  Qd[11] = t23 * (t341 - t1109) / 24.0;
  Qd[12] = 0.0;
  Qd[13] = 0.0;
  Qd[14] = 0.0;
  Qd[15] = 0.0;
  Qd[16] = 0.0;
  Qd[17] = 0.0;
  Qd[18] = t252;
  t45 = r_t252_tmp * gyro[2] / 36.0;
  Qd[19] = ((((dt * gyroVar[1] - t3 * (t329 - Qd_tmp)) + b_t408_tmp *
              (((gyroBiasVar[1] / 3.0 + p_t252_tmp / 3.0) + b_t328_tmp / 3.0) -
               t19 * gyroVar[1] / 3.0)) + e_t252_tmp * (((t253 + c_t328_tmp *
    gyro[1] / 6.0) - q_t252_tmp * gyro[2] / 6.0) - t45)) + k_t475_tmp *
            (((((gyroBiasVar[1] * (t255 + t333) * -0.2 + s_t252_tmp / 20.0) +
                d_t328_tmp / 20.0) + t25 * t25 * gyroVar[1] / 5.0) + gyroVar[2] *
              (t331 - t21 * gyro[0] * 2.0) / 5.0) + gyroVar[0] * (t254 - t30 *
              gyro[2] * 2.0) / 5.0)) + b_t527_tmp * ((((t21 * t21 * gyroVar[2] /
    7.0 + t30 * t30 * gyroVar[0] / 7.0) + t27 * t27 * gyroBiasVar[1] / 7.0) +
    t254_tmp * gyroBiasVar[0] / 252.0) + t331_tmp * gyroBiasVar[2] / 252.0);
  Qd[20] = t328;
  Qd_tmp = c_Qd_tmp / 4.0;
  c_Qd_tmp = d_t350_tmp * gyro[2] / 8.0;
  t115_tmp = t27 * t84_tmp * gyroBiasVar[1] / 6.0;
  t144 = t21 * t258 * gyroVar[2] / 6.0;
  t137 = t351_tmp * gyro[2] / 12.0;
  t155 = e_t408_tmp * gyro[1] / 36.0;
  t136_tmp = t352_tmp * gyro[2] / 12.0;
  t116_tmp = t2 * ((t360_tmp / 3.0 + t368_tmp * gyro[2] / 3.0) - t374_tmp / 3.0)
    * dt;
  t139 = t2 * t58_tmp * gyroVar[1] / 2.0;
  t110_tmp = ((t27 * t92 * gyroBiasVar[1] / 7.0 + t21 * t75 * gyroVar[2] / 7.0) +
              t364_tmp * gyro[1] / 42.0) + t365_tmp * gyro[2] / 42.0;
  Qd[21] = ((((t381 - t3 * ((((((-t366 - t367) - t368) + t346_tmp / 4.0) +
    t408_tmp / 4.0) + Qd_tmp) + c_Qd_tmp)) - t116_tmp) - t139) + e_t252_tmp *
            (((((((((-t369 - t370) - t371) + t115_tmp) + t144) + t25 * t339 *
                 gyroVar[1] / 6.0) + t137) + t155) - f_t408_tmp * gyro[1] / 12.0)
             + t136_tmp)) - b_t527_tmp * (t110_tmp + t30 * t338 * gyroVar[0] /
    7.0);
  Qd[22] = t475;
  Qd[23] = t604;
  t147 = m_t475_tmp * gyro[0];
  t83 = t756_tmp * gyro[0];
  t217 = t83 * gyro[1] / 10.0;
  t87 = -t3 * ((b_t756_tmp / 4.0 + t756_tmp * gyro[2] / 4.0) - t740_tmp / 4.0);
  t74 = (((t747_tmp / 72.0 + t25 * t92 * gyroVar[1] / 6.0) + t739_tmp * gyro[2] /
          72.0) - t21 * t70 * gyroVar[2] / 6.0) - t30 * t98 * gyroVar[0] / 6.0;
  t63_tmp = b_t741_tmp * gyro[2] / 252.0;
  t252_tmp_tmp = t2 * t84_tmp * dt * gyroVar[1] / 3.0;
  Qd[24] = (((t87 + k_t475_tmp * (((((-t750 + b_t634_tmp / 30.0) - l_t475_tmp /
    5.0) + m_t475_tmp * gyro[2] / 5.0) + t252_tmp * t84_tmp / 5.0) + t217)) -
             e_t252_tmp * ((t74 + t147 * gyro[1] / 12.0) + i_t604_tmp * gyro[2]
              / 12.0)) - t252_tmp_tmp) - b_t527_tmp * ((((-t749 - t21 * t424 *
    gyroVar[2] / 7.0) + t30 * t430 * gyroVar[0] / 7.0) + t27 * t409 * gyroBiasVar[1]
    / 42.0) + t63_tmp);
  l_t475_tmp = d_t604_tmp * gyro[0];
  i_t604_tmp = t859_tmp / 4.0;
  t60_tmp = (t856_tmp / 30.0 + t861_tmp / 5.0) - t25 * t154 * gyroVar[1] / 5.0;
  t59_tmp = t873_tmp * gyro[1] / 10.0;
  t78 = t862_tmp * gyro[2] / 10.0;
  t54_tmp = t857_tmp * gyro[2] / 72.0;
  t5 = b_t858_tmp * gyro[2] / 12.0;
  t99 = (t27 * t107 * gyroBiasVar[1] / 42.0 + t21 * t128 * gyroVar[2] / 7.0) +
    t870_tmp * gyro[1] / 252.0;
  Qd[25] = (((t871 - t3 * ((-t867 + t569_tmp / 4.0) + i_t604_tmp)) + e_t252_tmp *
             ((((((-t866 + t25 * t553 * gyroVar[1] / 6.0) + t574_tmp / 72.0) +
                 t54_tmp) + c_t252_tmp * t146_tmp / 6.0) - l_t475_tmp * gyro[1]
               / 12.0) + t5)) + k_t475_tmp * (((t60_tmp + d_t604_tmp * gyro[2] /
    5.0) - t59_tmp) - t78)) - b_t527_tmp * ((t99 + t30 * t550 * gyroVar[0] / 7.0)
    - h_t604_tmp * gyro[2] / 252.0);
  t574_tmp = t184 * gyro[0];
  h_t604_tmp = t40 * gyro[2] / 4.0 + t120 / 4.0;
  t4 = ((t977_tmp / 30.0 + t982_tmp * gyro[2] / 5.0) + t988_tmp * gyro[1] /
        10.0) - t25 * t204 * gyroVar[1] / 5.0;
  t215_tmp = t41 * gyro[2] / 10.0;
  t89 = t30 * t187_tmp * gyroVar[0] / 6.0;
  t6 = t21 * t210 * gyroVar[2] / 6.0;
  h_t252_tmp = t24 * gyroVar[1] / 3.0;
  t717 = (t30 * t197 * gyroVar[0] / 7.0 + t27 * t166 * gyroBiasVar[1] / 42.0) +
    t50 * gyro[2] / 252.0;
  Qd[26] = (((t3 * (h_t604_tmp - t159 / 4.0) - e_t252_tmp * ((((((-t989 - t990)
    + t89) + t6) - t25 * t718 * gyroVar[1] / 6.0) + t184 * gyro[2] / 72.0) +
    t181 * gyro[2] / 12.0)) - k_t475_tmp * ((t4 + t167_tmp / 5.0) - t215_tmp))
            - h_t252_tmp) + b_t527_tmp * ((t717 + t21 * t723 * gyroVar[2] / 7.0)
    + t574_tmp * gyro[1] / 252.0);
  Qd[27] = t1108;
  Qd[28] = t1121;
  t181 = t2 * (t342 + t343) * dt * gyroBiasVar[2] * -0.041666666666666664;
  Qd[29] = t181;
  Qd[30] = 0.0;
  Qd[31] = 0.0;
  Qd[32] = 0.0;
  Qd[33] = 0.0;
  Qd[34] = 0.0;
  Qd[35] = 0.0;
  Qd[36] = t300;
  Qd[37] = t328;
  Qd[38] = ((((dt * gyroVar[2] + t3 * (t329 - t330)) + b_t408_tmp *
              (((gyroBiasVar[2] / 3.0 + h_t300_tmp / 3.0) + e_t328_tmp / 3.0) -
               t32 * gyroVar[2] / 3.0)) - e_t252_tmp * (((f_t328_tmp * gyro[2] /
    6.0 - b_t300_tmp * gyro[1] * gyro[2] / 6.0) - t45) + b_Qd_tmp)) +
            k_t475_tmp * (((((gyroBiasVar[2] * (t333 + t334) * -0.2 + i_t300_tmp /
    20.0) + g_t328_tmp / 20.0) + t35 * t35 * gyroVar[2] / 5.0) + gyroVar[1] *
              (t331 - t34 * gyro[0] * 2.0) / 5.0) + gyroVar[0] * (t332 - t39 *
              gyro[1] * 2.0) / 5.0)) + b_t527_tmp * ((((t34 * t34 * gyroVar[1] /
    7.0 + t39 * t39 * gyroVar[0] / 7.0) + t36 * t36 * gyroBiasVar[2] / 7.0) +
    t332_tmp * gyroBiasVar[0] / 252.0) + t331_tmp * gyroBiasVar[1] / 252.0);
  Qd[39] = t408;
  b_Qd_tmp = (((t34 * t125_tmp * gyroVar[1] / 6.0 + t39 * t149 * gyroVar[0] / 6.0)
               + t36 * t146_tmp * gyroBiasVar[2] / 6.0) + h_t475_tmp * gyro[1] *
              gyro[2] / 36.0) + i_t475_tmp * gyro[2] / 12.0;
  t167_tmp = d_t475_tmp * gyro[1] / 12.0;
  t45 = q_t475_tmp * gyro[2] / 36.0;
  t51 = t445_tmp * gyro[1] * gyro[2] / 12.0;
  t50 = t3 * (((((((((t434_tmp / 5.0 + b_t501_tmp / 5.0) + t34 * t107 * gyroVar[1]
                     / 5.0) + t435_tmp * gyro[1] / 5.0) + t39 * t114 * gyroVar[0]
                   / 5.0) + t328_tmp * t125_tmp / 10.0) - t35 * t131 * gyroVar[2]
                 / 5.0) - t501_tmp / 10.0) - c_t475_tmp * gyro[1] / 10.0) -
              j_t475_tmp * gyro[2] / 10.0) * dt - t3 * ((((((t444_tmp *
    gyro[1] / 4.0 + g_t475_tmp * gyro[2] / 8.0) + t441_tmp / 4.0) + t485_tmp /
    4.0) - t447_tmp / 4.0) - t35 * t111_tmp * gyroVar[2] / 4.0) - t437_tmp *
    gyro[1] * gyro[2] / 8.0);
  t40 = t2 * ((t475_tmp * gyro[1] / 3.0 + b_t437_tmp / 3.0) - b_t475_tmp / 3.0)
    * dt;
  t41 = t2 * t111_tmp * gyroVar[2] / 2.0;
  t120 = e_t252_tmp * ((((t36 * t128 * gyroBiasVar[2] / 7.0 + t34 * t260 *
    gyroVar[1] / 7.0) + t39 * t138 * gyroVar[0] / 7.0) + e_t475_tmp * gyro[2] /
                        42.0) - f_t475_tmp * gyro[1] * gyro[2] / 42.0) *
    dt;
  Qd[40] = (((t50 + e_t252_tmp * (((((b_Qd_tmp - t300_tmp * t263_tmp / 6.0) -
    t167_tmp) - g_t252_tmp * t216_tmp / 12.0) - t45) - t51)) - t40) - t41) -
    t120;
  Qd[41] = t634;
  t49 = t765_tmp / 4.0 + t756_tmp * gyro[1] / 4.0;
  t44 = ((t741_tmp / 30.0 + t743_tmp / 5.0) + t744_tmp * gyro[1] * gyro[2] /
         10.0) - t35 * t70 * gyroVar[2] / 5.0;
  t83 = t83 * gyro[2] / 10.0;
  t53 = t34 * t84_tmp * gyroVar[1] / 6.0;
  t48 = t39 * t98 * gyroVar[0] / 6.0;
  t62 = t17 * gyroVar[2] / 3.0;
  t67 = (t34 * t92 * gyroVar[1] / 7.0 + t36 * t57 * gyroBiasVar[2] / 42.0) +
    t749_tmp * gyro[2] / 252.0;
  Qd[42] = (((t3 * (t49 - b_t604_tmp / 4.0) - e_t252_tmp * ((((((-t763 - t764) +
    t53) + t48) - t35 * t424 * gyroVar[2] / 6.0) + t439_tmp / 72.0) + t147 *
    gyro[2] / 12.0)) - k_t475_tmp * ((t44 + m_t475_tmp * gyro[1] / 5.0) - t83))
            - t62) + b_t527_tmp * ((t67 + t39 * t430 * gyroVar[0] / 7.0) +
    b_t634_tmp * gyro[1] * gyro[2] / 252.0);
  b_t604_tmp = t863_tmp * gyro[1] * gyro[2] / 10.0;
  b_t634_tmp = -t3 * ((t867_tmp * gyro[1] / 4.0 + t879_tmp / 4.0) - t858_tmp /
                      4.0);
  m_t475_tmp = (((t857_tmp * gyro[1] / 72.0 + t35 * t128 * gyroVar[2] / 6.0) +
                 b_t856_tmp / 72.0) - t39 * t135 * gyroVar[0] / 6.0) - t34 * t154
    * gyroVar[1] / 6.0;
  t439_tmp = t870_tmp * gyro[2] / 252.0;
  t147 = t2 * t146_tmp * dt * gyroVar[2] / 3.0;
  Qd[43] = (((b_t634_tmp + k_t475_tmp * (((((-t873 + t573_tmp / 30.0) + t564_tmp
    / 5.0) - d_t604_tmp * gyro[1] / 5.0) + t300_tmp * t146_tmp / 5.0) +
    b_t604_tmp)) - e_t252_tmp * ((m_t475_tmp + l_t475_tmp * gyro[2] / 12.0) +
              t569_tmp * gyro[1] * gyro[2] / 12.0)) - t147) - b_t527_tmp *
    ((((-t872 + t34 * t553 * gyroVar[1] / 7.0) - t39 * t550 * gyroVar[0] / 7.0) +
      t36 * t531 * gyroBiasVar[2] / 42.0) + t439_tmp);
  l_t475_tmp = b_t976_tmp / 4.0;
  t569_tmp = (t995_tmp / 72.0 + c_t300_tmp * t187_tmp / 6.0) + t990_tmp * gyro
    [2] / 12.0;
  t573_tmp = t34 * t204 * gyroVar[1] / 6.0;
  d_t604_tmp = (t975_tmp / 30.0 + t982_tmp * gyro[1] / 5.0) - t35 * t210 *
    gyroVar[2] / 5.0;
  t564_tmp = t988_tmp * gyro[2] / 10.0;
  b_t47_tmp = t976_tmp * gyro[1] * gyro[2] / 10.0;
  t47_tmp = (t36 * t162 * gyroBiasVar[2] / 42.0 + t39 * t197 * gyroVar[0] / 7.0) +
    t977_tmp * gyro[1] * gyro[2] / 252.0;
  Qd[44] = (((t1002 - t3 * ((t998 + t158 / 4.0) - l_t475_tmp)) + e_t252_tmp *
             ((((t569_tmp - t573_tmp) + t35 * t723 * gyroVar[2] / 6.0) + t184 *
               gyro[1] / 72.0) - t159 * gyro[1] * gyro[2] / 12.0)) +
            k_t475_tmp * (((d_t604_tmp + b_t668_tmp / 5.0) - t564_tmp) -
             b_t47_tmp)) - b_t527_tmp * ((t47_tmp + t34 * t718 * gyroVar[1] / 7.0)
    - t574_tmp * gyro[2] / 252.0);
  t574_tmp = t2 * (t341 + t1109) * dt * gyroBiasVar[0] *
    -0.041666666666666664;
  Qd[45] = t574_tmp;
  Qd[46] = t1123;
  Qd[47] = t1134;
  Qd[48] = 0.0;
  Qd[49] = 0.0;
  Qd[50] = 0.0;
  Qd[51] = 0.0;
  Qd[52] = 0.0;
  Qd[53] = 0.0;
  t159 = t77 * gyroVar[1];
  t158 = t86 * gyroVar[2];
  t184 = (t93 + t62_tmp / 6.0) * gyroBiasVar[2];
  b_t668_tmp = t101 * gyroVar[0];
  Qd[54] = ((((t361 - t3 * ((((t7 + b_t668_tmp / 4.0) - t41_tmp) - t158 * gyro
    [1] / 4.0) - t53_tmp)) - t40_tmp) - e_t252_tmp * ((((((t109_tmp + t11 * t86 *
    gyroVar[2] / 6.0) - t8 * t101 * gyroVar[0] / 6.0) - t184 * gyro[1] / 12.0) -
    n_t252_tmp) - t159 * gyro[0] * gyro[1] / 12.0) - l_t252_tmp)) - k_t475_tmp
            * (((k_t252_tmp - t159 * gyro[2] / 5.0) - j_t252_tmp) + t158 *
               gyro[0] * gyro[2] / 10.0)) + b_t527_tmp * ((g_t300_tmp - t14 *
    t77 * gyroVar[1] / 7.0) - t184 * gyro[0] * gyro[2] / 42.0);
  Qd[55] = ((((t381 + t3 * ((((((t366 + t367) + t368) + t159 / 4.0) -
    gyroBiasVar[1] * t84_tmp / 4.0) - Qd_tmp) - c_Qd_tmp)) - e_t252_tmp *
              (((((((((t369 + t370) + t371) + t25 * t77 * gyroVar[1] / 6.0) -
                    t115_tmp) - t144) - t137) + b_t668_tmp * gyro[0] * gyro[1]
                 / 12.0) - t155) - t136_tmp)) - t116_tmp) - t139) - b_t527_tmp *
    (t110_tmp + t30 * t101 * gyroVar[0] / 7.0);
  Qd[56] = t408;
  Qd_tmp = t84_tmp * t92;
  c_Qd_tmp = t70 * t424;
  t7 = t98 * t430;
  Qd[57] = ((((dt * ((t796_tmp + b_t796_tmp) + c_t796_tmp) - t3 * ((t47 *
    t79 * gyroVar[0] / 2.0 + t58_tmp * t82 * gyroVar[1] / 2.0) - t57 * t258 *
    gyroVar[2] / 2.0)) - e_t252_tmp * (((((Qd_tmp * gyroBiasVar[1] / 3.0 - t82 *
    t420_tmp * gyroVar[1] / 3.0) - c_Qd_tmp * gyroBiasVar[2] / 3.0) + t7 *
    gyroBiasVar[0] / 3.0) - t411 * t419 * gyroVar[0] / 3.0) + t426 * t431 *
    gyroVar[2] / 3.0)) + b_t408_tmp * (((((d_t796_tmp / 3.0 + e_t796_tmp / 3.0) +
    f_t796_tmp / 3.0) + t845 * gyroVar[2] / 3.0) + t846 * gyroVar[1] / 3.0) + t848
              * gyroVar[0] / 3.0)) + k_t475_tmp * (((((t851 * gyroBiasVar[0] / 5.0
    + t850 * gyroBiasVar[2] / 5.0) + gyroVar[1] * (t409 * t420_tmp * 2.0 + t82 *
    t82) / 5.0) + t84_tmp * t84_tmp * gyroBiasVar[1] / 5.0) - gyroVar[2] * (t57 *
    t75 * 2.0 - t258 * t258) / 5.0) - gyroVar[0] * (t47 * t419 * 2.0 - t411 *
              t411) / 5.0)) + b_t527_tmp * (((((t843 * gyroBiasVar[2] / 7.0 +
    t844 * gyroBiasVar[1] / 7.0) + t847 * gyroBiasVar[0] / 7.0) + t419 * t419 *
    gyroVar[0] / 7.0) + t426 * t426 * gyroVar[2] / 7.0) + t420_tmp * t420_tmp *
    gyroVar[1] / 7.0);
  t41_tmp = (t3 * (((((t125_tmp * t409 * gyroVar[1] / 4.0 + t114 * t411 * gyroVar
                       [0] / 4.0) + t82 * t107 * gyroVar[1] / 4.0) + t111_tmp *
                     t431 * gyroVar[2] / 4.0) - t57 * t131 * gyroVar[2] / 4.0) -
                   t47 * t149 * gyroVar[0] / 4.0) + b_t527_tmp * (((((t263_tmp *
    t426 * gyroVar[2] / 7.0 + t92 * t216_tmp * gyroBiasVar[1] / 7.0) + t220 * t430
    * gyroBiasVar[0] / 7.0) - t138 * t419 * gyroVar[0] / 7.0) - t527_tmp *
    gyroBiasVar[2] / 7.0) - t445_tmp * t420_tmp / 7.0)) - ((t530_tmp + b_t530_tmp)
    - c_t530_tmp) * dt;
  t53_tmp = e_t252_tmp * (((((((((((t98 * t220 * gyroBiasVar[0] / 6.0 + t138 *
    t411 * gyroVar[0] / 6.0) + t82 * t260 * gyroVar[1] / 6.0) + d_t523_tmp *
    gyroBiasVar[2] / 6.0) + t84_tmp * t216_tmp * gyroBiasVar[1] / 6.0) + t263_tmp *
    t431 * gyroVar[2] / 6.0) - b_t523_tmp * gyroBiasVar[1] / 6.0) - t125_tmp *
    t420_tmp * gyroVar[1] / 6.0) - t131 * t426 * gyroVar[2] / 6.0) - t149 * t419 *
    gyroVar[0] / 6.0) - t523_tmp * gyroBiasVar[0] / 6.0) - c_t523_tmp *
    gyroBiasVar[2] / 6.0);
  t40_tmp = t2 * (((((t516_tmp / 3.0 + b_t516_tmp / 3.0) + d_t516_tmp * gyroVar[0]
                     / 3.0) - t57 * t111_tmp * gyroVar[2] / 3.0) - c_t516_tmp /
                   3.0) - e_t516_tmp * gyroVar[1] / 3.0) * dt;
  t109_tmp = ((((t507_tmp * gyroBiasVar[2] / 5.0 + t47 * t138 * gyroVar[0] / 5.0)
                + t114 * t419 * gyroVar[0] / 5.0) + t107 * t420_tmp * gyroVar[1] /
               5.0) + t82 * t125_tmp * gyroVar[1] / 5.0) + t149 * t411 * gyroVar[0]
    / 5.0;
  n_t252_tmp = t963_tmp * gyroBiasVar[0] / 5.0;
  l_t252_tmp = t131 * t431 * gyroVar[2] / 5.0;
  k_t252_tmp = t260 * t409 * gyroVar[1] / 5.0;
  j_t252_tmp = t426 * gyroVar[2];
  Qd[58] = ((t41_tmp + k_t475_tmp * ((((((t109_tmp - b_t350_tmp * t263_tmp / 5.0)
    - n_t252_tmp) - l_t252_tmp) - k_t252_tmp) - h_t475_tmp * t84_tmp / 5.0) -
              j_t252_tmp * t111_tmp / 5.0)) - t53_tmp) - t40_tmp;
  Qd[59] = t668;
  Qd[60] = t796;
  g_t300_tmp = d_t819_tmp * gyroVar[2] / 4.0;
  t159 = e_t819_tmp * gyroVar[1] / 4.0;
  t158 = j_t819_tmp * gyroBiasVar[0] / 36.0;
  t184 = t128 * t431 * gyroVar[2] / 6.0;
  b_t668_tmp = ((t135 * t411 * gyroVar[0] / 5.0 + t82 * t154 * gyroVar[1] / 5.0) +
                t146_tmp * t431 * gyroVar[2] / 5.0) - f_t819_tmp * gyroVar[2] /
    5.0;
  t115_tmp = (t819_tmp * gyroBiasVar[0] / 42.0 + b_t819_tmp * gyroBiasVar[1] /
              42.0) + t128 * t426 * gyroVar[2] / 7.0;
  Qd[61] = (((-t882 - t3 * ((((c_t819_tmp - t889_tmp) + t890) - g_t300_tmp) -
    t159)) + k_t475_tmp * ((b_t668_tmp - g_t819_tmp * gyroVar[0] / 5.0) -
              h_t819_tmp * gyroVar[1] / 5.0)) - e_t252_tmp * ((((((((-t880 - t881)
    + i_t819_tmp * gyroBiasVar[1] / 36.0) + t158) + t184) + t146_tmp * t426 *
    gyroVar[2] / 6.0) + t82 * t553 * gyroVar[1] / 6.0) + k_t819_tmp * gyroBiasVar[2]
              / 36.0) - t411 * t550 * gyroVar[0] / 6.0)) + b_t527_tmp *
    (((t115_tmp + t419 * t550 * gyroVar[0] / 7.0) - t420_tmp * t553 * gyroVar[1] /
      7.0) - l_t819_tmp * gyroBiasVar[2] / 42.0);
  t144 = t187_tmp * t411 * gyroVar[0] / 5.0 + t842_tmp * gyroVar[0] / 5.0;
  t137 = t82 * t204 * gyroVar[1] / 5.0;
  t155 = t210 * t431 * gyroVar[2] / 5.0;
  t136_tmp = g_t842_tmp * gyroBiasVar[1] / 36.0;
  t116_tmp = t187_tmp * t419 * gyroVar[0] / 6.0;
  t139 = t210 * t426 * gyroVar[2] / 6.0;
  t110_tmp = (b_t842_tmp * gyroBiasVar[2] / 42.0 + t197 * t419 * gyroVar[0] / 7.0)
    + c_t842_tmp * gyroBiasVar[1] / 42.0;
  Qd[62] = (((-t1005_tmp - t3 * ((((d_t842_tmp - t1011_tmp) + t1012) + t1013) +
    t1014)) + k_t475_tmp * ((((t144 - t137) - t155) + e_t842_tmp * gyroVar[2] /
    5.0) - f_t842_tmp * gyroVar[1] / 5.0)) + e_t252_tmp * ((((((((-t1003 - t1004)
    + t136_tmp) + t116_tmp) + t139) - t82 * t718 * gyroVar[1] / 6.0) + h_t842_tmp
    * gyroBiasVar[0] / 36.0) + t431 * t723 * gyroVar[2] / 6.0) - t976_tmp *
             t420_tmp / 6.0)) - b_t527_tmp * (((t110_tmp + t420_tmp * t718 *
    gyroVar[1] / 7.0) + t426 * t723 * gyroVar[2] / 7.0) + i_t842_tmp * gyroBiasVar
    [0] / 42.0);
  Qd[63] = t1112;
  Qd[64] = t1125;
  Qd[65] = t1137;
  t23 = t2 * t52;
  t17 = t23 * accBiasVar[0] * -0.5;
  Qd[66] = t17;
  Qd[67] = t22 * accBiasVar[1];
  t24 = -t2 * t853 * accBiasVar[2];
  Qd[68] = t24;
  Qd[69] = 0.0;
  Qd[70] = 0.0;
  Qd[71] = 0.0;
  t16 = t142 + t118 * gyro[1] / 6.0;
  t22 = f_t475_tmp * gyro[0];
  Qd[72] = ((((t444 + t3 * ((f_t300_tmp + t485_tmp_tmp * gyro[2] / 4.0) -
    e_t300_tmp)) - e_t252_tmp * (((d_t381_tmp - j_t300_tmp * t16 / 12.0) + t141 *
    gyroVar[1] * gyro[0] * gyro[1] / 12.0) - c_t408_tmp)) - k_t475_tmp *
             ((((((t370_tmp + t157 * gyroBiasVar[0] / 5.0) - b_t381_tmp) -
                 t371_tmp) - d_t408_tmp) - b_t357_tmp) - t357_tmp)) - e_t381_tmp)
    + b_t527_tmp * ((((t434 + d_t252_tmp * t16 / 7.0) + t14 * t141 * gyroVar[1] /
                      7.0) + t15 * t157 * gyroBiasVar[0] / 7.0) + t22 * gyro[1] /
                    42.0);
  Qd[73] = t475;
  Qd[74] = (((t50 + e_t252_tmp * (((((b_Qd_tmp - t35 * t263_tmp * gyroVar[2] /
    6.0) - t22 / 12.0) - t167_tmp) - t45) - t51)) - t40) - t41) - t120;
  Qd[75] = ((t41_tmp - t53_tmp) - t40_tmp) + k_t475_tmp * ((((((t109_tmp - t57 *
    t263_tmp * gyroVar[2] / 5.0) - n_t252_tmp) - b_t963_tmp * gyroBiasVar[1] / 5.0)
    - t111_tmp * t426 * gyroVar[2] / 5.0) - l_t252_tmp) - k_t252_tmp);
  b_Qd_tmp = t128 * t146_tmp;
  t41_tmp = t135 * t550;
  t53_tmp = t154 * t553;
  Qd[76] = ((((dt * ((t919_tmp + b_t919_tmp) + c_t919_tmp) + t3 * ((t111_tmp
    * t131 * gyroVar[2] * -0.5 + t114 * t149 * gyroVar[0] / 2.0) + t437_tmp *
    t125_tmp / 2.0)) - e_t252_tmp * (((((b_Qd_tmp * gyroBiasVar[2] / 3.0 + t131 *
    t540 * gyroVar[2] / 3.0) - t41_tmp * gyroBiasVar[0] / 3.0) + t53_tmp *
    gyroBiasVar[1] / 3.0) + t532 * t545 * gyroVar[0] / 3.0) - t536 * t543 *
    gyroVar[1] / 3.0)) + b_t408_tmp * (((((d_t919_tmp / 3.0 + e_t919_tmp / 3.0) +
    f_t919_tmp / 3.0) + t966 * gyroVar[0] / 3.0) + t967 * gyroVar[2] / 3.0) + t969
              * gyroVar[1] / 3.0)) + k_t475_tmp * (((((t970 * gyroBiasVar[0] / 5.0
    + t972 * gyroBiasVar[1] / 5.0) + t146_tmp * t146_tmp * gyroBiasVar[2] / 5.0) -
    gyroVar[1] * (t107 * t543 * 2.0 - t536 * t536) / 5.0) - gyroVar[0] * (t114 *
    t545 * 2.0 - t532 * t532) / 5.0) - gyroVar[2] * (t531 * t540 * 2.0 - t131 *
              t131) / 5.0)) + b_t527_tmp * (((((t965 * gyroBiasVar[0] / 7.0 +
    t964 * gyroBiasVar[2] / 7.0) + t968 * gyroBiasVar[1] / 7.0) + t540 * t540 *
    gyroVar[2] / 7.0) + t543 * t543 * gyroVar[1] / 7.0) + t545 * t545 * gyroVar[0] /
    7.0);
  Qd[77] = t702;
  Qd[78] = t819;
  Qd[79] = t919;
  t40_tmp = e_t944_tmp * gyroVar[1] / 4.0;
  t109_tmp = t131 * t210 * gyroVar[2] / 5.0;
  n_t252_tmp = t187_tmp * t532 * gyroVar[0] / 5.0;
  l_t252_tmp = t204 * t536 * gyroVar[1] / 5.0;
  k_t252_tmp = (((t197 * t532 * gyroVar[0] / 6.0 + t944_tmp * gyroBiasVar[1] /
                  36.0) + t210 * t540 * gyroVar[2] / 6.0) + b_t944_tmp *
                gyroBiasVar[2] / 36.0) + t187_tmp * t545 * gyroVar[0] / 6.0;
  f_t300_tmp = t204 * t543 * gyroVar[1] / 6.0;
  e_t300_tmp = (c_t944_tmp * gyroBiasVar[1] / 42.0 + d_t944_tmp * gyroBiasVar[2] /
                42.0) + t197 * t545 * gyroVar[0] / 7.0;
  Qd[80] = (((-t1020 + t3 * (((((t735 - t938_tmp) - t940_tmp) + t1025) + t1026)
    - t40_tmp)) - k_t475_tmp * (((((t1021 - t109_tmp) - n_t252_tmp) - l_t252_tmp)
    + f_t944_tmp * gyroVar[1] / 5.0) + g_t944_tmp * gyroVar[2] / 5.0)) -
            e_t252_tmp * ((((k_t252_tmp - f_t300_tmp) + t131 * t723 * gyroVar[2] /
    6.0) + h_t944_tmp * gyroBiasVar[0] / 36.0) - t536 * t718 * gyroVar[1] / 6.0))
    + b_t527_tmp * (((e_t300_tmp + t543 * t718 * gyroVar[1] / 7.0) + t540 * t723 *
                     gyroVar[2] / 7.0) - i_t944_tmp * gyroBiasVar[0] / 42.0);
  Qd[81] = t1115;
  Qd[82] = t1127;
  Qd[83] = t1139;
  d_t381_tmp = -t2 * t973 * accBiasVar[0];
  Qd[84] = d_t381_tmp;
  c_t408_tmp = t2 * t108;
  t370_tmp = c_t408_tmp * accBiasVar[1] * -0.5;
  Qd[85] = t370_tmp;
  Qd[86] = t1148_tmp * accBiasVar[2];
  Qd[87] = 0.0;
  Qd[88] = 0.0;
  Qd[89] = 0.0;
  b_t381_tmp = t183 * gyroVar[1];
  Qd[90] = ((((-t3 * (((((b_t346_tmp + t190 * gyroVar[0] / 4.0) - t347_tmp) -
                        c_t350_tmp) + b_t381_tmp * gyro[2] / 4.0) - t350_tmp) +
               k_t475_tmp * ((n_t475_tmp - b_t381_tmp * gyro[0] * gyro[1] /
    10.0) - c_t381_tmp)) - t359_tmp) - p_t475_tmp) + e_t252_tmp * (((((t381_tmp
    + t8 * t190 * gyroVar[0] / 6.0) + t14 * t183 * gyroVar[1] / 6.0) - o_t475_tmp)
              - b_t447_tmp) - t269_tmp)) - b_t527_tmp * ((t563_tmp + d_t252_tmp *
    (t191 + t177 * gyro[0] / 6.0) / 7.0) + m_t252_tmp * (t180 + t177_tmp / 6.0)
    / 42.0);
  Qd[91] = t604;
  Qd[92] = t634;
  Qd[93] = t668;
  Qd[94] = t702;
  b_t381_tmp = t187_tmp * t197;
  t371_tmp = t204 * t718;
  d_t408_tmp = t210 * t723;
  Qd[95] = ((((dt * ((t1055_tmp + b_t1055_tmp) + c_t1055_tmp) - t3 * ((t162 *
    t179 * gyroVar[2] / 2.0 + t168_tmp * t207 * gyroVar[0] / 2.0) - t166 * t266 *
    gyroVar[1] / 2.0)) - e_t252_tmp * (((((b_t381_tmp * gyroBiasVar[0] / 3.0 +
    t207 * t714 * gyroVar[0] / 3.0) - t371_tmp * gyroBiasVar[1] / 3.0) +
    d_t408_tmp * gyroBiasVar[2] / 3.0) - t705 * t710 * gyroVar[2] / 3.0) + t720 *
    t724 * gyroVar[1] / 3.0)) + b_t408_tmp * (((((d_t1055_tmp / 3.0 + e_t1055_tmp
    / 3.0) + f_t1055_tmp / 3.0) + t1096 * gyroVar[1] / 3.0) + t1097 * gyroVar[0] /
    3.0) + t1099 * gyroVar[2] / 3.0)) + k_t475_tmp * (((((t1101 * gyroBiasVar[1] /
    5.0 + t1102 * gyroBiasVar[2] / 5.0) + t187_tmp * t187_tmp * gyroBiasVar[0] /
    5.0) - gyroVar[1] * (t166 * t194 * 2.0 - t266 * t266) / 5.0) - gyroVar[2] *
              (t162 * t710 * 2.0 - t705 * t705) / 5.0) - gyroVar[0] * (t703 *
              t714 * 2.0 - t207 * t207) / 5.0)) + b_t527_tmp * (((((t1094 *
    gyroBiasVar[1] / 7.0 + t1095 * gyroBiasVar[0] / 7.0) + t1098 * gyroBiasVar[2] /
    7.0) + t710 * t710 * gyroVar[2] / 7.0) + t714 * t714 * gyroVar[0] / 7.0) +
    t720 * t720 * gyroVar[1] / 7.0);
  Qd[96] = t842;
  Qd[97] = t944;
  Qd[98] = t1055;
  Qd[99] = t1117;
  Qd[100] = t1130;
  Qd[101] = t1141;
  Qd[102] = t1143;
  b_t357_tmp = -t2 * t1104 * accBiasVar[1];
  Qd[103] = b_t357_tmp;
  t357_tmp = t2 * t163;
  e_t381_tmp = t357_tmp * accBiasVar[2] * -0.5;
  Qd[104] = e_t381_tmp;
  Qd[105] = 0.0;
  Qd[106] = 0.0;
  Qd[107] = 0.0;
  b_t346_tmp = t215 * gyroVar[2];
  t347_tmp = t58_tmp * gyroBiasVar[1];
  c_t350_tmp = b_t346_tmp * gyro[0];
  t350_tmp = t88 * gyroVar[0];
  n_t475_tmp = b_t346_tmp * gyro[1];
  Qd[108] = (((t748 - t3 * ((t744 + t350_tmp / 4.0) - g_t604_tmp)) + e_t252_tmp *
              (((t172_tmp + t8 * t88 * gyroVar[0] / 6.0) - t347_tmp * gyro[2] /
                72.0) - c_t350_tmp * gyro[2] / 12.0)) + k_t475_tmp * (((t104 +
    n_t475_tmp / 5.0) - t102) - t178)) - b_t527_tmp * ((t175_tmp + t11 * t215 *
    gyroVar[2] / 7.0) + t347_tmp * gyro[0] * gyro[1] / 252.0);
  c_t381_tmp = t350_tmp * gyro[0];
  Qd[109] = (((t87 - k_t475_tmp * (((((t750 + t347_tmp / 30.0) - t25 * t84_tmp *
    gyroVar[1] / 5.0) + c_t350_tmp / 5.0) - t350_tmp * gyro[2] / 5.0) - t217)) -
              e_t252_tmp * ((t74 + c_t381_tmp * gyro[1] / 12.0) + n_t475_tmp *
    gyro[2] / 12.0)) + b_t527_tmp * ((((t749 - t30 * t88 * gyroVar[0] / 7.0) +
    t27 * t58_tmp * gyroBiasVar[1] / 42.0) + t21 * t215 * gyroVar[2] / 7.0) -
              t63_tmp)) - t252_tmp_tmp;
  Qd[110] = (((t3 * (t49 - b_t346_tmp / 4.0) - k_t475_tmp * ((t44 + t350_tmp *
    gyro[1] / 5.0) - t83)) + e_t252_tmp * ((((((t763 + t764) - t53) - t48) +
    t35 * t215 * gyroVar[2] / 6.0) + g_t252_tmp * t58_tmp / 72.0) - c_t381_tmp *
    gyro[2] / 12.0)) - t62) + b_t527_tmp * ((t67 + t39 * t88 * gyroVar[0] / 7.0)
    - t347_tmp * gyro[1] * gyro[2] / 252.0);
  Qd[111] = t796;
  Qd[112] = t819;
  Qd[113] = t842;
  Qd[114] = ((b_t408_tmp * ((t796_tmp / 3.0 + b_t796_tmp / 3.0) + c_t796_tmp /
    3.0) + k_t475_tmp * (((((d_t796_tmp / 20.0 + e_t796_tmp / 20.0) + f_t796_tmp
    / 20.0) + t851 * gyroVar[0] / 5.0) + t850 * gyroVar[2] / 5.0) + t84_tmp *
    t84_tmp * gyroVar[1] / 5.0)) - e_t252_tmp * ((Qd_tmp * gyroVar[1] / 3.0 -
    c_Qd_tmp * gyroVar[2] / 3.0) + t7 * gyroVar[0] / 3.0)) + b_t527_tmp *
    (((((t843 * gyroVar[2] / 7.0 + t844 * gyroVar[1] / 7.0) + t847 * gyroVar[0] /
        7.0) + t845 * gyroBiasVar[2] / 252.0) + t846 * gyroBiasVar[1] / 252.0) +
     t848 * gyroBiasVar[0] / 252.0);
  Qd[115] = t963;
  Qd[116] = t1074;
  Qd_tmp = t3 * t47 * gyroBiasVar[0] * -0.041666666666666664;
  Qd[117] = Qd_tmp;
  c_Qd_tmp = t3 * t409 * gyroBiasVar[1] * -0.041666666666666664;
  Qd[118] = c_Qd_tmp;
  Qd[119] = t1142;
  t7 = t23 * dt * accBiasVar[0] * -0.16666666666666666;
  Qd[120] = t7;
  Qd[121] = t1146;
  b_t346_tmp = t2 * t853 * dt * accBiasVar[2] * -0.33333333333333331;
  Qd[122] = b_t346_tmp;
  Qd[123] = 0.0;
  Qd[124] = 0.0;
  Qd[125] = 0.0;
  t347_tmp = t220 * gyroVar[0];
  t22 = t111_tmp * gyroBiasVar[2];
  c_t350_tmp = t22 * gyro[0];
  t350_tmp = t216_tmp * gyroVar[1];
  Qd[126] = (((t3 * (t572_tmp - t347_tmp / 4.0) + e_t252_tmp * ((((((t860 + t861)
    - b_t572_tmp) - t566_tmp) + t8 * t220 * gyroVar[0] / 6.0) + d_t300_tmp *
    t111_tmp / 72.0) + i_t252_tmp * t216_tmp / 12.0)) - k_t475_tmp * ((t568_tmp
    - t350_tmp * gyro[2] / 5.0) - b_t569_tmp)) - t173_tmp) + b_t527_tmp *
    ((t193 - t14 * t216_tmp * gyroVar[1] / 7.0) - c_t350_tmp * gyro[2] / 252.0);
  n_t475_tmp = t347_tmp * gyro[0];
  c_t381_tmp = gyroVar[1] * t216_tmp;
  Qd[127] = (((t871 + t3 * ((t867 + c_t381_tmp / 4.0) - i_t604_tmp)) +
              k_t475_tmp * (((t60_tmp + t347_tmp * gyro[2] / 5.0) - t59_tmp) -
    t78)) - e_t252_tmp * ((((((t866 - t21 * t146_tmp * gyroVar[2] / 6.0) + t25 *
    t216_tmp * gyroVar[1] / 6.0) + c_t350_tmp / 72.0) - t54_tmp) + n_t475_tmp *
    gyro[1] / 12.0) - t5)) - b_t527_tmp * ((t99 + t30 * t220 * gyroVar[0] / 7.0)
    + t22 * gyro[1] * gyro[2] / 252.0);
  Qd[128] = (((b_t634_tmp - k_t475_tmp * (((((t873 + t22 / 30.0) - t35 *
    t146_tmp * gyroVar[2] / 5.0) + t350_tmp * gyro[0] / 5.0) + t347_tmp * gyro
    [1] / 5.0) - b_t604_tmp)) - e_t252_tmp * ((m_t475_tmp + n_t475_tmp * gyro[2]
    / 12.0) - t350_tmp * gyro[1] * gyro[2] / 12.0)) + b_t527_tmp * ((((t872 +
    t36 * t111_tmp * gyroBiasVar[2] / 42.0) + t34 * t216_tmp * gyroVar[1] / 7.0) +
    t39 * t220 * gyroVar[0] / 7.0) - t439_tmp)) - t147;
  Qd[129] = (((-t2 * t557 - t3 * ((((c_t819_tmp + t890) - g_t300_tmp) - t889_tmp)
    - t159)) + e_t252_tmp * ((((((((t880 + t881) + t70 * t111_tmp * gyroBiasVar[2]
    / 36.0) + t82 * t216_tmp * gyroVar[1] / 6.0) - t158) - t184) + t220 * t411 *
    gyroVar[0] / 6.0) - t856_tmp * t84_tmp / 36.0) - j_t252_tmp * t146_tmp / 6.0))
             + k_t475_tmp * ((b_t668_tmp - t47 * t220 * gyroVar[0] / 5.0) +
              t216_tmp * t409 * gyroVar[1] / 5.0)) + b_t527_tmp * (((t115_tmp +
    c_t381_tmp * t420_tmp / 7.0) + t220 * t419 * gyroVar[0] / 7.0) + t424 *
    gyroBiasVar[2] * t111_tmp / 42.0);
  Qd[130] = t919;
  Qd[131] = t944;
  Qd[132] = t963;
  Qd[133] = ((b_t408_tmp * ((t919_tmp / 3.0 + b_t919_tmp / 3.0) + c_t919_tmp /
    3.0) + k_t475_tmp * (((((d_t919_tmp / 20.0 + e_t919_tmp / 20.0) + f_t919_tmp
    / 20.0) + t970 * gyroVar[0] / 5.0) + t972 * gyroVar[1] / 5.0) + t146_tmp *
    t146_tmp * gyroVar[2] / 5.0)) - e_t252_tmp * ((b_Qd_tmp * gyroVar[2] / 3.0 -
    t41_tmp * gyroVar[0] / 3.0) + t53_tmp * gyroVar[1] / 3.0)) + b_t527_tmp *
    (((((t965 * gyroVar[0] / 7.0 + t964 * gyroVar[2] / 7.0) + t968 * gyroVar[1] /
        7.0) + t966 * gyroBiasVar[0] / 252.0) + t967 * gyroBiasVar[2] / 252.0) +
     t969 * gyroBiasVar[1] / 252.0);
  Qd[134] = t1093;
  Qd[135] = t1118;
  b_Qd_tmp = t3 * t107 * gyroBiasVar[1] * -0.041666666666666664;
  Qd[136] = b_Qd_tmp;
  t41_tmp = t3 * t531 * gyroBiasVar[2] * -0.041666666666666664;
  Qd[137] = t41_tmp;
  t53_tmp = t2 * t973 * dt * accBiasVar[0] * -0.33333333333333331;
  Qd[138] = t53_tmp;
  j_t252_tmp = c_t408_tmp * dt * accBiasVar[1] * -0.16666666666666666;
  Qd[139] = j_t252_tmp;
  Qd[140] = t1148;
  Qd[141] = 0.0;
  Qd[142] = 0.0;
  Qd[143] = 0.0;
  g_t300_tmp = t223 * gyroVar[1];
  c_t408_tmp = t185 * gyroVar[2];
  t347_tmp = c_t408_tmp * gyro[0];
  c_t350_tmp = c_t408_tmp * gyro[1];
  t350_tmp = t168_tmp * gyroBiasVar[0];
  n_t475_tmp = g_t300_tmp * gyro[0];
  Qd[144] = (((b_t564_tmp - k_t475_tmp * (((((t976 + t350_tmp / 30.0) - t8 *
    t187_tmp * gyroVar[0] / 5.0) - c_t350_tmp / 5.0) + g_t300_tmp * gyro[2] /
    5.0) - t668_tmp)) - e_t252_tmp * ((t565_tmp + n_t475_tmp * gyro[1] / 12.0)
    + t347_tmp * gyro[2] / 12.0)) + b_t527_tmp * ((((t975 - t11 * t185 *
    gyroVar[2] / 7.0) + t14 * t223 * gyroVar[1] / 7.0) + t15 * t168_tmp *
    gyroBiasVar[0] / 42.0) - e_t604_tmp)) - f_t604_tmp;
  c_t381_tmp = t350_tmp * gyro[0];
  Qd[145] = (((t3 * (h_t604_tmp - g_t300_tmp / 4.0) - k_t475_tmp * ((t4 +
    t347_tmp / 5.0) - t215_tmp)) + e_t252_tmp * ((((((t989 + t990) - t89) - t6)
    + t25 * t223 * gyroVar[1] / 6.0) + t_t252_tmp * t168_tmp / 72.0) - c_t350_tmp
    * gyro[2] / 12.0)) - h_t252_tmp) + b_t527_tmp * ((t717 + t21 * t185 *
    gyroVar[2] / 7.0) - c_t381_tmp * gyro[1] / 252.0);
  Qd[146] = (((t1002 - t3 * ((t998 + c_t408_tmp / 4.0) - l_t475_tmp)) +
              e_t252_tmp * ((((t569_tmp + t35 * t185 * gyroVar[2] / 6.0) -
    t573_tmp) - t350_tmp * gyro[1] / 72.0) - g_t300_tmp * gyro[1] * gyro[2] /
    12.0)) + k_t475_tmp * (((d_t604_tmp + n_t475_tmp / 5.0) - t564_tmp) -
              b_t47_tmp)) - b_t527_tmp * ((t47_tmp + t34 * t223 * gyroVar[1] /
    7.0) + c_t381_tmp * gyro[2] / 252.0);
  Qd[147] = (((-t3 * ((((d_t842_tmp + t1012) + t1013) + t1014) - t1011_tmp) -
               t1005_tmp) + k_t475_tmp * ((((t144 + t57 * t185 * gyroVar[2] / 5.0)
    - t137) - t223 * t409 * gyroVar[1] / 5.0) - t155)) - e_t252_tmp *
             ((((((((t1003 + t1004) + t82 * t223 * gyroVar[1] / 6.0) - t136_tmp)
                  + t98 * t168_tmp * gyroBiasVar[0] / 36.0) - t116_tmp) - t185 *
                t431 * gyroVar[2] / 6.0) + t204 * t420_tmp * gyroVar[1] / 6.0) -
              t139)) - b_t527_tmp * (((t110_tmp + t185 * t426 * gyroVar[2] / 7.0)
    + t223 * t420_tmp * gyroVar[1] / 7.0) - t168_tmp * t430 * gyroBiasVar[0] /
    42.0);
  Qd[148] = (((-t2 * t734 + t3 * (((((t735 + t1025) + t1026) - t40_tmp) -
    t940_tmp) - t938_tmp)) - k_t475_tmp * (((((t1021 + t107 * t223 * gyroVar[1] /
    5.0) - t109_tmp) + t185 * t531 * gyroVar[2] / 5.0) - n_t252_tmp) - l_t252_tmp))
             - e_t252_tmp * ((((k_t252_tmp + t131 * t185 * gyroVar[2] / 6.0) -
    t135 * t168_tmp * gyroBiasVar[0] / 36.0) - f_t300_tmp) - t223 * t536 *
              gyroVar[1] / 6.0)) + b_t527_tmp * (((e_t300_tmp + t185 * t540 *
    gyroVar[2] / 7.0) + t223 * t543 * gyroVar[1] / 7.0) + t550 * gyroBiasVar[0] *
    t168_tmp / 42.0);
  Qd[149] = t1055;
  Qd[150] = t1074;
  Qd[151] = t1093;
  Qd[152] = ((b_t408_tmp * ((t1055_tmp / 3.0 + b_t1055_tmp / 3.0) + c_t1055_tmp /
    3.0) + k_t475_tmp * (((((d_t1055_tmp / 20.0 + e_t1055_tmp / 20.0) +
    f_t1055_tmp / 20.0) + t1101 * gyroVar[1] / 5.0) + t1102 * gyroVar[2] / 5.0) +
    t187_tmp * t187_tmp * gyroVar[0] / 5.0)) - e_t252_tmp * ((b_t381_tmp *
    gyroVar[0] / 3.0 - t371_tmp * gyroVar[1] / 3.0) + d_t408_tmp * gyroVar[2] / 3.0))
    + b_t527_tmp * (((((t1094 * gyroVar[1] / 7.0 + t1095 * gyroVar[0] / 7.0) +
                       t1098 * gyroVar[2] / 7.0) + t1096 * gyroBiasVar[1] / 252.0)
                     + t1097 * gyroBiasVar[0] / 252.0) + t1099 * gyroBiasVar[2] /
                    252.0);
  t40_tmp = t3 * t703 * gyroBiasVar[0] * -0.041666666666666664;
  Qd[153] = t40_tmp;
  Qd[154] = t1131;
  t109_tmp = t3 * t162 * gyroBiasVar[2] * -0.041666666666666664;
  Qd[155] = t109_tmp;
  Qd[156] = t1144;
  n_t252_tmp = t2 * t1104 * dt * accBiasVar[1] * -0.33333333333333331;
  Qd[157] = n_t252_tmp;
  l_t252_tmp = t357_tmp * dt * accBiasVar[2] * -0.16666666666666666;
  Qd[158] = l_t252_tmp;
  Qd[159] = 0.0;
  Qd[160] = 0.0;
  Qd[161] = 0.0;
  Qd[162] = t1106;
  Qd[163] = t1108;
  Qd[164] = t574_tmp;
  Qd[165] = t1112;
  Qd[166] = t1115;
  Qd[167] = t1117;
  Qd[168] = Qd_tmp;
  Qd[169] = t1118;
  Qd[170] = t40_tmp;
  Qd[171] = dt * gyroBiasVar[0];
  memset(&Qd[172], 0, sizeof(double) << 3);
  Qd[180] = t42;
  Qd[181] = t1121;
  Qd[182] = t1123;
  Qd[183] = t1125;
  Qd[184] = t1127;
  Qd[185] = t1130;
  Qd[186] = c_Qd_tmp;
  Qd[187] = b_Qd_tmp;
  Qd[188] = t1131;
  Qd[189] = 0.0;
  Qd[190] = dt * gyroBiasVar[1];
  Qd[191] = 0.0;
  Qd[192] = 0.0;
  Qd[193] = 0.0;
  Qd[194] = 0.0;
  Qd[195] = 0.0;
  Qd[196] = 0.0;
  Qd[197] = 0.0;
  Qd[198] = t2 * dt * gyroBiasVar[2] * (t341 - dt * gyro[0] * gyro[2]) /
    24.0;
  Qd[199] = t181;
  Qd[200] = t1134;
  Qd[201] = t1137;
  Qd[202] = t1139;
  Qd[203] = t1141;
  Qd[204] = t1142;
  Qd[205] = t41_tmp;
  Qd[206] = t109_tmp;
  Qd[207] = 0.0;
  Qd[208] = 0.0;
  Qd[209] = dt * gyroBiasVar[2];
  memset(&Qd[210], 0, 9U * sizeof(double));
  Qd[219] = t17;
  Qd[220] = d_t381_tmp;
  Qd[221] = t1143;
  Qd[222] = t7;
  Qd[223] = t53_tmp;
  Qd[224] = t1144;
  Qd[225] = 0.0;
  Qd[226] = 0.0;
  Qd[227] = 0.0;
  Qd[228] = dt * accBiasVar[0];
  memset(&Qd[229], 0, sizeof(double) << 3);
  Qd[237] = t2 * accBiasVar[1] * (t44_tmp - quat[1] * quat[2]);
  Qd[238] = t370_tmp;
  Qd[239] = b_t357_tmp;
  Qd[240] = t1146;
  Qd[241] = j_t252_tmp;
  Qd[242] = n_t252_tmp;
  Qd[243] = 0.0;
  Qd[244] = 0.0;
  Qd[245] = 0.0;
  Qd[246] = 0.0;
  Qd[247] = dt * accBiasVar[1];
  Qd[248] = 0.0;
  Qd[249] = 0.0;
  Qd[250] = 0.0;
  Qd[251] = 0.0;
  Qd[252] = 0.0;
  Qd[253] = 0.0;
  Qd[254] = 0.0;
  Qd[255] = t24;
  Qd[256] = t2 * accBiasVar[2] * (t103_tmp - quat[2] * quat[3]);
  Qd[257] = e_t381_tmp;
  Qd[258] = b_t346_tmp;
  Qd[259] = t1148;
  Qd[260] = l_t252_tmp;
  Qd[261] = 0.0;
  Qd[262] = 0.0;
  Qd[263] = 0.0;
  Qd[264] = 0.0;
  Qd[265] = 0.0;
  Qd[266] = dt * accBiasVar[2];
  memset(&Qd[267], 0, 18U * sizeof(double));
  Qd[285] = dt * gpsBiasVar[0];
  memset(&Qd[286], 0, 18U * sizeof(double));
  Qd[304] = dt * gpsBiasVar[1];
  memset(&Qd[305], 0, 18U * sizeof(double));
  Qd[323] = dt * gpsBiasVar[2];
}