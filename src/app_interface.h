/**
 * @file app_interface.h
 * @brief Application Configuration Parameters and Macros for PPOI_Nav System
 *
 * This header file defines the main application configuration parameters, macro definitions, and structure types
 * for the PPOI_Nav navigation system. It includes system operation modes, filtering parameters, observation controls,
 * high-precision GNSS integration models, file paths, and other commonly used configuration options.
 * These parameters provide a unified interface and foundational support for modules such as navigation computation,
 * integrated positioning, observation data processing, and system modeling.
 *
 * @author  LEADOR PPOI Team - Yejin
 * @date    2024-09-04
 * @version 1.0
 * @copyright Copyright (c) 2020-2025 PPOI_Nav Project
 *                          PSINS: A C++ Inertial Navigation Library, https://psins.org.cn/
 *
 * @mainpage PPOI_Nav Application Configuration Parameters
 * @section intro_sec Introduction
 * This file provides the essential application-level configuration parameters, macro definitions, and structure types
 * for the PPOI_Nav navigation and integrated positioning system. It is suitable for navigation computation,
 * integrated positioning, observation data processing, and other related scenarios, offering unified configuration
 * management and system parameter support.
 *
 * @section macro_sec Macros & Parameters Overview
 * - System modes and observation controls: CAL_TYPE, NAV_TYPE, MASK_MEAS, OD_ENABLE, etc.
 * - Filtering and algorithm parameters: N_STEP, YAW_ALIGN, YAW_STD, FB_att_TAU, Qt_gyr_h, etc.
 * - Model and feature configuration: IPOS_SER, IPOSDY_OLD, LOST_ASUME, GNSS_CHECK_LG69T, etc.
 * - State covariance and process noise: Pk_*, Qt_*, FB_*, etc.
 * - Structure definitions: sim32bin_t (simulation/observation data), config_t (configuration parameters), CONFIG_t (generic configuration item)
 *
 * @section history_sec History
 * - 2024-09-04: Initial version, defined main parameters and structures.
 * - 2024-09-10: Added model configuration options and improved comments.
 *
 * @section reference_sec References
 * - [1] PPOI_Nav Project Documentation and Design Specifications
 * - [2] PSINS: A C++ Inertial Navigation Library, https://psins.org.cn/
 * - [3] RTKLIB: An Open Source Program Package for GNSS Positioning, http://www.rtklib.com/
 *
 * @section usage_sec Usage
 * Include this header file directly to access all essential application-level configuration parameters, macro definitions,
 * and structure types for the PPOI_Nav system. It provides parameter support for navigation computation, integrated positioning,
 * and related system modules.
 *
 * @defgroup app_config Application Configuration Parameters
 * @defgroup app_struct Application Structure Types
 */

#ifndef MYNAV_APPCONFIGPARA_H
#define MYNAV_APPCONFIGPARA_H
#include <stdint.h>

//#define DSPRELTIME
#ifndef  DSPRELTIME
	#define  TRACE
	#define  FILEIO
	#define  CONFIGSIM
#else
	#include "bsw.h"
#endif

#define SYS_INITING           0x01
#define SYS_READY             0x02
#define SYS_GNSS_TIME_READY   0x03
#define SYS_PPS_TIMING        0x04
#define SYS_PPS_READY         0x05
#define SYS_IMU_READY         0x06
#define SYS_GNSS_POS_READY    0x07
#define SYS_ALG_ALIGNING      0x08
#define SYS_ALG_NAVIGATION    0x09

/* IMU framing constants ------------------------------------------------------
* Definitions for IMU protocol framing used by the parser state machine.
* - `IMU_HEAD_44`: start-of-frame marker
* - `IMU_BACK_44`: end-of-frame marker
* - `IMU_FRAME_LENGTH`: expected frame byte length
*-------------------------------------------------------------------------------*/
#define IMU_HEAD_44 0XAA
#define IMU_BACK_44 0XAC
#define IMU_FRAME_LENGTH 44

//#define IMU_HEAD0_40 0x55
//#define IMU_HEAD1_40 0xAA
//#define IMU_FRAME_LENGTH 40

#define GNSS_MAX_NUM     128
 /* GNSS sentence parsing state definitions ---------------------------------------
  * Defines bitmask values for different GNSS sentence parsing states.
  *-------------------------------------------------------------------------------*/
#define GNSSPARSE_GGA       0001   /**< State: Parsing GGA sentence  */
#define GNSSPARSE_VTG       0002   /**< State: Parsing VTG sentence vel */
#define GNSSPARSE_VEL       0002   /**< State: Parsing VEL sentence vel */
#define GNSSPARSE_SBF4007   0002   /**< State: Parsing SBF4007 sentence vel */
#define GNSSPARSE_PVT       0003   /**< State: Parsing PVT sentence vel */
#define GNSSPARSE_REL       0004   /**< State: Parsing REL sentence heading & rms*/
#define GNSSPARSE_TAR       0004   /**< State: Parsing TAR sentence heading & rms*/
#define GNSSPARSE_PSSN      0004   /**< State: Parsing PSSN sentence heading & rms*/
#define GNSSPARSE_HEA       0004   /**< State: Parsing HEADING sentence heading */
#define GNSSPARSE_ZDA       0010   /**< State: Parsing ZDA sentence date */
#define GNSSPARSE_GST       0020   /**< State: Parsing GST sentence rms_pos */

#define GNSSPARSE_PARSE_NOW 0037

#define CHECK_GN_GP(ch_0,ch1)    ((ch_0 == 'G') && ((ch1 == 'P') || ch1 == 'N'))
#define CHECK_UM_BEST(ch_0,ch1)  ((ch_0 == 'B') && (ch1 == 'E'))
#define CHECK_UM_HEA(ch_0,ch1)   ((ch_0 == 'H') && (ch1 == 'E'))
#define CHECK_YY_PQ(ch_0,ch1)    ((ch_0 == 'P') && (ch1 == 'Q'))
#define CHECK_UBLOX_HEAD(ch_0,ch1)((ch_0 == 0xb5) && (ch1 == 0x62))
#define CHECK_UBLOX_PVT(ch_0,ch1) ((ch_0 == 0x01) && (ch1 == 0x07))
#define CHECK_UBLOX_REL(ch_0,ch1) ((ch_0 == 0x01) && (ch1 == 0x3c))
#define CHECK_G5_SBF(ch_0,ch1)    ((ch_0 == 0x24) && (ch1 == 0x40)) /* == $@*/
#define CHECK_G5_PSSN(ch_0,ch1)   ((ch_0 == 'P') && (ch1 == 'S'))


#define VARMAX          5        /**< Maximum number of data points. */

#define MAX_MAT_DIM     19                        /*the max dim of vect*/
#define MAX_MAT_DIM_2   MAX_MAT_DIM*MAX_MAT_DIM   /*the max dim of matrix*/

#define ALIGN_NONE 0
#define ALIGN_ANT  1    /* using the ant for coast align*/
#define ALIGN_TRJ  2    /* using the trj for coast align*/
#define ALIGN_DONE 3

#define MAXLEAPS    64                  /* max number of leap seconds table */
#define RE_WGS84    6378137.0           /* earth semimajor axis (WGS84) (m) */
#define FE_WGS84    (1.0/298.257223563) /* earth flattening (WGS84) */

#define PI      3.1415926535897932      /* pi */
#define PI_2    (PI/2.0)                /**< PI divided by 2 */
#define PI_4    (PI/4.0)                /**< PI divided by 4 */
#define _2PI    (2.0*PI)                /**< Twice the value of PI */
#define DEG     (PI/180.0)              /**< Conversion factor from degrees to radians */
#define MIN     (DEG/60.0)              /**< Conversion factor from minutes to radians */
#define SEC     (MIN/60.0)              /**< Conversion factor from seconds to radians */
#define HUR     3600.0                  /**< Number of seconds in an hour (hur) */
#define SHUR    60.0                    /**< Square root of the number of seconds in an hour */
#define DPS     (DEG/1.0)               /**< Conversion factor from degrees per second to radians per second */
#define DPH     (DEG/HUR)               /**< Conversion factor from degrees per hour to radians per second */
#define DPSH    (DEG/SHUR)              /**< Conversion factor from degrees per square root of hour to radians per second */
#define G0      9.7803267714            /**< Standard acceleration due to gravity (G0) */
#define MG      (G0/1.0e3)              /**< Conversion factor from meters per second squared to milli-g */
#define UG      (G0/1.0e6)              /**< Conversion factor from meters per second squared to micro-g */
#define UGPSHZ  (UG/1)                  /**< Conversion factor from micro-g to micro-g per square root of hertz */
#define RE      6378137.0               /**< Earth's equatorial radius (RE) */
#define PPM     1.0e-6                  /**< Parts per million (PPM) */


#define f0_earth (1.0/298.257)           /**< Flattening factor (f) */
#define wie0     7.2921151467e-5

#define EPS		2.220446049e-16     /* the min value in this space*/
#define INF		3.402823466e+30     /* INF */
#define INFp5	INF*0.5             /* INF*0.5 */

#define assert(b)  {};                            /*safe action*/
#define swapt(a,b,tmp) {tmp c = a; a = b;b = c;}  /*swapt the a and b*/
#define asinEx(x)		asin(range(x, -1.0, 1.0)) /*get value the sin of x*/
#define acosEx(x)		acos(range(x, -1.0, 1.0)) /*get value the cos of x*/
#define min(x,y)        ( (x)<=(y)?(x):(y) )

#define velMax  400.0
#define hgtMin  -RE * 0.01
#define hgtMax  RE * 0.01
#define latMax  85.0 * DEG

#define fXYZU(X,Y,Z,U)	1.0*(X)*(U),1.0*(Y)*(U),1.0*(Z)*(U)
#define fXXZU(X,Z,U)	fXYZU(X,X,Z,U)
#define fXYZ(X,Y,Z)		fXYZU(X,Y,Z,1.0)
#define fXXZ(X,Z)		fXYZ(X,X,Z)
#define fXXX(X)			fXYZ(X,X,X)

#define fdLLH(LL,H)		fXXZ((LL)/RE,(H))
#define fdPOS(LLH)		fdLLH(LLH,LLH)

#define CC180C360(yaw)  ( (yaw)>0.0 ? (_2PI-(yaw)) : -(yaw) )   // counter-clockwise +-180deg -> clockwise 0~360deg for yaw
#define C360CC180(yaw)  ( (yaw)>=PI ? (_2PI-(yaw)) : -(yaw) )   // clockwise 0~360deg -> counter-clockwise +-180deg for yaw
#define pow2(x)			((x)*(x))
#define Ant_Mode_One       1
#define Ant_Mode_LR_L      2
#define Ant_Mode_LR_R      3
#define Ant_Mode_FB_F      4
#define Ant_Mode_FB_B      5
#define Ant_Mode_Ang       6
#define Out_Point_IMU      1
#define Out_Point_GNSS     2
#define Out_Point_Other    3

#define SECONDS_PER_WEEK 604800
#define MAX_TIME_DIFF    0.001


#define CAL_TYPE   0x0        /*< Calibration mode identifier */
#define NAV_TYPE   0x1        /*< Navigation mode identifier  */

/*  zupt od pos vel */
#define MASK_MEAS  0357077    /*< Enable all measurements; use detailed macros for control */
#define OD_ENABLE  0x00       /*< Disable odometer and virtual odometer */

#define N_STEP     6          /*< Step size for filter or algorithm (e.g., (2*(nq+nr) +3) / 20 = 3.8 < N) */
#define YAW_ALIGN  5.0        /*< Yaw alignment RMS in degrees */
#define YAW_ALIGN_INIT  5.0   /*< Yaw alignment RMS in degrees in init process */
#define YAW_STD    5          /*< Yaw standard deviation in degrees */

#define IPOS_SER              /*< Use IPOS3 100Hz serial mode */
#ifdef IPOS_SER
#define FS          100 
#define TS          1.0/100    /*< Time step for IPOS3 serial (100Hz) */
#define DT_POS     -0.00       /*< Position time delay compensation */
#define DT_VEL     -0.00       /*< Velocity time delay compensation */
#define DT_YAW     -0.00       /*< Yaw time delay compensation */
#define DT_GNSS    -0.12       /*< GNSS time delay compensation */
#endif
#define AVPINUM       12+1

 /* para will be */
#define FB_att_TAU     TS*10.0     /*< Attitude feedback time constant */
#define FB_vel_TAU     TS*10.0     /*< Velocity feedback time constant */
#define FB_pos_TAU     TS*10.0     /*< Position feedback time constant */
#define FB_eb_TAU      TS*10.0     /*< Gyro bias feedback time constant */
#define FB_db_TAU      TS*10.0     /*< Accel bias feedback time constant */
#define FB_od_TAU      TS*10.0     /*< Odometer feedback time constant */
#define FB_gnss_yaw_TAU TS*1.0    /*< GNSS yaw feedback time constant */

//#define _3
#ifdef _3
	#define Qt_gyr_h        0.3*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
	#define Qt_gyr_v        0.9*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */
	#define Pk_eb_h        10.0*DPH    /*< Horizontal gyro bias initial covariance */
	#define Pk_eb_v        30.0*DPH    /*< Vertical gyro bias initial covariance */
	#define Qt_acc_h     1200.0*UGPSHZ /*< Horizontal accel noise (ug/sqrt(Hz)) */
	#define Qt_acc_v     1500.0*UGPSHZ /*< Vertical accel noise (ug/sqrt(Hz)) */
	#define Pk_db_h        1.0*MG      /*< Horizontal accel bias initial covariance */
	#define Pk_db_v        3.0*MG      /*< Vertical accel bias initial covariance */
	#define Rt_yaw_gnss    0.5*DEG     /*< GNSS yaw measurement noise */
	#define RB_Init         0.00       /*< Infinite value for constraints */
	#define RB_Norm         0.50       /*< Infinite value for constraints */
	#define THR_ZIHR        0.23 * DPS  /*< ZIHR threshold to detect zero angular rate */
#endif

//#define CX
#ifdef CX
	#define Qt_gyr_h        0.5*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
	#define Qt_gyr_v        0.8*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */
	#define Pk_eb_h        15.0*DPH    /*< Horizontal gyro bias initial covariance */
	#define Pk_eb_v        18.0*DPH    /*< Vertical gyro bias initial covariance */
	#define Qt_acc_h     1000.0*UGPSHZ /*< Horizontal accel noise (ug/sqrt(Hz)) */
	#define Qt_acc_v     1200.0*UGPSHZ /*< Vertical accel noise (ug/sqrt(Hz)) */
	#define Pk_db_h        1.0*MG       /*< Horizontal accel bias initial covariance */
	#define Pk_db_v        3.0*MG       /*< Vertical accel bias initial covariance */
	#define Rt_yaw_gnss   10.0*DEG      /*< GNSS yaw measurement noise */
	#define RB_Init         0.00        /*< Infinite value for constraints */
	#define RB_Norm         0.50        /*< Infinite value for constraints */
	#define THR_ZIHR        0.18 * DPS  /*< ZIHR threshold to detect zero angular rate */
#endif

#define R_3
#ifdef R_3
	#define Qt_gyr_h        0.5*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
	#define Qt_gyr_v        0.8*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */
	#define Pk_eb_h        10.0*DPH    /*< Horizontal gyro bias initial covariance */
	#define Pk_eb_v        12.0*DPH    /*< Vertical gyro bias initial covariance */
	#define Qt_acc_h     1000.0*UGPSHZ /*< Horizontal accel noise (ug/sqrt(Hz)) */
	#define Qt_acc_v     1200.0*UGPSHZ /*< Vertical accel noise (ug/sqrt(Hz)) */
	#define Pk_db_h        1.0*MG      /*< Horizontal accel bias initial covariance */
	#define Pk_db_v        3.0*MG      /*< Vertical accel bias initial covariance */
	#define Rt_yaw_gnss   10.0*DEG     /*< GNSS yaw measurement noise */
	#define RB_Init         0.00       /*< Infinite value for constraints */
	#define RB_Norm         0.50       /*< Infinite value for constraints */
	#define THR_ZIHR        0.20 * DPS  /*< ZIHR threshold to detect zero angular rate */
#endif

//#define HSX
#ifdef HSX
#define Qt_gyr_h        0.8*DPSH   /*< Horizontal gyro noise (deg/sqrt(hr)) */
#define Qt_gyr_v        1.0*DPSH   /*< Vertical gyro noise (deg/sqrt(hr)) */
#define Pk_eb_h        15.0*DPH    /*< Horizontal gyro bias initial covariance */
#define Pk_eb_v        18.0*DPH    /*< Vertical gyro bias initial covariance */
#define Qt_acc_h     1000.0*UGPSHZ /*< Horizontal accel noise (ug/sqrt(Hz)) */
#define Qt_acc_v     1200.0*UGPSHZ /*< Vertical accel noise (ug/sqrt(Hz)) */
#define Pk_db_h        1.0*MG       /*< Horizontal accel bias initial covariance */
#define Pk_db_v        3.0*MG       /*< Vertical accel bias initial covariance */
#define Rt_yaw_gnss   10.0*DEG      /*< GNSS yaw measurement noise */
#define RB_Init         0.50        /*< Infinite value for constraints */
#define RB_Norm         0.50        /*< Infinite value for constraints */
#define THR_ZIHR        0.15 * DPS  /*< ZIHR threshold to detect zero angular rate */
#endif




//#define LOW_PASS_IIR        /*< Enable 10Hz IIR low-pass filter */
//#define LOST_ASUME          /*< Use speed & heading assumption when lost */
//#define GNSS_CHECK_UBOX       /*< Enable GNSS check for UBLOX receiver */
//#define GNSS_CHECK_UM482    /*< Enable GNSS check for UM482 receiver */
#define GNSS_CHECK_UM982    /*< Enable GNSS check for UM982 receiver */
//#define GNSS_CHECK_LG69T      /*< Enable GNSS check for LG69T receiver */

#define NAV_PARSE
/*******************************************************************************/
#define Pk_att_h       150*MIN     /*< Horizontal attitude initial covariance */
#define Pk_att_v       300*MIN     /*< Vertical attitude initial covariance */
#define Pk_vel_h       1.0         /*< Horizontal velocity initial covariance */
#define Pk_vel_v       3.0         /*< Vertical velocity initial covariance */
#define Pk_pos_h       1.0/RE      /*< Horizontal position initial covariance */
#define Pk_pos_v       3.0         /*< Vertical position initial covariance */

#define Pk_att_h_min   1e-3*DEG    /*< Minimum horizontal attitude covariance */
#define Pk_att_v_min   1e-3*DEG    /*< Minimum vertical attitude covariance */
#define Pk_vel_h_min   1e-3        /*< Minimum horizontal velocity covariance */
#define Pk_vel_v_min   1e-3        /*< Minimum vertical velocity covariance */
#define Pk_pos_h_min   1e-3/RE     /*< Minimum horizontal position covariance */
#define Pk_pos_v_min   1e-3        /*< Minimum vertical position covariance */
#define Pk_eb_h_min    1e-2*DPH    /*< Minimum horizontal gyro bias covariance */
#define Pk_eb_v_min    1e-2*DPH    /*< Minimum vertical gyro bias covariance */
#define Pk_db_h_min    1e-2*MG     /*< Minimum horizontal accel bias covariance */
#define Pk_db_v_min    1e-2*MG     /*< Minimum vertical accel bias covariance */

/** Rt in GNSS */
#define Rt_Vn_gnss_h     0.5       /*< GNSS horizontal velocity measurement noise */
#define Rt_Vn_gnss_v     0.8       /*< GNSS vertical velocity measurement noise */
#define Rt_Pos_gnss_h    0.5       /*< GNSS horizontal position measurement noise */
#define Rt_Pos_gnss_v    0.8       /*< GNSS vertical position measurement noise */

#define Rt_vn_od_h       0.5       /*< Odometer horizontal velocity measurement noise */
#define Rt_vn_od_v       0.8       /*< Odometer vertical velocity measurement noise */
#define Rt_zupt          1.0       /*< ZUPT measurement noise */
#define Rt_nhc           1.0       /*< NHC measurement noise */
#define Rt_zihr          1.0*DEG   /*< ZIHR measurement noise */
#define Rt_yaw_static    0.1*DEG   

#define Qt_pos     0           /*< Process noise for position state */
#define Qt_eb      0           /*< Process noise for gyro bias state */
#define Qt_db      0           /*< Process noise for accel bias state */
#define Qt_Other   0           /*< Process noise for other states */

#define Pk_od_pitch 1.0*DEG    /*< Odometer pitch initial covariance */
#define Pk_od_scale 0.1        /*< Odometer scale initial covariance */
#define Pk_od_yaw   1.0*DEG    /*< Odometer yaw initial covariance */
#define Pk_gnss_yaw 1.0*DEG    /*< GNSS yaw initial covariance */

#define Pk_od_pitch_min 1e-3*DEG   /*< Minimum odometer pitch covariance */
#define Pk_od_scale_min 1e-4       /*< Minimum odometer scale covariance */
#define Pk_od_yaw_min   1e-3*DEG   /*< Minimum odometer yaw covariance */
#define Pk_gnss_yaw_min 1e-3*DEG   /*< Minimum GNSS yaw covariance */

#define Pk_att_h_max    20*DEG    /*< Maximum horizontal attitude covariance */
#define Pk_att_v_max    20*DEG    /*< Maximum vertical attitude covariance */
#define Pk_vel_h_max    5.0       /*< Maximum horizontal velocity covariance */
#define Pk_vel_v_max    5.0       /*< Maximum vertical velocity covariance */
#define Pk_pos_h_max    100.0/RE  /*< Maximum horizontal position covariance */
#define Pk_pos_v_max    100.0     /*< Maximum vertical position covariance */
#define Pk_eb_h_max     360*DPH   /*< Maximum horizontal gyro bias covariance */
#define Pk_eb_v_max     360*DPH   /*< Maximum vertical gyro bias covariance */
#define Pk_db_h_max     300.0*MG  /*< Maximum horizontal accel bias covariance */
#define Pk_db_v_max     300.0*MG  /*< Maximum vertical accel bias covariance */
#define Pk_od_pitch_max 10.0*DEG   /*< Maximum odometer pitch covariance */
#define Pk_od_scale_max 0.3        /*< Maximum odometer scale covariance */
#define Pk_od_yaw_max   10.0*DEG   /*< Maximum odometer yaw covariance */
#define Pk_gnss_yaw_max 10.0*DEG   /*< Maximum GNSS yaw covariance */

#define Xk_att_h_max    INF        /*< Maximum horizontal attitude state value */
#define Xk_att_v_max    INF        /*< Maximum vertical attitude state value */
#define Xk_vel_h_max    INF        /*< Maximum horizontal velocity state value */
#define Xk_vel_v_max    INF        /*< Maximum vertical velocity state value */
#define Xk_pos_h_max    INF        /*< Maximum horizontal position state value */
#define Xk_pos_v_max    INF        /*< Maximum vertical position state value */
#define Xk_eb_h_max     600.0*DPH  /*< Maximum horizontal gyro bias state value */
#define Xk_eb_v_max     100.0*DPH  /*< Maximum vertical gyro bias state value */
#define Xk_db_h_max     100.0*MG   /*< Maximum horizontal accel bias state value */
#define Xk_db_v_max     100.0*MG   /*< Maximum vertical accel bias state value */
#define Xk_od_pitch_max 90.0*DEG   /*< Maximum odometer pitch state value */
#define Xk_od_scale_max 0.5        /*< Maximum odometer scale state value */
#define Xk_od_yaw_max   90.0*DEG   /*< Maximum odometer yaw state value */
#define Xk_gnss_yaw_max 90.0*DEG   /*< Maximum GNSS yaw state value */

#define FB_att_h   30*MIN      /*< Horizontal attitude feedback gain */
#define FB_att_v   60*MIN      /*< Vertical attitude feedback gain */
#define FB_vel_h   1.0         /*< Horizontal velocity feedback gain */
#define FB_vel_v   3.0         /*< Vertical velocity feedback gain */
#define FB_pos_h   10.0/RE     /*< Horizontal position feedback gain */
#define FB_pos_v   30.0        /*< Vertical position feedback gain */
#define FB_eb_h    1.0*DPH     /*< Horizontal gyro bias feedback gain */
#define FB_eb_v    3.0*DPH     /*< Vertical gyro bias feedback gain */
#define FB_db_h    10*MG       /*< Horizontal accel bias feedback gain */
#define FB_db_v    10*MG       /*< Vertical accel bias feedback gain */
#define FB_od_pitch 5e-1*DEG   /*< Odometer pitch feedback gain */
#define FB_od_scale 1e-1       /*< Odometer scale feedback gain */
#define FB_od_yaw   5e-1*DEG   /*< Odometer yaw feedback gain */
#define FB_gnss_yaw 5e-1*DEG   /*< GNSS yaw feedback gain */

#ifndef DSPRELTIME
/*-----**************************************************************************--*/
/* Simulation data structure ---------------------------------------------------
 * Stores simulation data including IMU measurements, GNSS data, odometry,
 * and base station information for navigation simulation.
 * Members:
 *  double t              - Timestamp
 *  double wm_x           - Angular velocity X (rad/s)
 *  double wm_y           - Angular velocity Y (rad/s)
 *  double wm_z           - Angular velocity Z (rad/s)
 *  double vm_x           - Linear acceleration X (m/s^2)
 *  double vm_y           - Linear acceleration Y (m/s^2)
 *  double vm_z           - Linear acceleration Z (m/s^2)
 *  double dS             - Odometry increment
 *  double lat_gps        - GPS latitude (rad)
 *  double lon_gps        - GPS longitude (rad)
 *  double hgt_gps        - GPS height (m)
 *  double dop            - Dilution of precision
 *  double eastvel_gps    - GPS east velocity (m/s)
 *  double northvel_gps   - GPS north velocity (m/s)
 *  double upvel_gps      - GPS up velocity (m/s)
 *  double satnum         - Number of satellites
 *  double None           - Unused field
 *  double yaw            - Yaw angle (rad)
 *  double yawrms         - Yaw RMS (rad)
 *  double lat_base       - Base station latitude (rad)
 *  double lon_base       - Base station longitude (rad)
 *  double hgt_base       - Base station height (m)
 *  double pitch_base     - Base station pitch (rad)
 *  double roll_base      - Base station roll (rad)
 *  double yaw_base       - Base station yaw (rad)
 *  double eastvel_base   - Base station east velocity (m/s)
 *  double northvel_base  - Base station north velocity (m/s)
 *  double uphvel_base    - Base station up velocity (m/s)
 *  double hdop           - GNSS HDOP
 *  double std_lat        - Latitude standard deviation (rad)
 *  double std_lon        - Longitude standard deviation (rad)
 *  double std_hgt        - Height standard deviation (m)
 *-------------------------------------------------------------------------------*/
typedef struct {
	double t;              /*< Timestamp */
	double wm_x;           /*< Angular velocity X (rad/s) */
	double wm_y;           /*< Angular velocity Y (rad/s) */
	double wm_z;           /*< Angular velocity Z (rad/s) */
	double vm_x;           /*< Linear acceleration X (m/s^2) */
	double vm_y;           /*< Linear acceleration Y (m/s^2) */
	double vm_z;           /*< Linear acceleration Z (m/s^2) */           
	double dS;             /*< Odometry increment */
	double lat_gps;        /*< GPS latitude (rad) */
	double lon_gps;        /*< GPS longitude (rad) */
	double hgt_gps;        /*< GPS height (m) */
	double dop;            /*< Dilution of precision */
	double eastvel_gps;    /*< GPS east velocity (m/s) */
	double northvel_gps;   /*< GPS north velocity (m/s) */
	double upvel_gps;      /*< GPS up velocity (m/s) */      
	double satnum;         /*< Number of satellites */
	double None;           /*< Unused field */
	double yaw;            /*< Yaw angle (rad) */
	double yawrms;         /*< Yaw RMS (rad) */
	double lat_base;       /*< Base station latitude (rad) */
	double lon_base;       /*< Base station longitude (rad) */
	double hgt_base;       /*< Base station height (m) */
	double pitch_base;     /*< Base station pitch (rad) */
	double roll_base;      /*< Base station roll (rad) */
	double yaw_base;       /*< Base station yaw (rad) */      
	double eastvel_base;   /*< Base station east velocity (m/s) */
	double northvel_base;  /*< Base station north velocity (m/s) */
	double uphvel_base;    /*< Base station up velocity (m/s) */
	double hdop;           /*< GNSS HDOP? */
	double std_lat;        /*< Latitude standard deviation (rad) */
	double std_lon;        /*< Longitude standard deviation (rad) */
	double std_hgt;        /*< Height standard deviation (m) */
} sim32bin_t;

/* offline CONFIG */
/*-----**************************************************************************--*/
/* Configuration option structure -----------------------------------------------
 * Defines a configuration option entry for system parameters.
 * Members:
 *  const char* name    - Option name
 *  int format          - Option format (0:int,1:double,2:string,3:enum)
 *  void* var           - Pointer to option variable
 *  const char* comment - Option comment/enum labels/unit
 *-------------------------------------------------------------------------------*/
typedef struct {         /* option type */
	const char* name;    /* option name */
	int format;          /* option format (0:int,1:double,2:string,3:enum) */
	void* var;           /* pointer to option variable */
	const char* comment; /* option comment/enum labels/unit */
} CONFIG_t;


#endif


/*-----**************************************************************************--*/
/* GNSS week and seconds structure ----------------------------------------------
 * Stores GPS week number and seconds of week.
 * Members:
 *  uint16_t ui_week - GPS week number
 *  double   lf_secs - Seconds of week
 *-------------------------------------------------------------------------------*/
typedef struct
{
	uint16_t  ui_week;
	double    lf_secs;
}gtime_t;

/* Calendar date and time structure ---------------------------------------------
 * Stores calendar date and time information.
 * Members:
 *  uint16_t year  - Year
 *  uint8_t  month - Month [1~12]
 *  uint8_t  day   - Day [1~31]
 *  uint8_t  hour  - Hour [0~24]
 *  uint8_t  min   - Minute [0~60]
 *  uint16_t sec   - Second [0~60]
 *-------------------------------------------------------------------------------*/
typedef struct {
	uint16_t year;     /*< year */
	uint32_t  month:8;    /*< month {1~12} */
	uint32_t  day:8;      /*< day {1~31} */
	uint32_t  hour:8;     /*< hour {0~24} */
	uint32_t  min:8;      /*< min {0~60} */
	uint16_t  sec;     /*< sec {0~60} ms */
}calender_t;

/* GNSS information structure --------------------------------------------------
 * Stores parsed GNSS data including position, velocity, accuracy, and status.
 * Members:
 *  Week_Sec_t week_secs      - GPS week and seconds
 *  Calender_t date           - Calendar date and time
 *  int32_t    lon            - Longitude [rad]
 *  int32_t    lat            - Latitude [rad]
 *  int32_t    height_ellipsoid - Height above ellipsoid [m]
 *  uint8_t    num_sv         - Number of satellites [0~52]
 *  uint8_t    fix_type       - Fix type [0,1,2,3,4,5]
 *  uint16_t   pos_dop        - Dilution of Precision
 *  uint16_t   age            - Age of data
 *  int32_t    vel_north      - North velocity [m/s]
 *  int32_t    vel_east       - East velocity [m/s]
 *  int32_t    vel_up         - Up velocity [m/s]
 *  int32_t    heading_motion - Heading of trajectory [rad]
 *  uint32_t   accu_lon       - Longitude accuracy [m]
 *  uint32_t   accu_lat       - Latitude accuracy [m]
 *  uint32_t   accu_height    - Height accuracy [m]
 *  int32_t    accu_heading   - Heading accuracy [deg]
 *  int32_t    heading        - Heading [rad]
 *  int32_t    baseline       - Baseline length [m]
 *  int32_t    GpsValFlag     - GPS validity flag
 *  int16_t    flag_time      - Time flag
 *-------------------------------------------------------------------------------*/
typedef struct {
	gtime_t week_secs;
	calender_t date;

	double    lon;                   /*< longitude {rad} */
	double    lat;                   /*< latitude  {rad} */
	float     height_ellipsoid;      /*< height    {m} */
	uint16_t  num_sv;                /*< num of sat {0~52}*/
	float     pos_dop;               /*< Dilution of Precision */
	uint16_t  age;
	float     vel_north;      /*< Velocity component in the north direction in meters per second.{m/s}*/
	float     vel_east;       /*< Velocity component in the east direction in meters per second.{m/s}*/
    float     vel_up;         /*< Velocity component in the up direction in meters per second.{m/s}*/
	float     heading_motion; /*< heading of trj  {rad}*/
	float     accu_lon;       /*< Accuracy estimate for longitude in meters {m} */
	float     accu_lat;       /*< Accuracy estimate for latitude in meters {m} */
	float     accu_height;    /*< Accuracy estimate for height in meters {m} */

	float     accu_heading;   /*< Accuracy estimate for heading {deg} */
	float     heading;        /*< heading {rad} */
	float     baseline;       /*< Baseline length in meters {m}*/
	uint32_t  GNSSFlag;
    uint16_t  fix_type:3;              /*< fix type {0,1,2,3,4,5}*/
    uint16_t  last:9;
    uint16_t  reserv:4;
}gnss_info_t;
/*-----**************************************************************************--*/

/*-----**************************************************************************--*/
/* IMU information structure ---------------------------------------------------
 * Holds decoded IMU outputs used by the application.
 * Members:
 *  uint16_t week - GPS week (when provided)
 *  double   secs - Seconds within week or time tag
 *  float    acc_x - X-axis linear acceleration (m/s^2)
 *  float    acc_y - Y-axis linear acceleration (m/s^2)
 *  float    acc_z - Z-axis linear acceleration (m/s^2)
 *  float    gyr_x - X-axis angular rate (deg/s)
 *  float    gyr_y - Y-axis angular rate (deg/s)
 *  float    gyr_z - Z-axis angular rate (deg/s)
 *  float    tmpt  - Temperature (see IMU frame scaling)
 *  uint16_t cnt   - Sample counter
 *-------------------------------------------------------------------------------*/
typedef struct {
	uint16_t week;    /*< GPS week (when provided) */
	double secs;      /*< Seconds within week or time tag */

	float acc_x;      /*< Acceleration X (m/s^2) */
	float acc_y;      /*< Acceleration Y (m/s^2) */
	float acc_z;      /*< Acceleration Z (m/s^2) */

	float gyr_x;      /*< Gyro X (deg/s) */
	float gyr_y;      /*< Gyro Y (deg/s) */
	float gyr_z;      /*< Gyro Z (deg/s) */

	float tmpt;       /*< Temperature (see IMU frame scaling) */
	int16_t cnt;     /*< Sample counter */
	uint16_t IMUFlag;
} imu_info_t;

/*-----**************************************************************************--*/
/* ODO (odometry) information structure -----------------------------------------
 * Stores wheel/odometry measurements collected per sample interval.
 * Members:
 *  uint16_t week        - GPS week or local week counter
 *  double   secs        - Seconds of week or timestamp (fractional seconds allowed)
 *  float    right_front - Right-front wheel odometry value
 *  float    right_back  - Right-rear wheel odometry value
 *  float    left_front  - Left-front wheel odometry value
 *  float    left_back   - Left-rear wheel odometry value
 *  float    mean        - Mean (aggregate) odometry value across wheels
 *-------------------------------------------------------------------------------*/
typedef struct {
	uint16_t week;
	double   secs;

	float    right_front;
	float    right_back;
	float    left_front;
	float    left_back;

	float    mean;
}odo_info_t;


typedef struct {
    long long cnt;
    long long cnt_max;
    imu_info_t* px_imu_data;
} imu_t;

typedef struct {
    uint32_t cnt;
    uint32_t cnt_max;
    gnss_info_t* px_gnss_data;
} gnss_t;

/*-----**************************************************************************--*/
/* KFAPP configuration parameters structure -------------------------------------
 * Configuration parameters for the KFAPP (Kalman Filter Application) module.
 * Contains file paths, enable flags, and various configuration values for GNSS,
 * IMU, navigation, and other system parameters.
 * Members:
 *  char srcfile_gnss[128]     - Source file path for GNSS data
 *  char resfile_gnss[128]     - Result file path for GNSS data
 *  char srcfile_imu[128]      - Source file path for IMU data
 *  char resfile_imu[128]      - Result file path for IMU data
 *  char srcfile_nav[128]      - Source file path for navigation data
 *  char resfile_nav[128]      - Result file path for navigation data
 *  char resfile_nav_gnss[128] - Result file path for navigation GNSS data
 *  char resfile_nav_imuod[128]- Result file path for navigation IMU/ODO data
 *  char simfile[128]          - Simulation file path
 *  char tracefile[128]        - Trace log file path
 *  int trace_lever            - Trace log level
 *  uint8_t gnss_enable        - GNSS enable flag
 *  uint8_t imu_enable         - IMU enable flag
 *  uint8_t imu_type           - IMU type
 *  uint8_t nav_enable         - Navigation enable flag
 *  uint8_t nav_type           - Navigation type
 *  float conf_lvGNSS_i        - GNSS level confidence parameter i
 *  float conf_lvGNSS_j        - GNSS level confidence parameter j
 *  float conf_lvGNSS_k        - GNSS level confidence parameter k
 *  float conf_dyawGNSS        - GNSS yaw confidence parameter
 *  float conf_lvOD_i          - Odometer level confidence parameter i
 *  float conf_lvOD_j          - Odometer level confidence parameter j
 *  float conf_lvOD_k          - Odometer level confidence parameter k
 *  float conf_odKappa_i       - Odometer kappa parameter i
 *  float conf_odKappa_j       - Odometer kappa parameter j
 *  float conf_odKappa_k       - Odometer kappa parameter k
 *  float conf_outputLV_i      - Output level parameter i
 *  float conf_outputLV_j      - Output level parameter j
 *  float conf_outputLV_k      - Output level parameter k
 *  float conf_calieb_i        - Calibration parameter i
 *  float conf_calieb_j        - Calibration parameter j
 *  float conf_calieb_k        - Calibration parameter k
 *  float conf_GNSSAgle        - GNSS angle parameter
 *  uint8_t conf_od            - Odometer configuration flag
 *  uint8_t ant_mode           - Antenna mode
 *  short imu_cnt              - IMU counter
 *-------------------------------------------------------------------------------*/
typedef struct KFAPP_ConfigPara {

#ifndef  DSPRELTIME
	char srcfile_gnss[128];
    char pattern_extertion[24];
    int  last_label;
//	char resfile_gnss[128];
//	char srcfile_imu[128];
//	char resfile_imu[128];

//	char srcfile_nav[128];
//	char resfile_nav[128];
//	char resfile_nav_gnss[128];
//	char resfile_nav_imuod[128];

//	char simfile[128];
//	char tracefile[128];
//	int  trace_lever;

	uint8_t gnss_enable;
//	uint8_t imu_enable ;
//	uint8_t imu_type   ;
//	uint8_t nav_enable ;
//	uint8_t nav_type   ;
//	uint8_t sim_exit   ;
#endif 
//	float conf_lvGNSS_i;
//	float conf_lvGNSS_j;
//	float conf_lvGNSS_k;
//	float conf_dyawGNSS;
//	float conf_lvOD_i;
//	float conf_lvOD_j;
//	float conf_lvOD_k;
//	float conf_odKappa_i;
//	float conf_odKappa_j;
//	float conf_odKappa_k;
//	float conf_outputLV_i;
//	float conf_outputLV_j;
//	float conf_outputLV_k;
//	float conf_calieb_i;
//	float conf_calieb_j;
//	float conf_calieb_k;
//	float conf_GNSSAgle;

//	short imu_cnt;
//	uint8_t conf_od;
//	uint8_t ant_mode;
//	uint8_t out_point;
//	uint8_t nav_type0;
}KFAPP_ConfigPara;

/*-----**************************************************************************--*/
/* KFAPP update data structure -------------------------------------------------
 * Update data structure for the KFAPP (Kalman Filter Application) module.
 * Contains the latest GNSS, IMU, and odometry data for processing.
 * Members:
 *  gnss_info_t gnss - GNSS information structure
 *  imu_info_t  imu  - IMU information structure
 *  odo_info_t  odo  - Odometry information structure
 *-------------------------------------------------------------------------------*/
typedef struct KFAPP_Update {
	gnss_info_t gnss;
	imu_info_t  imu;
	odo_info_t  odo;
    uint32_t    cnt: 6;
	uint32_t    flag_:1;
}KFAPP_Update;

/*-----**************************************************************************--*/
/* NAVIGATION solution structure ------------------------------------------------
 * High-level navigation solution produced by sensor fusion (GNSS/IMU/ODO).
 * Members:
 *  uint16_t week      - GPS week number
 *  double   secs      - Seconds within week or timestamp (fractional seconds)
 *  double   lat       - Geodetic latitude
 *  double   lon       - Geodetic longitude
 *  float    hgt       - Height / altitude (meters)
 *  float    yaw       - Yaw (heading) angle
 *  float    roll      - Roll angle
 *  float    pitch     - Pitch angle
 *  float    vnE       - Velocity component in East direction (m/s)
 *  float    vnN       - Velocity component in North direction (m/s)
 *  float    vnU       - Velocity component in Up direction (m/s)
 *  float    std_nav[9] - Navigation solution standard deviations / covariance
 *  float    eb[3]      - Estimated accelerometer biases
 *  float    db[3]      - Estimated gyro biases
 *  float    std_bias[6] - Standard deviations for bias estimates
 *  uint32_t flag_nav   - Navigation status flags (2 bits)
 *  uint32_t flag_gnss  - GNSS status flags (2 bits)
 *
 * Notes:
 *  - Per-field units and scaling may vary by project configuration; consult
 *    the top-of-file comments or external documentation for precise scaling.
 *-------------------------------------------------------------------------------*/
typedef struct {
	uint16_t  week;
	double    secs;
	double    lat;
	double    lon;
	float     hgt;
	float     yaw;
	float     roll;
	float     pitch;
	float     vnE;
	float     vnN;
	float     vnU;
	float     std_nav[9];
	float     std_rt[18];
	float     eb[3];
	float     db[3];
	float     std_bias[6];
	float     yaw_gnss_delta;
	uint32_t  measflag;
	uint32_t  flag_nav : 2;
	uint32_t  flag_gnss : 2;
}nav_info_t;

#pragma pack(push)
#pragma pack(1)
/* IMU44_t (44-byte IMU frame) ------------------------------------------------
 * Packed on-wire representation of the IMU 44-byte frame. Fields map to the
 * serial protocol and are used to assemble `imu_packet_t` for decoding.
 * Members:
 *  - head         : Frame header byte (start marker, e.g. 0xAA)
 *  - weeka:index  : High nibble: week high bits (weeka), low nibble: frame index
 *  - weekb        : Week low byte (combined with weeka to form full week)
 *  - tk           : Time tag (seconds / timestamp as double)
 *  - wmx/wmy/wmz  : Angular rates around X/Y/Z axes (degrees)
 *  - vmx/vmy/vmz  : Linear velocities or accelerations on X/Y/Z (m/s)
 *  - tmpt         : Temperature value (integer, typically 0.01 degC resolution)
 *  - odo          : Odometer or additional sensor counter (implementation-specific)
 *  - reser0       : Reserved byte
 *  - reser1/flag0 : Reserved bits and small flag nibble
 *  - flag_type    : Frame/flag type indicator
 *  - check        : Simple checksum or matching byte used during parsing
 *  - back         : Frame tail byte (end marker, e.g. 0xAC)
 *
 * Notes:
 *  - The exact scaling/units for some fields (vmx/vmy/vmz, odo) depend on
 *    the IMU vendor/protocol; consult IMU documentation if precise units are
 *    required. The code expects `tk` to be usable as a time tag when copying
 *    into `imu_info_t`.
 *-------------------------------------------------------------------------------*/
typedef struct {
	uint8_t head;

	uint8_t weeka : 4;
	uint8_t index : 4;
	uint8_t weekb; /* week = weeka*256 + weekb */

	double  tk;
	float   wmx;
	float   wmy;
	float   wmz;
	float   vmx;
	float   vmy;
	float   vmz;
	int16_t tmpt;
	int16_t odo;

	uint8_t reser0;
	uint8_t reser1 : 4;
	uint8_t flag0 : 4;
	uint8_t flag_type;
	uint8_t check;
	uint8_t back;
} IMU44_t;

typedef struct {
//	uint16_t headA:8;
//	uint16_t headB:8;
//
//	uint8_t  pcnum;
	int32_t  index;

	int32_t  wmx;
	int32_t  wmy;
	int32_t  wmz;
	int32_t  vmx;
	int32_t  vmy;
	int32_t  vmz;
	int16_t tmpt_a;
	int16_t tmpt_b;

	uint32_t code;
} IMU40_t;

/* imu_packet_t (packet union description):
 *  - data: parsed IMU44_t view of the frame
 *  - buff: raw bytes received from IMU (length IMU_FRAME_LENGTH)
 */
typedef union
{
	IMU44_t data;
	uint8_t buff[IMU_FRAME_LENGTH];
}imu_packet_t;

/*-----**************************************************************************--*/
/* UBLOX PVT message structure ------------------------------------------------
 * Stores UBX-NAV-PVT message fields. Members:
 *  itow      - GPS time of week (ms)
 *  year...sec- Calendar date/time
 *  tacc/nano - Time accuracy and fractional seconds
 *  fixtype   - GNSS fix type
 *  numSv     - Number of satellites used
 *  lon/lat   - Scaled longitude/latitude
 *  height/hmsl - Heights above ellipsoid and mean sea level
 *  hacc/vacc/sacc/headacc - Accuracy metrics
 *  veln/vele/veld/gSpeed - Velocity components and ground speed
 *  headmot/headveh - Heading of motion / vehicle heading
 *  pdop      - Position dilution of precision
 *-------------------------------------------------------------------------------*/
typedef struct {
	uint8_t res0[4];

	uint32_t itow;
	uint16_t year;
	uint16_t  month:8;
	uint16_t  day:8;

	uint32_t  hour:8;
	uint32_t  min:8;
	uint32_t  sec:8;
	uint32_t  res1:8;

	uint32_t tacc;
	int      nano;

	uint32_t  fixtype:8;
	uint32_t  res2:8;
	uint32_t  res3:8;
	uint32_t  numSv:8;

	int lon;
	int lat;
	int height;
	int hmsl;

	uint32_t hacc;
	uint32_t vacc;

	int veln;
	int vele;
	int veld;
	int gSpeed;
	int headmot;

	uint32_t sacc;
	uint32_t headacc;
	uint16_t pdop;
	uint16_t res4:8;
	uint16_t res5:8;

	int headveh;
	short magDec;
	uint16_t magAcc;
}UBLOX_PVT_t;
/* (End UBLOX PVT) */

/* UBLOX REL message structure ------------------------------------------------
 * Stores UBX-NAV-RELPOS fields (relative positioning). Members:
 *  itow            - GNSS time of week (ms)
 *  relposn/relpose/relposd - Relative position N/E/D (mm)
 *  relposLength    - Baseline length (mm)
 *  relposHeading   - Relative heading (scaled)
 *  relposHp*       - High-precision fractional parts
 *  accN/accE/accD  - Accuracy estimates
 *  bitfields       - Status flags (gnssFixOk, diffSoln, relPosValid, etc.)
 *-------------------------------------------------------------------------------*/
typedef struct {
	uint8_t res0[4];
	uint8_t version;
	uint8_t res1;
	uint16_t  ref;

	uint32_t  itow;
	int     relposn;
	int     relpose;
	int     relposd;
	int     relposLength;
	int     relposHeading;

	uint8_t  res2[4];

	char   relposHpn;
	char   relposHpe;
	char   relposHpd;
	char   relposHpLength;

	uint32_t accN;
	uint32_t accE;
	uint32_t accD;
	uint32_t accLength;
	uint32_t accHeading;

	uint8_t  res3[4];

	uint32_t gnssFixOk : 1;
	uint32_t diffSoln : 1;
	uint32_t relPosValid : 1;
	uint32_t carrSoln : 2;
	uint32_t isMoving : 1;
	uint32_t refPosMiss : 1;
	uint32_t refObsMiss : 1;
	uint32_t relPosHeading : 1;
	uint32_t resflag : 23;
}UBLOX_REL_t;


typedef struct {
//    uint8_t  sync[2]; /* 0x24 0x40 */
    uint16_t crc;
    uint16_t id_num:13;
    uint16_t id_version:3;

    uint16_t msg_length;
    uint32_t tow; /* ms */
    uint16_t wnc;

    uint16_t type_pvt:4;
    uint16_t resevred0:4;
    uint16_t err_code_pvt:8;

    double   lat;  /*rad*/
    double   lon;  /*rad*/
    double   hgt;  /*m*/
    float    hgt_datum;

    float    vn;
    float    ve;
    float    vu;

    uint8_t resevred1[40];
}G5_SBF_t;

#define MSG_HEADA 0xBD
#define MSG_HEADB 0xDB
#define MSG_NAV   0x0B

typedef struct {
	uint16_t  headA:8;
	uint16_t  headB:8;
	uint8_t   message_type;
	int16_t   pitch;
	int16_t   roll;
	int16_t   yaw;
	int16_t   wmm[3];
	int16_t   vmm[3];
	int32_t   lat;
	int32_t   lon;
	int32_t   hgt;
	int16_t   velE;
	int16_t   velN;
	int16_t   velU;
	uint8_t   flag;
	uint16_t  gnss_week;
	uint16_t  num_sv:8;
	uint16_t  rtk_cyle:8;
	int16_t   od_vel;
	uint16_t  data[3];
	uint32_t  gnss_secs;
	uint16_t  type0:8;
	uint16_t  check0:8;
	int16_t   wmm0[3];
	int16_t   vmm0[3];
	uint8_t   gnss_sol_type:4;
	uint8_t   type_main:4;
	float     gnss_main;
	uint8_t   check1;
}nav_message76_t;

typedef struct {
	uint16_t  headA:8;
	uint16_t  headB:8;
	uint8_t   message_type;
	/* first 16 bytes */
	uint16_t  is_psc_bsw_ok:1;
	uint16_t  is_spi_ok:1;
	uint16_t  is_timer0_ok:1;
	uint16_t  is_timer1_ok:1;

	uint16_t  is_uart0_ok:1;
	uint16_t  is_uart1_ok:1;
	uint16_t  is_uart2_ok:1;
	uint16_t  is_pps_osc_ok:1;
	uint16_t  resved0:8;

	/* sencond 16 bytes */
	uint16_t  is_pps_osc_calibrate:1;
	uint16_t  is_isr_imu_ok:1;
	uint16_t  is_isr_gnss_ok:1;
	uint16_t  is_isr_time_ok:1;

	uint16_t  ctrl_flag:1;
	uint16_t  out_flag:1;
	uint16_t  align_type:1;
	uint16_t  cfg_flag:1;

	uint16_t  resved1:8;
	/* third 16 bytes */
	uint16_t  sys_statues:4;
	uint16_t  cnt_pps_interrput:4;
	uint16_t  resved2:8;

	int16_t   wmm[3];
	int16_t   vmm[3];

	int32_t   imu_dt;
	int32_t   gnss_dt;
	int32_t   odo_ddt;

	int16_t   imu_cnt;
	int16_t   osc_frq;
	int16_t   gnss_cnt;

	uint8_t   flag;
	uint16_t  gnss_week;
	uint16_t  num_sv:8;
	uint16_t  rtk_cyle:8;
	int16_t   od_vel;
	uint16_t  data[3];
	uint32_t  gnss_secs;
	uint16_t  type0:8;
	uint16_t  check0:8;
	int16_t   wmm0[3];
	int16_t   vmm0[3];
	uint8_t   gnss_sol_type:4;
	uint8_t   type_main:4;
	int32_t   gnss_main;
	uint8_t   check1;
}bit_message76_t;

#define MSG_HEADA 0xBD
#define MSG_HEADB 0xDB
#define MSG_BIT   0x0A
#define MSG_NAV   0x0B

#define NAV_FRAME_LENGTH_A 76
#define NAV_FRAME_LENGTH_B 81
#define NAV_FRAME_LENGTH_C 86

typedef struct {
	uint8_t head[3];
	int16_t att[3];
	int16_t wmm[3];
	int16_t vmm[3];
	int32_t pos[3];
	int16_t vel[3];
	uint8_t flag;
	uint16_t gnss_week;
	uint8_t num_sv;
	uint8_t rtk_cyle;
	int16_t od_vel;
	uint16_t data[3];
	uint32_t gnss_secs;
	uint8_t type0;
	uint8_t check0;
	int16_t wmm0[3];
	int16_t vmm0[3];
	uint8_t type_main;
	float gnss_main;
	uint8_t check1;
}nav_frame_t;

typedef union
{
	nav_frame_t data;
	uint8_t buff[NAV_FRAME_LENGTH_A];
}nav_packet_t;

#pragma pack(pop)

/** trace log function */
/* open trace log file ---------------------------------------------------------
* open a trace log file for writing debug or trace information
* args   : const char* file   I   file name to open for trace logging
* return : none
*-----------------------------------------------------------------------------*/
void traceopen(const char* file);

/* close trace log file --------------------------------------------------------
* close the currently open trace log file
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void traceclose(void);

/* set trace log level ---------------------------------------------------------
* set the verbosity level for trace logging
* args   : int        level   I   trace level (higher means more verbose)
* return : none
*-----------------------------------------------------------------------------*/
void tracelevel(int level);

/* write trace log message -----------------------------------------------------
* write a formatted message to the trace log at the specified level
* args   : int        level   I   trace level
*          const char* format I   printf-style format string
*          ...                I   variable arguments
* return : none
*-----------------------------------------------------------------------------*/
void trace(int level, const char* format, ...);

/* write trace log message with timestamp --------------------------------------
* write a formatted message with timestamp to the trace log at the specified level
* args   : int        level   I   trace level
*          const char* format I   printf-style format string
*          ...                I   variable arguments
* return : none
*-----------------------------------------------------------------------------*/
void tracet(int level, const char* format, ...);

#ifdef __cplusplus
extern "C" {
#endif
/* initialize CKF algorithm ---------------------------------------------------
* initialize the CKF (Kalman Filter) algorithm parameters and state
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void ckf_alg_init(void* ptr_kfapp,int nav_type);

/* perform coast alignment for CKF -------------------------------------------
* perform coast alignment using the provided update parameters
* args   : KFAPP_Update* ptr_UpPara I   pointer to update parameters structure
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int  ckf_coast_align(KFAPP_Update* ptr_UpPara);

/* initialize CKF parameters --------------------------------------------------
* initialize the CKF parameters using configuration and update structures
* args   : KFAPP_ConfigPara* ptr_Config I   pointer to configuration parameters
*          KFAPP_Update* ptr_UpPara     I   pointer to update parameters structure
* return : none
*-----------------------------------------------------------------------------*/
void ckf_para_init(KFAPP_ConfigPara* ptr_Config, KFAPP_Update* ptr_UpPara);

/* update CKF state -----------------------------------------------------------
* update the CKF state using the provided update parameters
* args   : KFAPP_Update* ptr_UpPara I   pointer to update parameters structure
* return : none
*-----------------------------------------------------------------------------*/
void ckf_update(KFAPP_Update* ptr_UpPara);

/* print out the kfapp result--------------------------------------------------
* print out the kfapp result with ptr_nav using the config
* args   : KFAPP_ConfigPara* ptr_Config I   pointer to configuration parameters
*          nav_info_t*       ptr_nav    O   pointer to configuration parameters
* return : none
*-----------------------------------------------------------------------------*/
void out_nav(KFAPP_ConfigPara* ptr_Config, nav_info_t* ptr_nav);

/* match_timestamps ------------------------------------------------------------
 * Matches timestamps between GNSS and IMU data arrays within a given tolerance.
 * args   : const Week_Sec_t* gnss_times   I   Pointer to array of GNSS timestamps
 *          const Week_Sec_t* imu_times    I   Pointer to array of IMU timestamps
 *          double tolerance               I   Maximum allowed time difference
 * return : int                            R   Number of matched timestamps or error code
 -------------------------------------------------------------------- */
int match_timestamps(const gtime_t* gnss_times, const gtime_t* imu_times, double tolerance);

#ifndef DSPRELTIME
	typedef struct {
		int cnt_file;
		char file_name[128][256];
	} file_list_t;
	int listFiles(char* dir, const char* matchchar);
	file_list_t* getFileList(void);
#endif 


#ifdef __cplusplus
}
#endif

#ifdef CONFIGSIM
/* search configuration by name ------------------------------------------------
* search for a configuration entry by name in a configuration array
* args   : const char* name      I   configuration name to search for
*          const CONFIG_t* configs I   array of configuration entries
* return : pointer to found CONFIG_t entry, or NULL if not found
*-----------------------------------------------------------------------------*/
CONFIG_t* searchconfigs(const char* name, const CONFIG_t* configs);

/* convert string to configuration ---------------------------------------------
* parse a string and set the configuration entry accordingly
* args   : CONFIG_t* config     O   pointer to configuration entry to set
*          const char* str      I   string to parse
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int str2config(CONFIG_t* config, const char* str);

/* convert configuration to string ---------------------------------------------
* convert a configuration entry to a string representation
* args   : const CONFIG_t* config I   configuration entry to convert
*          char* str              O   output string buffer
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int config2str(const CONFIG_t* config, char* str);

/* convert configuration to buffer ---------------------------------------------
* convert a configuration entry to a buffer (for binary or compact storage)
* args   : const CONFIG_t* config I   configuration entry to convert
*          char* buff             O   output buffer
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int config2buf(const CONFIG_t* config, char* buff);

/* load configurations from file -----------------------------------------------
* load an array of configuration entries from a file
* args   : const char* file      I   file name to load from
*          CONFIG_t* configs     O   array to store loaded configurations
* return : number of configurations loaded, or -1 on error
*-----------------------------------------------------------------------------*/
int loadconfigs(const char* file, CONFIG_t* configs);

/* save configurations to file -------------------------------------------------
* save an array of configuration entries to a file
* args   : const char* file      I   file name to save to
*          const char* mode      I   file open mode ("w", "a", etc.)
*          const char* comment   I   comment to write at the top of the file
*          const CONFIG_t* configs I   array of configurations to save
* return : number of configurations saved, or -1 on error
*-----------------------------------------------------------------------------*/
int saveconfigs(const char* file, const char* mode, const char* comment, const CONFIG_t* configs);
#endif


#ifndef DSPRELTIME
/* open simulation data file --------------------------------------------------
* open a simulation data file for reading
* args   : char* file_name I   name of the file to open
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int sim_open(char* file_name);

/* load simulation data from file ---------------------------------------------
* load simulation data from the opened file into the data structure
* args   : sim32bin_t* data_sim O   pointer to simulation data structure
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int load_data(sim32bin_t* data_sim);

/* close simulation data file -------------------------------------------------
* close the currently open simulation data file
* args   : none
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int sim_close();

/* convert simulation data to KFAPP format -----------------------------------
* convert simulation data structure to KFAPP update structure
* args   : sim32bin_t* ptr_sim       I   pointer to simulation data
*          KFAPP_Update* ptr_UpPara  O   pointer to KFAPP update structure
* return : none
*-----------------------------------------------------------------------------*/
void sim2kfapp(sim32bin_t* ptr_sim, KFAPP_Update* ptr_UpPara);
#endif


/* IMU_Get --------------------------------------------------------------------
 * Return pointer to latest decoded IMU information.
 * args   : none
 * return : imu_info_t*  R pointer to internal IMU info struct
 -------------------------------------------------------------------- */
imu_info_t* imu_get(void);

/* IMU_Parse ------------------------------------------------------------------
 * Feed a single incoming byte from the IMU serial stream into the IMU parser
 * state machine.
 * args   : uint8_t ch1   I   incoming byte
 * return : int           R   parser status (0: frame parsed, 1: in-progress,
 *                            2: checksum/match error)
 *-------------------------------------------------------------------------------*/
int imu_parse(uint8_t ch1);

/* GNSS_Parse -----------------------------------------------------------------
 * Feed a single incoming byte from the GNSS serial stream (NMEA/other) into
 * the GNSS parser state machine.
 * args   : uint8_t ch1   I   incoming byte
 * return : void          R   (parsing is driven internally; parsed frames
 *                            are exposed via `GNSS_get()`)
 *-------------------------------------------------------------------------------*/
void gnss_parse(uint8_t ch1);

/* GNSS_get -------------------------------------------------------------------
 * Return pointer to the latest decoded GNSS information.
 * args   : none
 * return : gnss_info_t*  R pointer to internal GNSS info struct
 *-------------------------------------------------------------------------------*/
gnss_info_t* gnss_get(void);


/* NAV_Get --------------------------------------------------------------------
* Return pointer to the latest high-level navigation solution produced by the
* fusion algorithm (GNSS/IMU/ODO).
* args   : none
* return : nav_info_t*   R pointer to internal navigation info struct
*-------------------------------------------------------------------------------*/
nav_info_t* nav_get(void);

/* NAV_Parse ------------------------------------------------------------------
* Feed a single incoming byte from the NAV framing stream into the NAV
* parser state machine. This parser extracts on-wire nav frames defined by
* `nav_frame_t`/`nav_packet_t`.
* args   : uint8_t ch1   I   incoming byte
* return : int           R   parser status (0: frame parsed, 1: in-progress,
*                            2: checksum/match error)
*-------------------------------------------------------------------------------*/
int nav_parse(uint8_t ch1);

int gnss_parse_by_file(const char* file_name_gnss, gnss_info_t* px_gnss, gnss_t* gnss_data, int last_label);

int nav_parse_by_file(const char* file_name_imu, imu_info_t* px_imu, imu_t* imu_data);

void add_gnss(gnss_info_t* gnss_seg, gnss_t* gnss_data);

void add_imu(imu_info_t* imu_seg, imu_t* imu_data);

void fill_sim(KFAPP_Update* ptr_update, sim32bin_t* ptr_sim);

#endif //MYNAV_APPCONFIGPARA_H
