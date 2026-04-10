/* mid_parse.c / mid_parse.cpp -------------------------------------------------
 * Source: src/mid_parse.cpp
 *
 * File purpose:
 *  - Implements parsers and helper utilities for GNSS (NMEA/UBLOX) and IMU
 *    binary frames used by the ORI_FILE_PARSE project.
 *  - Provides configuration file parsing (keyword=value) utilities used to
 *    load and save runtime options.
 *  - Exposes parser entry-points and accessors:
 *      `GNSS_Parse(uint8_t)`  - NMEA/UBLOX/Proprietary GNSS stream parser
 *      `IMU_Parse(uint8_t)`   - IMU binary-frame streaming parser
 *      `GNSS_get()` / `IMU_Get()` - access latest parsed info structures
 *
 * author:  PPOI_team YEJIN
 * 
 * 
 * Contents summary:
 *  - small string/helper utilities (chop, str2num, enum helpers)
 *  - configuration helpers (CONFIG_t, load/save, conversion helpers)
 *  - GNSS sentence parsers for GGA, GST, ZDA, HEADING, BESTVEL and UBLOX
 *    message handlers (PVT/REL)
 *  - IMU frame definition (IMU44_t), parser state machine, and decoded
 *    `imu_info_t` population routines
 *
 * Threading / reentrancy:
 *  - The parsers and static state in this file are not thread-safe. Callers
 *    must serialize access when feeding bytes from multiple sources/threads.
 *
 * Notes on units and scaling:
 *  - Many fields are stored as scaled integers (see `include/mid_parse.h`)
 *    or as platform-specific units. Consumers should consult the header
 *    for per-field unit notes and apply scaling before arithmetic.
 *
 * Author: (project)
 * Last modified: 2025-12-09
 *-------------------------------------------------------------------------------*/
#include "app_interface.h"
#include <math.h>

/* Parsing state flag -----------------------------------------------------------
* Indicates the current state of the GNSS sentence parser.
*-------------------------------------------------------------------------------*/
static int flag_parse_gnss = 0;
/* Parsing buffer index ---------------------------------------------------------
 * Index for parsing buffer during GNSS sentence parsing.
 *-------------------------------------------------------------------------------*/
static int index_parse_gnss = 0;
/* Parsing buffer ---------------------------------------------------------------
 * Buffer for accumulating characters during GNSS sentence parsing.
 *-------------------------------------------------------------------------------*/
static uint8_t str_parse[GNSS_MAX_NUM] = { 0 };
static uint8_t ch_0;
/* Temporary variable for floating-point calculations ---------------------------
 * Used for intermediate results during parsing.
 *-------------------------------------------------------------------------------*/
static double tmp = 0.0;
/* GNSS information structure instance ------------------------------------------
 * Holds the latest parsed GNSS information.
 *-------------------------------------------------------------------------------*/
static gnss_info_t sx_gnss_info = { 0 };
/* Frame name buffer and parsing flags ------------------------------------------
 * Used for frame identification and parsing state control.
 *-------------------------------------------------------------------------------*/
static uint8_t Frame_name[10] = { 0 }, Rflag = 0, num = 0;
/* Checksum accumulator for NMEA parsing ----------------------------------------
 * Used to accumulate checksum during NMEA sentence parsing.
 *-------------------------------------------------------------------------------*/
static uint32_t check_parse_nema = 0;

static int16_t pos_int = 0;
 /* IMU information structure instance ------------------------------------------
  * Holds the latest parsed IMU packet buffer and decoded IMU information.
  *-------------------------------------------------------------------------------*/
static imu_packet_t sx_imu_packet = { 0 };

/* Latest decoded IMU information ---------------------------------------------
 * Populated from `sx_imu_packet` when a full frame is received and parsed.
 * Use `IMU_Get()` to obtain a pointer to this structure.
 *-------------------------------------------------------------------------------*/
static imu_info_t sx_imu_info = { 0 };

/* Checksum / matcher accumulator for IMU parsing -----------------------------
 * Running accumulator used by the IMU parser state machine to validate
 * frame boundary / simple checksum criteria while receiving bytes.
 *-------------------------------------------------------------------------------*/
static uint32_t check_parse_imu = 0;

/* IMU parser state flag ------------------------------------------------------
 * State of the IMU parsing state machine (e.g. 0 = idle, 1 = receiving).
 *-------------------------------------------------------------------------------*/
static int flag_parse_imu = 0;

/* IMU parsing buffer index ---------------------------------------------------
 * Current write index into `sx_imu_packet.buff` while building an incoming
 * IMU frame byte-by-byte.
 *-------------------------------------------------------------------------------*/
static int index_parse_imu = 0;

 /* IMU information structure instance ------------------------------------------
  * Holds the latest parsed IMU packet buffer and decoded IMU information.
  *-------------------------------------------------------------------------------*/
static nav_packet_t sx_nav_packet = { 0 };


static odo_info_t sx_odo_info = { 0 };

/* Checksum / matcher accumulator for IMU parsing -----------------------------
 * Running accumulator used by the IMU parser state machine to validate
 * frame boundary / simple checksum criteria while receiving bytes.
 *-------------------------------------------------------------------------------*/
static uint32_t check_parse_nav = 0;

/* IMU parser state flag ------------------------------------------------------
 * State of the IMU parsing state machine (e.g. 0 = idle, 1 = receiving).
 *-------------------------------------------------------------------------------*/
static int flag_parse_nav = 0;

/* IMU parsing buffer index ---------------------------------------------------
 * Current write index into `sx_imu_packet.buff` while building an incoming
 * IMU frame byte-by-byte.
 *-------------------------------------------------------------------------------*/
static int index_parse_nav = 0;
/* frame2data ---------------------------------------------------------------
 * transfer parsed IMU frame buffer into imu_info structure
 * args   : none
 * return : int  (0:ok)
 *-----------------------------------------------------------------------------*/
static int navframe2data()
{
	int tmp_data0 = 0;
	uint32_t tmp_data1 = 0;

	/* imu info*/
	sx_imu_info.week = sx_nav_packet.data.gnss_week;
	sx_imu_info.secs = sx_nav_packet.data.gnss_secs * 2.5e-4;
	sx_imu_info.gyr_x = sx_nav_packet.data.wmm[1] * 300.0f / 32768 * TS * DEG;
	sx_imu_info.gyr_y = sx_nav_packet.data.wmm[0] * 300.0f / 32768 * TS * DEG;
	sx_imu_info.gyr_z = -sx_nav_packet.data.wmm[2] * 300.0f / 32768 * TS * DEG;

	sx_imu_info.acc_x = sx_nav_packet.data.vmm[1] * 12.0f / 32768 * TS * G0;
	sx_imu_info.acc_y = sx_nav_packet.data.vmm[0] * 12.0f / 32768 * TS * G0;
	sx_imu_info.acc_z = -sx_nav_packet.data.vmm[2] * 12.0f / 32768 * TS * G0;

	/* odo info*/
	sx_odo_info.week = sx_nav_packet.data.gnss_week;
	sx_odo_info.secs = sx_nav_packet.data.gnss_secs * 2.5e-4;
	sx_odo_info.mean = sx_nav_packet.data.od_vel * 1e-2;

	return 0;
}


/* nav_parse ----------------------------------------------------------------
 * state machine to parse incoming IMU byte stream framed by head/tail markers
 * args   : uint8_t ch1   I   incoming byte from IMU stream
 * return : int           R   0: a full frame parsed and converted to data
 *                            2: checksum/match error
 *                            1: parsing in-progress or no valid frame yet
 *-----------------------------------------------------------------------------*/
int nav_parse(uint8_t ch1)
{
	int res = 1;

	switch (flag_parse_nav)
	{
	case 0:
		if (MSG_HEADA == ch1) {
			check_parse_nav = 0;
			check_parse_nav &= ch1;
			flag_parse_nav = 1;
			index_parse_nav = 0;
			sx_nav_packet.buff[index_parse_nav] = ch1;
			index_parse_nav++;
		}
		break;
	case 1:
		if (MSG_HEADB == ch1) {
			flag_parse_nav = 2;
			check_parse_nav &= ch1;
			sx_nav_packet.buff[index_parse_nav] = ch1;
			index_parse_nav++;
		}
		break;
	case 2:
		if (MSG_NAV == ch1) {
			flag_parse_nav = 3;
			check_parse_nav &= ch1;
			sx_nav_packet.buff[index_parse_nav] = ch1;
			index_parse_nav++;
		}
		break;
	case 3:
		if (index_parse_nav < NAV_FRAME_LENGTH_A - 1)
		{
			check_parse_nav &= ch1;
			sx_nav_packet.buff[index_parse_nav] = ch1;
			index_parse_nav++;
		}
		else if (index_parse_nav == NAV_FRAME_LENGTH_A - 1)
		{
			sx_nav_packet.buff[index_parse_nav] = ch1;
			navframe2data();
			flag_parse_nav = 0;
			index_parse_nav = 0;
			res = 0;
			memset(&sx_nav_packet, 0, sizeof(sx_nav_packet));
		}
		else {
			index_parse_nav = 0;
			flag_parse_nav = 0;
		}

		break;

	}

	return res;
}

/* nav_get ------------------------------------------------------------------
 * return pointer to the most recently parsed IMU information
 * args   : none
 * return : nav_info_t*   R   pointer to internal imu info struct
 *-----------------------------------------------------------------------------*/
nav_info_t* nav_get(void) {
	return &sx_nav_packet;
}

/* frame2data ---------------------------------------------------------------
 * transfer parsed IMU frame buffer into imu_info structure
 * args   : none
 * return : int  (0:ok)
 *-----------------------------------------------------------------------------*/
static int imuframe2data()
{
	sx_imu_info.acc_x = sx_imu_packet.data.vmx;
	sx_imu_info.acc_y = sx_imu_packet.data.vmy;
	sx_imu_info.acc_z = sx_imu_packet.data.vmz;

	sx_imu_info.gyr_x = sx_imu_packet.data.wmx;
	sx_imu_info.gyr_y = sx_imu_packet.data.wmy;
	sx_imu_info.gyr_z = sx_imu_packet.data.wmz;

	sx_imu_info.tmpt = sx_imu_packet.data.tmpt * 0.01;

	sx_imu_info.secs = sx_imu_packet.data.tk;
	sx_imu_info.week = sx_imu_packet.data.weeka * 256 + sx_imu_packet.data.weekb;

	return 0;
}

///* frame2data ---------------------------------------------------------------
// * transfer parsed IMU frame buffer into imu_info structure
// * args   : none
// * return : int  (0:ok)
// *-----------------------------------------------------------------------------*/
//static int imuframe2data()
//{
//	sx_imu_info.acc_x = sx_imu_packet.data.vmx * 0.000001 / FS;
//	sx_imu_info.acc_y = sx_imu_packet.data.vmy * 0.000001 / FS;
//	sx_imu_info.acc_z = sx_imu_packet.data.vmz * 0.000001 / FS;
//
//	sx_imu_info.gyr_x = sx_imu_packet.data.wmx * 0.000001 / FS * DEG;
//	sx_imu_info.gyr_y = sx_imu_packet.data.wmy * 0.000001 / FS * DEG;
//	sx_imu_info.gyr_z = sx_imu_packet.data.wmz * 0.000001 / FS * DEG;
//
//	sx_imu_info.tmpt = (sx_imu_packet.data.tmpt_a + sx_imu_packet.data.tmpt_b) * 0.008 / 2.0;
//
//	return 0;
//}


/* IMU_Parse ----------------------------------------------------------------
 * state machine to parse incoming IMU byte stream framed by head/tail markers
 * args   : uint8_t ch1   I   incoming byte from IMU stream
 * return : int           R   0: a full frame parsed and converted to data
 *                            2: checksum/match error
 *                            1: parsing in-progress or no valid frame yet
 *-----------------------------------------------------------------------------*/
int imu_parse(uint8_t ch1)
{
    int res = 1;
    if (IMU_HEAD_44 == ch1)
        flag_parse_imu = 0;

    switch (flag_parse_imu)
    {
        case 0:
            if (IMU_HEAD_44 == ch1)
            {
                check_parse_imu = 0;
                check_parse_imu &= ch1;
                flag_parse_imu = 1;
                index_parse_imu = 0;
                sx_imu_packet.buff[index_parse_imu] = ch1;
                index_parse_imu++;
            }
        break;
        case 1:
            if (IMU_BACK_44 != ch1 || index_parse_imu < IMU_FRAME_LENGTH -1 )
            {
                sx_imu_packet.buff[index_parse_imu] = ch1;
                index_parse_imu++;
                if (IMU_FRAME_LENGTH - 3 != index_parse_imu)
                    check_parse_imu &= ch1;
                if (IMU_FRAME_LENGTH - 2 == index_parse_imu && check_parse_imu != ch1)
                    res = 2;
            }
            else if (IMU_BACK_44 == ch1 && IMU_FRAME_LENGTH - 1 == index_parse_imu)
            {
                sx_imu_packet.buff[index_parse_imu] = ch1;
                index_parse_imu = 0;
                imuframe2data();
                flag_parse_imu = 0;
                index_parse_imu = 0;
                res = 0;
                memset(&sx_imu_packet, 0, sizeof(sx_imu_packet));
            }
            else {
                index_parse_imu = 0;
                flag_parse_imu = 0;
            }
        break;
    }
    return res;
}

///* IMU_Parse ----------------------------------------------------------------
// * state machine to parse incoming IMU byte stream framed by head/tail markers
// * args   : uint8_t ch1   I   incoming byte from IMU stream
// * return : int           R   0: a full frame parsed and converted to data
// *                            2: checksum/match error
// *                            1: parsing in-progress or no valid frame yet
// *-----------------------------------------------------------------------------*/
//int imu_parse(uint8_t ch1)
//{
//	int res = 1;
//
//	switch (flag_parse_imu)
//	{
//	case 0:
//		if (IMU_HEAD0_40 == ch1)
//		{
//			check_parse_imu = 0;
//			check_parse_imu &= ch1;
//			flag_parse_imu = 1;
//			index_parse_imu = 0;
//			sx_imu_packet.buff[index_parse_imu] = ch1;
//			index_parse_imu++;
//		}
//		break;
//	case 1:
//		if (IMU_HEAD1_40 == ch1)
//		{
//			check_parse_imu &= ch1;
//			flag_parse_imu = 2;
//			sx_imu_packet.buff[index_parse_imu] = ch1;
//			index_parse_imu++;
//		}
//		else {
//			flag_parse_imu = 0;
//		}
//		break;
//	case 2:
//		if (index_parse_imu < IMU_FRAME_LENGTH - 1)
//		{
//			sx_imu_packet.buff[index_parse_imu] = ch1;
//			index_parse_imu++;
//			if (IMU_FRAME_LENGTH - 3 != index_parse_imu)
//				check_parse_imu &= ch1;
//		}
//		else if (IMU_FRAME_LENGTH - 1 == index_parse_imu)
//		{
//			if (check_parse_imu == ch1) {
//				sx_imu_packet.buff[index_parse_imu] = ch1;
//				check_parse_imu = 0;
//				index_parse_imu = 0;
//				imuframe2data();
//				flag_parse_imu = 0;
//				index_parse_imu = 0;
//				res = 0;
//				memset(&sx_imu_packet, 0, sizeof(sx_imu_packet));
//				sx_imu_info.IMUFlag = 1;
//				isr_sync_imu();
//			}
//		}
//		else {
//			index_parse_imu = 0;
//			flag_parse_imu = 0;
//		}
//		break;
//	}
//	return res;
//}

/* IMU_Get ------------------------------------------------------------------
 * return pointer to the most recently parsed IMU information
 * args   : none
 * return : imu_info_t*   R   pointer to internal imu info struct
 *-----------------------------------------------------------------------------*/
imu_info_t* imu_get(void) {
	memset(&sx_imu_info, 0, sizeof(sx_imu_info));
	return &sx_imu_info;
}

/* string to number ------------------------------------------------------------
* convert substring in string to number
* args   : char   *s        I   string ("... nnn.nnn ...")
*          int    i,n       I   substring position and width
* return : converted number (0.0:error)
*-----------------------------------------------------------------------------*/
static double str2num(const char* s, int i, int n)
{
    double value = 0.0;
    double frac_div = 1.0;
    int sign = 1;
    uint8_t in_fraction = 0;
    uint8_t digits_found = 0;

    if (!s || i < 0 || n <= 0) return 0.0;
    const char* p = s + i;
    const char* end = p + n;

    if (p < end && *p == '-') {
        sign = -1;
        p++;
    } else if (p < end && *p == '+') {
        p++;
    }

    for (; p < end && *p != '\0'; p++) {
        if (*p >= '0' && *p <= '9') {
            digits_found = 1;
            if (in_fraction) {
                frac_div *= 10.0;
                value += (*p - '0') / frac_div;
            } else {
                value = value * 10.0 + (*p - '0');
            }
        } else if (*p == '.' && !in_fraction) {
            in_fraction = 1;
        } else if (*p == ' ' || *p == '\t') {
//            continue;
            break;
        } else {
            break;
        }
    }

    if (!digits_found) {
        return 0.0;
    }

    return sign * value;
}

/* Convert calendar date to GPS week and seconds --------------------------------
 * Converts a calendar date to GPS week number and seconds of week.
 * args   : Calender_t* date      I   Pointer to calendar date structure
 *          Week_Sec_t* week_sec  O   Pointer to week/seconds structure
 * return : int                   R   0 on success, 1 on error
 *-------------------------------------------------------------------------------*/
static int calender2weekSecs(calender_t* date, gtime_t* week_sec) {
	const int doy[] = { 1,32,60,91,121,152,182,213,244,274,305,335 };
	if (date->year < 1970 || date->year > 2099 || date->month < 1 || date->month > 12) return 1;
	uint16_t sec_i = (uint16_t)(date->sec/1e3);
	float sec_f = date->sec/1e3 - sec_i;
	/* leap year if year%4==0 in 1901-2099 */
	int days = (date->year - 1970) * 365 + (date->year - 1969) / 4 + doy[date->month - 1] + date->day - 2 + (date->year % 4 == 0 && date->month >= 3 ? 1 : 0);
    long long ll_sec = (long long)days * 86400 + date->hour * 3600 + date->min * 60 + sec_i - 315964800 + 18;
	week_sec->ui_week = (uint32_t)(ll_sec / (86400 * 7));
	week_sec->lf_secs = (ll_sec - week_sec->ui_week * 86400 * 7);
	week_sec->lf_secs += sec_f;
	return 0;
}


/* Parse GGA sentence from GNSS data --------------------------------------------
 * Parses a GGA sentence and updates GNSS information structure.
 * args   : uint8_t ch1   I   Input character
 * return : int           R   1 if complete, -1 on error, 0 otherwise
 * $GPGGA,014840.00,3607.9202030,N,11420.6409278,E,1,28,0.6,75.1078,M,-16.60,M,,*71
 *-------------------------------------------------------------------------------*/
int Recv_gga(uint8_t ch1) {
	int ret = 0;
	switch (flag_parse_gnss) {
	case 0:
		if (ch1 == ',') {
			flag_parse_gnss = 1;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		break;
	case 1:
		if (ch1 == ',') {
			flag_parse_gnss = 2;
			if (index_parse_gnss > 1) {
				double f = str2num((const char*)str_parse, 0, 20);
				pos_int = (int)(f / 100);
				tmp = pos_int + (f - pos_int * 100) / 60.0;
                sx_gnss_info.lat = tmp * DEG;
			}
			else  ret = -1;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		else str_parse[index_parse_gnss++] = ch1;
		break;

	case 2:
		if (ch1 == ',') { flag_parse_gnss = 3; }
		else if (ch1 == 'S') { sx_gnss_info.lat = -1.0 * sx_gnss_info.lat; }
		break;

	case 3:
		if (ch1 == ',') {
			flag_parse_gnss = 4;
			if (index_parse_gnss > 1) {
				double f = str2num((const char*)str_parse, 0, 20);
				pos_int = (int)(f / 100);
				tmp = pos_int + (f - pos_int * 100) / 60.0;
                sx_gnss_info.lon = tmp * DEG;
			}
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;

	case 4:
		if (ch1 == ',') flag_parse_gnss = 5;
		else if (ch1 == 'W')	sx_gnss_info.lon = -1.0 * sx_gnss_info.lon;
		break;

	case 5:
		if (ch1 == ',') 	flag_parse_gnss = 6;
		else sx_gnss_info.fix_type = ch1 - '0';
		break;

	case 6:
		if (ch1 == ',') {
			flag_parse_gnss = 7;
			sx_gnss_info.num_sv = str2num((const char*)str_parse, 0, 10);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;

	case 7:
		if (ch1 == ',') {
			flag_parse_gnss = 8;
			tmp = str2num((const char*)str_parse, 0, 10);
			sx_gnss_info.pos_dop = tmp;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;

	case 8:
		if (ch1 == ',') {
			flag_parse_gnss = 9;
			tmp = str2num((const char*)str_parse, 0, 10);
			sx_gnss_info.height_ellipsoid = tmp;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;

	case 9:
		if (ch1 == ',') {
			flag_parse_gnss = 10;
		}
		break;

	case 10:
		if (ch1 == ',') {
			flag_parse_gnss = 11;
			tmp = str2num((const char*)str_parse, 0, 10);
			sx_gnss_info.height_ellipsoid += tmp;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;

	case 11:
		if (ch1 == ',')
		{
			flag_parse_gnss = 12;
		}
		break;

	case 12:
		if (ch1 == ',') {
            sx_gnss_info.age = (str2num((const char*)str_parse, 0, 10) * 10);
			flag_parse_gnss = 0;
            index_parse_gnss = 0;
			memset(str_parse, '\0', GNSS_MAX_NUM);
			ret = 1;
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	}

	return ret;
}

/* Parse GMC sentence from GNSS data --------------------------------------------
 * Parses a GMC sentence and updates GNSS information structure.
 * args   : uint8_t ch1   I   Input character
 * return : int           R   1 if complete, -1 on error, 0 otherwise
 *        0      1      2        3    4           5    6       7     8       9 10
$GNRMC,061937.00,A,3024.46135200,N,11424.56160238,E,24.0486,222.6154,090426,0.0,E,A*1D
 *-------------------------------------------------------------------------------*/
int Recv_rmc(uint8_t ch1) {
    int ret = 0;
    switch (flag_parse_gnss) {
    case 0:
        if (ch1 == ',') {
            flag_parse_gnss = 1;
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse , '\0' , GNSS_MAX_NUM);
        }
        break;
    case 1:
        if (ch1 == ',') {
            flag_parse_gnss = 2;
        }
        break;
    case 2:
        if (ch1 == ',') {
            flag_parse_gnss = 3;
            if (index_parse_gnss > 1) {
                double f = str2num((const char*)str_parse, 0, 20);
                pos_int = (int)(f / 100);
                tmp = pos_int + (f - pos_int * 100) / 60.0;
                sx_gnss_info.lat = tmp * DEG;
            }
            else  ret = -1;
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse , '\0' , GNSS_MAX_NUM);
        }
        else str_parse[index_parse_gnss++] = ch1;
        break;

    case 3:
        if (ch1 == ',') { flag_parse_gnss = 4; }
        else if (ch1 == 'S') { sx_gnss_info.lat = -1.0 * sx_gnss_info.lat; }
        break;

    case 4:
        if (ch1 == ',') {
            flag_parse_gnss = 5;
            if (index_parse_gnss > 1) {
                double f = str2num((const char*)str_parse, 0, 20);
                pos_int = (int)(f / 100);
                tmp = pos_int + (f - pos_int * 100) / 60.0;
                sx_gnss_info.lon = tmp * DEG;
            }
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse , '\0' , GNSS_MAX_NUM);
        }
        else  str_parse[index_parse_gnss++] = ch1;
        break;

    case 5:
        if (ch1 == ',') flag_parse_gnss = 6;
        else if (ch1 == 'W')	sx_gnss_info.lon = -1.0 * sx_gnss_info.lon;
        break;

    case 6:
        if (ch1 == ',') {
            flag_parse_gnss = 7;
            tmp = str2num((const char*)str_parse, 0, 10);
            tmp *= 0.51444;
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse, '\0', GNSS_MAX_NUM);
        }
        else  str_parse[index_parse_gnss++] = ch1;
        break;
    case 7:
        if (ch1 == ',') {
            flag_parse_gnss = 0;
            double trjMotion = str2num((const char*)str_parse, 0, 10);
            sx_gnss_info.heading_motion = trjMotion * DEG;
            trjMotion *= DEG;
            sx_gnss_info.vel_east  = tmp * sin(trjMotion);
            sx_gnss_info.vel_north = tmp * cos(trjMotion);
            index_parse_gnss = 0;
            ret = 1;
            str_parse[0] = '\0';
            memset(str_parse, '\0', GNSS_MAX_NUM);
        }
        else  str_parse[index_parse_gnss++] = ch1;
        break;
    }
    return ret;
}

/* Parse GST sentence from GNSS data --------------------------------------------
 * Parses a GST sentence and updates accuracy fields in GNSS information.
 * args   : uint8_t ch1   I   Input character
 * return : int           R   1 if complete, 0 otherwise
 * $GPGST,014840.00,2.99,1.36,1.11,-28.8269,1.31,1.17,2.80*75
 *-------------------------------------------------------------------------------*/
int Recv_gst(uint8_t ch1) {
	int ret = 0;
	switch (flag_parse_gnss) {
	case 0:
		if (ch1 == ',')
		{
			flag_parse_gnss = 1;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		break;
	case 1:
		if (ch1 == ',') {
			flag_parse_gnss = 2;
		}
		break;
	case 2:
		if (ch1 == ',') {
			flag_parse_gnss = 3;
		}
		break;
	case 3:
		if (ch1 == ',') {
			flag_parse_gnss = 4;
		}
		break;
	case 4:
		if (ch1 == ',') {
			flag_parse_gnss = 5;
		}
		break;

	case 5:
		if (ch1 == ',') {
			str_parse[index_parse_gnss++] = '\0';
			flag_parse_gnss = 6;
            tmp = str2num((const char*)str_parse, 0, 5);
			sx_gnss_info.accu_lat = tmp;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;

	case 6:
		if (ch1 == ',') {
			str_parse[index_parse_gnss++] = '\0';
			flag_parse_gnss = 7;
            tmp = str2num((const char*)str_parse, 0, 5);
			sx_gnss_info.accu_lon = tmp;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else	str_parse[index_parse_gnss++] = ch1;
		break;

	case 7:
		if (ch1 == '*') {
			str_parse[index_parse_gnss++] = '\0';
            tmp = str2num((const char*)str_parse, 0, 5);
			sx_gnss_info.accu_height = tmp;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
			flag_parse_gnss = 0x00;
			ret = 1;
		}
		else {
			str_parse[index_parse_gnss++] = ch1;
		}
		break;
	}

	return ret;
}
/* Parse ZDA sentence from GNSS data --------------------------------------------
 * Parses a ZDA sentence and updates date/time fields in GNSS information.
 * args   : uint8_t ch1   I   Input character
 * return : int           R   1 if complete, -1 on error, 0 otherwise
 * $GPZDA,014841.00,10,03,2025,,*69
 *-------------------------------------------------------------------------------*/
int Reci_zda(uint8_t ch1) {
	int ret = 0;
	double zda_f;

	switch (flag_parse_gnss) {
    case 15:
        if (ch1 == ',') {
            flag_parse_gnss = 1;
            if (index_parse_gnss > 1) {
                zda_f = str2num((const char*)str_parse, 0, 10);
                sx_gnss_info.date.hour = (uint8_t)(zda_f / 10000);
                sx_gnss_info.date.min = (uint8_t)(zda_f / 100 - (uint32_t)sx_gnss_info.date.hour * 100);
                sx_gnss_info.date.sec = (zda_f - (uint32_t)sx_gnss_info.date.hour * 10000 - (uint32_t)sx_gnss_info.date.min * 100)*1e3;
            }
            else {
                ret = -1;
            }
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse , '\0' , GNSS_MAX_NUM);
        }
        else  str_parse[index_parse_gnss++] = ch1;
        break;
	case 0:
        if (ch1 == ',') {
            flag_parse_gnss = 1;
            if (index_parse_gnss > 1) {
                zda_f = str2num((const char*)str_parse, 0, 10);
                sx_gnss_info.date.hour = (uint8_t)(zda_f / 10000);
                sx_gnss_info.date.min = (uint8_t)(zda_f / 100 - (uint32_t)sx_gnss_info.date.hour * 100);
                sx_gnss_info.date.sec = (zda_f - (uint32_t)sx_gnss_info.date.hour * 10000 - (uint32_t)sx_gnss_info.date.min * 100)*1e3;
            }
            else {
                ret = -1;
            }
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse , '\0' , GNSS_MAX_NUM);
        }
        else
            str_parse[index_parse_gnss++] = ch1;

//        if (ch1 == ',') flag_parse_gnss = 15;
//        else  str_parse[index_parse_gnss++] = ch1;
		break;

	case 1:
		if (ch1 == ',') {
			flag_parse_gnss = 2;
			sx_gnss_info.date.day = str2num((const char*)str_parse, 0, 2);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		else {
			str_parse[index_parse_gnss++] = ch1;
		}
		break;

	case 2:
		if (ch1 == ',') {
			flag_parse_gnss = 3;
			sx_gnss_info.date.month = str2num((const char*)str_parse, 0, 2);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
		}
		else {
			str_parse[index_parse_gnss++] = ch1;
		}
		break;
	case 3:
		if (ch1 == ',') {
			flag_parse_gnss = 0;
			sx_gnss_info.date.year = str2num((const char*)str_parse, 0, 4);
            calender2weekSecs(&sx_gnss_info.date, &sx_gnss_info.week_secs);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse , '\0' , GNSS_MAX_NUM);
			ret = 1;
		}
		else {
			str_parse[index_parse_gnss++] = ch1;
		}
		break;
	default:
		flag_parse_gnss = 0;
	}

	return ret;
}

/* Parse Heading sentence from GNSS data --------------------------------------------
* Parses a heading sentence and updates date/time fields in GNSS information.
* args   : uint8_t ch1   I   Input character
* return : int           R   1 if complete, -1 on error, 0 otherwise
* note:  :
*             0  1  2     3    4      5         6   7   8          8        9         10    11     12     13       14
* #HEADINGA,COM2,0,23.0,FINE,2376,266819.200,112748,24,18;SOL_COMPUTED,NARROW_INT,0.7935,309.5608,-1.6492,0.0000,0.2902,0.7118,"999",52,46,46,27,3,00,3,f3*91c43d2c
*-------------------------------------------------------------------------------*/
int Reci_hea(uint8_t ch1) {
	int ret = 0;

	switch (flag_parse_gnss) {
	case 0:
		if (ch1 == ',') {
			flag_parse_gnss = 1;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		break;

	case 1:
		if (ch1 == ',') {
			flag_parse_gnss = 2;
		}
		break;

	case 2:
		if (ch1 == ',') {
			flag_parse_gnss = 3;
		}
		break;
	case 3:
		if (ch1 == ',') {
			flag_parse_gnss = 4;
		}
		break;
	case 4:
		if (ch1 == ',') {
			flag_parse_gnss = 5;
		}
		break;
	case 5:
		if (ch1 == ',') {
			flag_parse_gnss = 6;
			sx_gnss_info.week_secs.lf_secs = str2num((const char*)str_parse, 0, 10);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 6:
		if (ch1 == ',') {
			flag_parse_gnss = 7;
		}
		break;
	case 7:
		if (ch1 == ',') {
			flag_parse_gnss = 8;
		}
		break;
	case 8:
		if (ch1 == ',') {
			flag_parse_gnss = 9;
		}
		break;
	case 9:
		if (ch1 == ',') {
			flag_parse_gnss = 10;
		}
		break;
	case 10:
		if (ch1 == ',') {
			flag_parse_gnss = 11;
			tmp = str2num((const char*)str_parse, 0, 10);
			sx_gnss_info.baseline = tmp;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 11:
		if (ch1 == ',') {
			flag_parse_gnss = 12;
			tmp = str2num((const char*)str_parse, 0, 10);
            sx_gnss_info.heading = tmp * DEG;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;

		break;
	case 12:
		if (ch1 == ',') {
			flag_parse_gnss = 13;
		}
		break;
	case 13:
		if (ch1 == ',') {
			flag_parse_gnss = 14;
		}
		break;
	case 14:
		if (ch1 == ',') {
			flag_parse_gnss = 0;
			tmp = str2num((const char*)str_parse, 0, 10);
			sx_gnss_info.accu_heading = tmp;
			index_parse_gnss = 0;
			ret = 1;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	default:
		flag_parse_gnss = 0;
	}

	return ret;
}

/* Parse BESTVEL sentence from GNSS data --------------------------------------------
* Parses a vel sentence and updates date/time fields in GNSS information.
* args   : uint8_t ch1   I   Input character
* return : int           R   1 if complete, -1 on error, 0 otherwise
* note:  :
*             0  1  2       3           4      5         6       7    8          8        9               10    11     12     13       14
* #BESTVELA,COM1,0,57.0,COARSESTEERING,2357,92938.000,02000000,10a2,17136;SOL_COMPUTED,DOPPLER_VELOCITY,0.000,0.000,15.9625,2.988786,0.0026,0*bf513cf7
*-------------------------------------------------------------------------------*/
int Reci_vel(uint8_t ch1) {
	int ret = 0;

	switch (flag_parse_gnss) {
	case 0:
		if (ch1 == ',') {
			flag_parse_gnss = 1;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		break;

	case 1:
		if (ch1 == ',') {
			flag_parse_gnss = 2;
		}
		break;

	case 2:
		if (ch1 == ',') {
			flag_parse_gnss = 3;
		}
		break;
	case 3:
		if (ch1 == ',') {
			flag_parse_gnss = 4;
		}
		break;
	case 4:
        if (ch1 == ',') {
            flag_parse_gnss = 5;
            sx_gnss_info.week_secs.ui_week = str2num((const char*)str_parse, 0, 10);
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse, '\0', GNSS_MAX_NUM);
        }
        else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 5:
		if (ch1 == ',') {
			flag_parse_gnss = 6;
			sx_gnss_info.week_secs.lf_secs = str2num((const char*)str_parse, 0, 10);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 6:
		if (ch1 == ',') {
			flag_parse_gnss = 7;
		}
		break;
	case 7:
		if (ch1 == ',') {
			flag_parse_gnss = 8;
		}
		break;
	case 8:
		if (ch1 == ',') {
			flag_parse_gnss = 9;
		}
		break;
	case 9:
		if (ch1 == ',') {
			flag_parse_gnss = 10;
		}
		break;
	case 10:
		if (ch1 == ',') {
			flag_parse_gnss = 11;
		}
		break;
	case 11:
		if (ch1 == ',') {
			flag_parse_gnss = 12;
		}
		break;
	case 12:
		if (ch1 == ',') {
			flag_parse_gnss = 13;
			tmp = str2num((const char*)str_parse, 0, 10);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 13:
		if (ch1 == ',') {
			flag_parse_gnss = 14;
			double trjMotion = str2num((const char*)str_parse, 0, 10);
			sx_gnss_info.heading_motion = trjMotion * DEG;
			trjMotion *= DEG;
			//trjMotion = C360CC180(trjMotion);
			sx_gnss_info.vel_east  = tmp * sin(trjMotion);
			sx_gnss_info.vel_north = tmp * cos(trjMotion);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 14:
		if (ch1 == ',') {
			flag_parse_gnss = 0;
			tmp = str2num((const char*)str_parse, 0, 10);
            sx_gnss_info.vel_up = tmp;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
			flag_parse_gnss = 0;
			ret = 1;
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	default:
		flag_parse_gnss = 0;
	}

	return ret;
}

/* Parse PQTMPVT sentence from GNSS data --------------------------------------------
* Parses a vel sentence and updates date/time fields in GNSS information.
* args   : uint8_t ch1   I   Input character
* return : int           R   1 if complete, -1 on error, 0 otherwise
* note:  :
*         0  1           2       3        4 5 6   7    8          9            10       11    12     13     14    15   16     17     18      
*$PQTMPVT,1,436825800,20260130,012007.800,2,3,45,18,30.44500935,114.39689686,39.218,-13.744,-0.003,0.000,-0.002,0.003,179.32,0.31,0.59*73
*-------------------------------------------------------------------------------*/
int Reci_PVT(uint8_t ch1) {
	int ret = 0;

	switch (flag_parse_gnss) {
	case 0:
		if (ch1 == ',') {
			flag_parse_gnss = 1;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		break;

	case 1:
		if (ch1 == ',') {
			flag_parse_gnss = 2;
		}
		break;

	case 2:
		if (ch1 == ',') {
			flag_parse_gnss = 3;
		}
		break;
	case 3:
		if (ch1 == ',') {
			flag_parse_gnss = 4;
		}
		break;
	case 4:
		if (ch1 == ',') {
			flag_parse_gnss = 5;
		}
		break;
	case 5:
		if (ch1 == ',') {
			flag_parse_gnss = 6;
		}
		break;
	case 6:
		if (ch1 == ',') {
			flag_parse_gnss = 7;
		}
		break;
	case 7:
		if (ch1 == ',') {
			flag_parse_gnss = 8;
		}
		break;
	case 8:
		if (ch1 == ',') {
			flag_parse_gnss = 9;
		}
		break;
	case 9:
		if (ch1 == ',') {
			flag_parse_gnss = 10;
		}
		break;
	case 10:
		if (ch1 == ',') {
			flag_parse_gnss = 11;
		}
		break;
	case 11:
		if (ch1 == ',') {
			flag_parse_gnss = 12;
		}
		break;
	case 12:
		if (ch1 == ',') {
			flag_parse_gnss = 13;
			sx_gnss_info.vel_north = str2num((const char*)str_parse, 0, 10);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 13:
		if (ch1 == ',') {
			flag_parse_gnss = 14;
			sx_gnss_info.vel_east = str2num((const char*)str_parse, 0, 10);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 14:
		if (ch1 == ',') {
			flag_parse_gnss = 0;
            sx_gnss_info.vel_up = -str2num((const char*)str_parse, 0, 10);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
			ret = 1;
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	//case 14:
	//	if (ch1 == ',') {
	//		flag_parse_gnss = 0;
	//	}
	//	break;
	default:
		flag_parse_gnss = 0;
	}

	return ret;
}

/* Parse PQTMTAR sentence from GNSS data --------------------------------------------
* Parses a vel sentence and updates date/time fields in GNSS information.
* args   : uint8_t ch1   I   Input character
* return : int           R   1 if complete, -1 on error, 0 otherwise
* note:  :
*         0  1         2 3   4     5     6   7          8     9     10   11
*$PQTMTAR,1,012007.800,4,,1.270,-0.389622,,224.738703,0.228066,,0.072529,45*45
*-------------------------------------------------------------------------------*/
int Reci_TAR(uint8_t ch1) {
	int ret = 0;

	switch (flag_parse_gnss) {
	case 0:
		if (ch1 == ',') {
			flag_parse_gnss = 1;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		break;
	case 1:
		if (ch1 == ',') {
			flag_parse_gnss = 2;
		}
		break;

	case 2:
		if (ch1 == ',') {
			flag_parse_gnss = 3;
		}
		break;
	case 3:
		if (ch1 == ',') {
			flag_parse_gnss = 4;
		}
		break;
	case 4:
		if (ch1 == ',') {
			flag_parse_gnss = 5;
		}
		break;
	case 5:
		if (ch1 == ',') {
			flag_parse_gnss = 6;
		}
		break;
	case 6:
		if (ch1 == ',') {
			flag_parse_gnss = 7;
		}
		break;
	case 7:
		if (ch1 == ',') {
			flag_parse_gnss = 8;
			sx_gnss_info.heading = str2num((const char*)str_parse, 0, 10);
			sx_gnss_info.heading *= DEG;
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;
	case 8:
		if (ch1 == ',') {
			flag_parse_gnss = 9;
		}
		break;
	case 9:
		if (ch1 == ',') {
			flag_parse_gnss = 10;
		}
		break;
	case 10:
		if (ch1 == ',') {
			flag_parse_gnss = 0;
			sx_gnss_info.accu_heading = str2num((const char*)str_parse, 0, 10);
			index_parse_gnss = 0;
			str_parse[0] = '\0';
			memset(str_parse, '\0', GNSS_MAX_NUM);
			ret = 1;
		}
		else  str_parse[index_parse_gnss++] = ch1;
		break;

	default:
		flag_parse_gnss = 0;
	}

	return ret;
}

/* Calculate UBLOX message checksum ---------------------------------------------
 * Calculates checksum A and B for UBLOX message validation.
 * args   : uint8_t* buff   I   Pointer to message buffer
 *          int len         I   Length of buffer
 *          uint8_t* chk_a  O   Pointer to checksum A
 *          uint8_t* chk_b  O   Pointer to checksum B
 * return : none
 *-------------------------------------------------------------------------------*/
void checksum(uint8_t* buff, int len, uint8_t* chk_a, uint8_t* chk_b)
{
	int i;
	for (i = 0; i < len; i++) {
		*chk_a = *chk_a + buff[i];
		*chk_b = *chk_b + *chk_a;
	}
	return;
}

/* Latest assembled UBLOX PVT buffer ----------------------------------------
 * Holds the most recently-received UBX-NAV-PVT message bytes copied into
 * a structured view. Updated by `Recv_pvt()` when a full PVT message is
 * received and validated.
 *-------------------------------------------------------------------------------*/
static UBLOX_PVT_t sx_pvt = { 0 };
/* Parse UBLOX PVT message ------------------------------------------------------
 * Parses a UBLOX PVT message and updates PVT structure.
 * args   : uint8_t ch1   I   Input character
 * return : int           R   1 if complete, 2 if checksum error, 0 otherwise
 *-------------------------------------------------------------------------------*/
int Recv_pvt(uint8_t ch1) {
	int ret = 0;
	switch (flag_parse_gnss) {
	case 0:
		if (ch1 == 92) {
			flag_parse_gnss = 1;
			str_parse[0] = '\0';
			memset(str_parse, 0, 100);
			str_parse[0] = 0x01;
			str_parse[1] = 0x07;
			str_parse[2] = 92;
			index_parse_gnss = 0;
		}
		break;
	case 1:
		str_parse[index_parse_gnss + 3] = ch1;
		if (index_parse_gnss++ >= 92) {
			flag_parse_gnss = 2;
		}
		break;
	case 2:
		//chk_a = 0, chk_b = 0;
		//checksum(str_parse, 96, &chk_a, &chk_b);
		//if (chk_a == ch1) {
		//	flag_parse_gnss = 3;
		//}
		//else {
		//	ret = 2;
		//	flag_parse_gnss = 0;
		//}
		flag_parse_gnss = 3;
		break;
	case 3:
		memcpy(&sx_pvt, str_parse, sizeof(UBLOX_PVT_t));
		ret = 1;
		flag_parse_gnss = 0;
		break;
	default:
		flag_parse_gnss = 0;
	}
	return ret;
}

/* Latest assembled UBLOX REL buffer ----------------------------------------
 * Holds the most recently-received UBX-NAV-RELPOS message bytes copied into
 * a structured view. Updated by `Recv_rel()` when a full REL message is
 * received and validated.
 *-------------------------------------------------------------------------------*/
static UBLOX_REL_t sx_rel = { 0 };
/* Parse UBLOX REL message ------------------------------------------------------
 * Parses a UBLOX REL message and updates REL structure.
 * args   : uint8_t ch1   I   Input character
 * return : int           R   1 if complete, 2 if checksum error, 0 otherwise
 *-------------------------------------------------------------------------------*/
int Recv_rel(uint8_t ch1) {
	int ret = 0;
	//	static uint8_t chk_a, chk_b;
	switch (flag_parse_gnss) {
	case 0:
		if (ch1 == 64) {
			flag_parse_gnss = 1;
			str_parse[0] = '\0';
			//			memset(str_parse, 0, 100);
			str_parse[0] = 0x01;
			str_parse[1] = 0x3c;
			str_parse[2] = 64;

			index_parse_gnss = 0;
		}
		break;
	case 1:
		str_parse[index_parse_gnss + 3] = ch1;
		if (index_parse_gnss++ >= 64) {
			flag_parse_gnss = 2;
		}
		break;
	case 2:
		//chk_a = 0, chk_b = 0;
		//checksum(str_parse, 68, &chk_a, &chk_b);
		//if (chk_a == ch1) {
		//	flag_parse_gnss = 3;
		//}
		//else {
		//	ret = 2;
		//	flag_parse_gnss = 0;
		//}
		flag_parse_gnss = 3;
		break;
	case 3:
		memcpy(&sx_rel, str_parse, sizeof(UBLOX_REL_t));
		ret = 1;
		flag_parse_gnss = 0;
		break;
	default:
		flag_parse_gnss = 0;
	}

	return ret;
}

/* Latest assembled UBLOX REL buffer ----------------------------------------
 * Holds the most recently-received UBX-NAV-RELPOS message bytes copied into
 * a structured view. Updated by `Recv_rel()` when a full REL message is
 * received and validated.
 *-------------------------------------------------------------------------------*/
static G5_SBF_t sx_BSF = { 0 };
/* Parse UBLOX REL message ------------------------------------------------------
 * Parses a UBLOX REL message and updates REL structure.
 * args   : uint8_t ch1   I   Input character
 * return : int           R   1 if complete, 2 if checksum error, 0 otherwise
 *-------------------------------------------------------------------------------*/
int Recv_SBF(uint8_t ch1) {
    int ret = 0;
    str_parse[index_parse_gnss++] = ch1;
    switch (flag_parse_gnss) {
    case 0:
        if (index_parse_gnss > 6) {
            memcpy(&sx_BSF,str_parse,6);
            switch(sx_BSF.id_num){
                case 4007:
                    flag_parse_gnss = 1;
                break;
                default:
                    flag_parse_gnss = 0;
                    ret = 2;
                break;
            }
        }
        break;
    case 1:
        if (index_parse_gnss == sx_BSF.msg_length - 2) {
            memcpy(&sx_BSF,str_parse,sx_BSF.msg_length);
            ret = 1;
            flag_parse_gnss = 0;
        }
        break;
    case 2:

        break;
    case 3:

        break;
    default:
        flag_parse_gnss = 0;
    }

    return ret;
}

/* Parse PPSN sentence from GNSS data --------------------------------------------
* Parses a vel sentence and updates date/time fields in GNSS information.
* args   : uint8_t ch1   I   Input character
* return : int           R   1 if complete, -1 on error, 0 otherwise
* note:  :
*        0      1      2        3  4   5     6  7   8   9  10 11
* $PSSN,HRP,100310.80,090226,44.748,,0.412,0.143,,0.365,34,2,4.865,W*2B
*-------------------------------------------------------------------------------*/
int Reci_PSSN(uint8_t ch1) {
    int ret = 0;

    switch (flag_parse_gnss) {
    case 0:
        if (ch1 == ',') {
            flag_parse_gnss = 1;
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse, '\0', GNSS_MAX_NUM);
        }
        break;
    case 1:
        if (ch1 == ',') {
            flag_parse_gnss = 2;
        }
        break;

    case 2:
        if (ch1 == ',') {
            flag_parse_gnss = 3;
        }
        break;
    case 3:
        if (ch1 == ',') {
            flag_parse_gnss = 4;
            sx_gnss_info.heading = str2num((const char*)str_parse, 0, 10);
            sx_gnss_info.heading *= DEG;
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse, '\0', GNSS_MAX_NUM);
        }
        else  str_parse[index_parse_gnss++] = ch1;
        break;
    case 4:
        if (ch1 == ',') {
            flag_parse_gnss = 5;
        }
        break;
    case 5:
        if (ch1 == ',') {
            flag_parse_gnss = 6;
        }
        break;
    case 6:
        if (ch1 == ',') {
            flag_parse_gnss = 0;
            sx_gnss_info.accu_heading = str2num((const char*)str_parse, 0, 10);
            index_parse_gnss = 0;
            str_parse[0] = '\0';
            memset(str_parse, '\0', GNSS_MAX_NUM);
            ret = 1;
        }
        else  str_parse[index_parse_gnss++] = ch1;
        break;

    default:
        flag_parse_gnss = 0;
    }

    return ret;
}

/* GNSS data parser state machine -----------------------------------------------
 * Main state machine for parsing incoming GNSS data streams.
 * - `ch_0`: stores the previously-received byte and is used by the
 *           state detection code to recognize multi-character frame
 *           headers (e.g. 'GP', 'BE', UBX sync sequences).
 * args   : uint8_t ch1   I   Input character
 * return : none
 *-------------------------------------------------------------------------------*/
void gnss_parse(uint8_t ch1)
{
	int ret;
	switch (Rflag)
	{
	case 0:
        if (CHECK_GN_GP(ch_0,ch1)  || CHECK_UM_BEST(ch_0,ch1) ||
                CHECK_UM_HEA(ch_0,ch1) || CHECK_YY_PQ(ch_0,ch1) ||
                CHECK_G5_PSSN(ch_0,ch1)          )
		{
			Rflag = 1;
			num = 0;
			check_parse_nema = 0;
		}
        else if ( CHECK_UBLOX_HEAD(ch_0,ch1))
		{
            num = 0;
            Rflag = 11;
            index_parse_gnss = 0;
		}
        else if (CHECK_G5_SBF(ch_0,ch1))
        {
            num = 0;
            index_parse_gnss = 0;
            Rflag = 21;
        }

		ch_0 = ch1;
		break;
	case 1:
		check_parse_nema ^= ch1;
		if (ch1 == ',')
		{
            /*RMC*/
            if (Frame_name[1] == 'M' && Frame_name[2] == 'C') {
                Rflag = 2;
            }
			/*GST*/
            else if (Frame_name[1] == 'S' && Frame_name[2] == 'T') {
				Rflag = 3;
			}
			/*GGA*/
			else if (Frame_name[1] == 'G' && Frame_name[2] == 'A') {
				Rflag = 4;
			}
			/*BESTVEL*/
            else if (Frame_name[1] == 'T' && (Frame_name[2] == 'V' || Frame_name[6] == 'V')) {
				Rflag = 5;
			}
			/*ZDA*/
			else if (Frame_name[1] == 'D' && Frame_name[2] == 'A') {
				Rflag = 6;
			}
			/*VTG*/
//			else if (Frame_name[1] == 'T' && Frame_name[2] == 'G') {
//				Rflag = 7;
//			}
			/*HEADINGA*/
			else if (Frame_name[0] == 'A' && Frame_name[1] == 'D') {
				Rflag = 8;
			}
			/*TMPVT*/
			else if (Frame_name[2] == 'P' && Frame_name[3] == 'V') {
				Rflag = 9;
			}
			/*TMTAR*/
			else if (Frame_name[2] == 'T' && Frame_name[3] == 'A') {
				Rflag = 10;
			}

            /*PSSN*/
            else if (Frame_name[0] == 'S' && Frame_name[1] == 'N') {
                Rflag = 22;
            }
			else {
				Rflag = 0;
			}
			num = 0;
		}
		else {
			Frame_name[num++] = ch1;
			if (num > 10) {
				Rflag = 0;
				num = 0;
				check_parse_nema = 0;
			}
		}
		break;

    case 2:
        check_parse_nema ^= ch1;
        ret = Recv_rmc(ch1);
        if (1 == ret) {
            Rflag = 0;
            sx_gnss_info.GNSSFlag |= GNSSPARSE_RMC;
            sx_gnss_info.last = GNSSPARSE_RMC;
        }
        else if (-1 == ret) {
            Rflag = 0;
        }break;

	case 3:
		check_parse_nema ^= ch1;
		ret = Recv_gst(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_GST;
            sx_gnss_info.last = GNSSPARSE_GST;
		}
		else if (-1 == ret) {
			Rflag = 0;
		}break;
	case 4:
		check_parse_nema ^= ch1;
		ret = Recv_gga(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_GGA;
            sx_gnss_info.last = GNSSPARSE_GGA;
		}
		else if (-1 == ret) {
			Rflag = 0;
		}break;
	case 5:
		check_parse_nema ^= ch1;
		ret = Reci_vel(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_VEL;
            sx_gnss_info.last = GNSSPARSE_VEL;
		}
		else if (-1 == ret) {
			Rflag = 0;
		}break;
	case 6:
		check_parse_nema ^= ch1;
		ret = Reci_zda(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_ZDA;
            sx_gnss_info.last = GNSSPARSE_ZDA;
		}
		else if (-1 == ret) {
			Rflag = 0;
		}break;
	case 8:
		check_parse_nema ^= ch1;
		ret = Reci_hea(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_HEA;
            sx_gnss_info.last = GNSSPARSE_HEA;
		}
		else if (-1 == ret) {
			Rflag = 0;
		}break;
	case 9:
		check_parse_nema ^= ch1;
		ret = Reci_PVT(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_PVT;
			sx_gnss_info.last = GNSSPARSE_PVT;
		}
		else if (-1 == ret) {
			Rflag = 0;
		}break;
	case 10:
		check_parse_nema ^= ch1;
		ret = Reci_TAR(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_TAR;
			sx_gnss_info.last = GNSSPARSE_TAR;
		}
		else if (-1 == ret) {
			Rflag = 0;
		}break;
	case 11:
		Frame_name[num++] = ch1;
		if (num >= 2) {
            if (CHECK_UBLOX_PVT(Frame_name[0],Frame_name[1])){
				Rflag = 12;
			}
            else if (CHECK_UBLOX_REL(Frame_name[0],Frame_name[1])) {
				Rflag = 13;
			}
			else {
				Rflag = 0;
			}
		}

		break;
	case 12:
		ret = Recv_pvt(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_PVT;
            sx_gnss_info.last = GNSSPARSE_PVT;

			if (sx_pvt.year > 2000) {
				sx_gnss_info.date.year = sx_pvt.year;
				sx_gnss_info.date.month = sx_pvt.month;
				sx_gnss_info.date.day = sx_pvt.day;
				sx_gnss_info.date.hour = sx_pvt.hour;
				sx_gnss_info.date.min = sx_pvt.min;
                sx_gnss_info.date.sec = sx_pvt.sec*1e3;
			}

			sx_gnss_info.lat = sx_pvt.lat * 1e-7 * DEG;
			sx_gnss_info.lon = sx_pvt.lon * 1e-7 * DEG;
			sx_gnss_info.height_ellipsoid = sx_pvt.height * 1e-3;
			sx_gnss_info.pos_dop = sx_pvt.pdop * 1e-2;
			sx_gnss_info.num_sv = sx_pvt.numSv;

			sx_gnss_info.vel_east = sx_pvt.vele * 1e-3;
			sx_gnss_info.vel_north = sx_pvt.veln * 1e-3;
            sx_gnss_info.vel_up = -sx_pvt.veld * 1e-3;

			sx_gnss_info.heading = sx_rel.relposHeading * 1e-5 * DEG;
			sx_gnss_info.accu_heading = sx_rel.accHeading * 1e-5;

			calender2weekSecs(&sx_gnss_info.date, &sx_gnss_info.week_secs);

			if (sx_pvt.fixtype) {
				sx_gnss_info.fix_type = 1;//s_gpsinfo.pvt.fixtype;
			}
			else {
				sx_gnss_info.fix_type = 0;
			}

		}
		else if (2 == ret) {
			Rflag = 0;
		}
		break;
	case 13:
		ret = Recv_rel(ch1);
		if (1 == ret) {
			Rflag = 0;
			sx_gnss_info.GNSSFlag |= GNSSPARSE_REL;
            sx_gnss_info.last = GNSSPARSE_REL;
		}
		else if (2 == ret) {
			Rflag = 0;
		}
		break;
    case 21:
        ret = Recv_SBF(ch1);
        if (1 == ret) {
            sx_gnss_info.vel_east = sx_BSF.ve;
            sx_gnss_info.vel_north = sx_BSF.vn;
            sx_gnss_info.vel_up = sx_BSF.vu;

            Rflag = 0;
            sx_gnss_info.GNSSFlag |= GNSSPARSE_SBF4007;
            sx_gnss_info.last = GNSSPARSE_SBF4007;
        }
        else if (2 == ret) {
            Rflag = 0;
        }
        break;
    case 22:
        check_parse_nema ^= ch1;
        ret = Reci_PSSN(ch1);
        if (1 == ret) {
            Rflag = 0;
            sx_gnss_info.GNSSFlag |= GNSSPARSE_PSSN;
            sx_gnss_info.last = GNSSPARSE_PSSN;
        }
        else if (-1 == ret) {
            Rflag = 0;
        }break;
        break;
    default:

		Rflag = 0;
        break;
	}

	if (GNSSPARSE_PARSE_NOW == sx_gnss_info.GNSSFlag & 0777)
	{
		//isr_sync_gnss();
        //sx_gnss_info.GNSSFlag = 0;
	}

	return;
}

/* GNSS_get -----------------------------------------------------------------
 * return pointer to the most recently parsed GNSS information
 * args   : none
 * return : GNSS_Info_t*  R   pointer to internal GNSS info struct
 *-----------------------------------------------------------------------------*/
gnss_info_t* gnss_get(void) {
	memset(&sx_gnss_info, 0, sizeof(sx_gnss_info));
	flag_parse_gnss = 0;
	index_parse_gnss = 0;
	memset(Frame_name,0,sizeof(Frame_name));
	return &sx_gnss_info;
}

void add_gnss(gnss_info_t* gnss_seg, gnss_t* gnss_data)
{
    gnss_info_t* px_gnss;
    if (gnss_data->cnt_max <= gnss_data->cnt)
    {
        if (gnss_data->cnt_max <= 0){
            gnss_data->cnt_max = 128;
            if (!(px_gnss = (gnss_info_t*)malloc(sizeof(gnss_info_t) * gnss_data->cnt_max))) return;
        }
        else{
            gnss_data->cnt_max *= 2;
            if (!(px_gnss = (gnss_info_t*)realloc(gnss_data->px_gnss_data, sizeof(gnss_info_t) * gnss_data->cnt_max)))
            {
                free(gnss_data->px_gnss_data);
                gnss_data->cnt_max = 0; gnss_data->cnt = 0;
                return ;
            }
        }
        gnss_data->px_gnss_data = px_gnss;
    }
    gnss_data->px_gnss_data[gnss_data->cnt++] = *gnss_seg;
}


void add_imu(imu_info_t* imu_seg,imu_t* imu_data)
{
    imu_info_t* px_imu;
    if (imu_data->cnt_max <= imu_data->cnt)
    {
        if (imu_data->cnt_max <= 0){
            imu_data->cnt_max = 128;
            if (!(px_imu = (imu_info_t*)malloc(sizeof(imu_info_t) * imu_data->cnt_max))) return;
        }
        else{
            imu_data->cnt_max *= 2;
            if (!(px_imu = (imu_info_t*)realloc(imu_data->px_imu_data, sizeof(imu_info_t) * imu_data->cnt_max)))
            {
                free(imu_data->px_imu_data);
                imu_data->cnt_max = 0; imu_data->cnt = 0;
                return ;
            }
        }
        imu_data->px_imu_data = px_imu;
    }
    imu_data->px_imu_data[imu_data->cnt++] = *imu_seg;
}






