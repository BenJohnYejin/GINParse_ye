

#include "app_interface.h"
#include <string.h>
#include <stdlib.h>

#ifndef _WIN32
#define FSEEK fseeko
#define FTELL ftello
#else
#define FSEEK _fseeki64
#define FTELL _ftelli64
#endif

#ifdef TRACE
#include <stdio.h>
#include <stdint.h>
#include <stdarg.h>
static FILE* fp_trace = NULL;     /* file pointer of trace */
static char file_trace[1024];   /* trace file */
static int level_trace = 0;       /* level of trace */
static uint32_t tick_trace = 0;   /* tick time at traceopen (ms) */

/** trace log function */
/* open trace log file ---------------------------------------------------------
* open a trace log file for writing debug or trace information
* args   : const char* file   I   file name to open for trace logging
* return : none
*-----------------------------------------------------------------------------*/
void traceopen(const char* file)
{
	if ((fp_trace = fopen(file, "wb"))) {
		printf("log open -->\n");
	}
	else {
		printf("log not open -->\n");
	}
	//if (!(fp_trace = fopen(file, "wb"))) fp_trace = stderr;
	//strcpy(file_trace, file);
}

/* close trace log file --------------------------------------------------------
* close the currently open trace log file
* args   : none
* return : none
*-----------------------------------------------------------------------------*/
void traceclose(void)
{
	if (fp_trace) fclose(fp_trace);
	fp_trace = NULL;
	file_trace[0] = '\0';
}

/* set trace log level ---------------------------------------------------------
* set the verbosity level for trace logging
* args   : int        level   I   trace level (higher means more verbose)
* return : none
*-----------------------------------------------------------------------------*/
void tracelevel(int level)
{
	level_trace = level;
}

int gettracelevel(void)
{
	return level_trace;
}

/* write trace log message -----------------------------------------------------
* write a formatted message to the trace log at the specified level
* args   : int        level   I   trace level
*          const char* format I   printf-style format string
*          ...                I   variable arguments
* return : none
*-----------------------------------------------------------------------------*/
void trace(int level, const char* format, ...)
{
	va_list ap;

	/* print error message to stderr */
	if (level <= 1) {
		//        va_start(ap, format); vfprintf(stderr, format, ap); va_end(ap);
	}
	if (!fp_trace || level > level_trace) return;
	va_start(ap, format);
	vfprintf(fp_trace, format, ap);
	va_end(ap);
	fflush(fp_trace);
}
#else
void traceopen(const char* file) {}
void traceclose(void) {}
void tracelevel(int level) {}
void trace(int level, const char* format, ...) {}
void tracet(int level, const char* format, ...) {}
#endif

#ifdef CONFIGSIM
#include "ctype.h"
/* discard space characters at tail ------------------------------------------*/
static void chop(char* str)
{
	char* p;
	if ((p = strchr(str, '#'))) *p = '\0'; /* comment */
	for (p = str + strlen(str) - 1; p >= str && !isgraph((int)*p); p--) *p = '\0';
}
/* enum to string ------------------------------------------------------------*/
static int enum2str(char* s, const char* comment, int val)
{
	char str[32], * p, * q, com[1024];
	int n;

	strcpy(com, comment);
	n = sprintf(str, "%d:", val);
	if (!(p = strstr(com, str))) {
		return sprintf(s, "%d", val);
	}
	if (!(q = strchr(p + n, ',')) && !(q = strchr(p + n, ')'))) {
		strcpy(s, p + n);
		return (int)strlen(p + n);
	}
	strncpy(s, p + n, q - p - n); s[q - p - n] = '\0';
	return (int)(q - p - n);
}
/* string to enum ------------------------------------------------------------*/
static int str2enum(const char* str, const char* comment, int* val)
{
	const char* p;
	char s[32];

	for (p = comment;; p++) {
		if (!(p = strstr(p, str))) break;
		if (*(p - 1) != ':') continue;
		for (p -= 2; '0' <= *p && *p <= '9'; p--);
		return sscanf(p + 1, "%d", val) == 1;
	}
	sprintf(s, "%30.30s:", str);
	if ((p = strstr(comment, s))) { /* number */
		return sscanf(p, "%d", val) == 1;
	}
	return 0;
}

/* search option ---------------------------------------------------------------
* search option record
* args   : char   *name     I  option name
*          opt_t  *opts     I  options table
*                              (terminated with table[i].name="")
* return : option record (NULL: not found)
*-----------------------------------------------------------------------------*/
CONFIG_t* searchopt(const char* name, const CONFIG_t* opts)
{
	int i;

	for (i = 0; *opts[i].name; i++) {
		if (strstr(opts[i].name, name)) return (CONFIG_t*)(opts + i);
	}
	return NULL;
}
/* string to option value ------------------------------------------------------
* convert string to option value
* args   : opt_t  *opt      O  option
*          char   *str      I  option value string
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int str2opt(CONFIG_t* opt, const char* str)
{
	switch (opt->format) {
	case 0: *(int*)opt->var = atoi(str); break;
	case 1: *(double*)opt->var = atof(str); break;
	case 2: strcpy((char*)opt->var, str);  break;
	case 3: return str2enum(str, opt->comment, (int*)opt->var);
	case 4: *(float*)opt->var = atof(str); break;
	case 5: *(short*)opt->var = atoi(str); break;
	case 6: *(uint8_t*)opt->var = atoi(str); break;
	default: return 0;
	}
	return 1;
}
/* option value to string ------------------------------------------------------
* convert option value to string
* args   : opt_t  *opt      I  option
*          char   *str      O  option value string
* return : length of output string
*-----------------------------------------------------------------------------*/
int config2str(const CONFIG_t* opt, char* str)
{
	char* p = str;
	switch (opt->format) {
	case 0: p += sprintf(p, "%d", *(int*)opt->var); break;
	case 1: p += sprintf(p, "%6.3f", *(double*)opt->var); break;
	case 2: p += sprintf(p, "%s", (char*)opt->var); break;
	case 3: p += enum2str(p, opt->comment, *(int*)opt->var); break;
	case 4: p += sprintf(p, "%6.3f", *(float*)opt->var); break;
	case 5: p += sprintf(p, "%d", *(short*)opt->var); break;
	case 6: p += sprintf(p, "%d", *(uint8_t*)opt->var); break;
	}
	return (int)(p - str);
}
/* option to string -------------------------------------------------------------
* convert option to string (keyword=value # comment)
* args   : opt_t  *opt      I  option
*          char   *buff     O  option string
* return : length of output string
*-----------------------------------------------------------------------------*/
int config2buf(const CONFIG_t* opt, char* buff)
{
	char* p = buff;
	int n;
	p += sprintf(p, "%-18s =", opt->name);
	p += config2str(opt, p);
	if (*opt->comment) {
		if ((n = (int)(buff + 30 - p)) > 0) p += sprintf(p, "%*s", n, "");
		p += sprintf(p, " # (%s)", opt->comment);
	}
	return (int)(p - buff);
}
/* load options ----------------------------------------------------------------
* load options from file
* args   : char   *file     I  options file
*          opt_t  *opts     IO options table
*                              (terminated with table[i].name="")
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int loadconfigs(const char* file, CONFIG_t* opts)
{
	FILE* fp;
	CONFIG_t* opt;
	char buff[2048], * p;
	int n = 0;

	if (!(fp = fopen(file, "r"))) {
		return 0;
	}
	while (fgets(buff, sizeof(buff), fp)) {
		n++;
		chop(buff);

		if (buff[0] == '\0') continue;

		if (!(p = strstr(buff, "="))) {
			fprintf(stderr, "invalid option %s (%s:%d)\n", buff, file, n);
			continue;
		}
		*p++ = '\0';
		chop(buff);
		if (!(opt = searchopt(buff, opts))) continue;

		if (!str2opt(opt, p)) {
			fprintf(stderr, "invalid option value %s (%s:%d)\n", buff, file, n);
			continue;
		}
	}
	fclose(fp);

	return 1;
}

/* save options to file --------------------------------------------------------
* save options to file
* args   : char   *file     I  options file
*          char   *mode     I  write mode ("w":overwrite,"a":append);
*          char   *comment  I  header comment (NULL: no comment)
*          opt_t  *opts     I  options table
*                              (terminated with table[i].name="")
* return : status (1:ok,0:error)
*-----------------------------------------------------------------------------*/
int saveconfigs(const char* file, const char* mode, const char* comment, const CONFIG_t* opts)
{
	FILE* fp;
	char buff[1024];
	int i;

	if (!(fp = fopen(file, mode))) {
		return 0;
	}
	if (comment) fprintf(fp, "# %s\n\n", comment);

	for (i = 0; *opts[i].name; i++) {
		config2buf(opts + i, buff);
		fprintf(fp, "%s\n", buff);
	}
	fclose(fp);
	return 1;
}
#endif



#ifndef DSPRELTIME

#include <math.h>

static FILE* fp_sim;
static char* format_sim = "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d";
static uint8_t fix_type = 0;
static int8_t flag_file = 0;


/* open simulation data file --------------------------------------------------
* open a simulation data file for reading
* args   : char* file_name I   name of the file to open
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int sim_open(char* file_name) {
	fp_sim = fopen(file_name, "rb");
	return 0;
}

/* load simulation data from file ---------------------------------------------
* load simulation data from the opened file into the data structure
* args   : sim32bin_t* data_sim O   pointer to simulation data structure
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int load_data(sim32bin_t* data_sim) {
	flag_file = fread(data_sim, sizeof(double), 32, fp_sim);
	return feof(fp_sim);
}


/* close simulation data file -------------------------------------------------
* close the currently open simulation data file
* args   : none
* return : status (0: success, -1: error)
*-----------------------------------------------------------------------------*/
int sim_close() {
	fclose(fp_sim);
	return 0;
}

/* convert simulation data to KFAPP format -----------------------------------
* convert simulation data structure to KFAPP update structure
* args   : sim32bin_t* ptr_sim       I   pointer to simulation data
*          KFAPP_Update* ptr_UpPara  O   pointer to KFAPP update structure
* return : none
*-----------------------------------------------------------------------------*/
void sim2kfapp(sim32bin_t* ptr_sim, KFAPP_Update* ptr_UpPara) {
	ptr_UpPara->gnss.lon = ptr_sim->lon_gps;
	ptr_UpPara->gnss.lat = ptr_sim->lat_gps;
	ptr_UpPara->gnss.height_ellipsoid = ptr_sim->hgt_gps;

	ptr_UpPara->gnss.accu_lon = ptr_sim->std_lon;
	ptr_UpPara->gnss.accu_lat = ptr_sim->std_lat;
	ptr_UpPara->gnss.accu_height = ptr_sim->std_hgt;

	ptr_UpPara->gnss.vel_east = ptr_sim->eastvel_gps;
	ptr_UpPara->gnss.vel_north = ptr_sim->northvel_gps;
    ptr_UpPara->gnss.vel_up = ptr_sim->upvel_gps;

	if (fabs(ptr_sim->yaw) > 0.001) {
		//ptr_UpPara->gnss.heading = ptr_sim->yaw;
		ptr_UpPara->gnss.heading = 0;
	}
	else {
		ptr_UpPara->gnss.heading = 0;
	}
	ptr_UpPara->gnss.pos_dop = ptr_sim->hdop;
	ptr_UpPara->gnss.accu_heading = ptr_sim->yawrms;
	ptr_UpPara->gnss.fix_type = ptr_sim->dop;
	ptr_UpPara->gnss.week_secs.lf_secs = ptr_sim->t;
	ptr_UpPara->gnss.week_secs.ui_week = 100;

	if (ptr_UpPara->gnss.fix_type > 0) ptr_UpPara->imu.cnt = 0;
	else ptr_UpPara->imu.cnt++;

	ptr_UpPara->imu.acc_x = ptr_sim->vm_x;
	ptr_UpPara->imu.acc_y = ptr_sim->vm_y;
	ptr_UpPara->imu.acc_z = ptr_sim->vm_z;

	ptr_UpPara->imu.gyr_x = ptr_sim->wm_x;
	ptr_UpPara->imu.gyr_y = ptr_sim->wm_y;
	ptr_UpPara->imu.gyr_z = ptr_sim->wm_z;

	ptr_UpPara->imu.secs = ptr_sim->t;
	ptr_UpPara->imu.week = 100;

	ptr_UpPara->odo.right_front = 0;
	ptr_UpPara->odo.right_back = 0;
	ptr_UpPara->odo.left_front = 0;
	ptr_UpPara->odo.left_back = 0;
	ptr_UpPara->odo.mean = ptr_sim->dS;

	ptr_UpPara->imu.secs = ptr_sim->t;
	ptr_UpPara->odo.week = 100;
}
#endif



int gnss_parse_by_file(const char* file_name_gnss, gnss_info_t* px_gnss, gnss_t* gnss_data, int last_label)
{
	FILE* fp_gnss = fopen(file_name_gnss, "rb");
	if (!fp_gnss) return 2;
	FSEEK(fp_gnss, 0, SEEK_END);
	long len_gnss = ftell(fp_gnss);
	FSEEK(fp_gnss, 0, SEEK_SET);
	uint8_t* buff_gnss = (uint8_t*)malloc(len_gnss);
	long len_read = fread(buff_gnss, 1, len_gnss, fp_gnss);
	if (len_read != len_gnss) {
		free(buff_gnss);
		fclose(fp_gnss);
		return 3;
	}
	fclose(fp_gnss);

	long index_gnss = 0;
	while (index_gnss < len_gnss)
	{
		gnss_parse(buff_gnss[index_gnss++]);
        if (GNSSPARSE_PARSE_NOW == px_gnss->GNSSFlag && last_label == px_gnss->last)
		{
			add_gnss(px_gnss, gnss_data);
			px_gnss->GNSSFlag = 0;
            px_gnss->last = 0;
		}
	}
	free(buff_gnss);
	return 0;
}

int nav_parse_by_file(const char* file_name_imu, imu_info_t* px_imu, imu_t* imu_data)
{
	FILE* fp_imu = fopen(file_name_imu, "rb");
	if (fp_imu == NULL) {
		return 1;
	}
	FSEEK(fp_imu, 0, SEEK_END);
	long len_imu = ftell(fp_imu);
	FSEEK(fp_imu, 0, SEEK_SET);
	uint8_t* buff_imu = (uint8_t*)malloc(len_imu);

	long len_read = fread(buff_imu, 1, len_imu, fp_imu);
	if (len_read != len_imu) {
		free(buff_imu);
		fclose(fp_imu);
		return 3;
	}
	fclose(fp_imu);

	long index_imu = 0;
	while (index_imu < len_imu)
	{
		if (!nav_parse(buff_imu[index_imu++])) add_imu(px_imu, imu_data);
	}
	free(buff_imu);
	return 0;
}

void fill_sim(KFAPP_Update* ptr_update, sim32bin_t* ptr_sim)
{
	ptr_sim->t = ptr_update->imu.secs;
	ptr_sim->wm_x = ptr_update->imu.gyr_x;
	ptr_sim->wm_y = ptr_update->imu.gyr_y;
	ptr_sim->wm_z = ptr_update->imu.gyr_z;

	ptr_sim->vm_x = ptr_update->imu.acc_x;
	ptr_sim->vm_y = ptr_update->imu.acc_y;
	ptr_sim->vm_z = ptr_update->imu.acc_z;

	ptr_sim->dS = 99.0;

	ptr_sim->dop = ptr_update->gnss.fix_type;
	ptr_sim->hdop = ptr_update->gnss.pos_dop;

	ptr_sim->lat_gps = ptr_update->gnss.lat;
	ptr_sim->lon_gps = ptr_update->gnss.lon;
	ptr_sim->hgt_gps = ptr_update->gnss.height_ellipsoid;

	ptr_sim->eastvel_gps = ptr_update->gnss.vel_east;
	ptr_sim->northvel_gps = ptr_update->gnss.vel_north;
    ptr_sim->upvel_gps = ptr_update->gnss.vel_up;

	ptr_sim->satnum = ptr_update->gnss.num_sv;

	ptr_sim->std_lat = ptr_update->gnss.accu_lat;
	ptr_sim->std_lon = ptr_update->gnss.accu_lon;
	ptr_sim->std_hgt = ptr_update->gnss.accu_height;
}

static file_list_t file_list = { 0 };

file_list_t* getFileList(void)
{
	return &file_list;
}

//#include <windows.h>
//#include <io.h>
//int listFiles(char* dir, const char* matchchar)
//{
//	char search_path[256];
//
//	snprintf(search_path, sizeof(search_path), "%s//%s", dir, matchchar);
//	int res = 0;
//	intptr_t handle;
//	_finddata_t findData;
//
//	handle = _findfirst(search_path, &findData);
//	if (handle == -1)
//		return res;
//
//	do
//	{
//		snprintf(file_list.file_name[res++], sizeof(file_list.file_name[res]), "%s//%s", dir, findData.name);
//
//	} while (_findnext(handle, &findData) == 0);
//	_findclose(handle);
//
//	file_list.cnt_file = res;
//
//	return res;
//}



#include <dirent.h>

int listFiles(char* dir, const char* matchchar)
{
    int res = 0;
    struct dirent *entry;
    DIR *dp = opendir(dir);
    if (dp == NULL) return 0;

    while ((entry = readdir(dp)))
    {
        if (strcmp(entry->d_name, ".") == 0 || strcmp(entry->d_name, "..") == 0)  continue;
        if (matchchar != NULL && *matchchar != '\0') {
            if (!strstr(entry->d_name, matchchar)) {
//            if (fnmatch(matchchar, entry->d_name, 0) != 0) {
                continue;
            }
        }

        snprintf(file_list.file_name[res++], sizeof(file_list.file_name[res]), "%s//%s", dir, entry->d_name);
    }

    file_list.cnt_file = res;
    return res;
}
