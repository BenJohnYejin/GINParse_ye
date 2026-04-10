
#include "src/app_interface.h"
#include <stdio.h>
#include <math.h>


static KFAPP_ConfigPara configPara;
static char* format_avp = "%7.3f %7.3f %9.3f %9.6f %9.6f %9.6f %15.10f %15.10f %7.3f ";
static char* format_ebdb = "%12.10f %12.10f %12.10f %12.10f %12.10f %12.10f ";
static char* format_gnss_vp = "%9.6f %9.6f %9.6f %15.10f %15.10f %7.3f ";
static char* format_odkappa = "%10.7f %7.4f %10.7f ";
static char* format_measflag = "%9d ";
static char* format_yaw = "%9.3f ";
static char* format_time = "%10.3f\n";

CONFIG_t s_config[] = {
    { "dir_src_gnss"        ,2  ,  (void*)&configPara.srcfile_gnss, "src gnss dir"},
    { "gnss_externsion"        ,2  ,  (void*)&configPara.pattern_extertion, "src gnss externsion"},
    { "gnss_mask"        ,0  ,  (void*)&configPara.gnss_mask, "src gnss mask"},
    { "last_gnss_nema_label" ,0  ,  (void*)&configPara.last_label, "gnss last nema label code(oct code GGA 0001 VEL 0002 HEA 0004 GST 0020 ZDA 0010"},

//    { "gnss_enable"          ,6  ,  (void*)&configPara.gnss_enable, "enable parse gnss file(0 no 1 yes)"},

	//{ "file_offline"         ,2  ,  (void*)&configPara.simfile    ,"offline file" },
	//{ "file_trace"           ,2  ,  (void*)&configPara.tracefile  ,"trace file" },
	//{ "file_result_gnss"     ,2  ,  (void*)&configPara.resfile_gnss, "result gnss file"},
	//{ "file_offline_imu"     ,2  ,  (void*)&configPara.srcfile_imu , "offline gnss file"},
	//{ "file_result_imu"      ,2  ,  (void*)&configPara.resfile_imu , "result gnss file"},

//    { "file_offline_nav"     ,2  ,  (void*)&configPara.srcfile_nav       , "offline nav file"},
	//{ "file_result_nav"      ,2  ,  (void*)&configPara.resfile_nav       , "result nav file"},
	//{ "file_result_nav_gnss" ,2  ,  (void*)&configPara.resfile_nav_gnss  , "result nav-gnss file"},
	//{ "file_result_nav_imuod",2  ,  (void*)&configPara.resfile_nav_imuod , "result nav-imu_od file"},


//	{ "imu_enable"           ,6  ,  (void*)&configPara.imu_enable, "result gnss file"},
//	{ "imu_type"             ,6  ,  (void*)&configPara.imu_type, "result gnss file"},

//	{ "nav_enable"           ,6  ,  (void*)&configPara.nav_enable , "offline gnss file"},
//	{ "nav_type"             ,6  ,  (void*)&configPara.nav_type , "result gnss file"},
//	{ "sim_exit"             ,6  ,  (void*)&configPara.sim_exit , "exsit the sim file"},
	

//	{ "trace_lever"   ,0  ,  (void*)&configPara.trace_lever,"trace lever" },
//	{ "conf_od "      ,6  ,  (void*)&configPara.conf_od    ,"enable od" },
//	{ "imu_cnt "      ,5  ,  (void*)&configPara.imu_cnt    ,"dt = t_imu-t_od s" },
//	{ "ant_mode "     ,6  ,  (void*)&configPara.ant_mode   ,"ant mode, 1->ONE 2->LR 3->RF 4->FB 5->BF 6->angle" },
//	{ "out_point "    ,6  ,  (void*)&configPara.out_point  ,"1 imu 2 gnss 3 using out_lv" },

//	{ "lvGNSS_i"      ,4  ,  (void*)&configPara.conf_lvGNSS_i,"ins->gnss lever x m" },
//	{ "lvGNSS_j"      ,4  ,  (void*)&configPara.conf_lvGNSS_j,"ins->gnss lever y m" },
//	{ "lvGNSS_k"      ,4  ,  (void*)&configPara.conf_lvGNSS_k,"ins->gnss lever z m" },
//	{ "angle_gnss"    ,4  ,  (void*)&configPara.conf_GNSSAgle,"ins->gnss angle deg" },

//	{ "lvOD_i"        ,4  ,  (void*)&configPara.conf_lvOD_i,"ins->od lever x m" },
//	{ "lvOD_j"        ,4  ,  (void*)&configPara.conf_lvOD_j,"ins->od lever y m" },
//	{ "lvOD_k"        ,4  ,  (void*)&configPara.conf_lvOD_k,"ins->od lever z m" },

//	{ "odKappa_i"     ,4  ,  (void*)&configPara.conf_odKappa_i,"odKappa x deg" },
//	{ "odKappa_j"     ,4  ,  (void*)&configPara.conf_odKappa_j,"odKappa y scale" },
//	{ "odKappa_k"     ,4  ,  (void*)&configPara.conf_odKappa_k,"odKappa z deg" },


	//{ "Qt_gyr_h",  4,  (void*)&modelpara.Qt_gyr_h,"Horizontal gyro noise (deg/sqrt(hr)" },
	//{ "Qt_gyr_v",  4,  (void*)&modelpara.Qt_gyr_v,"Vertical gyro noise (deg/sqrt(hr)) " },
	//{ "Pk_eb_h",   4,  (void*)&modelpara.Pk_eb_h,"Horizontal gyro bias initial covariance DPH" },
	//{ "Pk_eb_v",   4,  (void*)&modelpara.Pk_eb_v,"Vertical gyro bias initial covariance DPH" },
	//{ "Qt_acc_h",   4,  (void*)&modelpara.Qt_acc_h,"Horizontal accel noise (ug/sqrt(Hz))" },
	//{ "Qt_acc_v",   4,  (void*)&modelpara.Qt_acc_v,"Vertical accel noise (ug/sqrt(Hz))" },
	//{ "Pk_db_h",   4,  (void*)&modelpara.Pk_db_h,"Horizontal accel bias initial covariance mg" },
	//{ "Pk_db_v",   4,  (void*)&modelpara.Pk_db_v,"Vertical accel bias initial covariance mg" },

	//{ "Rt_Vn_gnss_h",   4,  (void*)&modelpara.Rt_Vn_gnss_h,"GNSS horizontal velocity measurement noise m/s" },
	//{ "Rt_Vn_gnss_v",   4,  (void*)&modelpara.Rt_Vn_gnss_v,"GNSS vertical velocity measurement noise m/s" },
	//{ "Rt_Pos_gnss_h",   4,  (void*)&modelpara.Rt_Pos_gnss_h,"GNSS horizontal position measurement noise m" },
	//{ "Rt_Pos_gnss_v",   4,  (void*)&modelpara.Rt_Pos_gnss_v,"GNSS vertical position measurement noise m" },

	//{ "Rt_vn_od_h",   4,  (void*)&modelpara.Rt_vn_od_h,"Odometer horizontal velocity measurement noise m/s" },
	//{ "Rt_vn_od_v",   4,  (void*)&modelpara.Rt_vn_od_v,"Odometer vertical velocity measurement noise m/s" },
	//{ "Rt_zupt",      4,  (void*)&modelpara.Rt_zupt,"ZUPT measurement noise m" },
	//{ "Rt_nhc",       4,  (void*)&modelpara.Rt_nhc,"NHC measurement noise m" },
	//{ "Rt_zihr",      4,  (void*)&modelpara.Rt_zihr,"ZIHR measurement noise DEG" },

	//{ "Pk_att_h",  4,  (void*)&modelpara.Pk_att_h,"Horizontal attitude initial covariance MIN" },
	//{ "Pk_att_v",  4,  (void*)&modelpara.Pk_att_v,"Vertical attitude initial covariance MIN " },
	//{ "Pk_vel_h",   4,  (void*)&modelpara.Pk_vel_h,"Horizontal velocity initial covariance m/s" },
	//{ "Pk_vel_v",   4,  (void*)&modelpara.Pk_vel_v,"Vertical velocity initial covariance m/s" },
	//{ "Pk_pos_h",   4,  (void*)&modelpara.Pk_pos_h,"Horizontal position initial covariance m" },
	//{ "Pk_pos_v",   4,  (void*)&modelpara.Pk_pos_v,"Vertical position initial covariance m" },

	//{ "Pk_od_pitch",  4,  (void*)&modelpara.Pk_od_pitch,"Odometer pitch initial covariance DEG" },
	//{ "Pk_od_scale",  4,  (void*)&modelpara.Pk_od_scale,"Odometer scale initial covariance NONE " },
	//{ "Pk_od_yaw",    4,  (void*)&modelpara.Pk_od_yaw,"Odometer yaw initial covariance DEG" },
	//{ "Pk_gnss_yaw",  4,  (void*)&modelpara.Pk_gnss_yaw,"GNSS yaw initial covariance DEG" },

	//{ "FB_att_TAU",  4,  (void*)&modelpara.FB_att_TAU,"Attitude feedback time constant TS" },
	//{ "FB_vel_TAU",  4,  (void*)&modelpara.FB_vel_TAU,"Velocity feedback time constant TS " },
	//{ "FB_pos_TAU",  4,  (void*)&modelpara.FB_pos_TAU,"Position feedback time constant TS" },
	//{ "FB_eb_TAU",   4,  (void*)&modelpara.FB_eb_TAU,"Gyro bias feedback time constant TS" },
	//{ "FB_db_TAU",   4,  (void*)&modelpara.FB_db_TAU,"Accel bias feedback time constant TS" },
	//{ "FB_od_TAU",   4,  (void*)&modelpara.FB_od_TAU,"Odometer feedback time constant TS" },
	//{ "FB_gnss_yaw_TAU",   4,  (void*)&modelpara.FB_gnss_yaw_TAU,"GNSS yaw feedback time constant TS" },

	//{ "FB_att_h",  4,  (void*)&modelpara.FB_att_h,"Horizontal attitude feedback gain MIN" },
	//{ "FB_att_v",  4,  (void*)&modelpara.FB_att_v,"Vertical attitude feedback gain MIN " },
	//{ "FB_vel_h",  4,  (void*)&modelpara.FB_vel_h,"Horizontal velocity feedback gain m/s" },
	//{ "FB_vel_v",  4,  (void*)&modelpara.FB_vel_v,"Vertical velocity feedback gain m/s" },
	//{ "FB_pos_h",  4,  (void*)&modelpara.FB_pos_h,"Horizontal position feedback gain m/s" },
	//{ "FB_pos_v",  4,  (void*)&modelpara.FB_pos_v,"Vertical position feedback gain m/s" },
	//{ "FB_eb_h",   4,  (void*)&modelpara.FB_eb_h,"Horizontal gyro bias feedback gain DPH" },
	//{ "FB_eb_v",   4,  (void*)&modelpara.FB_eb_v,"Horizontal gyro bias feedback gain DPH" },
	//{ "FB_db_h", 4,  (void*)&modelpara.FB_db_h,"Horizontal accel bias feedback gain MG" },
	//{ "FB_db_v", 4,  (void*)&modelpara.FB_db_v,"Vertical accel bias feedback gain MG" },

	//{ "FB_od_pitch", 4,  (void*)&modelpara.FB_od_pitch,"Odometer pitch feedback gain DEG" },
	//{ "FB_od_scale", 4,  (void*)&modelpara.FB_od_scale,"Odometer scale feedback gain NONE" },
	//{ "FB_od_yaw",   4,  (void*)&modelpara.FB_od_yaw,"Odometer yaw feedback gain DEG" },
	//{ "FB_gnss_yaw", 4,  (void*)&modelpara.FB_gnss_yaw,"GNSS yaw feedback gain DEG" },

	{ "",0, NULL,"" }
};

static gnss_t gnss_data;

void print_gnss(char* file_gnss, gnss_t* gnss_data) 
{
	uint32_t index_gnss = 0;
	FILE* fp_out = fopen(file_gnss, "wb");
	while (index_gnss < gnss_data->cnt)
	{
		fprintf(fp_out, "%4d %8.3f ", gnss_data->px_gnss_data[index_gnss].week_secs.ui_week, gnss_data->px_gnss_data[index_gnss].week_secs.lf_secs);
		fprintf(fp_out, "%25.21f %25.21f %8.3f ", gnss_data->px_gnss_data[index_gnss].lon / DEG, gnss_data->px_gnss_data[index_gnss].lat / DEG, gnss_data->px_gnss_data[index_gnss].height_ellipsoid);
        fprintf(fp_out, "%9.4lf %9.4lf %9.4lf %8.3f ", gnss_data->px_gnss_data[index_gnss].vel_east, gnss_data->px_gnss_data[index_gnss].vel_north, gnss_data->px_gnss_data[index_gnss].vel_up, gnss_data->px_gnss_data[index_gnss].heading_motion);
		fprintf(fp_out, "%8.3f %6.3f %6.3f %6.3f %6.3f %1d %6.3f %3d %2.1f \n",
			gnss_data->px_gnss_data[index_gnss].heading / DEG, gnss_data->px_gnss_data[index_gnss].accu_heading,
			gnss_data->px_gnss_data[index_gnss].accu_lon, gnss_data->px_gnss_data[index_gnss].accu_lat, gnss_data->px_gnss_data[index_gnss].accu_height,
			gnss_data->px_gnss_data[index_gnss].fix_type, gnss_data->px_gnss_data[index_gnss].pos_dop, gnss_data->px_gnss_data[index_gnss].num_sv, gnss_data->px_gnss_data[index_gnss].age);
		index_gnss++;
	}
    fclose(fp_out);
}


/* config used */
static char  file_config[128] = "./gnss_parse_config.txt";

int main(int argc, char* argv[]) {
	/* load config*/
	loadconfigs(file_config, s_config);
    //saveconfigs(file_config,"w","gnss_parse_config", s_config);

	file_list_t* file_list_;
	file_list_ = getFileList();
	int cnt_file = 0;
    cnt_file = listFiles(configPara.srcfile_gnss, configPara.pattern_extertion);

	int j = 0;
	for (; j < cnt_file; j++)
	{
		char file_name_nav[64];
		char file_name_sim[64];
        strcpy(file_name_nav, file_list_->file_name[j]);
		printf("%s src file start load \n", file_name_nav);
		gnss_info_t* px_gnss = gnss_get();
		strcpy(file_name_sim, file_name_nav);
		char* ptr = strrchr(file_name_sim, '.');
		file_name_sim[ptr - file_name_sim] = '\0';
		strcat(file_name_sim, ".lat");
        gnss_parse_by_file(file_name_nav, px_gnss, &gnss_data, &configPara);
		print_gnss(file_name_sim, &gnss_data);
        if(gnss_data.cnt > 0){
            gnss_data.cnt = 0;
            gnss_data.cnt_max = 0;
            free(gnss_data.px_gnss_data);
        }
	}
	
	return 0;
}

