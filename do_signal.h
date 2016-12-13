#ifndef SIGNAL_T_H
#define SIGNAL_T_H

#include "rtklib.h"
#include <Windows.h>
#define CONFIG_FILE "/tmp/config.rtk"
#define LOCAL_RINEX_FILE "20161019.16p"
#define FILE_B "C7_E1B.txt"
#define FILE_C "C8_E1C.txt"
#define OUT_FILE "Gal_Sig"
#define NULL_FILE "Null_Sig"
#define MAX_SAT 10
#define EXIST_NUM 9 /*现存9颗伽利略在轨卫星*/
#define NAV_LEN 75000 /* GAL 250bit/s,GPS 50bit/s, 300s len */
#define SAT_NUMS 9
#define MS_PROCESS 1800
#define GAL_SATNUM_OFFSET 32
#define PTHREAD_NUM 3
#define END_COMING 0x00000007

/*
#define thread_t    pthread_t
#define lock_t      pthread_mutex_t
#define initlock(f) pthread_mutex_init(f,NULL)
#define lock(f)     pthread_mutex_lock(f)
#define unlock(f)   pthread_mutex_unlock(f)
#define FILEPATHSEP '/'
*/
typedef enum{
	STATIC_MODE=0,
	DYN_MODE
}Gen_Mode;

typedef struct SCENE{
	Gen_Mode mode; 
	double static_user_pos[3];
	char Sig_Path[256];
	char Nav_Path[256];
	char Rinex_Path[256];
	char Kml_Path[256];
}G_PARAS;

typedef struct{
	char *filedir;
	double *userP;
	char *buffer;
	int len;
}_g_config;

typedef struct{
	double x;
	double y;
	double z;
}_user_pos;

typedef struct{
	int sat;
    unsigned char data[4092];
}_ca_code;
//TODO：ca码双系统合并
typedef struct{
    int sat;
    int data[1024];
}_gps_ca_code;

typedef struct{
	int sat;
    unsigned char *data;
}_nav_data;

typedef struct{
	double space_update;
	int t_user;
	double azel;
	double transTime_est;
	double mstoProcess;
	double fs;
	double fo;
	double f_t;
	double codeFreqbasis;
	double codePeriod;
	double codeLength;
    double d_subFreqbasis; //double of the sub carrier, 2*6.138M
	double f_navbasis;
	double CNR;
	double deltaP;
	int PRN[32];
    double rr[3]; //user ecef position
    double ll[3]; //user Latitude,longitude and height
    _ca_code bin[MAX_SAT];//栈上直接申请空间
    _ca_code cin[MAX_SAT];
    _gps_ca_code ca[MAX_SAT];
    _nav_data nav[MAX_SAT];//堆上动态申请
}_init_paras;

typedef struct{
	double codeTime; //传播时间 s
	double carrierTime;
	double t_svCodeTime; //发射时间 S
	double t_svCarrierTime;
    double fd;
	double dist;
}_time_code_carrier;

typedef struct{
    int PRN;
	int times; //sv_time len
	double *codechange;
	double *carrierchange;
	_time_code_carrier *sv_time;
}T_MODEL;

typedef struct{
	int PRN;
	double *s;
}SIGNAL_I;

typedef struct{
    lock_t lock;
    //pthread_cond_t ok;
    _init_paras *p;
    T_MODEL *t;
    SIGNAL_I *s;
    unsigned char *wr_buf;
    double t_user;
    int ok_nums;
    int pthread_num;
    int pthread_id;
    int status_mask[16];
}Multi_Signal;

int INIT_PARAS(_init_paras *paras, int final_second);
void FREE_PARAS(_init_paras *paras, int sat_num);
extern int Get_ok_sat(const _init_paras init_paras, nav_t *nav, int *sat, gtime_t time);
extern int main_signal(int min, int Is_Simple_Way);
#endif
