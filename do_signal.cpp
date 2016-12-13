#include "do_signal.h"
#include "do_nav.h"
#include "do_util.h"
#include "do_files.h"
#include "rtklib.h"
//#include <unistd.h>
#include <stdio.h>

_g_config g_config = {0};
extern G_PARAS g_paras;
static double static_user_pos[3] = {39.9075*D2R,116.3881*D2R,39.07};

const int e1c_sub_code[25] = {0,0,1,1,1,0,0,0,0,0,0,0,1,0,1,0,1,1,0,1,1,0,0,1,0};
static const int g2CodePhase[32][2]={2,6,3,7,4,8,5,9,1,9,2,10,1,8,2,9,3,10,2,3,3,4,5,6,6,7,7,8,8,9,9,10,
1,4,2,5,3,6,4,7,5,8,6,9,1,3,4,6,5,7,6,8,7,9,8,10,1,6,2,7,3,8,4,9};
double base_a[2][12];
double base_b[2][12];
static Multi_Signal sig_arg; //多线程共享信号相关参数

static int tow[MAX_SAT],frame[MAX_SAT];
static double temp[MAX_SAT];
static int End_Flag = 0x00000000;
static const int End_Bit[3]={0x01,0x02,0x04};

//multi-pthread support
static thread_t p_tid[PTHREAD_NUM];
static int p_id[PTHREAD_NUM];
//pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;/*初始化互斥锁*/
//pthread_cond_t cond = PTHREAD_COND_INITIALIZER;/*初始化条件变量*/

static eph_t *my_seleph(gtime_t time, int sat, int iode, const nav_t *nav);

static int Generate_Baseband(char *file_b, char *file_c, _init_paras *p){
    double sub_a[12];
    double sub_b[12];
    read_e1b_e1c(file_b,file_c,p->bin,p->cin);//key
    generate_sub_carrier(10,1,12,sub_a,sub_b);//key
    generate_baseband(sub_a,sub_b,base_a,base_b);//key
    return 0;
}

static void init_read_config(){
	g_config.filedir = CONFIG_FILE;
	g_paras.static_user_pos[0]*=D2R;
	g_paras.static_user_pos[1]*=D2R;
	g_config.userP = g_paras.static_user_pos;	
}

int INIT_PARAS(_init_paras *paras, int final_second){
	int i;
	double ecef[3];
    init_read_config();//初始化配置文件
    paras->space_update = 0.1;//计算卫星位置，估算信号传输时间的时间间隔
    paras->azel = 10;//卫星最低仰角
    paras->transTime_est = 0.085;//传输时间估计
    paras->mstoProcess = final_second;//生成信号时长
    paras->fs = 2e7;//5.72e6;//信号采样率
    paras->fo = 4123968;//4e7/7-(3.542e7 - 2.80e8/9);//中频载波频率
    paras->codeFreqbasis = 1.023e6;//测距吗速率，1.023M HZ,码长 4092,周期 4ms 左右
	paras->codePeriod = 1/paras->codeFreqbasis;
    paras->codeLength = 4092;//码长 4092
    paras->d_subFreqbasis=2*6.138e6; //cboc(1,6)子载波6频率的2倍
	paras->f_navbasis = 1000;//TODO::
    paras->f_t = 1.57542e9;//E1 射频中心频率
	// llh to ecef
	pos2ecef(g_config.userP, ecef);
    paras->rr[0] = ecef[0];
    paras->rr[1] = ecef[1];
    paras->rr[2] = ecef[2];
	paras->ll[0] = g_config.userP[0];
	paras->ll[1] = g_config.userP[1];
	paras->ll[2] = g_config.userP[2];
	for(i=0;i<MAX_SAT;i++){
        //paras->nav[i].data = (int *)malloc(sizeof(int)*NAV_LEN);//导航电文存储区
	}
    return 0;
}

void FREE_PARAS(_init_paras *paras,int sat_num){
	int i;
	for(i=0;i<sat_num;i++){
                free(paras->nav[i].data);
        }
}

//return 接收机与卫星几何距离
//geodist:计算卫星与接收机几何距离，rs:卫星ecef;rr:接收机ecef;e:相对向量
//satazel:计算方位角，接收机：pos{lat,lon,h},azel:satellite azimuth/elevation angle
double topocent(const double *rs, const double *rr, const double *pos, double *azel)
{
    double e[3];
    double r = geodist(rs,rr,e);
	if(r > 0.0)
		satazel(pos,e,azel);
	return r;
}

extern int Get_ok_sat(const _init_paras init_paras, nav_t *nav, int *sat, gtime_t time)
{
    /*gtime_t t;
    char file[]=LOCAL_RINEX_FILE;
    double ep[]={2016,6,26,6,41,25};*/
    double rs[6],dts[2];
    double var;
    int i,j=0,svh,ret;
    double azel[2]={0};
    //readrnx(file,1,"",NULL,nav,NULL);
    //t=epoch2time(ep);
    for (i=0;i<NSATGAL;i++) {
        ret=satpos(time,time,satno(SYS_GAL,i+1),EPHOPT_BRDC,nav,rs,dts,&var,&svh);
        if(ret)topocent(rs,init_paras.rr,static_user_pos,azel);
        printf("Debug: sat %d degree is %f, ret==%d \n", satno(SYS_GAL,i+1), azel[1]*R2D, ret );
        if(azel[1]*R2D > init_paras.azel){
            sat[j]=satno(SYS_GAL,i+1);
            j++;
            printf("Debug: sat %d degree is %f \n", satno(SYS_GAL,i+1), azel[1]*R2D );
        }
        azel[0]=azel[1]=0;
    }
    printf("Get_ok_sat num is %d \n",j);
    return j;
}

static eph_t *my_seleph(gtime_t time, int sat, int iode, const nav_t *nav)
{
    double t,tmax,tmin;
    int i,j=-1;
    
    trace(4,"seleph  : time=%s sat=%2d iode=%d\n",time_str(time,3),sat,iode);
    
    switch (satsys(sat,NULL)) {
        case SYS_QZS: tmax=MAXDTOE_QZS+1.0; break;
        case SYS_GAL: tmax=MAXDTOE_GAL+1.0; break;
        case SYS_CMP: tmax=MAXDTOE_CMP+1.0; break;
        default: tmax=MAXDTOE+1.0; break;
    }
    tmin=tmax+1.0;
    
    for (i=0;i<nav->n;i++) {
        if (nav->eph[i].sat!=sat) continue;
        if (iode>=0&&nav->eph[i].iode!=iode) continue;
        if ((t=fabs(timediff(nav->eph[i].toe,time)))>tmax) continue;
        if (iode>=0) return nav->eph+i;
        if (t<=tmin) {j=i; tmin=t;} /* toe closest to time */
    }
    if (iode>=0||j<0) {
        trace(2,"no broadcast ephemeris: %s sat=%2d iode=%3d\n",time_str(time,0),
              sat,iode);
        return NULL;
    }
    return nav->eph+j;
}

///////////////////////////////////////////////////
int find_index(int num, int *data, int len){
	int i;
	for(i=0;i<len;i++){
		if(data[i]==num)return i;
	}
	return -1;
}

//nav data generate
int read_nav_bin(char *file,  _init_paras *p, int *sat, int len){
	FILE *fp;
	char buff[NAV_LEN+4];
	char *data, *num;
	int i,n,index;
	if(!(fp = fopen(file,"r"))){
		printf("ERROR: open nav bin file wrong\n");
		return -1;
	}
	while (fgets(buff,NAV_LEN+4,fp)) {
		num = strtok(buff, "#");
		n = (int)str2num(num,0,2);
		printf("n is %d \n",n);//TODO::n=0 ????;
		index = find_index(n,sat,len);
		if( index != -1){
			data = strtok(NULL,"#");
			printf("indexxxxxxxxxxxxxxxxxx is %d \n",index);
			for(i=0;i<NAV_LEN;i++){
				p->nav[index].data[i]=(data[i]-48)*2-1;
			}
            printf("the %dth nav first bit is %d", index, p->nav[index].data[0]);
		}
		printf("\n END of NAV \n");
	}
    return 0;
}
//GPS ca code generate,output 0,1
int generateCAcodeS(int SatelliteID, int *CAOutput){
    int reg_value[10]={1,1,1,1,1,1,1,1,1,1};
    int N=10,Cyc=1023,feedback;
    int G1[1023] = {0};
    int G2[1023] = {0};
    int PhaseSelector[2];
    int i,j;
    for(i=0;i<Cyc;i++){
        G1[i] = reg_value[9];
        feedback = reg_value[2]^reg_value[9];
        for(j=N;j>1;j--){
            reg_value[j-1] = reg_value[j-2];
        }
        reg_value[0] = feedback;
    }
    for(i=0;i<N;i++){
        reg_value[i]=1;
    }
    PhaseSelector[0] = g2CodePhase[SatelliteID-1][0] - 1;
    PhaseSelector[1] = g2CodePhase[SatelliteID-1][1] - 1;
    for(i=0;i<Cyc;i++){
        G2[i] = reg_value[PhaseSelector[0]]^reg_value[PhaseSelector[1]];
        feedback = reg_value[1]^reg_value[2]^reg_value[5]^reg_value[7]^reg_value[8]^reg_value[9];
        for(j=N;j>1;j--){
            reg_value[j-1] = reg_value[j-2];
        }
        reg_value[0] = feedback;
    }
    for(i=0;i<Cyc;i++){
        CAOutput[i] = G1[i]^G2[i];
        //if(CAOutput[i] == 0)
            //CAOutput[i] = -1;
    }
	return 0;
}
//ca end
gtime_t time2gtime(double t1){
	gtime_t t;
	t.time = (time_t)t1;
	t.sec = t1 - (int)t1;
	return t;
}

double gtime2time(gtime_t t1){
	double t;
	t = t1.time*1000 + t1.sec;
	return t;
}

void eph2pos2speed(gtime_t time, const eph_t *eph, double *rs, double *dts, double *var){
    int i;
    double rst[3],dtst[1],tt=1E-3;
	////**************domon: caculate the position of sv
    eph2pos(time,eph,rs,dts,var);
	////**************domon: 为后续差分消计算(卫星速度和时钟变化率)做准备
    time=timeadd(time,tt);
    eph2pos(time,eph,rst,dtst,var);
    //domon:存储多余的变化率参数
    /* satellite velocity and clock drift by differential approx */
    for (i=0;i<3;i++) rs[i+3]=(rst[i]-rs[i])/tt;
    dts[1]=(dtst[0]-dts[0])/tt;
}
// 计算码传播时延 
//   input : t               -- reciever time
//          eph             -- ephemise for one satllite
//          userposition    -- 用户位置(XYZ)

//
//  output: codeTime    --码延时时间
//         carrierTime --载波延时
//计算每一颗星的发射及接收时间
int calculateTime(double t, int week, double delt_t,int sat, eph_t *eph, const double *rr,const double *userP, _time_code_carrier *sv_time){

double t_sv0;
double var;
int svh;
double rs[6],dts[2],e[3],azel[2];
double Xsu[3],Vt,Vr,nameta,Rsu;
double r, traveltime, dtrp, tau;
int flag=1;
int i=1,k;
gtime_t gtime;
//printf("Debug: time: %f, delt_t: %f \n", t, delt_t);
t_sv0=t-delt_t;//信号初始发射时刻
while(flag){
    gtime = gpst2time(week,t_sv0);
    //计算卫星位置和速度
    eph2pos(gtime,eph,rs,dts,&var);
    //printf("rrrrrrrrrrrrrrs is %13.3f %13.3f %13.3f \n",rs[0],rs[1],rs[2]);
    //satpos(t_sv0,t_sv0,sat,EPHOPT_BRDC,nav,rs,dts,&var,&svh);
    r = geodist(rs, rr, e);//初始真实距离
    traveltime= r / CLIGHT;//初始传播时间
    tau=fabs(traveltime-delt_t);
        if(tau<1e-9){
            //建立传播过程中误差模型
            r = topocent(rs,rr,userP,azel);//角度
            //对流层延时
            dtrp = tropmodel(gpst2time(week,t),userP,azel,0.5);//0.5 relative humidity
			dtrp = 0;
//             trop = 2.47/(sin(el * dtr)+0.0121);
//             //电离层延时
//             iono = ionospheric(el*dtr, az*dtr,userP,t, eph);//sec
//             //码传播延时
//             codeTime=traveltime+trop/c+iono/c-satClkCorr;%
//             carrierTime=traveltime+trop/c-iono/c-satClkCorr;
			sv_time->dist = r;
            sv_time->codeTime=traveltime+dtrp/CLIGHT-dts[0];//
            sv_time->carrierTime=traveltime+dtrp/CLIGHT-dts[0];
            sv_time->t_svCodeTime=t-sv_time->codeTime;//准确的发射时刻
            sv_time->t_svCarrierTime=t-sv_time->carrierTime;//准确的发射时刻
            //计算多普勒初值sat speed ,rs[3-5]
            eph2pos2speed(gpst2time(week,t_sv0),eph,rs,dts,&var);
			//satpos(t_sv0,t_sv0,sat,EPHOPT_BRDC,nav,rs,dts,&var,&svh);
            r = geodist(rs, rr, e);//初始真实距离
            traveltime= r / CLIGHT;//初始传播时间
            //地球自转修正后的卫星位置，赛格纳效应
            //多普勒  
			for(k=0;k<3;k++)
				Xsu[k]=r*e[k];
			// e .* satspeed
            Vt=dot(Xsu,rs+3,3);
            Rsu=r;
            Vr=-Vt/Rsu;
            nameta=CLIGHT/FREQ1;
            sv_time->fd=Vr/nameta;
            flag=0;
	    //printf("Debug:tau = %f, r = %f, traveltime = %f \n",tau,r,traveltime);
            break;
		}
        i=i+1;
        delt_t=traveltime;
        t_sv0=t-traveltime;
	printf("Debug:tau = %10.12f, r = %10.12f, traveltime = %10.12f \n",tau,r,traveltime);
}
return 0;
}

int INIT_T_MODEL(T_MODEL *T, int times, int sat){
	T->PRN = sat;
	T->times = times;
	T->sv_time = (_time_code_carrier *)malloc(sizeof(_time_code_carrier)*times);
	T->codechange = (double *)malloc(sizeof(double)*(times*4/3));
	T->carrierchange = (double *)malloc(sizeof(double)*(times*4/3));
	return 0;
}

int FREE_T_MODEL(T_MODEL *T){
	printf("free struct T_MODEL \n");
	T->PRN = 0;
	T->times = 0;
	free(T->sv_time); T->sv_time = NULL;
	free(T->codechange); T->codechange = NULL;
	free(T->carrierchange); T->carrierchange = NULL;
	return 0;
}

int INIT_SIGNAL_I(SIGNAL_I *S, int size, int sat){
	S->PRN = sat;
	S->s = (double *)malloc(sizeof(double)*size);
    //memset(S->s,0,sizeof(double)*size);
	return 0;
}

int FREE_SIGNAL_I(SIGNAL_I *S){
	printf("free struct SIGNAL_I \n");
	S->PRN = 0;
	free(S->s); S->s = NULL;
	return 0;
}
//建立时间模型
//三阶多项式建模
//输入： rr    -- 用户坐标（XYZ）
//      userp     --  用户经纬度坐标
//      nav    --星历
//      sat     --  可见星星星号
//      t       -- 本地时间
//      time   -- 0.1s的次数
//输出： T    --码伪距、载波伪距、发射时刻
int getTimeModelStatic(T_MODEL *T, _init_paras *paras, nav_t *nav, int *sat, int sat_nums, gtime_t t, LLH *llh){

	double A[16]={1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1};
	double code_tau[4],code_Q[4];
	double carrier_tau[4],carrier_Q[4];
	//用于存放动态结果
	double d_ecef[3],d_llh[3];
	eph_t  *eph;
	int Q = 0,k,i,j,m,n,week,index,delta;
    int times = (int)(paras->mstoProcess/paras->space_update);
	int den = (int)(1/paras->space_update + 0.5);
    double space_update = paras->space_update;
    //int sat_nums = SAT_NUMS;//TODO::kkkkk
	double tow;
	tow = time2gpst(t,&week);
	for(m=0;m<4;m++){
		for(n=0;n<4;n++){
			switch(n){
				case 0: A[4*m+n]=1.0;break;
				case 1: A[4*m+n]=pow(m*space_update,n);break;
				case 2: A[4*m+n]=pow(m*space_update,n)/2;break;
				case 3: A[4*m+n]=pow(m*space_update,n)/6;break;
				default: break;
			}
		}
	}
	if(matinv(A, 4))printf("A -1 wrong\n");//inveser A
	for(k=0;k<sat_nums;k++){
		if (!(eph=my_seleph(t,sat[k],-1,nav))){
			printf("time t: %f eph is not available \n", t.time + t.sec);
			continue;
		}
		Q=0;
        //INIT_T_MODEL(T+k,times,sat[k]);
		printf("Debug: sat:%d times:%d \n", sat[k], times);
		for(i=0; i<times; i++){
			//添加动态部分，一阶插值计算接收机位置
			if(g_paras.mode == DYN_MODE && llh){
				index=i/den;
				delta=i%den;
				//get right llh
				for(m=0;m<3;m++){
					d_llh[m]=(double)(llh[index].userp[m]*(den-delta)+llh[index+1].userp[m]*delta)/den;
				}
				// llh to ecef
				pos2ecef(d_llh, d_ecef);
				for(m=0;m<3;m++){
					paras->rr[m] = d_ecef[m];
					paras->ll[m] = d_llh[m];
				}
			}
			calculateTime(tow+i*space_update, week, paras->transTime_est,sat[k], eph, paras->rr, paras->ll, T[k].sv_time+i);
			if((i%3 == 0) && (i != 0)){
				printf("-------------i : %d \n",i);
				for(j=0;j<4;j++){
					code_tau[j] = T[k].sv_time[i+j-3].codeTime;
					carrier_tau[j] = T[k].sv_time[i+j-3].carrierTime;
				}
				matmul("TN",4,1,4,1.0,A,code_tau,0.0,code_Q);
				matmul("TN",4,1,4,1.0,A,carrier_tau,0.0,carrier_Q);
				j=0;
				for(j=0;j<4;j++){
					T[k].codechange[Q+j] = code_Q[j];
					T[k].carrierchange[Q+j] = carrier_Q[j];
				}
				//printf("Debug:getTimeModelStatic Q: %d, and i = %d \n", Q, i);
				Q+=4;
				if(Q>(times*4/3))printf("Debug:getTimeModelStatic Q: %d is out of range, and i = %d \n", Q, i);
			}
		}
	}
	return 0;
}

double mod(double x, double y){
	if(y == 0)return -1;
    int a = (int)(x/y);
	return x - a*y;
}

static void encode_signal_2bit(SIGNAL_I *S,double amp,int i){
	double step,a[4],v;
	int j;
	step = amp/2;
	v = S->s[i];
	for(j=0;j<4;j++){
		a[j] = -amp+j*step;
	}
	if(v >= a[0] && v < a[1])
		S->s[i] = 3;
	else if(v >= a[1] && v < a[2])
		S->s[i] = 2;
	else if(v >= a[2] && v < a[3])
		S->s[i] = 0;
	else
		S->s[i] = 1;
}
static void encode_signal(SIGNAL_I *S,double amp,int i){
    double zoom = 127.0/amp;
    S->s[i]*=zoom;
}
//信号生成线程，线程数3，每个线程处理0.1s的数据
#ifdef WIN32
static DWORD WINAPI signal_cb(void *arg)
#else
static void *signal_cb(void *arg)
#endif
{
    Multi_Signal *ms=&sig_arg;
    T_MODEL *T=ms->t;
    SIGNAL_I *S=ms->s;
    _init_paras *paras=ms->p;
    double Period=3*paras->space_update;
    double fs=paras->fs;
    double f_t=paras->f_t;
    double fo=paras->fo;
    double codeFreqbasis=paras->codeFreqbasis;
    double d_sub=paras->d_subFreqbasis;
    double t_user=ms->t_user;
    int sat_nums=ms->ok_nums;
    int p_id=*(int *)arg;
    int p_num=ms->pthread_num;
    int dots=Period*fs/p_num;//TODO::ensure 整除!!!!!!!!!
    int times=floor(ms->p->mstoProcess/Period);
    int offset=p_id*dots;
    int wr_offset=0;
    double ts=1/fs;
    int i,j,k,n,bin_code,cin_code,index_sub,cin_sub_code,nav_up,index_nav,index_code,wr_index;
    double t,index_m,index_tsv,index_c,index_carrier,theta,carrier,s_over;
    double *codeTimeCoef, *carrierTimeCoef;
	unsigned char k1,k2,k3,k4,debugggg;
    for(i=0;i<times;i++){
        //printf("INFO: Everything is OK, pthread_num = %d, Process = %d \n", p_id, i);
        s_over = i*Period;
        if(ms->status_mask[p_id]==0){
            for(j=0;j<sat_nums;j++){
                //Galileo Part
                if(T[j].PRN > MAXPRNGPS){
                    for(k=offset,n=0;n<dots;k++,n++){
                        t=k*ts; //TODO::leap is not sure
                        codeTimeCoef = T[j].codechange+4*i;
                        carrierTimeCoef = T[j].carrierchange+4*i;
                        index_m=((codeTimeCoef[3]/6*t+codeTimeCoef[2]/2)*t+codeTimeCoef[1])*t+codeTimeCoef[0];//每个时间点的传播时间
                        //用累加器实现三阶多项式
                        index_tsv=(t_user+t)-index_m;//每个时间点的发射时刻-------------(t_user+t == new t_user time)
                        //************************************domon change,remove floor, proved wrong
                        index_c=index_tsv-tow[j]-floor(temp[j])/1000;// domon:相对于初始时刻的偏移量，举例：0时刻帧内偏移 500，接下来的9时刻偏移 503，则相对偏移 3
                        //domon：最终相对于首个时刻的码偏移
                        //index_c=index_c+index-deltaP(j)*ts;%伪距调整
                        //载波
                        index_carrier=((carrierTimeCoef[3]/6*t+carrierTimeCoef[2]/2)*t+carrierTimeCoef[1])*t+carrierTimeCoef[0];//每个时间点的传播时间
                        theta=f_t*index_carrier;
                        //carrier=cos(2*PI*fo*t-2*PI*theta);//软件
                        carrier=cos(2*PI*fo*(t+s_over)+2*PI*theta);//CGS9000: - , OEM: +
                        index_code=floor(mod(index_c*codeFreqbasis,paras->codeLength));
                        index_sub=(int)floor(index_c*d_sub)%12; //12 values of sub_carrier
                        bin_code=paras->bin[j].data[index_code];
                        cin_code=paras->cin[j].data[index_code];
                        cin_sub_code=e1c_sub_code[(int)floor((index_c*codeFreqbasis)/paras->codeLength)%25]; //辅码25个
                        //data_NAV =getgpsdata(frame[j],signalbits(j,:));
                        index_nav=floor(index_c*250+floor(temp[j])/4);//查找导航电文数据index，250bit/s, 4ms/bit
                        nav_up=paras->nav[j].data[index_nav];
                        //printf("index_m: %10.12f,index_c: %10.12f,index_code: %d,carrier:%10.12f,ca_code:%d,nav_up:%d,k:%d \n",index_m,index_c,index_code,carrier,ca_code,nav_up,k);
                        S[j].s[k]=0.7017*carrier*(base_a[bin_code^nav_up][index_sub]-base_b[cin_code^cin_sub_code][index_sub]);//max abs 1.9069251784911846308935518430544
                        //*二分之根号二后 MAX 1.3484
                    }
                }
                else{
                    //GPS part
                    for(k=offset,n=0;n<dots;k++,n++){
                        t=k*ts; //TODO::leap is not sure
                        codeTimeCoef = T[j].codechange+4*i;
                        carrierTimeCoef = T[j].carrierchange+4*i;
                        index_m=((codeTimeCoef[3]/6*t+codeTimeCoef[2]/2)*t+codeTimeCoef[1])*t+codeTimeCoef[0];//每个时间点的传播时间
                        //用累加器实现三阶多项式
                        index_tsv=(t_user+t)-index_m;//每个时间点的发射时刻-------------(t_user+t == new t_user time)
                        //************************************domon change,remove floor, proved wrong
                        index_c=index_tsv-tow[j]-floor(temp[j])/1000;// domon:相对于初始时刻的偏移量，举例：0时刻帧内偏移 500，接下来的9时刻偏移 503，则相对偏移 3
                        //domon：最终相对于首个时刻的码偏移
                        //index_c=index_c+index-deltaP(j)*ts;%伪距调整
                        //载波
                        index_carrier=((carrierTimeCoef[3]/6*t+carrierTimeCoef[2]/2)*t+carrierTimeCoef[1])*t+carrierTimeCoef[0];//每个时间点的传播时间
                        theta=f_t*index_carrier;
                        //carrier=cos(2*PI*fo*t-2*PI*theta);//软件
                        carrier=cos(2*PI*fo*(t+s_over)+2*PI*theta);//CGS9000: - , OEM: +
                        index_code=floor(mod(index_c*codeFreqbasis,1023));
                        cin_code=paras->ca[j].data[index_code];//GPS
                        index_nav=floor(index_c*50+floor(temp[j])/20);//查找导航电文数据index，50bit/s, 20ms/bit
                        nav_up=paras->nav[j].data[index_nav];
                        //printf("index_m: %10.12f,index_c: %10.12f,index_code: %d,carrier:%10.12f,ca_code:%d,nav_up:%d,k:%d \n",index_m,index_c,index_code,carrier,ca_code,nav_up,k);
                        S[j].s[k]=carrier*(((cin_code^nav_up)==0)?1:-1);//max abs 1.9069251784911846308935518430544
                        double debug_s = S[j].s[k];
                        debug_s++;
                    }
                }
            }
            //printf("Debug: offset: %d, k: %d,index_c = %9.10f, index_nav=%d,index_code=%d,p_id=%d,wr_offset=%d \n", offset, k, index_c,index_nav,index_code,p_id,wr_offset);
            t_user+=Period;
			wr_index=offset/4;
            for(k=offset,n=0;n<dots;k++,n++){
                //S[sat_nums]对应多星信号，由于采用缓冲，所以有sat_nums的地方都要加上读写偏移wr_offset，bug2
                S[sat_nums].s[k+wr_offset] = 0;
                for(j=0;j<sat_nums;j++){
                    S[sat_nums].s[k+wr_offset]+=S[j].s[k];
                }
                encode_signal_2bit(S+sat_nums,sat_nums+0.35,k+wr_offset);//7.628=1.907*4(sat_nums), 0.35 for 1 Gal
				if(k%4 == 3){
					//ms->wr_buf[wr_index+wr_offset]=(unsigned char)S[sat_nums].s[k+wr_offset]<<6 + (unsigned char)S[sat_nums].s[k-1+wr_offset]<<4 + (unsigned char)S[sat_nums].s[k-2+wr_offset]<<2 + (unsigned char)S[sat_nums].s[k-3+wr_offset];
					k1=(unsigned char)S[sat_nums].s[k+wr_offset]<<6;
					k2=(unsigned char)S[sat_nums].s[k-1+wr_offset]<<4;
					k3=(unsigned char)S[sat_nums].s[k-2+wr_offset]<<2;
					k4=(unsigned char)S[sat_nums].s[k-3+wr_offset];
					ms->wr_buf[wr_index+wr_offset]=k1+k2+k3+k4;
					//debugggg= ms->wr_buf[wr_index+wr_offset];
					wr_index++;
				}
                //ms->wr_buf[k+wr_offset]=(char)S[sat_nums].s[k+wr_offset];
            }
            ms->status_mask[p_id]=1;
            p_id=(p_id+p_num)%(2*p_num);
            wr_offset=(p_id<p_num)?0:dots*p_num;
        }
        else{
            i--;//重新来过
            printf("Debug: write so slowly, aaaaaaaaaaaaa \n");
            Sleep(100);
        }
    }
    //线程结束标志置位
    End_Flag|=End_Bit[p_id%p_num];
	return 0;
}
//信号文件写入线程，创立2个信号生成空间，一个用于信号生成缓冲区，相当于可以容纳0.6s的信号
#ifdef WIN32
static DWORD WINAPI write_cb(void *arg)
#else
static void *write_cb(void *arg)
#endif
{
    int i,n,ready;
    double Period=3*sig_arg.p->space_update;
    double fs=sig_arg.p->fs;
    int p_num=sig_arg.pthread_num;
    int dots=Period*fs;
    char *buf=(char *)arg;
	FILE *fp = fopen(g_paras.Sig_Path,"wb+");
    //Generate 0 data file to test
    FILE *fp_null = fopen(NULL_FILE,"wb+");
    char *buf_null = (char *)malloc(sizeof(char)*dots);
    memset(buf_null,0,sizeof(char)*dots);
    //test end
    int id_offset=0,buf_offset=0,count=0;
    while(1){
        ready=1;
        for(i=id_offset,n=0;n<p_num;n++,i++){
            if(sig_arg.status_mask[i]==0)ready=0;
        }
        if(ready){
            //Generate 0 data file to test
            //fwrite(buf_null,sizeof(char),dots,fp_null);
            //test end
            n=fwrite(buf+buf_offset,sizeof(unsigned char),dots/4,fp);
            if(n!=(dots/4)){
                printf("ERROR: Write Wrong !!! result=%d \n", n);
                fclose(fp);
                exit(-1);
            }
            for(i=id_offset,n=0;n<p_num;n++,i++){
                sig_arg.status_mask[i]=0;
            }
            buf_offset=(buf_offset+dots)%(2*dots);
            id_offset=(id_offset+p_num)%(2*p_num);
            count++;
            printf("INFO: Everything is OK, dots = %d, pthread_num = %d, Process = %d, offset=%d \n", dots/4, p_num, count, buf_offset);
        }
        else{//printf("INFO: Data is not ready for Writing !! \n");
            Sleep(100);
        }
        //信号生成线程结束，退出写线程
        if(End_Flag==END_COMING)break;
    }
    fclose(fp);
	return 0;
}
/***************************************
 * 各线程开始前的准备工作
 * in：T, trans paras time model
 * in: t_user, first recived user time(time in a week)
 * in: setting, init set paras
 * in: ok_nums, sat num to be dealed with
 * S: to store signal for being written to file
 * Func: caculate the nav start time: tow[i] and prn code start time: temp[i]
 * ************************************/
int signalT(_init_paras setting, T_MODEL *T, SIGNAL_I *S, double t_user, int ok_nums){
    int i,ret;
    double t_sv;
    thread_t tid_write;
    sig_arg.ok_nums=ok_nums;
    sig_arg.s=S;
    sig_arg.p=&setting;
    sig_arg.t=T;
    sig_arg.t_user=t_user;
    sig_arg.pthread_num=PTHREAD_NUM;
    for(i=0;i<ok_nums;i++){
        int sat_id=T[i].PRN;
        //Galileo, tow=nav start time(nav data offset 0), temp=nav start pos to generate signal(mod 4 for galileo,mod 1 for GPS)
        if(T[i].PRN > MAXPRNGPS){
            t_sv=T[i].sv_time[0].t_svCodeTime;
            tow[i]=t_user-1;//floor(t_sv/2)*2;//TODO:: Galileo时间,奇数秒 2s/page, tow+=2, not sure
            // 6000----代表一帧对应的 6000ms
            //domon:: note for bug: make 4ms aligned for galileo system
            temp[i]=floor((t_sv-tow[i])*1000/4)*4;//get the int 4ms start to same with 4ms ca code
            double debug1= temp[i];
            frame[i]=1999-floor(temp[i]);//以整数毫秒为帧的起始，为了对其进行ca码与数据码对齐,暂时没有使用
        }
        else{
            t_sv=T[i].sv_time[0].t_svCodeTime;
            tow[i]=t_user-6;//floor(t_sv/2)*2;//TODO:: Galileo时间, 2s/page, tow+=2, not sure
            // 6000----代表一帧对应的 6000ms
            //domon:: note for bug: make 4ms aligned for galileo system
            temp[i]=floor((t_sv-tow[i])*1000);//get the int 1ms start to same with 1ms ca code
            double debug2= temp[i];
            frame[i]=5999-floor(temp[i]);//以整数毫秒为帧的起始，为了对其进行ca码与数据码对齐,暂时没有使用
        }
    }
	/*************************
	reference
	#ifdef WIN32
    if (!(svr->thread=CreateThread(NULL,0,strsvrthread,svr,0,NULL))) {
	#else
    if (pthread_create(&svr->thread,NULL,strsvrthread,svr)) {
	#endif
	*************************/
    for(i=0;i<PTHREAD_NUM;i++){
        p_id[i]=i;
#ifdef WIN32
    if (!(p_tid[i] =CreateThread(NULL,0,signal_cb,p_id+i,0,NULL)))
#else
        ret=pthread_create(p_tid+i,NULL,signal_cb,p_id+i);
#endif
        if (ret != 0) {
                printf("FATAL: Failed to create a new thread (rate counter) - exiting \n");
        }
    }
#ifdef WIN32
    if (!(tid_write =CreateThread(NULL,0,write_cb,sig_arg.wr_buf,0,NULL))) 
#else
        ret=pthread_create(&tid_write,NULL,write_cb,sig_arg.wr_buf);
#endif
    if (ret != 0) {
            printf("FATAL: Failed to create a new thread (rate counter) - exiting \n");
    }
    printf("INFO: Bye Bye !!!!! \n");
	for(i=0;i<PTHREAD_NUM;i++){
#ifdef WIN32
		WaitForSingleObject(p_tid[i],INFINITE);
		CloseHandle(p_tid[i]);
#else
		pthread_join(p_tid[i],NULL);
#endif
	}
#ifdef WIN32
    WaitForSingleObject(tid_write,INFINITE);
    CloseHandle(tid_write);
#else
    pthread_join(tid_write,NULL);
#endif
	return 0;
}
///////////////////////////////////////////////////

/*int raw_main(){
	T_MODEL T[SAT_NUMS];//set sat nums to make
    SIGNAL_I S[SAT_NUMS+1];//0-8 five sats, 9--->plus result
	nav_t nav={0};
    //int sat[9]={3,8,14,16,23,25,26,29,32};
    int sat[SAT_NUMS]={3,8,14,16,23,25,26,29,32};
	int week,i,j;
	double ep[]={2015,6,26,1,15,6};
	double delt = 0.085;
	_init_paras test_pos;
	init_read_config();
	init_user_paras(&test_pos);
	rr[0]=test_pos.user_pos.x;
	rr[1]=test_pos.user_pos.y;
	rr[2]=test_pos.user_pos.z;
	printf("pos %f %f %f ....\n", test_pos.user_pos.x,test_pos.user_pos.y,test_pos.user_pos.z);
	for(i=0;i<10;i++){
		generateCAcodeS(sat[i],test_pos.ca[i].data);
		for(j=0;j<1023;j++){
            printf("%d ",test_pos.ca[i].data[j]);
		}
	}
    read_nav_bin("E:\\test_rtklib_ok\\build-test_rtk-Desktop_Qt_5_6_0_MinGW_32bit-Debug\\debug\\final_dat_439200", &test_pos, sat, SAT_NUMS);
	printf("\n\n\n\n\n");
	for(i=0;i<NAV_LEN;i++){
				//p->nav[index].data[i]=(data[i]-48)*2-1;
				printf("%d", test_pos.nav[0].data[i]);
	}
    Get_ok_sat(test_pos,&nav,sat);
    getTimeModelStatic(T,rr,static_user_pos,&nav,sat,9,epoch2time(ep),delt,1800);
	double t_user = time2gpst(epoch2time(ep),&week);
	signalT(test_pos,T,S,0,t_user,NULL,"oooooo");
	printf("it is ok, free T\n");
	//5===sat nums
    for(i=0;i<SAT_NUMS;i++){
		FREE_T_MODEL(T+i);
		FREE_SIGNAL_I(S+i);
	}
    FREE_SIGNAL_I(S+SAT_NUMS);
	free_paras(&test_pos);
}*/
/**********************************************************
 * in: min, signal len
 * in: Is_Simple_Way, true means use nav data from reciever,false means generate on your own
 * 
 * *******************************************************/
extern int main_signal(int min, int Is_Simple_Way){
    int sat[MAX_SAT];
    //double rr[3] = {0};//user ecef
    int i,j,k,dots,ok_nums;
    //char *wr_buf;
    nav_t nav={0};
	LLH *llh=NULL;
    _init_paras gal_paras;
    T_MODEL T[MAX_SAT];//set sat nums to make
    SIGNAL_I S[MAX_SAT];//0-8 five sats, 9--->plus result
    double t_user;
    //TODO::tt:time of user start
    gtime_t tt;
    double ep[]={2016,6,26,1,15,6};

	//这里-6操作是为了兼容，自己组的导航电文为min，生成的信号要比其短至少6s，保证电文不会用完
    INIT_PARAS(&gal_paras, min*60-6);//min*60-6
	//添加动态部分,多加1s，解决插值的临界问题
	if(g_paras.mode == DYN_MODE){
		llh = (LLH *)malloc(sizeof(LLH)*(60*min+1));
		if(g_paras.Kml_Path)
			prase_mkl(g_paras.Kml_Path,60*min+1,llh);
		else
			printf("ERROR: no kml path given to me \n");
	}
	//采用采集的的导航电文，称之为简单模式
	if(Is_Simple_Way){
        ok_nums=Decode_and_Encode_Simple_Way(&nav,&gal_paras,min,sat,&tt);
    }
    else{
        INIT_NAV(&nav);
        //tt: get the time of user == tow + 2 same with gps tow==next frame start
        ok_nums=Decode_and_Encode(&nav,&gal_paras,min,sat,&tt);
        //ok_nums=Get_ok_sat(gal_paras,&nav,sat);
    }
	if(ok_nums < 1){
		printf("ERROR: read nav hex data error \n");
		Sleep(10000);
		exit(-1);
	}
    dots=3*gal_paras.fs*gal_paras.space_update; //用于三阶多项式拟合，3个最小时间段-最终生成信号点数
    for(k=0;k<ok_nums;k++){
        INIT_T_MODEL(T+k,gal_paras.mstoProcess/gal_paras.space_update,sat[k]);
        INIT_SIGNAL_I(S+k,dots,0);
    }
    INIT_SIGNAL_I(S+ok_nums,2*dots,0);//多颗星合成信号存储位置，带一个缓冲区
    //final quantization signal double buffer malloc
    sig_arg.wr_buf=(unsigned char *)malloc(2*dots*sizeof(unsigned char));

    //伪码生成，Gal 1st,GPS 2nd
    Generate_Baseband(FILE_B,FILE_C,&gal_paras);
    for(i=0;i<ok_nums;i++){
        if(sat[i]<=32){
            gal_paras.ca[i].sat = sat[i];
            generateCAcodeS(sat[i],gal_paras.ca[i].data);
            for(j=0;j<1023;j++){
                //printf("%d ",gal_paras.ca[i].data[j]);
            }
        }
    }
    printf("\n");
    getTimeModelStatic(T,&gal_paras,&nav,sat,ok_nums,tt,llh);
    signalT(gal_paras,T,S,time2gpst(tt,NULL),ok_nums);

    for(k=0;k<ok_nums;k++){
        FREE_T_MODEL(T+k);
        FREE_SIGNAL_I(S+k);
    }
	FREE_SIGNAL_I(S+ok_nums);
    //final quantization signal double buffer free
    free(sig_arg.wr_buf);
    FREE_PARAS(&gal_paras,ok_nums);
    FREE_NAV(&nav);
	if(g_paras.mode == DYN_MODE) free(llh);
    printf("sat ok sum-num is %d \n", ok_nums);
    return 0;
}

