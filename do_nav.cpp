#include <stdio.h>
#include <string.h>
#include "do_nav.h"
#include "do_files.h"
#include "rtklib.h"

/* get fields (little-endian) ------------------------------------------------
 * #define U1(p) (*((unsigned char *)(p)))
 * #define I1(p) (*((char *)(p)))
 * static unsigned short U2(unsigned char *p) {unsigned short u; memcpy(&u,p,2); return u;}
 * static unsigned int   U4(unsigned char *p) {unsigned int   u; memcpy(&u,p,4); return u;}
 * static short          I2(unsigned char *p) {short          i; memcpy(&i,p,2); return i;}
 * static int            I4(unsigned char *p) {int            i; memcpy(&i,p,4); return i;}
 * static float          R4(unsigned char *p) {float          r; memcpy(&r,p,4); return r;}
 * static double         R8(unsigned char *p) {double         r; memcpy(&r,p,8); return r;}
 * *********************************************/
typedef void (*Func)(int, int, nav_t *, u_int8 *);

extern G_PARAS g_paras;

static const unsigned char sync[10]={0,1,0,1,1,0,0,0,0,0};
//对应30s长的子帧的两种字类型组成
static const int word_type[2][15]={2,4,6,7,8,-1,-1,-1,-1,-1,1,3,5,0,0, \
                       2,4,6,9,10,-1,-1,-1,-1,-1,1,3,5,0,0};
static _gal_utc gal;
static int g_tow,g_week,g_ioda;
static int g_init_tow;//init tow decode frome trimble
/* extract field (big-endian) ------------------------------------------------*/
#define U1(p)       (*((unsigned char *)(p)))
#define I1(p)       (*((char *)(p)))

static unsigned short U2(unsigned char *p)
{
    union {unsigned short u2; unsigned char b[2];} buff;
    buff.b[0]=p[1]; buff.b[1]=p[0];
    return buff.u2;
}
static unsigned int U4(unsigned char *p)
{
    union {unsigned int u4; unsigned char b[4];} buff;
    buff.b[0]=p[3]; buff.b[1]=p[2]; buff.b[2]=p[1]; buff.b[3]=p[0];
    return buff.u4;
}
static double R8(unsigned char *p)
{
    union {double r8; unsigned char b[8];} buff;
    buff.b[0]=p[7]; buff.b[1]=p[6]; buff.b[2]=p[5]; buff.b[3]=p[4];
    buff.b[4]=p[3]; buff.b[5]=p[2]; buff.b[6]=p[1]; buff.b[7]=p[0];
    return buff.r8;
}

/*******************
 * in: hex string buffer
 * out: unsigned byte array
 * return: num of bytes
 * *******************/
int hexstr2byte(char *buf,unsigned char *ret){
    int i,len=strlen(buf)/2;
    char p[3];
    if(!ret)return -1;
    for(i=0;i<len;i++){
        strncpy(p,buf+i*2,2);
        sscanf(p,"%x",ret+i);
    }
    return len;
}

/****************************
 *
 * return: sat num
 * *************************/
int Get_raw_trimble_data(char *eph_file, char *alm_file, char *utc_file,_raw_trimble *raw){
    char line[LINE_BUF_LEN];
    char *s;
    int count=0;
    FILE *fe,*fa,*fu;
    if (!(fe=fopen(eph_file,"r"))) {
        fprintf(stderr,"nav file open error : %s\n",eph_file);
        return -1;
    }
    if (!(fa=fopen(alm_file,"r"))) {
        fprintf(stderr,"nav file open error : %s\n",alm_file);
        return -1;
    }
    if (!(fu=fopen(utc_file,"r"))) {
        fprintf(stderr,"nav file open error : %s\n",utc_file);
        return -1;
    }
    while (!feof(fe) && fgets(line, LINE_BUF_LEN, fe)){
        s = line;
        if (s[strlen(s) - 1] == '\n')//change the line end to string end
            s[strlen(s) - 1] = '\0';
        hexstr2byte(s, (unsigned char *)raw->eph+count);
        count++;
    }
    count=0;
    while (!feof(fa) && fgets(line, LINE_BUF_LEN, fa)){
        s = line;
        if (s[strlen(s) - 1] == '\n')//change the line end to string end
            s[strlen(s) - 1] = '\0';
        hexstr2byte(s, (unsigned char *)raw->alm+count);
        count++;
    }
    while (!feof(fu) && fgets(line, LINE_BUF_LEN, fu)){
        s = line;
        if (s[strlen(s) - 1] == '\n')//change the line end to string end
            s[strlen(s) - 1] = '\0';
        hexstr2byte(s, (unsigned char *)raw->utc_ion);
    }
    raw->sat_num=count;
    printf("INFO: the num of galileo is %d:\n", count);
    return count;
}

/****************************refered to init_raw()
 * func: init nav for future use
 * in: nav_t nav
 * return: status
 * *************************/
extern int INIT_NAV(nav_t *nav){
    int i;
    eph_t  eph0 ={0,-1,-1};
    alm_t  alm0 ={0,-1};
    geph_t geph0={0,-1};
    nav->eph  =NULL;
    nav->alm  =NULL;
    nav->geph =NULL;
    if (!(nav->eph  =(eph_t  *)malloc(sizeof(eph_t )*MAXSAT))||
        !(nav->alm  =(alm_t  *)malloc(sizeof(alm_t )*MAXSAT))||
        !(nav->geph =(geph_t *)malloc(sizeof(geph_t)*NSATGLO))) {
        FREE_NAV(nav);
        return 0;
    }
    nav->n =MAXSAT;
    nav->na=MAXSAT;
    nav->ng=NSATGLO;
    for (i=0;i<MAXSAT   ;i++) nav->eph  [i]=eph0;
    for (i=0;i<MAXSAT   ;i++) nav->alm  [i]=alm0;
    for (i=0;i<NSATGLO  ;i++) nav->geph [i]=geph0;
    return 1;
}
/***************************
 * free nav above
 * ************************/
extern void FREE_NAV(nav_t *nav)
{
    printf("free_raw:\n");

    free(nav->eph  ); nav->eph  =NULL; nav->n =0;
    free(nav->alm  ); nav->alm  =NULL; nav->na=0;
    free(nav->geph ); nav->geph =NULL; nav->ng=0;
}

//reference: rtklib: decode_galephemerisb();
/* decode trimble sv_data(55H) mesaage: decoded galileo ephmemeris ------------------*/
static int decode_trimble_eph(nav_t *nav, unsigned char *buff, int len)
{
    eph_t eph={0};
    unsigned char *p=buff;
    double tow,week,toc,ura,sqrtA;
    int prn, hsdvs;
    unsigned char model1,model2,sisa;

    printf("trimble nav message: len=%d\n",len);

    if (len>=7) {
        prn       =U1(p);     	 p+=1; /* add the gps offset */ // 5
        eph.code  =U1(p);        p+=1; // data source           // 6
        week      =U2(p);        p+=2;                          // 7-8
        tow       =U4(p);        p+=4;                          // 9-12
        eph.iode  =U2(p);        p+=2; // IODnav                // 13-14
        eph.toes  =U4(p);        p+=4;                          // 15-18
        eph.crs   =R8(p);        p+=8;                          // 19-26
        eph.deln  =R8(p)*PI;     p+=8;                          // 17-34
        eph.M0    =R8(p)*PI;     p+=8;                          // 35-42
        eph.cuc   =R8(p)*PI;     p+=8;                          // 43-50
        eph.e     =R8(p);        p+=8;                          // 51-58
        eph.cus   =R8(p)*PI;     p+=8;                          // 59-66
        sqrtA     =R8(p);        p+=8;                          // 67-74
        eph.cic   =R8(p)*PI;     p+=8;                          // 75-82
        eph.OMG0  =R8(p)*PI;     p+=8;                          // 83-90
        eph.cis   =R8(p)*PI;     p+=8;                          // 91-97
        eph.i0    =R8(p)*PI;     p+=8;                          // 98-105
        eph.crc   =R8(p);        p+=8;                          // 106-113
        eph.omg   =R8(p)*PI;     p+=8;                          // 114-122
        eph.OMGd  =R8(p)*PI;     p+=8;                          // 123-130
        eph.idot  =R8(p)*PI;     p+=8;                          // 131-138
        sisa      =U1(p);        p+=1;                          // 139
        hsdvs     =U2(p);        p+=2;                          // 140-141
        toc       =U4(p);        p+=4;                          // 142-145
        eph.f0    =R8(p);        p+=8;                          // 146-153
        eph.f1    =R8(p);        p+=8;                          // 154-161
        eph.f2    =R8(p);        p+=8;                          // 162-169
        eph.tgd[0]=R8(p);        p+=8; // BGD E5a/E1            // 170-177
        model1    =U1(p);        p+=1;                          // 178
        eph.tgd[1]=R8(p);        p+=8; // BGD E5b/E1            // 179-186
        model2    =U1(p);                                       // 187
    }
    else {
        printf("binex 0x01-04: length error len=%d\n",len);
        return -1;
    }
    eph.sat=prn + MAXPRNGPS; //for galileo
    eph.week=week-1024; // galileo week, trimble pdf say it,Bug 5 -1024 means gal's start time is not same with gps
    eph.A=sqrtA*sqrtA;
    eph.iodc=eph.iode;
    eph.toe=gst2time(eph.week,eph.toes);
    eph.toc=eph.toe;
    nav->eph[eph.sat-1]=eph;
    g_init_tow=tow;//global init tow :bug 4:make init tow once
    g_week=week-1024;//global
    printf("decode eph sat : %d ,week num =%d, tow= %d, toes= %f, toc=%f, crs= %10.13f \n",eph.sat, eph.week, g_tow, eph.toes, toc, eph.crs);
    return 2;
}

/* decode trimble sv_data(55H) mesaage: decoded galileo ephmemeris ------------------*/
static int decode_trimble_alm(nav_t *nav, unsigned char *buff, int len)
{
    alm_t alm={0};
    unsigned char *p=buff;
    double tow,toc,ura,sqrtA;
    int prn, week, dec_t, hsdvs;
    unsigned char ioda,a_health,a_src,model1,model2;

    printf("trimble message alm: len=%d\n",len);
    if (len>=7) {
        prn       =U1(p);     	 p+=1; /* add the gps offset */ // 5
        dec_t     =U4(p);        p+=4; // data source           // 6
        week      =U2(p);        p+=2;                          // 7-8
        alm.toas  =U4(p);        p+=4;                          // 9-12
        sqrtA     =R8(p);        p+=8; // IODnav                // 13-14
        alm.e     =R8(p);        p+=8;                          // 15-18
        alm.i0    =R8(p)*PI;     p+=8;                          // 19-26
        alm.OMGd  =R8(p)*PI;     p+=8;                          // 17-34
        alm.OMG0  =R8(p)*PI;     p+=8;                          // 35-42
        alm.omg   =R8(p)*PI;     p+=8;                          // 43-50
        alm.M0    =R8(p)*PI;     p+=8;                          // 51-58
        alm.svh   =U1(p);        p+=1;                          // 59-66
        alm.f0    =R8(p);        p+=8;                          // 67-74
        alm.f1    =R8(p);        p+=8;                          // 75-82
        a_src     =U1(p);        p+=1;                          // 83-90
        ioda      =U1(p);
    }
    alm.sat=prn+MAXPRNGPS;//gps offset should replace it
    alm.A=sqrtA*sqrtA;
    alm.week=week-1024;
    alm.toa=gst2time(alm.week,alm.toas);
    g_ioda=ioda;
    nav->alm[alm.sat-1]=alm;
    printf("decode alm sat : %d ,week num =%d \n",alm.sat, alm.week);
    return 2;
}

static int decode_trimble_utc_ion(nav_t *nav, unsigned char *buff, int len){
    unsigned char *p=buff+1;//not care prn, so plus 1
    double ai[4];
    int i,bi[4];

    if (len<=7) {
        printf("oem4 galclockb length error: len=%d\n",len);
        return -1;
    }
    ai[0]=R8(p); p+=8;
    ai[1]=R8(p); p+=8;
    ai[2]=R8(p); p+=8;
    ai[3]=R8(p); p+=8;
    bi[0]=R8(p); p+=8;
    bi[1]=R8(p); p+=8;
    bi[2]=R8(p); p+=8;
    bi[3]=R8(p); p+=8;
    gal.a0   =R8(p); p+=8;
    gal.a1   =R8(p); p+=8;
    gal.tot  =R8(p); p+=8;
    gal.dtls =R8(p); p+=8;
    gal.dtlsf=R8(p); p+=8;
    gal.ion_t=R8(p); p+=8;
    gal.wnt  =U1(p); p+=1;
    gal.wnlsf=U1(p); p+=1;
    gal.dn   =U1(p); p+=1;
    //reserved is ignored

    nav->utc_gal[0]=gal.a0;
    nav->utc_gal[1]=gal.a1;
    nav->utc_gal[2]=gal.tot; /* utc reference tow (s) */
    nav->utc_gal[3]=gal.wnt; /* utc reference week */
    for (i=0;i<3;i++) nav->ion_gal[i]=ai[i];
    printf("decode utc and ion paras \n");
    return 2;
}
//uint 8 转化为 10101010
void byte2bin(unsigned char *hex, int len, unsigned char *bin){
    int i,j;
    for(i=0;i<len;i++)
        for(j=0;j<8;j++){
            bin[8*i+j]=(hex[i]>>(7-j))&0x01;
        }
}


//10101010 8bytes---->1byte,右对齐，len == bin length
void bin2byte_right_align(unsigned char *bin, int len, unsigned char *crc){
    int n=len/8;
    int k=len%8;
    int i,j;
    unsigned char s=0;
    for(i=0;i<k;i++){
        s^=bin[i]<<(k-1-i);
    }
    crc[0]=s;
    s=0;
    for(i=0;i<n;i++){
        for(j=0;j<8;j++){
            s^=bin[8*i+j+k]<<(7-j);
        }
        crc[i+1]=s;
        s=0;
    }
}

/*************************************
 * func: 128bit word to even & odd page
 * in: word, 128bit==15byte galileo word
 * out: buf1, even page 120bit
 * out: buf2, odd page 120bit
 * **********************************/
void word2bits(unsigned char *word, unsigned char *buf1, unsigned char *buf2){
    unsigned char reserved1[5]={0x55,0x55,0x55,0x55,0x55};
    unsigned char sar_spare[3]={0xaa,0xaa,0xa8};
    unsigned char reserved2[1]={0x55};
    unsigned char crc_in[25],bits[256],crc[3];
    unsigned int iiiiiiiiiii;
    union _crc_out{unsigned int crc_i;unsigned char crc_c[4];}crc_out;
    bits[0]=bits[1]=0;               //1-2
    byte2bin(word,14,bits+2);        //3-114
    bits[114]=1;bits[115]=0;         //115-116
    byte2bin(word+14,2,bits+116);    //117-132
    byte2bin(reserved1,5,bits+132);  //133-172
    byte2bin(sar_spare,3,bits+172);  //173-196, 196 bits is protected by crc24
    bin2byte_right_align(bits,196,crc_in); //right align for crc, 196 bits
    crc_out.crc_i=crc24q(crc_in,25);
    crc[0]=crc_out.crc_c[2];
    crc[1]=crc_out.crc_c[1];
    crc[2]=crc_out.crc_c[0];
    byte2bin(crc,3,bits+196);//197-220
    byte2bin(reserved2,1,bits+220);  //221-228
    //domon:: note for bug: replace word to bits
    memcpy(buf1,bits,sizeof(unsigned char)*114);    //114 + 6 bit tail == 120bit
    memcpy(buf2,bits+114,sizeof(unsigned char)*114);//114 + 6 bit tail == 120bit
}

/*****************************************
 * func: 1/2 FEC coding, k=7
 * in: 120*byte(0/1)
 * out: 240*byte(0/1),should be memset to 0
 *
 * **************************************/
void FEC_Encoding(unsigned char *in, unsigned char *out){
    const unsigned char g1[5]={0,1,2,3,6};//1111001, 1 offset to ist 1
    const unsigned char g2[5]={0,2,3,5,6};//1011011, 1 offset to ist 1
    unsigned char cp[126]={0};
    int i,j;
    unsigned char s1,s2;
    memcpy(cp+6,in,120*sizeof(unsigned char));// add 6 zero to the k=7 FEC Encoder
    for(i=0;i<120;i++){
        s1=0;s2=0;
        for(j=0;j<5;j++){
            s1+=cp[i+6-g1[j]];
            s2+=cp[i+6-g2[j]];
        }
        s2+=1;
        out[2*i]=s1%2;
        out[2*i+1]=s2%2;
    }
}

/********************************************
 * func: interleaving block encoding, 30*8 which is shown in the ICD
 * in: 240*byte(0/1)
 * out: 240*byte(0/1)
 * in: row, interleaving 30 in galile
 * out: col, interleaving 8 in galile
 * head: sync head 10*byte(0/1)
 * *****************************************/
void Interleaving_Encoding(unsigned char *in, int row, int col, unsigned char *out){
    int i,j;
    for(i=0;i<col;i++)
        for(j=0;j<row;j++)
            out[i*row+j]=in[j*col+i];
}

void encode_word1to4(int prn, nav_t *nav, u_int8 *word1, u_int8 *word2, u_int8 *word3, u_int8 *word4){
    eph_t *eph=nav->eph+prn-1;
    unsigned int sqrtA,e;
    int i=0,week,toe,toc,i0,OMG0,omg,M0,deln,idot,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,bgd1,bgd2,oshs,osdvs;
    int nbit;
    //trace(3,"encode_word 1-4: sync=%d\n",sync);

    //if (satsys(rtcm->ephsat,&prn)!=SYS_GAL) return 0;
    if (eph->sat!=prn) return;
    week=eph->week%4092;
    toe  =ROUND(eph->toes/60.0);
    toc  =ROUND(time2gpst(eph->toc,NULL)/60.0);
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_5 );
    crc  =ROUND(eph->crc/P2_5 );
    cus  =ROUND(eph->cus/P2_29);
    cuc  =ROUND(eph->cuc/P2_29);
    cis  =ROUND(eph->cis/P2_29);
    cic  =ROUND(eph->cic/P2_29);
    af0  =ROUND(eph->f0 /P2_34);
    af1  =ROUND(eph->f1 /P2_46);
    af2  =ROUND(eph->f2 /P2_59);
    bgd1 =ROUND(eph->tgd[0]/P2_32); /* E5a/E1 */
    bgd2 =ROUND(eph->tgd[1]/P2_32); /* E5b/E1 */
    oshs =(eph->svh>>4)&3;
    osdvs=(eph->svh>>3)&1;
    setbitu(word1,i,6,PAGE_TYPE1); i+=6;
    setbitu(word1,i,10,eph->iode); i+=10;
    setbitu(word1,i,14,toe      ); i+=14;
    setbitu(word1,i,32,M0       ); i+=32;
    setbitu(word1,i, 32,e       ); i+=32;
    setbitu(word1,i,32,sqrtA    ); i+=32; i=0;
    setbitu(word2,i,6,PAGE_TYPE2); i+=6;
    setbits(word2,i,10,eph->iode); i+=10;
    setbits(word2,i,32,OMG0     ); i+=32;
    setbits(word2,i,32,i0       ); i+=32;
    setbits(word2,i,32,omg      ); i+=32;
    setbits(word2,i,14,idot     ); i+=14; i=0;
    setbitu(word3,i,6,PAGE_TYPE3); i+=6;
    setbitu(word3,i,10,eph->iode); i+=10;
    setbits(word3,i,24,OMGd     ); i+=24;
    setbits(word3,i,16,deln     ); i+=16;
    setbitu(word3,i,16,cuc      ); i+=16;
    setbits(word3,i,16,cus      ); i+=16;
    setbitu(word3,i,16,crc      ); i+=16;
    setbitu(word3,i,16,crs      ); i+=16; i=0;
    setbitu(word4,i,6,PAGE_TYPE4); i+=6;
    setbitu(word4,i,10,eph->iode); i+=10;
    setbits(word4,i,6,prn       ); i+=6;
    setbits(word4,i,16,cic      ); i+=16;
    setbits(word4,i,16,cis      ); i+=16;
    setbits(word4,i,14,toc      ); i+=14;
    setbits(word4,i,31,af0      ); i+=31;
    setbits(word4,i,21,af1      ); i+=21;
    setbits(word4,i,6,af2       ); i+=6;
    setbitu(word4,i, 2,0        ); i+=2; /* reserved */
    nbit=i;
    //return nbit;
}
//This Prn should be plused with GPSoffset--MAXPRNGPS
void encode_word1(int prn, int sv_id, nav_t *nav, u_int8 *word){
    eph_t *eph=nav->eph+prn-1;
    unsigned int sqrtA,e;
    int i=0,iode,toe,M0;
    int nbit;
    //trace(3,"encode_word 1-4: sync=%d\n",sync);

    //if (satsys(rtcm->ephsat,&prn)!=SYS_GAL) return 0;
    if (eph->sat!=prn) return;
    iode = eph->iode;
    toe  =ROUND_U(eph->toes/60.0);
    sqrtA=ROUND_U(sqrt(eph->A)/P2_19);
    e    =ROUND_U(eph->e/P2_33);
    M0   =ROUND(eph->M0  /P2_31/SC2RAD);
    setbitu(word,i,6,PAGE_TYPE1); i+=6;
    setbitu(word,i,10,iode     ); i+=10;
    setbitu(word,i,14,toe      ); i+=14;
    setbits(word,i,32,M0       ); i+=32;
    setbitu(word,i,32,e        ); i+=32;
    setbitu(word,i,32,sqrtA    ); i+=32;
    nbit=i;
    //return nbit;
}
void encode_word2(int prn, int sv_id, nav_t *nav, u_int8 *word){
    eph_t *eph=nav->eph+prn-1;
    int i=0,iode,i0,OMG0,omg,idot;
    int nbit;
    //trace(3,"encode_word 1-4: sync=%d\n",sync);

    //if (satsys(rtcm->ephsat,&prn)!=SYS_GAL) return 0;
    if (eph->sat!=prn) return;
    iode = eph->iode;
    i0   =ROUND(eph->i0  /P2_31/SC2RAD);
    OMG0 =ROUND(eph->OMG0/P2_31/SC2RAD);
    omg  =ROUND(eph->omg /P2_31/SC2RAD);
    idot =ROUND(eph->idot/P2_43/SC2RAD);
    setbitu(word,i,6,PAGE_TYPE2); i+=6;
    setbitu(word,i,10,iode     ); i+=10;
    setbits(word,i,32,OMG0     ); i+=32;
    setbits(word,i,32,i0       ); i+=32;
    setbits(word,i,32,omg      ); i+=32;
    setbitu(word,i,14,idot     ); i+=14;
    nbit=i;
    //return nbit;
}
void encode_word3(int prn, int sv_id, nav_t *nav, u_int8 *word){
    eph_t *eph=nav->eph+prn-1;
    int i=0,iode,deln,OMGd,crs,crc,cus,cuc;
    int nbit;
    //trace(3,"encode_word 1-4: sync=%d\n",sync);

    //if (satsys(rtcm->ephsat,&prn)!=SYS_GAL) return 0;
    if (eph->sat!=prn) return;
    iode = eph->iode;
    deln =ROUND(eph->deln/P2_43/SC2RAD);
    OMGd =ROUND(eph->OMGd/P2_43/SC2RAD);
    crs  =ROUND(eph->crs/P2_5 );
    crc  =ROUND(eph->crc/P2_5 );
    cus  =ROUND(eph->cus/P2_29);
    cuc  =ROUND(eph->cuc/P2_29);
    setbitu(word,i,6,PAGE_TYPE3); i+=6;
    setbitu(word,i,10,iode     ); i+=10;
    setbits(word,i,24,OMGd     ); i+=24;
    setbits(word,i,16,deln     ); i+=16;
    setbits(word,i,16,cuc      ); i+=16;
    setbits(word,i,16,cus      ); i+=16;
    setbits(word,i,16,crc      ); i+=16;
    setbits(word,i,16,crs      ); i+=16;
    nbit=i;
    //return nbit;
}
void encode_word4(int prn, int sv_id, nav_t *nav, u_int8 *word){
    eph_t *eph=nav->eph+prn-1;
    int i=0,iode,toc,cis,cic,af0,af1,af2;
    int nbit;
    //trace(3,"encode_word 1-4: sync=%d\n",sync);

    //if (satsys(rtcm->ephsat,&prn)!=SYS_GAL) return 0;
    if (eph->sat!=prn) return;
    iode =eph->iode;
    toc  =ROUND(time2gpst(eph->toc,NULL)/60.0);
    cis  =ROUND(eph->cis/P2_29);
    cic  =ROUND(eph->cic/P2_29);
    af0  =ROUND(eph->f0 /P2_34);
    af1  =ROUND(eph->f1 /P2_46);
    af2  =ROUND(eph->f2 /P2_59);
    setbitu(word,i,6,PAGE_TYPE4); i+=6;
    setbitu(word,i,10,iode     ); i+=10;
    //domon: bug3: remove the galileo offset in RTKLIB
    setbitu(word,i,6,prn-GAL_SATNUM_OFFSET); i+=6;
    setbits(word,i,16,cic      ); i+=16;
    setbits(word,i,16,cis      ); i+=16;
    setbitu(word,i,14,toc      ); i+=14;
    setbits(word,i,31,af0      ); i+=31;
    setbits(word,i,21,af1      ); i+=21;
    setbits(word,i,6,af2       ); i+=6;
    setbitu(word,i, 2,0        ); i+=2; /* reserved */
    nbit=i;
    //return nbit;
}
void encode_word5(int prn, int sv_id, nav_t *nav, u_int8 *word){
    eph_t *eph=nav->eph+prn-1;
    int i=0,ai0,ai1,ai2,bgd1,bgd2,oshs1,oshs2,osdvs1,osdvs2,week,tow;
    int Dis_Flag=0,spare=0;
    int nbit;
    ai0 =ROUND_U(nav->ion_gal[0]/P2_2);
    ai1 =ROUND(nav->ion_gal[1]/P2_8);
    ai2 =ROUND(nav->ion_gal[2]/P2_15);
    bgd1=ROUND(eph->tgd[0]/P2_32);
    bgd2=ROUND(eph->tgd[1]/P2_32);
    //oshs1 =(eph->svh>>4)&3;
    //osdvs1=(eph->svh>>3)&1;//??????????? TODO::
    oshs1=oshs2=osdvs1=osdvs2=0;
	week=eph->week%4092;
    tow=g_tow;
    setbitu(word,i,6,PAGE_TYPE5);       i+=6;
    setbits(word,i,11,ai0);             i+=11;
    setbits(word,i,11,ai1);             i+=11;
    setbits(word,i,14,ai2);             i+=14;
    setbitu(word,i,5,Dis_Flag);         i+=5;
    setbits(word,i,10,bgd1);            i+=10;
    setbits(word,i,10,bgd2);            i+=10;
    setbitu(word,i,2,oshs1);            i+=2;
    setbitu(word,i,2,oshs2);            i+=2;
    setbitu(word,i,1,osdvs1);           i+=1;
    setbitu(word,i,1,osdvs2);           i+=1;
    setbitu(word,i,12,week);            i+=12;
    setbitu(word,i,20,tow);             i+=20;//keyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
    setbitu(word,i,23,spare);           i+=23;
    nbit=i;
    //return nbit;
}
void encode_word6(int prn, int sv_id, nav_t *nav, u_int8 *word){
    int nbit,spare=0,i=0;
    int a0,a1,dtls,tot,wnt,wnlsf,dn,dtlsf,tow;
    a0=ROUND(gal.a0/P2_30);
    a1=ROUND(gal.a1/P2_50);
    dtls=ROUND(gal.dtls);
    tot=ROUND_U(gal.tot/3600.0);
    wnt=ROUND_U(gal.wnt);
    wnlsf=ROUND_U(gal.wnlsf);
    dn=ROUND_U(gal.dn);
    dtlsf=ROUND(gal.dtlsf);
    tow=g_tow;
    setbitu(word,i,6,PAGE_TYPE6);   i+=6;
    setbits(word,i,32,a0);          i+=32;
    setbits(word,i,24,a1);          i+=24;
    setbits(word,i,8,dtls);         i+=8;
    setbits(word,i,8,tot);          i+=8;
    setbits(word,i,8,wnt);          i+=8;
    setbits(word,i,8,wnlsf);        i+=8;
    setbits(word,i,3,dn);           i+=3;
    setbits(word,i,8,dtlsf);        i+=8;
    setbitu(word,i,20,tow);         i+=20;//keyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
    setbitu(word,i,3,spare);        i+=3;
    nbit=i;
    //return nbit;
}
void encode_word7(int prn, int sv_id, nav_t *nav, u_int8 *word){
    alm_t *alm=nav->alm+sv_id-1;
    unsigned int e;
    int i=0,delt_sqrtA,week,toa,toc,delt_i,OMG0,omg,M0,deln,ioda,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,bgd1,bgd2,oshs,osdvs;
    int nbit,reserved=0;
    ioda=g_ioda;
    week=ROUND_U(alm->week%4);
    toa =ROUND_U(alm->toas/600.0);
    delt_sqrtA=ROUND((sqrt(alm->A)-SQRTA_REF)/P2_9);
    e=   ROUND_U(alm->e/P2_16);
    omg= ROUND(alm->omg/P2_15/SC2RAD);
    delt_i=ROUND((alm->i0-I_REF)/P2_14/SC2RAD);
    OMG0=ROUND(alm->OMG0/P2_15/SC2RAD);
    OMGd=ROUND(alm->OMGd/P2_33/SC2RAD);
    M0=  ROUND(alm->M0/P2_15/SC2RAD);
    setbitu(word,i,6,PAGE_TYPE7);    i+=6;
    setbitu(word,i,4,ioda);          i+=4;
    setbitu(word,i,2,week);          i+=2;
    setbitu(word,i,10,toa);          i+=10;
    setbitu(word,i,6,sv_id);         i+=6;
    setbits(word,i,13,delt_sqrtA);   i+=13;
    setbitu(word,i,11,e);            i+=11;
    setbits(word,i,16,omg);          i+=16;
    setbits(word,i,11,delt_i);       i+=11;
    setbits(word,i,16,OMG0);         i+=16;//keyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
    setbits(word,i,11,OMGd);         i+=11;
    setbits(word,i,16,M0);           i+=16;
    setbitu(word,i,6,reserved);      i+=6;
    nbit=i;
    //return nbit;
}
void encode_word8(int prn, int sv_id, nav_t *nav, u_int8 *word){
    alm_t *alm_pre=nav->alm+sv_id-1;
    alm_t *alm_next=alm_pre+1;
    unsigned int e;
    int i=0,delt_sqrtA,week,toa,toc,delt_i,OMG0,omg,M0,deln,ioda,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,bgd1,bgd2,oshs1,oshs2,osdvs;
    int nbit,spare=0;
    ioda=g_ioda;
    af0 =ROUND(alm_pre->f0/P2_19);
    af1 =ROUND(alm_pre->f1/P2_38);
    oshs1=oshs2=0;//health:0
    delt_sqrtA=ROUND((sqrt(alm_next->A)-SQRTA_REF)/P2_9);
    e=ROUND_U(alm_next->e/P2_16);
    omg=ROUND(alm_next->omg/P2_15/SC2RAD);
    delt_i=ROUND((alm_next->i0-I_REF)/P2_14/SC2RAD);
    OMG0=ROUND(alm_next->OMG0/P2_15/SC2RAD);
    OMGd=ROUND(alm_next->OMGd/P2_33/SC2RAD);
    setbitu(word,i,6,PAGE_TYPE8);    i+=6;
    setbitu(word,i,4,ioda);          i+=4;
    setbits(word,i,16,af0);          i+=16;
    setbits(word,i,13,af1);          i+=13;
    setbitu(word,i,2,oshs1);         i+=2;
    setbitu(word,i,2,oshs2);         i+=2;
    setbitu(word,i,6,sv_id);         i+=6;
    setbits(word,i,13,delt_sqrtA);   i+=13;
    setbitu(word,i,11,e);            i+=11;
    setbits(word,i,16,omg);          i+=16;//keyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
    setbits(word,i,11,delt_i);       i+=11;
    setbits(word,i,16,OMG0);         i+=16;
    setbits(word,i,11,OMGd);         i+=11;
    setbitu(word,i,1,spare);         i+=1;
    nbit=i;
    //return nbit;
}
void encode_word9(int prn, int sv_id, nav_t *nav, u_int8 *word){
    alm_t *alm_pre=nav->alm+sv_id-1;
    alm_t *alm_next=alm_pre+1;
    unsigned int e;
    int i=0,delt_sqrtA,week,toa,toc,delt_i,OMG0,omg,M0,deln,ioda,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,bgd1,bgd2,oshs1,oshs2,osdvs;
    int nbit,spare=0;
    ioda=g_ioda;
    week=ROUND_U(alm_pre->week%4);
    toa =ROUND_U(alm_pre->toas/600.0);
    M0  =ROUND(alm_pre->M0/P2_15/SC2RAD);
    af0 =ROUND(alm_pre->f0/P2_19);
    af1 =ROUND(alm_pre->f1/P2_38);
    oshs1=oshs2=0;//health:0
    delt_sqrtA=ROUND((sqrt(alm_next->A)-SQRTA_REF)/P2_9);
    e=ROUND_U(alm_next->e/P2_16);
    omg=ROUND(alm_next->omg/P2_15/SC2RAD);
    delt_i=ROUND((alm_next->i0-I_REF)/P2_14/SC2RAD);
    setbitu(word,i,6,PAGE_TYPE9);    i+=6;
    setbitu(word,i,4,ioda);          i+=4;
    setbitu(word,i,2,week);          i+=2;
    setbitu(word,i,10,toa);          i+=10;
    setbits(word,i,16,M0);           i+=16;
    setbits(word,i,16,af0);          i+=16;
    setbits(word,i,13,af1);          i+=13;
    setbitu(word,i,2,oshs1);         i+=2;
    setbitu(word,i,2,oshs2);         i+=2;
    setbitu(word,i,6,sv_id);         i+=6;//keyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
    setbits(word,i,13,delt_sqrtA);   i+=13;
    setbitu(word,i,11,e);            i+=11;
    setbits(word,i,16,omg);          i+=16;
    setbits(word,i,11,delt_i);       i+=11;
    nbit=i;
    //return nbit;
}
void encode_word10(int prn, int sv_id, nav_t *nav, u_int8 *word){
    alm_t *alm=nav->alm+sv_id-1;
    unsigned int sqrtA,e;
    int i=0,week,toa,toc,i0,OMG0,omg,M0,deln,ioda,OMGd,crs,crc;
    int cus,cuc,cis,cic,af0,af1,af2,bgd1,bgd2,oshs1,oshs2,osdvs;
    unsigned int a0g,a1g,t0g,wn0g;
    int nbit,spare=0;
    ioda=g_ioda;
    OMG0=ROUND(alm->OMG0/P2_15/SC2RAD);
    OMGd=ROUND(alm->OMGd/P2_33/SC2RAD);
    M0  =ROUND(alm->M0/P2_15/SC2RAD);
    af0 =ROUND(alm->f0/P2_19);
    af1 =ROUND(alm->f1/P2_38);
    oshs1=oshs2=0;//health:0
    a0g=65536;a1g=4092;t0g=256;wn0g=64;//GGTO set invalid
    setbitu(word,i,6,PAGE_TYPE10);   i+=6;
    setbitu(word,i,4,ioda);          i+=4;
    setbits(word,i,16,OMG0);         i+=16;
    setbits(word,i,11,OMGd);         i+=11;
    setbits(word,i,16,M0);           i+=16;
    setbits(word,i,16,af0);          i+=16;
    setbits(word,i,13,af1);          i+=13;
    setbitu(word,i,2,oshs1);         i+=2;
    setbitu(word,i,2,oshs2);         i+=2;
    setbits(word,i,16,a0g);          i+=16;//keyyyyyyyyyyyyyyyyyyyyyyyyyyyyy
    setbits(word,i,12,a1g);          i+=12;
    setbits(word,i,8,t0g);           i+=8;
    setbits(word,i,6,wn0g);          i+=6;
    nbit=i;
    //return nbit;
}
void encode_word0(int prn, int sv_id, nav_t *nav, u_int8 *word){
    int Time=2;//set Time filed to 10
    int spare=0;// 88 bits
    eph_t *eph=nav->eph+prn-1;
    int week=eph->week%4096;
    int i=0,nbit,ioda,tow;
    ioda=g_ioda;
    tow=g_tow;
    setbitu(word,i,6,PAGE_TYPE0);   i+=6;
    setbitu(word,i,2,ioda);         i+=2;
    setbitu(word,i,32,spare);       i+=32;
    setbitu(word,i,32,spare);       i+=32;
    setbitu(word,i,24,spare);       i+=24;
    setbitu(word,i,12,week);        i+=12;
    setbitu(word,i,20,tow);         i+=20;
    nbit=i;
    //return nbit;
}
void encode_reserved(int prn, int sv_id, nav_t *nav, u_int8 *word){
    unsigned int reserved=0xaaaaaaaa;
    int i=0,j,nbit;
    for(j=0;j<4;j++){
        setbitu(word,i,32,reserved);
        i+=32;
    }
    nbit=i;
    //return nbit;
}
/********************************************
 *
 * out: word, 500bit for even&odd page
 * *****************************************/
void do_encode(int prn, int sv_id, nav_t *nav, unsigned char *word, Func encode){
    unsigned char byte[16];
    unsigned char even[120],odd[120];
    unsigned char buf1[240],buf2[240];
    encode(prn, sv_id, nav, byte);
    word2bits(byte,even,odd);
    FEC_Encoding(even,buf1);
    FEC_Encoding(odd,buf2);
    memcpy(word,sync,10*sizeof(unsigned char));     //copy sync head for nav data
    memcpy(word+250,sync,10*sizeof(unsigned char)); //copy sync head for nav data
    Interleaving_Encoding(buf1,30,8,word+10); //offset 10 bit for the sync head
    Interleaving_Encoding(buf2,30,8,word+260);//offset 10 bit for the sync head
}

void Get_word_1to4(){}

/********************************************
 *
 * out: word, 500bit for even&odd page
 * *****************************************/
void Get_word_by_type(int type, int prn, int sv_id, nav_t *nav, unsigned char *word){
    switch(type){
        case 0: g_tow+=2;break;
        case 5: g_tow+=20;break;
        case 6: g_tow+=6;break;
        default: break;
    }
    switch(type){
        case 0: do_encode(prn, sv_id, nav, word, encode_word0);break;
        case 1: do_encode(prn, sv_id, nav, word, encode_word1);break;
        case 2: do_encode(prn, sv_id, nav, word, encode_word2);break;
        case 3: do_encode(prn, sv_id, nav, word, encode_word3);break;
        case 4: do_encode(prn, sv_id, nav, word, encode_word4);break;
        case 5: do_encode(prn, sv_id, nav, word, encode_word5);break;
        case 6: do_encode(prn, sv_id, nav, word, encode_word6);break;
        case 7: do_encode(prn, sv_id, nav, word, encode_word7);break;
        case 8: do_encode(prn, sv_id, nav, word, encode_word8);break;
        case 9: do_encode(prn, sv_id, nav, word, encode_word9);break;
        case 10:do_encode(prn, sv_id, nav, word, encode_word10);break;
        case -1:do_encode(prn, sv_id, nav, word, encode_reserved);break;
        defult: printf("wrong type to encode word \n");
    }
}

int Pre_decode_for_nav(nav_t *nav){
    _raw_trimble raw;
    int i,n;
    n = Get_raw_trimble_data(FILE_EPH,FILE_ALM,FILE_UTC,&raw);
    if(n<0){printf("no valid trimble data \n");return 0;}
    for(i=0;i<n;i++){
        if(decode_trimble_eph(nav,(unsigned char *)raw.eph+i,LINE_BUF_LEN)==-1){
            printf("ERROR: decode eph error \n");
            return 0;
        }
        if(decode_trimble_alm(nav,(unsigned char *)raw.alm+i,LINE_BUF_LEN)==-1){
            printf("ERROR: decode alm error \n");
            return 0;
        }
    }
    if(decode_trimble_utc_ion(nav,(unsigned char *)raw.utc_ion,LINE_BUF_LEN)==-1){
        printf("ERROR: decode utc and ion error \n");
        return 0;
    }
    return n;
}

static unsigned char * Do_nav_by_time(int mins, int prn, nav_t *nav){
    int i,j,row,type_val;
    unsigned char *ret_nav;
    int sv_id=9; //almanac from sv_id, default 0, ++ == 1(first)
    ret_nav=(unsigned char *)malloc(500*30*mins*sizeof(unsigned char)); //malloc space for nav data
    if(!ret_nav)printf("memory is ran out, shit!!!!! \n");
    memset(ret_nav,0,500*30*mins*sizeof(unsigned char));
    for(i=0;i<2*mins;i++){
        row=i%2;
        for(j=0;j<15;j++){
            type_val=word_type[row][j];
            if(type_val==7 || type_val>8)sv_id++;
            Get_word_by_type(type_val, prn, sv_id+MAXPRNGPS, nav, ret_nav+7500*i+500*j); //7500bit/30s,500bit/2s;
            if(sv_id == 36)sv_id=0;
        }
    }
	/*******************debugggggggggggggggg*****************/
    for(i=0;i<10;i++){
        unsigned char debug=ret_nav[i+250];
        unsigned char debug2=ret_nav[i+1500];
        printf("%d", debug);
    }
	/******************* debug end **************************/
    return ret_nav;
}
/*****************************************
 * 捷星广达原始导航电文读入,每行64/80个16进制字符串，这里的64,80在程序里写死(Gal-- 64*4=256 > 250)(Gps-- 80*4=320 > 300)
 * 文件中排列顺序需满足，有待优化
 * Gal 1st, 1s/line(250bit/s)
 * Gps 2rd, 6s/line(300bit/6s)
 *
 * affects ****NAV SAT**** important
 * **************************************/
static int readrawnav(const char *nav_file, nav_t *nav, _init_paras *paras, int mins, int *sat, gtime_t *t){
    char line[LINE_BUF_LEN];
    int local_paras[4];
    unsigned char temp_nav[320];
    char *s,*p,*p1;
    int i,k,word_index,bit_index,index,sat_index=0,count=0;
    int week=0,tow=0,prn=0,len=0,bits_per_gps_frame=300;
    gtime_t tuser;
    FILE *fp=NULL;

	/*******************debugggggggggggggggg*****************/
    FILE *fp_w=NULL;
    char local_char_nav[9001]={0};
    if (!(fp_w=fopen("20161025nav_debug","w"))) {
        fprintf(stderr,"raw 1010 nav file open error : 20161025nav_debug \n");
        return -1;
    }
	/******************* debug end **************************/

    if (!(fp=fopen(nav_file,"r"))) {
        fprintf(stderr,"raw nav file open error : %s\n",nav_file);
        return -1;
    }  
    while (!feof(fp) && fgets(line, LINE_BUF_LEN, fp)){
        s = line;
        if (s[strlen(s) - 1] == '\n')//change the line end to string end
            s[strlen(s) - 1] = '\0';
        if(strncmp(s,"###E",4)==0){
            p=s+4;
            index=0;
            if((p1 = strtok(p, ","))) {
                sscanf(p1,"%d",local_paras+index);
                index++;
            }
            while((p1 = strtok(NULL, ","))) {
                sscanf(p1,"%d",local_paras+index);
                index++;
            }
            if(index!=4){printf("Error: Raw Nav Data File is not right \n");}
            else{
                //paras->nav[]
                sat[sat_index]=local_paras[0]+MAXPRNGPS;
                paras->nav[sat_index].sat=local_paras[0]+MAXPRNGPS;
                paras->nav[sat_index].data=(unsigned char *)malloc(250*local_paras[3]*sizeof(unsigned char)); //malloc space for nav data
                //use gps tuser
                //tuser=gpst2time(local_paras[1],local_paras[2]);
                //int ok_nums=Get_ok_sat(*paras,nav,sat,tuser);
                for(i=0;i<local_paras[3];i++){
                    fgets(line, LINE_BUF_LEN, fp);
                    hex_str2bin(line,64,temp_nav);
                    memcpy(paras->nav[sat_index].data+i*250,temp_nav,sizeof(unsigned char)*250);
                }
                /*******************debugggggggggggggggg*****************/
                for(i=0;i<10;i++){
                    unsigned char debug=paras->nav[sat_index].data[i+250];
                    unsigned char debug2=paras->nav[sat_index].data[i+1500];
                    printf("%d", debug);
                }
                sat_index++;
            }
        }
        else if(strncmp(s,"###G",4)==0){
            p=s+4;
            index=0;
            if((p1 = strtok(p, ","))) {
                sscanf(p1,"%d",local_paras+index);
                index++;
            }
            while((p1 = strtok(NULL, ","))) {
                sscanf(p1,"%d",local_paras+index);
                index++;
            }
            if(index!=4){printf("Error: Raw Nav Data File is not right \n");}
            else{
                //paras->nav[]
                sat[sat_index]=local_paras[0];
                paras->nav[sat_index].sat=local_paras[0];
                paras->nav[sat_index].data=(unsigned char *)malloc(300*local_paras[3]*sizeof(unsigned char)); //malloc space for nav data
                tuser=gpst2time(local_paras[1],local_paras[2]);
                //int ok_nums=Get_ok_sat(*paras,nav,sat,tuser);
                for(i=0;i<local_paras[3];i++){
                    fgets(line, LINE_BUF_LEN, fp);
                    hex_str2bin(line,80,temp_nav);
                    for(k=1;k<bits_per_gps_frame;k=k+32){
                        if(temp_nav[k]==1){
                            //Bit30*=1,inveser the 24bit in this word
                            bit_index=k+1;
                            while(bit_index<k+25){
                                temp_nav[bit_index]=(temp_nav[bit_index]+1)%2;
                                bit_index++;
                            }
                        }
                    }
                    for(word_index=0;word_index<10;word_index++){
                        memcpy(paras->nav[sat_index].data+i*300+word_index*30,temp_nav+word_index*32+2,sizeof(unsigned char)*30);
                    }
                }
                /*******************debugggggggggggggggg*****************/
                for(i=0;i<10;i++){
                    unsigned char debug=paras->nav[sat_index].data[i+810];
                    unsigned char debug2=paras->nav[sat_index].data[i+1500];
                    printf("%d", debug);
                }

                memcpy(local_char_nav,paras->nav[sat_index].data,sizeof(unsigned char)*9000);
                for(i=0;i<9000;i++){
                    local_char_nav[i]+=48;
                }
                fprintf(fp_w,"%2d#",local_paras[0]);
                fwrite(local_char_nav,sizeof(char),9000,fp_w);
                fprintf(fp_w,"\n");
				/******************* debug end **************************/

                sat_index++;
            }
        }
        //hexstr2byte(s, raw->eph+count);
        count++;
    }
    fclose(fp_w);
    *t=tuser;
    return sat_index;
}
/***********************************************************
 * func: generate mins nav message to be stored in paras.nav
 * in: nav, must be init before the function
 * in: paras, store nav message
 * in: mins, the message length
 * out: sat, store ok sats
 * out: t, signal, tuser starts here
 * ********************************************************/
extern int Decode_and_Encode(nav_t *nav, _init_paras *paras, int mins, int *sat, gtime_t *t){
    int i,n,ok_nums;
    n=Pre_decode_for_nav(nav);
    printf("INFO: decode from trimble files, get %d sats \n", n);
    if(!n){
        printf("ERROR: pre_decode wrong \n");
        return -1;
    }
    //double ep[6]={2016,7,27,8,0,0};
    //time2epoch(gpst2time(g_week,g_tow),ep);
    //printf("%f %f %f %f %f %f \n",ep[0],ep[1],ep[2],ep[3],ep[4],ep[5]);
    //find meeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeee
    *t=gst2time(g_week,g_init_tow+2);//TODO:: +2 to make signal +2s to suit the g_tow
    ok_nums=Get_ok_sat(*paras,nav,sat,*t);
    if(!ok_nums){
        printf("ERROR: find no one sat is ok \n");
        return -1;
    }
    for(i=0;i<ok_nums;i++){
        //20161010 test
        //g_tow=g_init_tow+2;
        g_tow=g_init_tow;
        paras->nav[i].data=Do_nav_by_time(mins,sat[i],nav);
        paras->nav[i].sat=sat[i];
    }
    return ok_nums;
}
/***********************************************************
 * func: generate mins nav message to be stored in paras.nav
 * in: nav, must be init before the function
 * in: paras, store nav message
 * in: mins, the message length
 * out: sat, store ok sats
 * out: t, signal, tuser starts here
 * ********************************************************/
extern int Decode_and_Encode_Simple_Way(nav_t *nav, _init_paras *paras, int mins, int *sat, gtime_t *t){

    int i,n,ok_nums;
	if(readrnx(g_paras.Rinex_Path,1,"",NULL,nav,NULL)!=1){
		printf("ERROR: read rinex file error \n");
		Sleep(10000);
		exit(-1);
	}
    ok_nums=readrawnav(g_paras.Nav_Path, nav, paras, mins, sat, t);
    return ok_nums;
}
/*int main(void)
{
    int i;
    char buffer[256]="0B0007700006B3A0";
    unsigned char p[256];
    int n = hexstr2byte(buffer,p);
    for(i=0;i<n;i++){
        printf("ret = %2x \n", p[i]);
    }
    decode_trimble(NULL,p,n);
    return 0;
}*/
