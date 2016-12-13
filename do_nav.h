#ifndef NAV_H
#define NAV_H

#include <time.h>
#include "rtklib.h"
#include "do_signal.h"

#define FILE_EPH "trimble_eph_bak.txt"
#define FILE_ALM "trimble_alm_bak.txt"
#define FILE_UTC "trimble_utc_bak.txt"
#define LOCAL_RAW_NAV "20161019nav.txt"
#define WORD_BYTES 16
#define EVEN 0
#define ODD  1
#define LINE_BUF_LEN 512
#define A0g  619315200 //wrong
#define SQRTA_REF   5440.588203494180 //sqrt(29600000)
#define I_REF       0.977384381116825 //(56/180*pi)
#define P2_2        0.25
#define P2_8        0.00390625
#define P2_9        0.001953125
#define P2_10       0.0009765625          /* 2^-10 */
#define P2_14       6.103515625000000E-5
#define P2_16       1.525878906250000E-5
#define P2_34       5.820766091346740E-11 /* 2^-34 */
#define P2_46       1.421085471520200E-14 /* 2^-46 */
#define P2_59       1.734723475976810E-18 /* 2^-59 */
#define ROUND(x)    ((int)floor((x)+0.5))
#define ROUND_U(x)  ((unsigned int)floor((x)+0.5))
#define MIN(x,y)    ((x)<(y)?(x):(y))

enum Page_Type{
    PAGE_TYPE0,
    PAGE_TYPE1,
	PAGE_TYPE2,
	PAGE_TYPE3,
	PAGE_TYPE4,
	PAGE_TYPE5,
	PAGE_TYPE6,
	PAGE_TYPE7,
	PAGE_TYPE8,
	PAGE_TYPE9,
	PAGE_TYPE10
};

typedef struct{
    double a0,a1;
    int tot,wnt,wnlsf,dn,dtls,dtlsf,ion_t;
}_gal_utc;

typedef struct{
    char eph[MAX_SAT][LINE_BUF_LEN];
    char alm[MAX_SAT][LINE_BUF_LEN];
    char utc_ion[LINE_BUF_LEN];
    int sat_num;
}_raw_trimble;

typedef unsigned char u_int8;
typedef char int8;
/* type definitions ----------------------------------------------------------*/

void FEC_Encoding(unsigned char *in, unsigned char *out);
void Interleaving_Encoding(unsigned char *in, int row, int col, unsigned char *out);
extern int INIT_NAV(nav_t *nav);
extern void FREE_NAV(nav_t *nav);
extern int Decode_and_Encode(nav_t *nav, _init_paras *paras, int mins, int *sat, gtime_t *t);
extern int Decode_and_Encode_Simple_Way(nav_t *nav, _init_paras *paras, int mins, int *sat, gtime_t *t);
#endif // NAV_H
