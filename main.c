#include <stdio.h>
#include <string.h>
#include <pthread.h>
#include "do_files.h"
#include "do_util.h"
#include "do_nav.h"
#include "do_signal.h"

//const int e1c_sub_code[25] = {0,0,1,1,1,0,0,0,0,0,0,0,1,0,1,0,1,1,0,1,1,0,0,1,0};
const char file_b[]="E:\\code_c\\Ga_Raw\\E1B.txt";
const char file_c[]="E:\\code_c\\Ga_Raw\\E1C.txt";
int bin_test[MAX_SAT_ID][4096];
int cin_test[MAX_SAT_ID][4096];
double sub_a[12];
double sub_b[12];
double base_a[2][12];
double base_b[2][12];
void test1(){
    int i;
    read_e1b_e1c(file_b,file_c,bin_test,cin_test);//key
    printf("999999 ");
    for(i=0;i<4096;i++){
        printf("%d",bin_test[9][i]);
    }
    printf("\n 999999 \n");
    for(i=0;i<4096;i++){
        printf("%d",cin_test[9][i]);
    }
    printf("\n");
}
void test2(){
    generate_sub_carrier(10,1,12,sub_a,sub_b);//key
    int i;
    for(i=0;i<12;i++){
        printf("%f ",sub_a[i]);
    }
    printf("\n");
    for(i=0;i<12;i++){
        printf("%f ",sub_b[i]);
    }
    printf("\n");

    int a[4]={0,0,1,1};
    int b[4]={0,1,0,1};
    for(i=0;i<4;i++){
        printf("check signed int === %d \n",a[i]^b[i]);
    }
    generate_baseband(sub_a,sub_b,base_a,base_b);//key
    for(i=0;i<12;i++){
        printf("base === %f \n",base_a[0][i]);//key
        printf("base === %f \n",base_b[0][i]);
    }
}

void test_fec_interleave(){
    unsigned char a[120]={1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,1,1,0,0,1,1,0,0,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,1,1, \
                          1,1,0,0,1,1,0,0,1,1,0,1,0,1,0,1,0,1,1,1,1,0,0,0,1,1,1,1,1,0,1,1,0,0,1,1,0,1,1,1,1,1,1,0,0,0, \
                          1,0,1,0,0,0,0,1,1,1,0,0,0,0,0,1,0,0,1,1,0,1,0,0,0,0,0,0};
    unsigned char b[240]={0},c[240];
    //FEC_Encoding(a,b);
    //Interleaving_Encoding(b,30,8,c);
    int i;
    for(i=0;i<240;i++)printf("%d",c[i]);
}

void *test_pthread(void *args){
    while(1){
        printf("ppppppppppppp \n");
        Sleep(5000);
    }
}

void test_signal(){
    pthread_t p;
    int res;
    int sat[MAX_SAT];
    double rr[3] = {0};
    int ok_num;
    nav_t nav={0};
    _init_paras gal_paras;
    INIT_PARAS(&gal_paras);
    //ok_num=Get_ok_sat(gal_paras,&nav,sat,);
    printf("sat ok sum-num is %d \n", ok_num);

    //pthread_create(&p,NULL,test_pthread,NULL);
    //res = pthread_join(p,NULL);

    main_signal(2,1);
}
//GST-GPST Time Offset Test
void test_GGTO(){
    gtime_t a,b;
    a=gpst2time(1025,0);
    b=gst2time(1,0);

    printf("GGTO A0g == %lf \n",timediff(b,a));
}

void test_nav_message(){
    nav_t nav={0};
    int sat[16];
    _init_paras gal_paras;
    gtime_t t;
    int min=2;
    INIT_PARAS(&gal_paras);
    INIT_NAV(&nav);
    Decode_and_Encode(&nav,&gal_paras,min,sat,&t);
    int i;
    for(i=0;i<100;i++)printf("%d",gal_paras.nav[0].data[i]);
    FREE_PARAS(&gal_paras);
    FREE_NAV(&nav);
}
int main(int argc, char *argv[])
{
    //test1();
    //test2();
    //test_fec_interleave();
    //test_signal();
    //test_GGTO();
    //test_nav_message();
    main_signal(4, 1);//mins == how long nav data, 1=simple way,0=common way
    return 0;
}
