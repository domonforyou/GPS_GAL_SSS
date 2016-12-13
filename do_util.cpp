#include "do_util.h"
#include <math.h>

//cboc(1,n) sub carrier
/*******************************
产生E1信号的cboc子载波
in: a,weight of e1b
in: b,weight of e1c
in: n,the num of array==2*key,like cboc(1,6(key))==12
out：e1b,e1c,输出的子载波数组
*******************************/
void generate_sub_carrier(double a,double b,int n,double *e1b,double *e1c){
    int num=n;
    int i,temp_a[12],temp_b[12];
    double sum=a+b;
    double weight_a=sqrt(a/sum);
    double weight_b=sqrt(b/sum);
    for(i=0;i<num;i++){
        if(i%2==0)temp_b[i]=1;
        else temp_b[i]=-1;
        if(i< num/2)temp_a[i]=1;
        else temp_a[i]=-1;
    }
    for(i=0;i<num;i++){
        e1b[i]=weight_a*temp_a[i]+weight_b*temp_b[i];
        e1c[i]=weight_a*temp_a[i]-weight_b*temp_b[i];
    }
}

//add data, pseudo code, sub_carrier together
//不具有通用性
void generate_baseband(double *e1b,double *e1c,double base_b[][12],double base_c[][12]){
    int a[2]={1,-1};//0^0 1^1,0^1 1^0 对应的真值;0,1代表 数据和伪码模二作为索引
    int i,j;

    for(i=0;i<2;i++)
        for(j=0;j<12;j++){
            base_b[i][j]=a[i]*e1b[j];
            base_c[i][j]=a[i]*e1c[j];
        }
}
