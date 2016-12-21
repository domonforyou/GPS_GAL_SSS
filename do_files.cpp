#include "do_files.h"

//将16进制字符串转化为整形的10010001,暂适用于大写字母
int hex_str2bin(char *hex, int n, unsigned char *bin){
    unsigned char temp;
    int i,j;
    unsigned char *p = bin;
    for(i=0;i<n;i++){
        if(*hex < 58)
            temp = *hex-'0';
        else
            temp = *hex-'A'+10;//TODO::a is different from A
        for(j=0;j<4;j++){
            p[j]=temp>>(3-j)&0x01;
        }
        hex++;
        p=p+4;
    }
	return 0;
}

void read_e1b_e1c(const char *file_e1b, const char *file_e1c, _ca_code *e1b, _ca_code *e1c){
    FILE *fp_b;
    FILE *fp_c;
    FILE *fp;
    int sat_id,sat_num=0;
    int i;
    char *s,*p1,*id;
    char line[MAX_BUF];
    if (!(fp_b=fopen(file_e1b,"r"))) {
        fprintf(stderr,"e1b file open error : %s\n",file_e1b);
        return;
    }
    if (!(fp_c=fopen(file_e1c,"r"))) {
        fprintf(stderr,"e1c file open error : %s\n",file_e1c);
        return;
    }
    for(i=0;i<2;i++){
        if(i==0)
            fp=fp_b;
        else
            fp=fp_c;
        while (!feof(fp) && fgets(line, MAX_BUF, fp)){
            s = line;
            if (s[strlen(s) - 1] == '\n')//change the line end to string end
                s[strlen(s) - 1] = '\0';
            if ((p1 = strchr(s, ';'))) {
                p1[0] = '\0';
            }
            if(p1){
                id=p1-2;//get sat num
                p1++;
                sscanf(id,"%d",&sat_id);
                if(sat_num < MAX_SAT_ID){
                    if(i==0){
                        e1b[sat_num].sat=sat_id+MAXPRNGPS;
                        hex_str2bin(p1,1023,e1b[sat_num].data);
                    }
                    else{
                        e1c[sat_num].sat=sat_id+MAXPRNGPS;
                        hex_str2bin(p1,1023,e1c[sat_num].data);
                    }
                }
                sat_num++;
            }
        }
        sat_num=0;
    }
}
//MAX_KML_BUF大小对仿真时间有所限制，还有一处导航电文长度对仿真时间也有所限制，这边default<3mins
int prase_mkl(const char *file_path, int size, LLH *llh){
	FILE *fp;
    int sat_id,sat_num=0;
    int i,count=0,index,get_it=0;
	int time[3]={0};
    char *s,*p,*p1,*id;
    char line[MAX_KML_BUF];
	char ret[MAX_KML_BUF]={0};
	bool is_enter,start=false;
    if (!(fp=fopen(file_path,"r"))) {
        fprintf(stderr,"kml file open error : %s\n",file_path);
        return -1;
    }
	 while (!feof(fp) && fgets(line, MAX_KML_BUF, fp)){
        s = line;
        if (s[strlen(s) - 1] == '\n')//change the line end to string end
            s[strlen(s) - 1] = '\0';
		if(get_it==1 && count<size){
			count++;
			strcat(ret, s);
			strcat(ret," ");
		}
		if(get_it==2){
			printf("ret=== %s\n",ret);
			break;
		}
		if(strstr(s,"<coordinates>") || strstr(s,"</coordinates>")){
            get_it++;
		}	
	 }
	p=ret;
	index=0;
	while(*p==' ' || *p == '\t')p++;
    if((p1 = strtok(p, " "))) {
		sscanf(p1,"%lf,%lf,%lf",&llh[0].userp[1],&llh[0].userp[0],&llh[0].userp[2]);
    }
    while((p1 = strtok(NULL, " ")) && index < size) {
		index++;
		sscanf(p1,"%lf,%lf,%lf",&llh[index].userp[1],&llh[index].userp[0],&llh[index].userp[2]);
    }
	/* degree to rad */
	for(i=0;i<size;i++){
		llh[i].userp[0]*=D2R;
		llh[i].userp[1]*=D2R;
	}
}