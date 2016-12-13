#ifndef DO_FILES_H
#define DO_FILES_H

#include <stdio.h>
#include <string.h>
#include "do_signal.h"

#define MAX_BUF 2048
#define MAX_KML_BUF (1024*16)
#define MAX_SAT_ID 10

typedef struct{
	double userp[3];
}LLH;
int hex_str2bin(char *hex, int n, unsigned char *bin);
void read_e1b_e1c(const char *file_e1b, const char *file_e1c, _ca_code *e1b, _ca_code *e1c);
int prase_mkl(const char *file_path, int size, LLH *llh);
#endif // DO_FILES_H
