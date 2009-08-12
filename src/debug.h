//#ifndef _DEBUG_H_
//#define _DEBUG_H_
#include <stdlib.h>
#include <stdio.h>

#define TOFILE 54
#define TONULL 55
#define TOSTDOUT 56


extern FILE *OUTPUTFILE;
extern int OUTPUTREDIRECT;

#ifdef __cplusplus
extern "C" {
#endif

void myprintf(const char *msg, ...);
void mydie(const char *msg, ...);
void myexit(int i);

#ifdef __cplusplus
} /* closing brace for extern "C" */
#endif

//#endif /* _DEBUG_H_ */
