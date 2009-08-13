#include "debug.h"
#include "stdarg.h"
// First, a few utility function for displaying messages or saving them to
// disk...


//int OUTPUTREDIRECT = TONULL;
int OUTPUTREDIRECT = TOSTDOUT;

FILE *OUTPUTFILE;

void myprintf(const char *msg, ...)
{
    static va_list argp;
    static int nbcalls=0;
    if (OUTPUTREDIRECT == TONULL) return;
    if (OUTPUTREDIRECT == TOFILE)
    {
        if (nbcalls == 0)
            OUTPUTFILE = fopen("./output", "w");
        if (nbcalls == 50) fflush(OUTPUTFILE);
        if (nbcalls == 10000)
        {
            nbcalls = 1;
            fclose(OUTPUTFILE);
            OUTPUTFILE = fopen("./output", "w");
        }
        va_start(argp, msg);
        vfprintf(OUTPUTFILE, msg, argp);
        va_end(argp);
    }
    else if (OUTPUTREDIRECT == TOSTDOUT)
    {
        va_start(argp, msg);
        vprintf(msg, argp);
        va_end(argp);
    }

}

void myexit(int i)
{
    myprintf("Exiting |\n");
    if (OUTPUTFILE) fflush(OUTPUTFILE);
    while (1) ;
    exit(i);
}


// exits program, write reason to "exitmessage.txt"
void mydie(const char *msg, ...)
{
    va_list argp;
    FILE *out;
    va_start(argp, msg);
    vprintf(msg, argp);
    va_end(argp);
    fflush(stdout);
    out = fopen("exitmessage.txt", "w");
    va_start(argp, msg);
    vfprintf(out, msg, argp);
    va_end(argp);
    fflush(out);
    myexit(-1);
}
