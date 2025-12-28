#include "log.h"
#include <stdio.h>
#include <stdarg.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/file.h>

void logMessage(const char *filename, const char *format, ...) {
    mkdir("logs", 0777);

    FILE *fp = fopen(filename, "a");
    if (!fp) return;

    int fd = fileno(fp); 
    
    flock(fd, LOCK_EX); 

    time_t t = time(NULL);
    struct tm tm_info;
    localtime_r(&t, &tm_info);

    char timebuf[32];
    strftime(timebuf, sizeof(timebuf), "%Y-%m-%d %H:%M:%S", &tm_info);

    fprintf(fp, "[%s] (PID %d) ", timebuf, getpid());

    va_list args;
    va_start(args, format);
    vfprintf(fp, format, args);
    va_end(args);

    fprintf(fp, "\n");

    fflush(fp);
    
    flock(fd, LOCK_UN);

    fclose(fp);
}