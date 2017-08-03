#ifdef ARDUINO

#include <Arduino.h>
#include "arduino-util.h"
#include <cstdarg>
#include <stdio.h>


char tmpbuf[160];
extern int us_getTicker(void);
extern int s_getTicker(void);
extern Stream *ser;

void
dprintf(const char *format, ...)
{
    static volatile bool busy;
    if (busy)
        return;
    busy = true;
    
    int secs = s_getTicker();
    int s = secs % 60;
    int m = secs / 60;
    int h = secs / 3600;
    int us = us_getTicker();
    while(us > 999999)
        us /= 10;	// get it to 6 digits only

    snprintf(tmpbuf, sizeof(tmpbuf)-1, "%02d:%02d:%02d.%.06d ", h, m, s, us);
    ser->write(tmpbuf, (int) sizeof "00:00:34.3436868 " -1);

	va_list arg;
	va_start(arg, format);
	int len = vsnprintf(tmpbuf, sizeof(tmpbuf)-3, format, arg);
	tmpbuf[len] = '\r';
	tmpbuf[len+1] = '\n';
	tmpbuf[len+2] = 0;
	ser->write(tmpbuf, len+3);
	va_end(arg);
    busy = false;
}

void
rprintf(const char *format, ...)
{
    va_list arg;
    va_start(arg, format);
    int len = vsnprintf(tmpbuf, sizeof(tmpbuf)-3, format, arg);
    tmpbuf[len] = 0;
    ser->write(tmpbuf, len+1);
    va_end(arg);
}

void
dump(const char *title, const void *data, int len)
{
    dprintf("dump(\"%s\", 0x%x, %d bytes)", title, data, len);
    
    int i, j, cnt;
    unsigned char *u;
    const int width = 16;
    const int seppos = 7;
    
    cnt = 0;
    u = (unsigned char *)data;
    while (len > 0) {
        rprintf("%08x: ", (unsigned int)data + cnt);
        cnt += width;
        j = len < width ? len : width;
        for (i = 0; i < j; i++) {
            rprintf("%2.2x ", *(u + i));
            if (i == seppos)
                ser->write(' ');
        }
        ser->write(' ');
        if (j < width) {
            i = width - j;
            if (i > seppos + 1)
                ser->write(' ');
            while (i--) {
                ser->print("   ");
            }
        }
        for (i = 0; i < j; i++) {
            int c = *(u + i);
            if (c >= ' ' && c <= '~')
                ser->write(c);
            else
                ser->write('.');
            if (i == seppos)
                ser->write(' ');
        }
        len -= width;
        u += width;
        ser->print("\r\n");
    }
    ser->print("--\r\n");
    
}
#endif
