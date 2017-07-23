#ifdef ARDUINO
#ifndef __ARDUINO_UTIL_H__
#define __ARDUINO_UTIL_H__

/*
 * The file is Licensed under the Apache License, Version 2.0
 * (c) 2017 Helmut Tschemernjak
 * 30826 Garbsen (Hannover) Germany
 */

extern void dprintf(const char *format, ...);

extern void dump(const char *title, const void *data, int len);

#endif // __ARDUINO_UTIL_H__

#endif
