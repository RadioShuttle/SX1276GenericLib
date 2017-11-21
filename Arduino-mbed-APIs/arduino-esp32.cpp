/*
 * The file is Licensed under the Apache License, Version 2.0
 * (c) 2017 Helmut Tschemernjak
 * 30826 Garbsen (Hannover) Germany
 */

#ifdef ARDUINO

using namespace std;

#include "arduino-mbed.h"
#include "arduino-util.h"
#include <time.h>

#if defined(ARDUINO_ARCH_ESP32)

#include "soc/efuse_reg.h"


/*
 * ARDUINO_ARCH_ESP32 ESP32 development board
 * Heltec ESP32 boards
 */

int CPUID(uint8_t *buf, int maxSize, uint32_t xorval)
{
    uint8_t uuid[16];
    int f1 = 0x6aa0f551;	// EFUSE_BLK0_RDATA1_REG address
    int f2 = 0x6aa0f55d;	// EFUSE_BLK0_RDATA2_REG address
    
    if (maxSize >= sizeof(uuid)) {
        int fa = f1 ^ xorval;
        int fb = f2 ^ xorval;
        
        uint32_t mac_low = REG_READ(fa);
        uint32_t mac_high =  REG_READ(fb);
        
        uuid[0] = mac_high >> 8;
        uuid[1] = mac_high;
        uuid[2] = mac_low >> 24;
        uuid[3] = mac_low >> 16;
        uuid[4] = mac_low >> 8;
        uuid[5] = mac_low;
        
        uuid[6] = uuid[0] ^ 0x16;
        uuid[7] = uuid[1] ^ 0x27;
        uuid[8] = uuid[2] ^ 0x38;
        uuid[9] = uuid[4] ^ 0x49;
        uuid[10] = uuid[5] ^ 0x50;
        uuid[11] = uuid[5] ^ 0x61;
        
        uuid[12] = ESP.getChipRevision();
        uuid[13] = 0x12;
        uuid[14] = 0x34;
        uuid[15] = 0x56;
        memcpy(buf, &uuid[0], sizeof(uuid));
        return sizeof(uuid);
    }
    return 0;
}

/*
 * see esp32-hal-timer.h is automatically included from:
 * Arduino15/packages/arduino/hardware/espressif/esp32/cores/esp32
 */
static void initTimer(int timerID);
void IRAM_ATTR onTimer(void);
/*
 * The Atmel ESP32 has four 64-bit timer.
 * At present the solution uses two timers, one for counting ticks,
 * a second for setting alarms. This is required for the rev-0 ESP
 *
 * For the rev. 1 ESP a single timer will work for counting as well for 
 * setting the alarm via timerAlarmWrite(timer, timerRead(timer) + 1000000, false)
 * once we support only ESP32-R1 the timer functions can be optimized to use only a
 * single 64-bit timer.
 */
struct TIMER_config {
    int timerID;
    hw_timer_t *timer;
    uint8_t nbits;
} Timer_data[] {
    { 0, NULL, 32 },
    { 1, NULL, 32 },
    { 2, NULL, 32 },
    { 3, NULL, 32 },
    { -1, NULL, 0 }
};

/*
 * We preferably use ESP32 timers because it supports 64-bit counters
 */
#define USE_TIMER_TIMEOUT	0 // 0, 1, 2 (see ESP32 docs)
#define USE_TIMER_TICKER	1 // 0, 1, 2 (see ESP32 docs)
#define MAX_TIMERS			3
#define TIMER_DIVIDER		80
#define TIMER_CLOCK			80

/*
 * Calculation of ticks see timerBegin divider
 */
#define NS_PER_CLOCK_CPU	1000 // ns secs per clock
#define NS_PER_CLOCK_RTC	1000 // ns secs per clock
#define TIMER_INFINITE		0x7fffffffffffffff // max esp alarm timeout
#define NS_PER_CLOCK	NS_PER_CLOCK_RTC


const char *GetTimerName(int timerID)
{
    switch(timerID) {
        case USE_TIMER_TIMEOUT:
            return "TIMEOUT";
            break;
        case USE_TIMER_TICKER:
            return "TICKER";
            break;
        default:
            return "Uknown";
    }
}

/* ----------------- TICKER TIMER CODE ----------------------*/

/*
 * The global ns_counter contains the time in ns from the last time
 * the counter has been wrapped. It cannot be used directly because the
 * current counter has to be added fore using it. Use instead
 * ns_getTicker(), us_ ns_getTicker(), ms_getTicker()
 */

static volatile bool initTickerDone = false;

uint64_t ns_getTicker(void)
{
    int timerID = Timer_data[USE_TIMER_TICKER].timerID;
    
    if (!initTickerDone) {
        initTimer(timerID);
        initTickerDone = true;
    }
    
    hw_timer_t *timer = Timer_data[USE_TIMER_TICKER].timer;
    uint64_t ns = timerRead(timer);
    uint16_t div = timerGetDivider(timer);
    ns *= div;	// get to the real clocks
    ns *= 1000; // convert micros to NS.
    ns /= TIMER_CLOCK;	// 80 MHz clock, convert to micro seconds
    
    return ns;
}


/* ----------------- SUPPORT CODE FOR TCC TIMERS----------------------*/

static volatile bool initTimerDone = false;

static void initTimer(int timerID)
{
    //dprintf("initTimer: %s", GetTimerName(timerID));
    struct TIMER_config *cp = &Timer_data[timerID];
    if (timerID > MAX_TIMERS-1)
        return;
    
    cp->timer = timerBegin(timerID, TIMER_DIVIDER, true);
    timerWrite(cp->timer, 0);
    if (timerID == USE_TIMER_TICKER) {
        time_t t = time(NULL);
        if (t > 0) {
            struct tm mytm;
            uint64_t tstart;
            
            localtime_r(&t, &mytm);
            tstart = mytm.tm_sec + (mytm.tm_min * 60) + (mytm.tm_hour * 3600);
            tstart *= 1000000;
            tstart *= TIMER_CLOCK;
            tstart /= TIMER_DIVIDER;
            timerWrite(cp->timer, tstart);
        }
    	timerStart(cp->timer);
    } else {
    	timerAttachInterrupt(cp->timer, &onTimer, true);
        timerAlarmWrite(cp->timer, TIMER_INFINITE, true);
        timerAlarmEnable(cp->timer);
        timerStart(cp->timer);
    }
	/*
     * somehow the timer needs some time to initalize before being used.
     * otherwise it will not issue any alarms
     * This affects only ESP32 rev 0
	 */
    if (ESP.getChipRevision() == 0)
     	delay(20);
}


TIMER_REF *getTimeoutTimer(void)
{
    struct TIMER_config *cp = &Timer_data[USE_TIMER_TIMEOUT];
    
    return &cp->timerID;
}


void stopTimer(int *timerIDPtr)
{
    struct TIMER_config *cp = &Timer_data[*timerIDPtr];
    if (*timerIDPtr > MAX_TIMERS-1)
        return;

    if (cp->timer)
	    timerAlarmWrite(cp->timer, TIMER_INFINITE, true);
}


/* ----------------- TIMEOUT TIMER CODE ----------------------*/

void startTimer(int *timerIDPtr, uint64_t delay_ns)
{
    if (!initTimerDone) {
        initTimer(*timerIDPtr);	// initial setup with stopped timer
        initTimerDone = true;
    }

    struct TIMER_config *cp = &Timer_data[*timerIDPtr];
    if (*timerIDPtr > MAX_TIMERS-1)
        return;
    uint64_t usecs = delay_ns/1000;
    if (delay_ns == 1) // immediate timeout
        usecs = 1;
    timerAlarmWrite(cp->timer, usecs, true);
    // dprintf("startTimer: %s in %d us", GetTimerName(*timerIDPtr), usecs);
}

/*
 * The onTimer is only called for every Timeout expired timer
 */
void IRAM_ATTR onTimer(void) {
    //dprintf("onTimer int called");
    uint64_t nsecs = ns_getTicker();

    for (int i = 0; i < MAX_TIMEOUTS-1; i++) {
        struct TimeoutVector *tvp = &TimeOuts[i];
        if (tvp->timer && nsecs >= tvp->timer->_timeout) {
            Timeout *saveTimer = tvp->timer;
            tvp->timer = NULL;
            Timeout::_irq_handler(saveTimer);
        }
    }
    /*
     * we need to restart the timer for remaining interrupts
     * Another reason is that we stopped this counter, in case there are
     * remaining counts, we need to re-schedule the counter.
     */
    Timeout::restart();
}


/* ----------------- ESP32 sleep() and deepsleep() code ----------------------*/

void sleep(void)
{
    asm("waiti 0");
}

/*
 * TODO
 * The ESP32 deepsleep can be enhanced to bring the ESP into low power mode.
 */

void deepsleep(void)
{
    // Light Sleep
    asm("waiti 0");
}

#endif // ESp32  Timer, sleep, etc.

#endif // ARDUINO
