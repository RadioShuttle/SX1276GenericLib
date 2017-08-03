/*
 * The file is Licensed under the Apache License, Version 2.0
 * (c) 2017 Helmut Tschemernjak
 * 30826 Garbsen (Hannover) Germany
 */

#ifdef ARDUINO

using namespace std;

#include "arduino-mbed.h"
#include "arduino-util.h"

Stream *ser;
void InitSerial(Stream *serial) {
    ser = serial;
}

static void pinInt00(void);
static void pinInt01(void);
static void pinInt02(void);
static void pinInt03(void);
static void pinInt04(void);
static void pinInt05(void);
static void pinInt06(void);
static void pinInt07(void);
static void pinInt08(void);
static void pinInt09(void);
static void pinInt10(void);
static void pinInt11(void);
static void pinInt12(void);
static void pinInt13(void);
static void pinInt14(void);
static void pinInt15(void);
static void pinInt16(void);
static void pinInt17(void);
static void pinInt18(void);
static void pinInt19(void);
static void pinInt20(void);
static void pinInt21(void);
static void pinInt22(void);
static void pinInt23(void);
static void pinInt24(void);
static void pinInt25(void);
static void pinInt26(void);
static void pinInt27(void);
static void pinInt28(void);
static void pinInt29(void);
static void pinInt30(void);
static void pinInt31(void);
static void pinInt32(void);
static void pinInt33(void);
static void pinInt34(void);
static void pinInt35(void);
static void pinInt36(void);
static void pinInt37(void);
static void pinInt38(void);
static void pinInt39(void);
static void pinInt40(void);
static void pinInt41(void);
static void pinInt42(void);
static void pinInt43(void);
static void pinInt44(void);
static void pinInt45(void);
static void pinInt46(void);
static void pinInt47(void);



#define MAX_MCU_PINS	48
class InterruptIn;
struct intPtrTable {
    void (*func)(void);
    InterruptIn	*context;
} intPtrTable[MAX_MCU_PINS]  = {
    { pinInt00, NULL },
    { pinInt01, NULL },
    { pinInt02, NULL },
    { pinInt03, NULL },
    { pinInt04, NULL },
    { pinInt05, NULL },
    { pinInt06, NULL },
    { pinInt07, NULL },
    { pinInt08, NULL },
    { pinInt09, NULL },
    { pinInt10, NULL },
    { pinInt11, NULL },
    { pinInt12, NULL },
    { pinInt13, NULL },
    { pinInt14, NULL },
    { pinInt15, NULL },
    { pinInt16, NULL },
    { pinInt17, NULL },
    { pinInt18, NULL },
    { pinInt19, NULL },
    { pinInt20, NULL },
    { pinInt21, NULL },
    { pinInt22, NULL },
    { pinInt23, NULL },
    { pinInt24, NULL },
    { pinInt25, NULL },
    { pinInt26, NULL },
    { pinInt27, NULL },
    { pinInt28, NULL },
    { pinInt29, NULL },
    { pinInt30, NULL },
    { pinInt31, NULL },
    { pinInt32, NULL },
    { pinInt33, NULL },
    { pinInt34, NULL },
    { pinInt35, NULL },
    { pinInt36, NULL },
    { pinInt37, NULL },
    { pinInt38, NULL },
    { pinInt39, NULL },
    { pinInt40, NULL },
    { pinInt41, NULL },
    { pinInt42, NULL },
    { pinInt43, NULL },
    { pinInt44, NULL },
    { pinInt45, NULL },
    { pinInt46, NULL },
    { pinInt47, NULL }
}; // our max MCUs pins



static void pinInt00(void) { InterruptIn::_irq_handler(intPtrTable[0].context); }
static void pinInt01(void) { InterruptIn::_irq_handler(intPtrTable[1].context); }
static void pinInt02(void) { InterruptIn::_irq_handler(intPtrTable[2].context); }
static void pinInt03(void) { InterruptIn::_irq_handler(intPtrTable[3].context); }
static void pinInt04(void) { InterruptIn::_irq_handler(intPtrTable[4].context); }
static void pinInt05(void) { InterruptIn::_irq_handler(intPtrTable[5].context); }
static void pinInt06(void) { InterruptIn::_irq_handler(intPtrTable[6].context); }
static void pinInt07(void) { InterruptIn::_irq_handler(intPtrTable[7].context); }
static void pinInt08(void) { InterruptIn::_irq_handler(intPtrTable[8].context); }
static void pinInt09(void) { InterruptIn::_irq_handler(intPtrTable[9].context); }
static void pinInt10(void) { InterruptIn::_irq_handler(intPtrTable[10].context); }
static void pinInt11(void) { InterruptIn::_irq_handler(intPtrTable[11].context); }
static void pinInt12(void) { InterruptIn::_irq_handler(intPtrTable[12].context); }
static void pinInt13(void) { InterruptIn::_irq_handler(intPtrTable[13].context); }
static void pinInt14(void) { InterruptIn::_irq_handler(intPtrTable[14].context); }
static void pinInt15(void) { InterruptIn::_irq_handler(intPtrTable[15].context); }
static void pinInt16(void) { InterruptIn::_irq_handler(intPtrTable[16].context); }
static void pinInt17(void) { InterruptIn::_irq_handler(intPtrTable[17].context); }
static void pinInt18(void) { InterruptIn::_irq_handler(intPtrTable[18].context); }
static void pinInt19(void) { InterruptIn::_irq_handler(intPtrTable[19].context); }
static void pinInt20(void) { InterruptIn::_irq_handler(intPtrTable[20].context); }
static void pinInt21(void) { InterruptIn::_irq_handler(intPtrTable[21].context); }
static void pinInt22(void) { InterruptIn::_irq_handler(intPtrTable[22].context); }
static void pinInt23(void) { InterruptIn::_irq_handler(intPtrTable[23].context); }
static void pinInt24(void) { InterruptIn::_irq_handler(intPtrTable[24].context); }
static void pinInt25(void) { InterruptIn::_irq_handler(intPtrTable[25].context); }
static void pinInt26(void) { InterruptIn::_irq_handler(intPtrTable[26].context); }
static void pinInt27(void) { InterruptIn::_irq_handler(intPtrTable[27].context); }
static void pinInt28(void) { InterruptIn::_irq_handler(intPtrTable[28].context); }
static void pinInt29(void) { InterruptIn::_irq_handler(intPtrTable[29].context); }
static void pinInt30(void) { InterruptIn::_irq_handler(intPtrTable[30].context); }
static void pinInt31(void) { InterruptIn::_irq_handler(intPtrTable[31].context); }
static void pinInt32(void) { InterruptIn::_irq_handler(intPtrTable[32].context); }
static void pinInt33(void) { InterruptIn::_irq_handler(intPtrTable[33].context); }
static void pinInt34(void) { InterruptIn::_irq_handler(intPtrTable[34].context); }
static void pinInt35(void) { InterruptIn::_irq_handler(intPtrTable[35].context); }
static void pinInt36(void) { InterruptIn::_irq_handler(intPtrTable[36].context); }
static void pinInt37(void) { InterruptIn::_irq_handler(intPtrTable[37].context); }
static void pinInt38(void) { InterruptIn::_irq_handler(intPtrTable[38].context); }
static void pinInt39(void) { InterruptIn::_irq_handler(intPtrTable[39].context); }
static void pinInt40(void) { InterruptIn::_irq_handler(intPtrTable[40].context); }
static void pinInt41(void) { InterruptIn::_irq_handler(intPtrTable[41].context); }
static void pinInt42(void) { InterruptIn::_irq_handler(intPtrTable[42].context); }
static void pinInt43(void) { InterruptIn::_irq_handler(intPtrTable[43].context); }
static void pinInt44(void) { InterruptIn::_irq_handler(intPtrTable[44].context); }
static void pinInt45(void) { InterruptIn::_irq_handler(intPtrTable[45].context); }
static void pinInt46(void) { InterruptIn::_irq_handler(intPtrTable[46].context); }
static void pinInt47(void) { InterruptIn::_irq_handler(intPtrTable[47].context); }


void wait_ms(uint32_t ms)
{
    uint32_t start = ms_getTicker();
    
    while (true) {
        uint32_t t = ms_getTicker();
        if (t < start) // warp.
            start = 0;
        if (t > (start + ms))
            break;
    }
}

struct TimeoutVector TimeOuts[MAX_TIMEOUTS];

void
InterruptIn::rise(Callback<void()> func) {
    if (_gpioPin >= MAX_MCU_PINS-1)
        return;
    if (func) {
        _func = func;
        intPtrTable[_gpioPin].context = this;
        attachInterrupt(MYdigitalPinToInterrupt(_gpioPin), intPtrTable[_gpioPin].func, RISING);
    } else {
        _func = InterruptIn::donothing;
        intPtrTable[_gpioPin].context = NULL;
        detachInterrupt(_gpioPin);
    }
};

void
InterruptIn::fall(Callback<void()> func) {
    if (func) {
        _func = func;
        intPtrTable[_gpioPin].context = this;
        attachInterrupt(MYdigitalPinToInterrupt(_gpioPin), intPtrTable[_gpioPin].func, FALLING);
    } else {
        _func = InterruptIn::donothing;
        intPtrTable[_gpioPin].context = NULL;
        detachInterrupt(_gpioPin);
    }
}


uint32_t s_getTicker(void)
{
    long long ns = ns_getTicker();
    ns /= (long long)1000000000; // to secs
    
    int secs = ns;
    return secs;
}


uint32_t ms_getTicker(void)
{
    uint32_t us = us_getTicker();
    
    us /= 1000; // to ms
    return us;
}

uint32_t us_getTicker(void)
{
    long long ns = ns_getTicker();

    ns /= (long long)1000; // to us
    uint32_t us = ns & 0xffffffff;
    
    return us;
}


void
Timeout::insert(void)
{
    noInterrupts();
    for (int i = 0; i < MAX_TIMEOUTS-1; i++) {
        struct TimeoutVector *tvp = &TimeOuts[i];
        if (tvp->timer == this) // already here, timer has been restartet.
            break;
        if (tvp->timer == NULL) {
            tvp->timer = this;
            break;
        }
    }
    interrupts();
}

void
Timeout::remove(void)
{
    noInterrupts();
    for (int i = 0; i < MAX_TIMEOUTS-1; i++) {
        struct TimeoutVector *tvp = &TimeOuts[i];
        if (tvp->timer == this) {
            tvp->timer = NULL;
            break;
        }
    }
    interrupts();
}


void
Timeout::restart()
{
    Tcc *t = getTimeout_tcc();
    uint64_t timeout = ~0;
    
    /*
     * find the lowest timeout value which is our the next timeout
     * zero means stop the timer.
     */
    noInterrupts();
    for (int i = 0; i < MAX_TIMEOUTS-1; i++) {
        struct TimeoutVector *tvp = &TimeOuts[i];
        if (tvp->timer) {
            if (tvp->timer->_timeout < timeout) {
                timeout = tvp->timer->_timeout;
            }
        }
    }
    interrupts();
    
    if (timeout == (uint64_t)~0) {
        stopTimer(t);
        return;
    }
    
    uint64_t nsecs = ns_getTicker();
    
    if (timeout > nsecs) {
        startTimer(t, (uint64_t)timeout - (uint64_t)nsecs);
        return;
    } else {
        startTimer(t, (uint64_t)1); // just one nsec to trigger interrrupt
    }
}

#endif // ARDUINO
