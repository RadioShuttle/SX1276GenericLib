/*
 * The file is Licensed under the Apache License, Version 2.0
 * (c) 2017 Helmut Tschemernjak
 * 30826 Garbsen (Hannover) Germany
 */

#ifdef ARDUINO

using namespace std;

#include "arduino-mbed.h"


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




void
InterruptIn::rise(Callback<void()> func) {
    if (_gpioPin >= MAX_MCU_PINS-1)
        return;
    if (func) {
        _func = func;
        intPtrTable[_gpioPin].context = this;
        attachInterrupt(digitalPinToInterrupt(_gpioPin), intPtrTable[_gpioPin].func, RISING);
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
        attachInterrupt(digitalPinToInterrupt(_gpioPin), intPtrTable[_gpioPin].func, FALLING);
    } else {
        _func = InterruptIn::donothing;
        intPtrTable[_gpioPin].context = NULL;
        detachInterrupt(_gpioPin);
    }
}


#define MAX_TIMEOUTS	10
class Timeout;
struct TimeoutVector {
    Timeout *timer;
    volatile uint32_t timeout; // us
} TimeOuts[MAX_TIMEOUTS];


#if defined(__SAMD21G18A__) || defined(__SAMD21J18A__)
/*
 * __SAMD21J18A__ is the SamD21 Explained Board
 * __SAMD21G18A__ is Genuino Zero-Board (compatible with the LoRa board)
 */

/*
 * see tcc.h included from
 * Arduino15/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/
 * Device/ATMEL/samd21/include/component/tcc.h
 */
static const struct TCC_CONFIG {
    Tcc *tcc_ptr;
    IRQn_Type tcc_irq;
    uint8_t nbits;
} TCC_CONFIG[] {
    { TCC0, TCC0_IRQn, 24 },
    { TCC1, TCC1_IRQn, 24 },
    { TCC2, TCC2_IRQn, 16 },
};
#define USE_TCC  0 // TCC0, TTC1, TTC2 are working using the Arduino D21


static bool initTimerDone = false;

static void initTimer() {
    Tcc *TC = TCC_CONFIG[USE_TCC].tcc_ptr;
    
    // Enable clock for TC, see gclk.h
    if (TC == TCC0 || TC == TCC1) {
        REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1);
    } else if (TC == TCC2) {
        REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC2_TC3_Val);
    }
    while (GCLK->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
    TC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
    while (TC->SYNCBUSY.bit.ENABLE == 1); // wait for sync
    
    TC->CTRLA.reg |= (TCC_CTRLA_PRESCALER_DIV1024 | TCC_CTRLA_RUNSTDBY); // Set perscaler
    
    TC->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ; // Set wave form configuration
    while (TC->SYNCBUSY.bit.WAVE == 1);	   // wait for sync
    
    TC->PER.bit.PER = 0xFFFFFF; // set counter top to max 24 bit
    while (TC->SYNCBUSY.bit.PER == 1); // wait for sync
    
    // the compare counter TC->CC[0].reg will be set in the startTimer
    // after the timeout calculation is known.
    
    // Interrupts
    TC->INTENSET.reg = 0;              // disable all interrupts
    TC->INTENSET.bit.OVF = 1;          // enable overfollow
    TC->INTENSET.bit.MC0 = 1;          // enable compare match to CC0
    
    NVIC_EnableIRQ( TCC_CONFIG[USE_TCC].tcc_irq); // Enable InterruptVector
    initTimerDone = true;
}

static void stopTimer(void)
{
    Tcc *TC = TCC_CONFIG[USE_TCC].tcc_ptr;
    
    TC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
    while (TC->SYNCBUSY.bit.ENABLE == 1); // wait for sync
}

static void startTimer(uint32_t delay_us)
{
    Tcc *TC = TCC_CONFIG[USE_TCC].tcc_ptr;
    
    if (!initTimerDone)
        initTimer();	// initial setup with stopped timer
    
    stopTimer();		// avoid timer interrupts while calculating
    
    /*
     * every 21333 ns equals one tick (1/(48000000/1024))
     * COUNT*DIVIDER*SECS until interrupt
     * 48 Mhz = (65536*1024)/1.398636s
     */
    long long nclocks = delay_us * 1000; // ns;
    nclocks = nclocks / 21333;
    int nCounts = nclocks;
   
    int bits = TCC_CONFIG[USE_TCC].nbits;
    int maxCounts = (uint32_t)(1<<bits)-1;

    if (nCounts > maxCounts) 	// if count exceeds timer capacity
        nCounts =  maxCounts;	// set the largest posible count.
    if (nCounts == 0)
        nCounts = 1;
    TC->CC[0].bit.CC = nCounts;
    while (TC->SYNCBUSY.bit.CC0 == 1); // wait for sync

    TC->CTRLA.reg |= TCC_CTRLA_ENABLE ; // Enable TC
    while (TC->SYNCBUSY.bit.ENABLE == 1); // wait for sync
#if 1
    Serial.print(millis(), DEC);
    Serial.print(" startTimer: nCounts=");
    Serial.println(nCounts, DEC);
#endif
}



#if USE_TCC == 0
void TCC0_Handler()
#elif USE_TCC == 1
void TCC1_Handler()
#elif USE_TCC == 2
void TCC2_Handler()
#endif
{
    static uint32_t last_usecs = 0;
    Tcc *TC = TCC_CONFIG[USE_TCC].tcc_ptr;
    uint32_t usecs = micros();
    uint32_t u_offset = 0;
    
    if (last_usecs && last_usecs < usecs) {
        /*
         * Problem is that the micros sometimes gives smaller values
         * compared to previuos micros. As a workaround we all 1ms.
         */
        u_offset = 1000;
    }
    last_usecs = usecs;

    /*
     * Overflow means the max timer exeeded, we need restart the timer
     * Interrupts and
     */
    if (TC->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
        Serial.print("OVF\r\n");
        TC->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    }
    
    if (TC->INTFLAG.bit.MC0 == 1) {  // A compare to cc0 caused the interrupt
        //Serial.print("MC0\r\n");
        TC->INTFLAG.bit.MC0 = 1;    // writing a one clears the MCO (match capture) flag
    }

    TC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
    while (TC->SYNCBUSY.bit.ENABLE == 1); // wait for sync
    
    for (int i = 0; i < MAX_TIMEOUTS-1; i++) {
        struct TimeoutVector *tvp = &TimeOuts[i];
        if (tvp->timer && tvp->timeout && usecs + u_offset >= tvp->timeout) {
            Timeout *saveTimer = tvp->timer;
            tvp->timer = NULL;
            tvp->timeout = 0;
            Timeout::_irq_handler(saveTimer);
        }
    }
    /*
     * we need to restart the timer for remaining interrupts
     * we provide the interrupt entry time in usecs which means
     * we don't count the irq_hander duration or debug prints
     */
    Timeout::restart(usecs);
}

#endif // D21 TCC Timer

void
Timeout::insert(void)
{
    noInterrupts();
    for (int i = 0; i < MAX_TIMEOUTS-1; i++) {
        struct TimeoutVector *tvp = &TimeOuts[i];
        if (tvp->timer == NULL) {
            tvp->timeout = _timeout;
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
            tvp->timeout = 0;
            break;
        }
    }
    interrupts();
}


void
Timeout::restart(uint32_t usecs)
{
    uint32_t timeout = ~0;
    
    /*
     * find the lowest timeout value which is our the next timeout
     * zero means stop the timer.
     */
    noInterrupts();
    for (int i = 0; i < MAX_TIMEOUTS-1; i++) {
        struct TimeoutVector *tvp = &TimeOuts[i];
        if (tvp->timer && tvp->timeout > 0) {
            if (tvp->timeout < timeout) {
                timeout = tvp->timeout;
            }
        }
    }
    interrupts();
    
    if (timeout == (uint32_t)~0) {
        stopTimer();
        return;
    }
    if (!usecs)
    	usecs = micros();
    
    if (timeout > usecs) {
        startTimer(timeout - usecs);
        return;
    } else {
        startTimer(1); // just one usec to trigger interrrupt
    }
}
#endif // ARDUINO
