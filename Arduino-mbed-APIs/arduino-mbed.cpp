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


#define MAX_TIMEOUTS	10
class Timeout;
struct TimeoutVector {
    Timeout *timer;
} TimeOuts[MAX_TIMEOUTS];


#if defined(__SAMD21G18A__) || defined(__SAMD21J18A__)
/*
 * __SAMD21J18A__ is the SamD21 Explained Board
 * __SAMD21G18A__ is Genuino Zero-Board (compatible with the LoRa board)
 */

/*
 * see tcc.h is automatically included from:
 * Arduino15/packages/arduino/tools/CMSIS-Atmel/1.1.0/CMSIS/
 * Device/ATMEL/samd21/include/component/tcc.h
 * See also tcc.c (ASF/mbed, e.g. Tcc_get_count_value)
 */
static void initTimer(Tcc *t);
static uint32_t getTimerCount(Tcc *t);

/*
 * The Atmel D21 has three TCC timer, other models have more.
 */
static const struct TCC_config {
    Tcc *tcc_ptr;
    IRQn_Type tcc_irq;
    uint8_t nbits;
} TCC_data[] {
    { TCC0, TCC0_IRQn, 24 },
    { TCC1, TCC1_IRQn, 24 },
    { TCC2, TCC2_IRQn, 16 },
    { NULL, (IRQn_Type)NULL, 0 }
};

/*
 * We preferably use the TCC timers because it supports 24-bit counters
 * versus TC Timer which supports only 8 or 16 bit counters
 * TCC0/1/2 timer work on the D21 using Arduino.
 */
#define USE_TCC_TIMEOUT	0 // 0=TCC0, 1=TTC1, 2=TTC2 (see TCC_data)
#define USE_TCC_TICKER	1


/*
 * every 21333 ns equals one tick (1/(48000000/1024)) // prescaler 1024, 48 MHz
 * every 61035 ns equals one tick (1/(32768/2))		  // prescaler 2, 32 kHz
 * COUNT*DIVIDER*SECS until interrupt
 * CPU 48 MHz = (65536*1024)/1.398636s
 * RTC 32 kHz = (65536*2)/4.0s
 */
#define NS_PER_CLOCK_CPU	21333 // ns secs per clock
#define NS_PER_CLOCK_RTC	61035 // ns secs per clock

#define NS_PER_CLOCK	NS_PER_CLOCK_RTC

/* ----------------- TICKER TIMER CODE ----------------------*/

/*
 * The global ns_counter contains the time in ns from the last time
 * the counter has been wrapped. It cannot be used directly because the
 * current counter has to be added fore using it. Use instead
 * ns_getTicker(), us_ ns_getTicker(), ms_getTicker()
 */

uint64_t ticker_ns;
static bool initTickerDone = false;

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


uint64_t ns_getTicker(void)
{
    Tcc *t = TCC_data[USE_TCC_TICKER].tcc_ptr;
    if (!initTickerDone) {
        initTimer(t);
        initTickerDone = true;

        // set counter top to max 16 bit for testing
        // t->PER.bit.PER = 0xffff;
        // while (t->SYNCBUSY.bit.PER == 1); // wait for sync

        t->CTRLA.reg |= TCC_CTRLA_ENABLE ;		// Enable TC
        while (t->SYNCBUSY.bit.ENABLE == 1);	// wait for sync
    }
    
    /*
     * if we are called from the interrupt level, the counter contains
     * somehow wrong data, therfore we needs to read it twice.
     * Another option was to add a little wait (loop 500x) 
     * in the TCC_TIMEOUT interrupt handler.
     */
    if (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) // check if we are in the interrupt
        getTimerCount(t);

    uint64_t counter_us = (uint64_t)NS_PER_CLOCK * (uint64_t)getTimerCount(t);
    uint64_t ns = ticker_ns + counter_us;

    return ns;
}

#if USE_TCC_TICKER == 0
void TCC0_Handler()
#elif USE_TCC_TICKER == 1
void TCC1_Handler()
#elif USE_TCC_TICKER == 2
void TCC2_Handler()
#endif
{
    Tcc *t = TCC_data[USE_TCC_TICKER].tcc_ptr;
    /*
     * Overflow means the timer top exeeded
     */
    if (t->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
        t->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
        // ser->println("T_OVF");

        /*
         * reading the count once is needed, otherwise
         * it will not wrap correct.
         */
        getTimerCount(t);
        
        int bits = TCC_data[USE_TCC_TICKER].nbits;
        int maxCounts = (uint32_t)(1<<bits);
		
        ticker_ns += (uint64_t)NS_PER_CLOCK * (uint64_t)maxCounts;
    }
    if (t->INTFLAG.bit.MC0 == 1) {  // A compare to cc0 caused the interrupt
        t->INTFLAG.bit.MC0 = 1;    // writing a one clears the MCO (match capture) flag
        // ser->println("T_MC0");
    }
}

/* ----------------- SUPPORT CODE FOR TCC TIMERS----------------------*/

static bool initTimerDone = false;

static void initTimer(Tcc *t)
{
    
    /*
     * enable clock for TCC, see gclk.h
     * GCLK_CLKCTRL_GEN_GCLK0 for 48 Mhz CPU
     * GCLK_CLKCTRL_GEN_GCLK1 for 32k extern crystal XOSC32K (ifdef CRYSTALLESS)
     * GCLK_CLKCTRL_GEN_GCLK1 for 32k internal OSC32K
     * see Arduino: arduino/hardware/samd/1.6.15/cores/arduino/startup.c
     * Use TCC_CTRLA_PRESCALER_DIV1024 for for 48 Mhz clock
     * Use TCC_CTRLA_PRESCALER_DIV2 for 32k clock
     */
    if (t == TCC0 || t == TCC1) {
        REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_TCC0_TCC1);
    } else if (t == TCC2) {
        REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_ID_TCC2_TC3_Val);
    }
    while (GCLK->STATUS.bit.SYNCBUSY == 1); // wait for sync
    
    t->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TCC
    while (t->SYNCBUSY.bit.ENABLE == 1); // wait for sync
    
    t->CTRLA.reg |= (TCC_CTRLA_PRESCALER_DIV2 | TCC_CTRLA_RUNSTDBY); // Set perscaler
    
    t->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ; // Set wave form configuration
    while (t->SYNCBUSY.bit.WAVE == 1);	   // wait for sync
    
    t->PER.bit.PER = 0xffffff; // set counter top to max 24 bit
    while (t->SYNCBUSY.bit.PER == 1); // wait for sync
    
    // the compare counter TC->CC[0].reg will be set in the startTimer
    // after the timeout calculation is known.
    
    // Interrupts
    t->INTENSET.reg = 0;              // disable all interrupts
    t->INTENSET.bit.OVF = 1;          // enable overfollow
    t->INTENSET.bit.MC0 = 1;          // enable compare match to CC0
    
    const struct TCC_config *cp = &TCC_data[0];
    while (cp->tcc_ptr) {
        if (cp->tcc_ptr == t) {
            NVIC_EnableIRQ(cp->tcc_irq); // Enable InterruptVector
            break;
        }
        cp++;
    }
}

#if 0
// Atmel ASF Code
static uint32_t getTimerCount(Tcc *t)
{
    uint32_t last_cmd;
    /* Wait last command done */
    do {
        while (t->SYNCBUSY.bit.CTRLB); /* Wait for sync */
        
        last_cmd = t->CTRLBSET.reg & TCC_CTRLBSET_CMD_Msk;
        if (TCC_CTRLBSET_CMD_NONE == last_cmd) {
            /* Issue read command and break */
            t->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val;
            break;
        } else if (TCC_CTRLBSET_CMD_READSYNC == last_cmd) {
            /* Command have been issued */
            break;
        }
    } while (1);
    
    while (t->SYNCBUSY.bit.COUNT); /* Wait for sync */

    return t->COUNT.reg;
}
#endif


static uint32_t getTimerCount(Tcc *t)
{
    
    noInterrupts();

    while (t->SYNCBUSY.bit.CTRLB); /* Wait for sync */

    t->CTRLBSET.bit.CMD = TCC_CTRLBSET_CMD_READSYNC_Val; /* Issue read command and break */

    while (t->SYNCBUSY.bit.COUNT); /* Wait for sync */
    
    uint32_t count = t->COUNT.reg;

    interrupts();
    
    return count;
}


static void stopTimer(Tcc *t)
{
    t->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
    while (t->SYNCBUSY.bit.ENABLE == 1); // wait for sync
}


/* ----------------- TIMEOUT TIMER CODE ----------------------*/

static void startTimer(Tcc *t, uint64_t delay_ns)
{
    if (!initTimerDone) {
        initTimer(t);	// initial setup with stopped timer
        initTimerDone = true;
    }
    
    stopTimer(t);		// avoid timer interrupts while calculating
    
    /*
     * every 21333 ns equals one tick (1/(48000000/1024))
     * COUNT*DIVIDER*SECS until interrupt
     * 48 Mhz = (65536*1024)/1.398636s
     */
    uint64_t nclocks = (uint64_t)delay_ns;
    nclocks /= (uint64_t)NS_PER_CLOCK;
    int nCounts = nclocks;
   
    int bits = TCC_data[USE_TCC_TIMEOUT].nbits;
    int maxCounts = (uint32_t)(1<<bits)-1;

    if (nCounts > maxCounts) 	// if count exceeds timer capacity
        nCounts =  maxCounts;	// set the largest posible count.
    if (nCounts <= 0)
        nCounts = 1;
    t->CC[0].bit.CC = nCounts;
    while (t->SYNCBUSY.bit.CC0 == 1); // wait for sync

    t->CTRLA.reg |= TCC_CTRLA_ENABLE ; // Enable TC
    while (t->SYNCBUSY.bit.ENABLE == 1); // wait for sync
#if 0
    ser->print(ms_getTicker(), DEC);
    ser->print(" startTimer: nCounts=");
    ser->println(nCounts, DEC);
#endif
}


#if USE_TCC_TIMEOUT == 0
void TCC0_Handler()
#elif USE_TCC_TIMEOUT == 1
void TCC1_Handler()
#elif USE_TCC_TIMEOUT == 2
void TCC2_Handler()
#endif
{
    Tcc *t = TCC_data[USE_TCC_TIMEOUT].tcc_ptr;
    uint64_t nsecs = ns_getTicker();
    
    /*
     * Overflow means the max timer exeeded, we need restart the timer
     * Interrupts and
     */
    if (t->INTFLAG.bit.OVF == 1) {  // A overflow caused the interrupt
        t->INTFLAG.bit.OVF = 1;    // writing a one clears the flag ovf flag
    }
    
    if (t->INTFLAG.bit.MC0 == 1) {  // A compare to cc0 caused the interrupt
        //ser->print("MC0\r\n");
        t->INTFLAG.bit.MC0 = 1;    // writing a one clears the MCO (match capture) flag
    }

    t->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
    while (t->SYNCBUSY.bit.ENABLE == 1); // wait for sync
    
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


#endif // D21 TCC Timer

void
Timeout::insert(void)
{
    noInterrupts();
    for (int i = 0; i < MAX_TIMEOUTS-1; i++) {
        struct TimeoutVector *tvp = &TimeOuts[i];
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
    Tcc *t = TCC_data[USE_TCC_TIMEOUT].tcc_ptr;
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

/* ----------------- D21 sleep() and deepsleep() code ----------------------*/

void sleep(void)
{
    /*
     * If we use the native USB port our Serial is SerialUSB
     * and if the SerialUSB and connected we should
     * not enter into sleep mode because this kills the Arduino USB emulation
     */
    if (ser && ser == (Stream *)&SerialUSB) {
        __WFI();
        return;
        // USB->CTRLA.bit.ENABLE = 0;
        // USB->HOST.CTRLA.reg = 0;
        // USB->HOST.CTRLA.bit.ENABLE &= USB_CTRLA_ENABLE;
    }

    
#if  1 // (SAMD20 || SAMD21)
    /* Errata: Make sure that the Flash does not power all the way down
     * when in sleep mode. */
    NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
#endif
    uint32_t saved_ms = ms_getTicker();
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // disbale SysTick
    
    SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;	// clear deep sleep
    PM->SLEEP.reg = 2; // SYSTEM_SLEEPMODE_IDLE_2 IDLE 2 sleep mode.

    __DSB(); // ensures the completion of memory accesses
    __WFI(); // wait for interrupt
    
    int count = ms_getTicker() - saved_ms;
    if (count > 0) { // update the Arduino Systicks
        for (int i = 0; i < count; i++) {
            SysTick_Handler();
        }
    }
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // enable SysTick
}

void deepsleep(void)
{
#if  1 // (SAMD20 || SAMD21)
    /* Errata: Make sure that the Flash does not power all the way down
     * when in sleep mode. */
    NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
#endif

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // disbale SysTick
    
    SCB->SCR |=  SCB_SCR_SLEEPDEEP_Msk; // standby mode
    
    __DSB(); // ensures the completion of memory accesses
    __WFI(); // wait for interrupt

    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // enable SysTick
}


#endif // ARDUINO
