/*
 * The file is Licensed under the Apache License, Version 2.0
 * (c) 2017 Helmut Tschemernjak
 * 30826 Garbsen (Hannover) Germany
 */

#ifdef ARDUINO

using namespace std;

#include "arduino-mbed.h"
#include "arduino-util.h"



#if defined(__SAMD21G18A__) || defined(__SAMD21J18A__)
/*
 * __SAMD21J18A__ is the SamD21 Explained Board
 * __SAMD21G18A__ is Genuino Zero-Board (compatible with the LoRa board)
 */

int
CPUID(uint8_t *buf, int maxSize, uint32_t xorval)
{
    int f1 = 0x55d5f559; // D21 128-bit UUID, first 32 bit.
    int f2 = 0x55d5f515; // D21 128-bit UUID, next 96 bit.

    if (maxSize >= 16 ) {
        int cnt = 0;
        int fa = f1 ^ xorval;
        uint32_t *first = (uint32_t *)fa;
        uint8_t *dst = (uint8_t *)first;
        for (int i = 0; i < (int)sizeof(uint32_t); i++)
        	*buf++ = *dst++;
        cnt += 4;
        int fb = f2 ^ xorval;
        uint32_t *next = (uint32_t *)fb;
        dst = (uint8_t *)next;
        for (int i = 0; i < (int)sizeof(uint32_t)*3; i++)
        	*buf++ = *dst++;
        cnt += 12;
        return cnt;
    }
    
    return 0;
}

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
const struct TCC_config {
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
 * versus TC Timer which supports only 8 or 16 bit counters only.
 * TCC0/1/2 timer work on the D21 using Arduino Zero.
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


Tcc *getTimeout_tcc(void)
{
    return TCC_data[USE_TCC_TIMEOUT].tcc_ptr;
}


void stopTimer(Tcc *t)
{
    t->CTRLA.reg &= ~TCC_CTRLA_ENABLE;   // Disable TC
    while (t->SYNCBUSY.bit.ENABLE == 1); // wait for sync
}


/* ----------------- TIMEOUT TIMER CODE ----------------------*/

void startTimer(Tcc *t, uint64_t delay_ns)
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


/* ----------------- D21 sleep() and deepsleep() code ----------------------*/

void sleep(void)
{
    /*
     * If we use the native USB port our Serial is SerialUSB
     * and if the SerialUSB and connected we should
     * not enter into sleep mode because this kills the Arduino USB emulation
     */
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk; // disbale SysTick
    uint32_t saved_ms = ms_getTicker();

    if (SerialUSB_active) {
        __DSB(); // ensures the completion of memory accesses
        __WFI(); // wait for interrupt
    } else {
#if  0 // (SAMD20 || SAMD21)
        /* Errata: Make sure that the Flash does not power all the way down
         * when in sleep mode. */
        NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
#endif
        
        SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;	// clear deep sleep
        PM->SLEEP.reg = 2; // SYSTEM_SLEEPMODE_IDLE_2 IDLE 2 sleep mode.
        
        __DSB(); // ensures the completion of memory accesses
        __WFI(); // wait for interrupt
    }
    
    int count = ms_getTicker() - saved_ms;
    if (count > 0) { // update the Arduino Systicks
        for (int i = 0; i < count; i++) {
            SysTick_Handler();
        }
    }
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk; // enable SysTick
}

/*
 * TODO
 * Check if we need to disable the USB GCLK->CLKCTRL.reg (see USBCore.cpp)
 * Check what else we need to disable?
 */

void deepsleep(void)
{
#if  0 // (SAMD20 || SAMD21)
    /* Errata: Make sure that the Flash does not power all the way down
     * when in sleep mode. */
    NVMCTRL->CTRLB.bit.SLEEPPRM = NVMCTRL_CTRLB_SLEEPPRM_DISABLED_Val;
#endif
    
    SCB->SCR |=  SCB_SCR_SLEEPDEEP_Msk; // standby mode
    //EIC->WAKEUP.bit.WAKEUPEN3 = 1; // enable wakeup on Pin 12/PA19/EXTINT[3] see variants.h
    
    __DSB(); // ensures the completion of memory accesses
    __WFI(); // wait for interrupt
}

#endif // D21 TCC Timer, sleep, etc-

#endif // ARDUINO
