/*
 * The file is Licensed under the Apache License, Version 2.0
 * (c) 2017 Helmut Tschemernjak
 * 30826 Garbsen (Hannover) Germany
 */



#ifdef ARDUINO
#ifndef __ARDUINO_MBED_H__
#define __ARDUINO_MBED_H__

#include <arduino.h>
#include "Callback-A.h"
#include <SPI.h>
#undef min
#undef max
#undef map

typedef int PinName;
#define NC	-1
/* we need to redefine out dprintf because stdio.h uses the same name */
#define dprint	dxprintf
#if ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606
 #define MYdigitalPinToInterrupt(x)	digitalPinToInterrupt(x)
#else
 #define MYdigitalPinToInterrupt(x)	(x)
#endif

class DigitalOut;
void InitSerial(Stream *serial, int timeout_ms, DigitalOut *led, bool waitForSerial);
extern Stream *ser;
extern bool SerialUSB_active;

/*
 * Arduino_d21.cpp
 */
extern void startTimer(Tcc *t, uint64_t delay_ns);
extern void stopTimer(Tcc *t);
extern uint64_t ns_getTicker(void);
extern Tcc *getTimeout_tcc(void);
extern int CPUID(uint8_t *buf, int maxSize, uint32_t xorval);


extern void sleep(void);
extern void deepsleep(void);

#define MAX_TIMEOUTS	10
class Timeout;
struct TimeoutVector {
    Timeout *timer;
};
extern TimeoutVector TimeOuts[];


/*
 * Arduino-mbed.cpp
 */
extern uint32_t s_getTicker(void);
extern uint32_t ms_getTicker(void);
extern uint32_t us_getTicker(void);
extern void wait_ms(uint32_t ms);


enum PinMode {
    PullUp = 1,
    PullDown = 2,
};

class DigitalInOut {
public:
    DigitalInOut(PinName pin) {
        _gpioPin = pin;
    }
    void write(int value) {
        digitalWrite(_gpioPin, value == 0 ? LOW : HIGH);
    };
    
    void output() {
        pinMode(_gpioPin, OUTPUT);
    };
    
    void input() {
        pinMode(_gpioPin, INPUT);
    };
    
    void mode(PinMode pull) {
        switch(pull) {
            case PullUp:
                pinMode(_gpioPin, INPUT_PULLUP);
                break;
            case PullDown:
                pinMode(_gpioPin, INPUT_PULLDOWN);
                break;
        }
    }
              
    int read() {
        if (digitalRead(_gpioPin) == HIGH)
            return 1;
        else
            return 0;
    };
    operator int() {
        return read();
    };
    
    DigitalInOut& operator= (int value) {
        // Underlying write is thread safe
        write(value);
        return *this;
    }
    
    DigitalInOut& operator= (DigitalInOut& rhs) {
        write(rhs.read());
        return *this;
    }
    
private:
    int _gpioPin;
};

class DigitalOut : public DigitalInOut {
public:
    
    DigitalOut(PinName pin) : DigitalInOut(pin) {
        output();
    }
    
    DigitalOut& operator= (int value) {
        write(value);
        return *this;
    }
    
};

class DigitalIn : public DigitalInOut {
public:
    
    DigitalIn(PinName pin) :  DigitalInOut(pin) {
        input();
    }
};

class XSPI {
public:
    XSPI(PinName mosi, PinName miso, PinName sclk) {
        _mosi = mosi;
        _miso = miso;
        _sclk = sclk;
        if (mosi == PIN_SPI_MOSI && miso == PIN_SPI_MISO && sclk == PIN_SPI_SCK)
            _spi = &SPI;
#if SPI_INTERFACES_COUNT > 1
        else if (mosi == PIN_SPI1_MOSI && miso == PIN_SPI1_MISO && sclk == PIN_SPI1_SCK)
            _spi = &SPI1;
#endif
#if SPI_INTERFACES_COUNT > 2
        else if (mosi == PIN_SPI2_MOSI && miso == PIN_SPI2_MISO && sclk == PIN_SPI2_SCK)
            _spi = &SPI2;
#endif
        else {
            _spi = NULL;
            return;
        }
        _hz = 1000000;
        _mode = SPI_MODE0;
        _spi->beginTransaction(SPISettings(_hz, MSBFIRST, _mode));
    }
    ~XSPI() {
        _spi->endTransaction();
    };
    
    void format(int bits, int mode = 0) {
        if (mode == 0)
            _mode = SPI_MODE0;
        else if (mode == 1)
            _mode = SPI_MODE1;
        else if (mode == 2)
            _mode = SPI_MODE2;
        else if (mode == 3)
            _mode = SPI_MODE3;
        else
            _mode = SPI_MODE0;
        _bits = bits;
        _spi->endTransaction();
        _spi->beginTransaction(SPISettings(_hz, MSBFIRST, _mode));
    }
    void frequency(int hz) {
        _hz = hz;
        _spi->endTransaction();
        _spi->beginTransaction(SPISettings(_hz, MSBFIRST, _mode));
    }
    
    int write(int value) {
        int ret = _spi->transfer(value);
        return ret;
    }

private:
    SPIClass *_spi;
    int _hz;
    int _mode;
    int _bits;
    int _mosi, _miso, _sclk;
};

class InterruptIn {
public:
    static void donothing(void) {
    }
    
    InterruptIn(PinName pin) :  _func() {
        _gpioPin = pin;
        _func = InterruptIn::donothing;
        pinMode(_gpioPin, INPUT);
    }
    
    ~InterruptIn() {
        detachInterrupt(MYdigitalPinToInterrupt(_gpioPin));
    };
    
    static void _irq_handler(InterruptIn *id) {
        if (id)
        	id->_func();
    }
    
    void rise(Callback<void()> func);
    
    void fall(Callback<void()> func);
    
    void high(Callback<void()> func);
    
    void low(Callback<void()> func);

    void mode(PinMode pull) {
        switch(pull) {
            case PullUp:
                pinMode(_gpioPin, INPUT_PULLUP);
                break;
            case PullDown:
                pinMode(_gpioPin, INPUT_PULLDOWN);
                break;
        }
    }
    int read () {
        if (digitalRead(_gpioPin) == HIGH)
            return 0;
        else
            return 1;
    }
private:
    int _gpioPin;
    Callback<void()> _func;
};



class Timer {
public:
    void start(void) {
        _time = ns_getTicker();
    }
    uint32_t read_sec(void) {
        int64_t n = ns_getTicker() - (uint64_t)_time;
        n /= (uint64_t)1000000000;
        return n;
    }
    uint32_t read_ms(void) {
        int64_t n = ns_getTicker() - (uint64_t)_time;
        n /= (uint64_t)1000000;
        return n;
    }
    uint32_t read_us(void) {
        int64_t n = ns_getTicker() - (uint64_t)_time;
        n /= (uint64_t)1000;
        return n;
    }
private:
    uint64_t _time;
};


class Timeout {
public:
    Timeout() : _func() {
    }
    ~Timeout() {
        detach();
    }
    
    void attach_sec(Callback<void()> func, uint32_t secs) {
        if (secs == 0)
            return detach();
        _func = func;
        _timeout = ns_getTicker() + (uint64_t)secs * (uint64_t)1000000000;
        insert();
        restart();
    }

    void attach(Callback<void()> func, uint32_t msecs) {
        if (msecs == 0)
            return detach();
        _func = func;
        _timeout = ns_getTicker() + (uint64_t)msecs * (uint64_t)1000000;
        insert();
        restart();
    }
    
    void attach_us(Callback<void()> func, long usecs) {
        if (usecs == 0)
            return detach();
        _func = func;
        _timeout = ns_getTicker() + (uint64_t)usecs * (uint64_t)1000;
        insert();
        restart();
    }
    
    void detach(void) {
        _func = NULL;
        remove();
        restart();
    }
    
    static void _irq_handler(Timeout *tp) {
        if (tp) {
            tp->_func();
        }
    }

    static void restart(void);
    uint64_t _timeout;	// in ns this lasts for 539 years.
protected:
    void insert(void);
    void remove(void);
private:
    Callback<void()> _func;
};

#endif // __ARDUINO_MBED_H__

#endif // ARDUINO
