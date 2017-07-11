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
#define	wait_ms	delay

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
        _spi->endTransaction();
    }
    
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
        _spi->beginTransaction(SPISettings(_hz, MSBFIRST, _mode));
    	_spi->endTransaction();
    }
    void frequency(int hz) {
        _hz = hz;
        _spi->beginTransaction(SPISettings(_hz, MSBFIRST, _mode));
        _spi->endTransaction();
    }
    
    int write(int value) {
        _spi->beginTransaction(SPISettings(_hz, MSBFIRST, _mode));
        int ret = _spi->transfer(value);
        _spi->endTransaction();
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
        detachInterrupt(digitalPinToInterrupt(_gpioPin));
    };
    
    static void _irq_handler(InterruptIn *id) {
        if (id)
        	id->_func();
    }
    
    void rise(Callback<void()> func);
    
    void fall(Callback<void()> func);
    
private:
    int _gpioPin;
    Callback<void()> _func;
};

class Timer {
public:
    void start(void) {
        _time = micros();
    }
    int read_ms(void) {
        return (micros() - _time) / 1000;
    }
    int read_us(void) {
        return micros() - _time;
    }
private:
    uint32_t _time;
};


class Timeout {
public:
    Timeout() : _func() {
    }
    ~Timeout() {
        detach();
    }
    
    void attach(Callback<void()> func, int msecs) {
        if (msecs == 0)
            return detach();
        _func = func;
        _timeout = micros() + (uint32_t)msecs * 1000;
        insert();
        restart();
    }
    
    void attach_us(Callback<void()> func, long usecs) {
        if (usecs == 0)
            return detach();
        _func = func;
        _timeout = micros() + usecs;
        insert();
        restart();
    }
    
    void detach() {
        _func = NULL;
        remove();
        restart();
    }
    
    static void _irq_handler(Timeout *tp) {
        if (tp) {
            tp->_func();
        }
    }

    static void restart(uint32_t usecs = 0);
protected:
    void insert(void);
    void remove(void);
private:
    Callback<void()> _func;
    uint32_t _timeout;	// in us this lasts form 49 days.
};

#endif // __ARDUINO_MBED_H__

#endif // ARDUINO
