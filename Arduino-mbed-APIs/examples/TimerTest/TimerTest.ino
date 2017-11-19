
#include "arduino-mbed.h"

void TestTimeoutFunc(void);
void TestTimeoutFunc55(void);
void TestTimeoutFunc10(void);
void TestTimeoutFunc1m(void);
void SwitchInput(void);

#ifdef ARDUINO_ARCH_ESP32
#define SW0	0
#define LED	2
#define MYSERIAL Serial
#else
#define SW0   3  // switch needs pullup
#define LED       LED_BUILTIN
#define MYSERIAL Serial
#endif

DigitalOut led(LED);
InterruptIn intr(SW0);
Timeout tp;
Timeout tp2;
Timeout tp3;
Timeout tp4;

void setup() {
  MYSERIAL.begin(230400);
  InitSerial(&MYSERIAL, 5000, &led, true);

  ser->println("TimerTest");
  ser->println("TimerTest2");

  tp.attach(callback(&TestTimeoutFunc), 1000);
  tp2.attach(callback(&TestTimeoutFunc55), 5500);
  tp3.attach(callback(&TestTimeoutFunc10), 10000);
  // tp4.attach(callback(&TestTimeoutFunc1m), 1);
  
  intr.mode(PullUp);
  intr.fall(callback(&SwitchInput));
}

void loop() {
  // led = !led;

  sleep(); // or deepsleep()
}



void TestTimeoutFunc(void) {
   tp.attach(callback(&TestTimeoutFunc), 1000);   
   led = !led;
   ser->print(ms_getTicker(), DEC);
   ser->println(" TestTimeoutFunc 1 sec");
}

void TestTimeoutFunc55(void) {
   tp2.attach(callback(&TestTimeoutFunc55), 5500);
   ser->print(ms_getTicker(), DEC);
   ser->println(" TestTimeoutFunc 5.5 sec");
}


void TestTimeoutFunc10(void) {
   tp3.attach(callback(&TestTimeoutFunc10), 10000);
   ser->print(ms_getTicker(), DEC);
   ser->println(" TestTimeoutFunc 10 sec");
}


void TestTimeoutFunc1m(void) {
   tp4.attach(callback(&TestTimeoutFunc1m), 1);
}

void SwitchInput(void) {
   ser->print(ms_getTicker(), DEC);
   ser->println(" SwitchInput");  
   led = !led;
}
