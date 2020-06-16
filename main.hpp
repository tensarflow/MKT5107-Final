#include "mbed.h"

#ifndef PI
#define PI 3.14159265358979323846
#endif
#ifndef TWO_PI
#define TWO_PI 6.28318530717958647692
#endif

class Counter {
public:
  Counter(PinName pin) : _interrupt(pin) {
    _interrupt.rise(callback(this, &Counter::increment));
  }

  void increment() { _count++; }

  int read() { return _count; }

private:
  InterruptIn _interrupt;
  volatile int _count = 0;
  uint16_t state = 0;
};