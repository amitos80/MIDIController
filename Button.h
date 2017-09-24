
#ifndef Button_h
#define Button_h

#include <Arduino.h>

class Button
{
  public:
    Button(int pin1, int debounce1);
    byte read();

  private:
    int pin;
    int debounce;
    unsigned long  countAt;
    int count;
    byte previousState;
    int countPrinted;
};

#endif