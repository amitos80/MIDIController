
#include "Button.h"


Button::Button(int pin1, int debounce1)
{
    pin = pin1;
    debounce = debounce1;
    countPrinted = 0;
    count = 0;
}

byte Button::read()
{
    byte buttonState = digitalRead(pin);
    return buttonState;
    if (buttonState != previousState) {
        unsigned long nowMillis = millis();
        if (nowMillis - countAt > debounce) {
            previousState = buttonState;
            countAt = 0;
            return buttonState;
        } else {
            if (countAt == 0) {
                countAt = millis();
            }
            return previousState;
        }
    } else {
        return previousState;
    }
}