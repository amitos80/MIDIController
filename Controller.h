#ifndef Controller_h
#define Controller_h

#include <Arduino.h>
#include "quaternionFilters.h"
#include "MPU9250.h"

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

//***********************************************************************
class Mux
{
  public:
    Mux(byte outpin_, byte numPins_, bool analog_);
    byte outpin;
    byte numPins;
    bool analog;
};
//************************************************************************
//Button (Pin Number, Command, Note Number, Channel, Debounce Time)
class Button
{
  public:
    Button(byte pin, byte command, byte value, byte channel, byte debounce);
    Button(Mux mux, byte muxpin, byte command, byte value, byte channel, byte debounce);
    byte getValue();
    void muxUpdate();
    void newValue(byte command, byte value, byte channel);
    byte Bcommand;
    byte Bvalue;
    byte Bchannel;
    byte Btoggle;

  private:
    byte _previous;
    byte _current;
    unsigned long _time;
    int _debounce;
    byte _pin;
    byte _muxpin;
    byte _numMuxPins;
    byte _value;
    byte _command;
    bool _busy;
    byte _status;
    byte _last;
    byte _enablepin;
};
//*************************************************************************
class Pot
{
  public:
    Pot(byte pin, byte command, byte control, byte channel);
    Pot(Mux mux, byte muxpin ,byte command, byte control, byte channel);
    void muxUpdate();
    void newValue(byte command, byte value, byte channel);
    byte getValue();
    byte Pcommand;
    byte Pcontrol;
    byte Pchannel;

  private:
    byte _pin;
    byte _muxpin;
    byte _numMuxPins;
    byte _control;
    int _value;
    int _oldValue;
    bool _changed;
    byte _enablepin;
};
//*************************************************************************
class Accl
{
  public:
    // Pin definitions
    int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
    int myLed  = 13;  // Set up pin 13 led for 
    MPU9250 myIMU;
    Accl(int c);
    void init();
    void updateValues();
};
#endif
