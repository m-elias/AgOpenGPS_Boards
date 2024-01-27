#include <Arduino.h>

class AnalogInput{
public:
  AnalogInput(){ }

  void init(Stream* _stream){
    pin = 0;
    stream = _stream;
    clearStats();
  }

  void init(byte _pin, Stream* _stream){
    pin = _pin;
    pinMode(pin, INPUT_DISABLE);
    stream = _stream;
    clearStats();
  }

  void clearStats(){
    highReading = 0;
    lowReading = 4096;
  }

  void readPin(uint16_t _reading, bool _print){
    reading = _reading;
    if (reading > highReading) highReading = reading;
    if (reading < lowReading) lowReading = reading;
    if (_print) printStats();
  }

  void printStats(){
    stream->print(" "); stream->print(reading);
    stream->print("<>"); stream->print(highReading - lowReading);
  }

  byte pin;
  uint16_t reading, highReading, lowReading;
  Stream* stream;
};