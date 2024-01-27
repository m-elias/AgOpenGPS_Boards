class Reset{
  public:
    Reset(uint8_t _btnIO, uint16_t _btnPressPeriod = 10000, uint8_t _ledIO = LED_BUILTIN){
      btnIO = _btnIO;
      pinMode(btnIO, INPUT_PULLUP);
      btnPressPeriod = _btnPressPeriod;
      ledIO = _ledIO;
      pinMode(ledIO, OUTPUT);
    }

    bool update(){
      if (digitalRead(btnIO) == LOW){
        uint32_t btnPressTime = millis();
        digitalWrite(ledIO, HIGH);
        while (digitalRead(btnIO) == LOW && millis() - btnPressTime <= btnPressPeriod){
          delay(5);
        }
        if (millis() - btnPressTime >= btnPressPeriod){  // btn held for more than btnPressPeriod (10s default)
          while (digitalRead(btnIO) == LOW){  // flash LED while btn is held to indicate factory/firmware reset is triggered
            digitalWrite(ledIO, !digitalRead(ledIO));
            delay(50);
          }
          digitalWrite(ledIO, LOW);
          return true;
        } else {  // short btn press, do reboot instead of settings reset
          reboot();
        }
      }
      return false;
    }

    void reboot(bool _pause = false){
      if (_pause) delay(100);   // for serial etc to finish
      SCB_AIRCR = 0x05FA0004;   // Teensy Reboot
    }

  private:
    uint8_t btnIO;
    uint16_t btnPressPeriod;
    uint8_t ledIO;
};
