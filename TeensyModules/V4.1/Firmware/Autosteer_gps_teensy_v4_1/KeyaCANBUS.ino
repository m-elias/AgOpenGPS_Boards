// KeyaCANBUS

#define lowByte(w) ((uint8_t)((w) & 0xFF))
#define highByte(w) ((uint8_t)((w) >> 8))

// templates for commands with matching responses, only need first 4 bytes
uint8_t keyaDisableCommand[] = { 0x23, 0x0C, 0x20, 0x01 };
uint8_t keyaDisableResponse[] = { 0x60, 0x0C, 0x20, 0x00 };

uint8_t keyaEnableCommand[] = { 0x23, 0x0D, 0x20, 0x01 };
uint8_t keyaEnableResponse[] = { 0x60, 0x0D, 0x20, 0x00 };

uint8_t keyaSpeedCommand[] = { 0x23, 0x00, 0x20, 0x01 };
uint8_t keyaSpeedResponse[] = { 0x60, 0x00, 0x20, 0x00 };

uint8_t keyaCurrentQuery[] = { 0x40, 0x00, 0x21, 0x01 };
uint8_t keyaCurrentResponse[] = { 0x60, 0x00, 0x21, 0x01 };

uint8_t keyaFaultQuery[] = { 0x40, 0x12, 0x21, 0x01 };
uint8_t keyaFaultResponse[] = { 0x60, 0x12, 0x21, 0x01 };

uint8_t keyaVoltageQuery[] = { 0x40, 0x0D, 0x21, 0x02 };
uint8_t keyaVoltageResponse[] = { 0x60, 0x0D, 0x21, 0x02 };

uint8_t keyaTemperatureQuery[] = { 0x40, 0x0F, 0x21, 0x01 };
uint8_t keyaTemperatureResponse[] = { 0x60, 0x0F, 0x21, 0x01 };

uint8_t keyaVersionQuery[] = { 0x40, 0x01, 0x11, 0x11 };
uint8_t keyaVersionResponse[] = { 0x60, 0x01, 0x11, 0x11 };

uint64_t KeyaID = 0x06000001; // 0x01 is default ID

const bool debugKeya = false;
bool lnNeeded = false;

// function not used anywhere
/*void keyaSend(uint8_t data[]) {
	//TODO Use this optimisation function once we're happy things are moving the right way
	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaID;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
	memcpy(KeyaBusSendData.buf, data, sizeof(data)); // using 'sizeof(data)' instead of '8' throws a compiler warning, data is only 8 bytes long
	Keya_Bus.write(KeyaBusSendData);
}*/

void CAN_Setup() {
	Keya_Bus.begin();
	Keya_Bus.setBaudRate(250000); // for official Keya motor
  //Keya_Bus.setBaudRate(500000);  // for identical motor from JinanLanJiu store https://www.aliexpress.com/item/1005005364248561.html
	delay(100);
	Serial.print("Initialised Keya CANBUS @ "); Serial.print(Keya_Bus.getBaudRate()); Serial.println("bps");
}

bool isPatternMatch(const CAN_message_t& message, const uint8_t* pattern, size_t patternSize) {
	return memcmp(message.buf, pattern, patternSize) == 0;
}

// only issue one query at a time, wait for respone
void  keyaCommand(uint8_t command[]) {
  CAN_message_t KeyaBusSendData;
  KeyaBusSendData.id = KeyaID;
  KeyaBusSendData.flags.extended = true;
  KeyaBusSendData.len = 8;
  memcpy(KeyaBusSendData.buf, command, 4);
  Keya_Bus.write(KeyaBusSendData);
}

void SteerKeya(int steerSpeed) {
	int actualSpeed = map(steerSpeed, -255, 255, -995, 995);
	if (steerSpeed == 0) {
    keyaCommand(keyaDisableCommand);
		if (debugKeya) Serial.println("steerSpeed zero - disabling");
		return; // don't need to go any further, if we're disabling, we're disabling
	}
	if (debugKeya) Serial.println("told to steer, with " + String(steerSpeed) + " so....");
	if (debugKeya) Serial.println("I converted that to speed " + String(actualSpeed));

	CAN_message_t KeyaBusSendData;
	KeyaBusSendData.id = KeyaID;
	KeyaBusSendData.flags.extended = true;
	KeyaBusSendData.len = 8;
  memcpy(KeyaBusSendData.buf, keyaSpeedCommand, 4);
	if (steerSpeed < 0) {
		KeyaBusSendData.buf[4] = highByte(actualSpeed);
		KeyaBusSendData.buf[5] = lowByte(actualSpeed);
		KeyaBusSendData.buf[6] = 0xff;
		KeyaBusSendData.buf[7] = 0xff;
		if (debugKeya) Serial.println("pwmDrive < zero - clockwise - steerSpeed " + String(steerSpeed));
	}
	else {
		KeyaBusSendData.buf[4] = highByte(actualSpeed);
		KeyaBusSendData.buf[5] = lowByte(actualSpeed);
		KeyaBusSendData.buf[6] = 0x00;
		KeyaBusSendData.buf[7] = 0x00;
		if (debugKeya) Serial.println("pwmDrive > zero - anticlock-clockwise - steerSpeed " + String(steerSpeed));
	}
	Keya_Bus.write(KeyaBusSendData);
  keyaCommand(keyaEnableCommand);
}


void KeyaBus_Receive() {
	CAN_message_t KeyaBusReceiveData;
	if (Keya_Bus.read(KeyaBusReceiveData)) {
		// parse the different message types

		// heartbeat 00:07:00:00:00:00:00:[ID]
		if (KeyaBusReceiveData.id == 0x07000001) {
      if (KeyaCurrentSensorReading < 0) Serial.println("Keya heartbeat detected, using Keya reported motor current for disengage");
			// 0-1 - Cumulative value of angle (360 def / circle)
			// 2-3 - Motor speed, signed int eg -500 or 500
			// 4-5 - Motor current, with "symbol" ? Signed I think that means, but it does appear to be a crap int. 1, 2 for 1, 2 amps etc
			//		is that accurate enough for us?
			// 6-7 - Control_Close (error code)
			// TODO Yeah, if we ever see something here, fire off a disable, refuse to engage autosteer or..?
      printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
      Serial.print(" HB ");

      // calc speed
      Serial.print(KeyaBusReceiveData.buf[2]); Serial.print(":"); Serial.print(KeyaBusReceiveData.buf[3]); Serial.print("=");
      if (KeyaBusReceiveData.buf[2] == 0xFF) {
        Serial.print("-");
        Serial.print(255 - KeyaBusReceiveData.buf[3]);
      } else {
        Serial.print(KeyaBusReceiveData.buf[3]);
      }
      Serial.print(" ");
      
      // calc current
      Serial.print(KeyaBusReceiveData.buf[4]); Serial.print(":"); Serial.print(KeyaBusReceiveData.buf[5]); Serial.print("=");
			if (KeyaBusReceiveData.buf[4] == 0xFF) {
        Serial.print("-");
        Serial.print(255 - KeyaBusReceiveData.buf[5]);
				KeyaCurrentSensorReading = (255 - KeyaBusReceiveData.buf[5]) * 20;
			} else {
        Serial.print(KeyaBusReceiveData.buf[5]);
				KeyaCurrentSensorReading = KeyaBusReceiveData.buf[5] * 20;
			}
      Serial.print(" ");

      // print error status
      Serial.print(KeyaBusReceiveData.buf[6]); Serial.print(":"); Serial.print(KeyaBusReceiveData.buf[7]); Serial.print(" ");// Serial.print(KeyaCurrentSensorReading);
      //  keyaCommand(keyaCurrentQuery);
      //  keyaCommand(keyaTemperatureQuery);
      //  keyaCommand(keyaVoltageQuery);
      //  keyaCommand(keyaVersionQuery);
      //  keyaCommand(keyaFaultQuery);
		}

    // parse query/command 00:05:08:00:00:00:00:[ID] responses
		if (KeyaBusReceiveData.id == 0x05800001) {

      // Disable command response
      if (isPatternMatch(KeyaBusReceiveData, keyaDisableResponse, sizeof(keyaDisableResponse))) {
        //printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        //Serial.print(" disable reply ");
      }

      // Enable command response
      else if (isPatternMatch(KeyaBusReceiveData, keyaEnableResponse, sizeof(keyaEnableResponse))) {
        //printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        //Serial.print(" enable reply ");
      }

      // Speed command response
      else if (isPatternMatch(KeyaBusReceiveData, keyaSpeedResponse, sizeof(keyaSpeedResponse))) {
        //printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        //Serial.print(" speed reply ");
      }

			// Current query response (this is also in heartbeat)
			else if (isPatternMatch(KeyaBusReceiveData, keyaCurrentResponse, sizeof(keyaCurrentResponse))) {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        Serial.print(" current reply "); Serial.print(KeyaBusReceiveData.buf[4]);
			}

      // Fault query response
			else if (isPatternMatch(KeyaBusReceiveData, keyaFaultResponse, sizeof(keyaFaultResponse))) {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        Serial.print(" fault reply "); Serial.print(KeyaBusReceiveData.buf[4]); Serial.print(":"); Serial.print(KeyaBusReceiveData.buf[5]);
			}

      // Voltage query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaVoltageResponse, sizeof(keyaVoltageResponse))) {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        Serial.print(" voltage reply "); Serial.print(KeyaBusReceiveData.buf[4]);
      }

      // Temperature query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaTemperatureResponse, sizeof(keyaTemperatureResponse))) {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        Serial.print(" temperature reply "); Serial.print(KeyaBusReceiveData.buf[4]);
      }

      // Version query response
      else if (isPatternMatch(KeyaBusReceiveData, keyaVersionResponse, sizeof(keyaVersionResponse))) {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        Serial.print(" version reply "); Serial.print(KeyaBusReceiveData.buf[4]); Serial.print(":"); Serial.print(KeyaBusReceiveData.buf[5]); Serial.print(":"); Serial.print(KeyaBusReceiveData.buf[6]); Serial.print(":"); Serial.print(KeyaBusReceiveData.buf[7]);
      }
      else
      {
        printIdAndReply(KeyaBusReceiveData.id, KeyaBusReceiveData.buf);
        Serial.print(" unknown reply ");
      }
		}

    if (lnNeeded){
      Serial.println();
      lnNeeded = false;
    }
	}
}

void printIdAndReply(uint32_t id, uint8_t buf[8]){
  Serial.print(id,HEX); Serial.print(" <> ");
  for (byte i = 0; i < 8; i++){
    if (buf[i] < 16) Serial.print("0");
    Serial.print(buf[i],HEX);
    if (i < 7) Serial.print(":");
  }
  lnNeeded = true;
}
