/*
 * Added by Matt for analog sprayer boom pressure reading & section mapping
 * 2 Jun 2022
 * 
 * Uses 5 analog inputs to trigger AoG section mapping
 * 
 */

//Heart beat hello AgIO
uint8_t helloFromMachine[] = { 0x80, 0x81, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

//Switch Control 0x77 0xEA(234) -  Main (5), No Sections (8), ON Group0 (9), OFF Group0 (10), ON Group1 (11), OFF Group1 (12), CRC
uint8_t PGN_234[] = {0x80, 0x81, 0x77, 0xEA, 8, 2, 0, 0, 5, 0, 0, 0, 0, 0 };
int8_t PGN_234_Size = sizeof(PGN_234) - 1;

//Pressure data 0x7F 0x33(51) -  
//uint8_t PGN_51[] = {0x80, 0x81, 0x7F, 0x33, 5, 0, 0, 0, 0, 0, 0 };
//int8_t PGN_51_Size = sizeof(PGN_51) - 1;

byte swOnGr0  = 0;
byte swOffGr0 = 0;

// for digital inputs, just update each time autosteer loops updates
//const uint16_t PRESSURE_SENSOR_LOOP_TIME = 500;  //2Hz
//uint32_t pressureSensorLastTime = 0;
//uint32_t pressureTime;

/*const byte numPressSensors = 5;
int pressSensorPins[numPressSensors] = { A0, A1, A5, A4, A6 };  // have to !Serial3.begin (Serial3 uses A0 & A1)
int pressSensorReadings[numPressSensors];
bool pressSensorState[numPressSensors];
bool pressSensorPrevState[numPressSensors];*/

uint8_t numSectionInputs = 5;
uint8_t sectionInputs[5] = { A13, A16, A5, A4, A11 };


/*struct Pressure {
  int zeroOffset = 102;       // actual raw zero readings on sprayer, 102-120
  float calFactor = 5.5;      // ~5.5 per psi @5V
  int hiTriggerLevel = 40;    // actual PSI on trigger level
  int loTriggerLevel = 30;    // actual PSI off trigger level
}; Pressure pressSensor;      // 10 bytes
*/
// ~5.5 per psi @5V
// 320 ~40 psi @5V
// 267 ~30 psi @5V
// 211 ~20 psi @5V
// 156 ~10 psi @5V

// function prototype needed
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport);

//void readPressureSensors(){
void readSectionInputs(){
  bool debug = 0;
  //if (debug) Serial << "\npSensors";
  if (debug) Serial.print("\r\nSec Inputs: ");
  
  //swOnGr0  = 0; // not used because sections with pressure within hysterysis (Between on/off thresholds) should remain unchanged
  //swOffGr0 = 0;
 
  for (int i = 0; i < numSectionInputs; i++){
    /*pressSensorReadings[i] = analogRead(pressSensorPins[i]);//    delay(1);
    if (debug) Serial << " " << pressSensorPins[i] << ":" << pressSensorReadings[i];
    pressSensorReadings[i] -= pressSensor.zeroOffset;
    if (pressSensorReadings[i] < 0) pressSensorReadings[i] = 0;
    pressSensorReadings[i] /= pressSensor.calFactor;
    PGN_51[i + 5] = pressSensorReadings[i];
    if (debug) Serial << "> " << pressSensorReadings[i];
    if (pressSensorReadings[i] > pressSensor.hiTriggerLevel){
      pressSensorState[i] = 1;
      bitSet(swOnGr0, i);    // set respective bits 1 to toggle section ON in AoG
      bitClear(swOffGr0, i); // set respective bits 0 to stop toggling section OFF in AoG
    } else if (pressSensorReadings[i] < pressSensor.loTriggerLevel){
      pressSensorState[i] = 0;
      bitSet(swOffGr0, i);   // inverse of above
      bitClear(swOnGr0, i);
    }
    if (debug) Serial << ":" << pressSensorState[i];*/

    if (digitalRead(sectionInputs[i])){  // sec off
      if (debug) Serial.print("0 ");
      bitSet(swOffGr0, i);   // inverse of below
      bitClear(swOnGr0, i);      
    } else { // sec on
      if (debug) Serial.print("1 ");
      bitSet(swOnGr0, i);    // set respective bits 1 to toggle section ON in AoG
      bitClear(swOffGr0, i); // set respective bits 0 to stop toggling section OFF in AoG
    }
    
  }

  PGN_234[9] = swOnGr0;
  PGN_234[10] = swOffGr0;

  int16_t CK_A = 0;
  for (uint8_t i = 2; i < PGN_234_Size; i++)
    CK_A = (CK_A + PGN_234[i]);
  PGN_234[PGN_234_Size] = CK_A;

  SendUdp(PGN_234, sizeof(PGN_234), Eth_ipDestination, portDestination);

  /*if (debug){
    Serial << " <ON> ";
    Serial.print(swOnGr0, BIN);
    Serial << " <OFF> ";
    Serial.print(swOffGr0, BIN);
  }
*/
/*
  if (debug){
    Serial << " PGN_234: ";
    for (int i = 0; i < sizeof(PGN_234); i++){
      Serial << PGN_234[i] << ":";
    }
  }

  CK_A = 0;
  for (uint8_t i = 2; i < PGN_51_Size; i++)
    CK_A = (CK_A + PGN_51[i]);
  PGN_51[PGN_51_Size] = CK_A;
  
  SendUdp(PGN_51, sizeof(PGN_51), Eth_ipDestination, portDestination);
*/
}
/*
void pressureSensorLoop(){
  pressureTime = systick_millis_count;
  if (pressureTime - pressureSensorLastTime >= PRESSURE_SENSOR_LOOP_TIME){
    pressureSensorLastTime = pressureTime;
    readPressureSensors();
  }
}*/
