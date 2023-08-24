/*
 * Adapted by Matt for analog combine feedhouse height input
 * from analog sprayer boom pressure reading & section mapping
 * 23 Aug 2023
 * 
 * Uses 1 analog inputs to trigger AoG section 1 mapping
 * 
 */

//Heart beat hello AgIO
uint8_t helloFromMachine[] = { 0x80, 0x81, 123, 123, 5, 0, 0, 0, 0, 0, 71 };

//Switch Control 0x77 0xEA(234) -  Main (5), No Sections (8), ON Group0 (9), OFF Group0 (10), ON Group1 (11), OFF Group1 (12), CRC
uint8_t PGN_234[] = {0x80, 0x81, 0x77, 0xEA, 8, 2, 0, 0, 5, 0, 0, 0, 0, 0 };
int8_t PGN_234_Size = sizeof(PGN_234) - 1;

byte swOnGr0  = 0;
byte swOffGr0 = 0;

struct AnalogTrigger {
  uint8_t pin = A13;             // input pin
  uint8_t thresh = 128;       // "WORK input" threshold for analog feedhouse height for 575R
  uint8_t hyst = 5;           // hysterysis 
};  AnalogTrigger analogWork; // 3 bytes

uint8_t numSectionInputs = 1;
uint8_t sectionInputs[5] = { analogWork.pin, A16, A5, A4, A11 };

// function prototype needed
void SendUdp(uint8_t *data, uint8_t datalen, IPAddress dip, uint16_t dport);

void sectionSetup(){
  Serial.println("\r\nSetting section inputs");
  for (int i = 0; i < numSectionInputs; i++){
    pinMode(sectionInputs[i], INPUT_PULLUP);
    analogRead(sectionInputs[i]);
    Serial.print(" - analog Work pin "); Serial.print(sectionInputs[i]); Serial.print(" = "); Serial.println(analogRead(sectionInputs[i]));
  }
}

void readSectionInputs(){
  bool debug = 0;
  if (debug) Serial.print("\r\nSec Inputs: ");
  
  for (int i = 0; i < numSectionInputs; i++){
    //analogRead(sectionInputs[i]); // discard first reading
    int aRead = analogRead(sectionInputs[i]) / 4;    // divided by 4 to compare to byte
    if (aRead < analogWork.thresh - analogWork.hyst) // turn on section
    {
      if (bitRead(swOnGr0, i) == 0){
        Serial.print("section "); Serial.print(i+1); Serial.print(" = "); Serial.print(aRead); Serial.println(" OFF -> ON");
      }
      bitSet(swOnGr0, i);    // set respective bits HIGH to toggle section ON in AoG
      bitClear(swOffGr0, i); // set respective bits LOW to stop toggling section OFF in AoG
    }
    else if (aRead > analogWork.thresh + analogWork.hyst)
    {
      if (bitRead(swOnGr0, i) == 1){
        Serial.print("section "); Serial.print(i+1); Serial.print(" = "); Serial.print(aRead); Serial.println(" ON -> OFF");
      }
      bitSet(swOffGr0, i);    // set respective bits 1 to toggle section OFF in AoG
      bitClear(swOnGr0, i);   // set respective bits 0 to stop toggling section ON in AoG
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
