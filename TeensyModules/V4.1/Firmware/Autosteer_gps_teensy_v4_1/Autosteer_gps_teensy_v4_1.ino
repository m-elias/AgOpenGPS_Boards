// Single antenna, IMU, & dual antenna code for AgOpenGPS
// If dual right antenna is for position (must enter this location in AgOpen), left Antenna is for heading & roll
//
//modified by Pat 20/05/2023 BNO_RVC.cpp set 100 to the yaw reset
//
//
// connection plan:
// Teensy Serial 5 RX (21) to F9P Position receiver TX1 (Position data)
// Teensy Serial 5 TX (20) to F9P Position receiver RX1 (RTCM data for RTK)
// Teensy Serial 8 RX (34) to F9P Heading receiver TX1 (Relative position from left antenna to right antenna)
// Teensy Serial 8 TX (35) to F9P Heading receiver RX1
// F9P Position receiver TX2 to F9P Heading receiver RX2 (RTCM data for Moving Base)
//
// Configuration of receiver
// Position F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 460800
// Serial 1 In - RTCM (Correction Data from AOG)
// Serial 1 Out - NMEA GGA
// CFG-UART2-BAUDRATE 460800
// Serial 2 Out - RTCM 1074,1084,1094,1230,4072.0 (Correction data for Heading F9P, Moving Base)  
// 1124 is not needed (China’s BeiDou system) - Save F9P brain power 
//
// Heading F9P
// CFG-RATE-MEAS - 100 ms -> 10 Hz
// CFG-UART1-BAUDRATE 460800
// Serial 1 Out - UBX-NAV-RELPOSNED
// CFG-UART2-BAUDRATE 460800
// Serial 2 In RTCM

String inoVersion = ("\r\nFirmware Version !! RVC & A0 TEST !!, 2024.01.26 - Proton Test\r\n");

//#include "reset.h"
//Reset(uint8_t _btnIO, uint16_t _btnPressPeriod = 10000, uint8_t _ledIO = LED_BUILTIN)
//Reset teensyReset(A12, 2000);  //Used for Teensy reboot (short press) & reset to firmware default (10s long press, work in progress)

/************************* User Settings *************************/
// Serial Ports
#define SerialAOG Serial                //AgIO USB conection
#define SerialRTK Serial3               //RTK radio
HardwareSerial* SerialGPS = &Serial5;   //Main postion receiver (GGA) (Serial2 must be used here with T4.0 / Basic Panda boards - Should auto swap)
HardwareSerial* SerialGPS2 = &Serial8;  //Dual heading receiver 
HardwareSerial* SerialIMU = &Serial6;   //IMU BNO-085
HardwareSerial* SerialGPSTmp = NULL;
//HardwareSerial* SerialAOG = &Serial;

const int32_t baudAOG = 115200;
const int32_t baudGPS = 460800;
const int32_t baudRTK = 115200;     // most are using Xbee radios with default of 115200

// Baudrates for detecting UBX receiver
uint32_t baudrates[]
{
  4800,
  9600,
  19200,
  38400,
  57600,
  115200,
  230400,
  460800,
  921600
};

const uint32_t nrBaudrates = sizeof(baudrates)/sizeof(baudrates[0]);

#define ImuWire Wire        //SCL=19:A5 SDA=18:A4
#define RAD_TO_DEG_X_10 572.95779513082320876798154814105

//Swap BNO08x roll & pitch?
//const bool swapRollPitch = false;

const bool invertRoll= true;  //Used for IMU with dual antenna
#define baseLineLimit 5       //Max CM differance in baseline

#define REPORT_INTERVAL 20    //BNO report time, we want to keep reading it quick & offen. Its not timmed to anything just give constant data.
uint32_t READ_BNO_TIME = 0;   //Used stop BNO data pile up (This version is without resetting BNO everytime)

//Roomba Vac mode for BNO085 and data
#include "BNO_RVC.h"
BNO_rvc rvc = BNO_rvc();
BNO_rvcData bnoData;
elapsedMillis bnoTimer;
bool bnoTrigger = false;

#include <ADC.h>
#include <ADC_util.h>
ADC* adcWAS = new ADC();  //16x oversampling medium speed 12 bit A/D object

//Status LED's
#define GGAReceivedLED 13         //Teensy onboard LED
/*#define Power_on_LED 5            //Red
#define Ethernet_Active_LED 6     //Green
#define GPSRED_LED 9              //Red (Flashing = NO IMU or Dual, ON = GPS fix with IMU)
#define GPSGREEN_LED 10           //Green (Flashing = Dual bad, ON = Dual good)
#define AUTOSTEER_STANDBY_LED 11  //Red
#define AUTOSTEER_ACTIVE_LED 12   //Green*/
uint32_t gpsReadyTime = 0;        //Used for GGA timeout

#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

struct ConfigIP {
    uint8_t ipOne = 192;
    uint8_t ipTwo = 168;
    uint8_t ipThree = 5;
}; ConfigIP defaultNetworkAddress;

struct ConfigIP networkAddress = defaultNetworkAddress;

// IP & MAC address of this module of this module
byte Eth_myip[4] = { 0, 0, 0, 0}; //This is now set via AgIO
byte mac[] = {0x00, 0x00, 0x56, 0x00, 0x00, 0x78};

unsigned int portMy = 5120;             // port of this module
unsigned int AOGNtripPort = 2233;       // port NTRIP data from AOG comes in
unsigned int AOGAutoSteerPort = 8888;   // port Autosteer data from AOG comes in
unsigned int portDestination = 9999;    // Port of AOG that listens
char Eth_NTRIP_packetBuffer[512];       // buffer for receiving ntrip data
char Eth_NEMA_packetBuffer[512];        // buffer for receiving NEMA data

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Eth_udpNEMA;      //Port 5120
EthernetUDP Eth_udpNtrip;     //Port 2233
EthernetUDP Eth_udpAutoSteer; //Port 8888

IPAddress Eth_ipDestination;

byte CK_A = 0;
byte CK_B = 0;
int relposnedByteCount = 0;

//Speed pulse output
elapsedMillis speedPulseUpdateTimer = 0;
byte speedPulsePin = 18;      // Speedpulse (MPH speed) PWM pin
byte speedPulseSlowPin = 19;

#include "zNMEAParser.h"
#include <Wire.h>
#include "BNO08x_AOG.h"

extern "C" uint32_t set_arm_clock(uint32_t frequency); // required prototype

bool useDual = false;
bool dualBaselineFail = true;
bool dualRTKFail = true;
bool dualDataFail = true;
bool dualReadyGGA = false;
bool dualReadyRelPos = false;

// booleans to see if we are using CMPS or BNO08x
bool useCMPS = false;
bool useBNO08x = false;
bool useBNO08xRVC = false;

// BNO08x address variables to check where it is
const uint8_t bno08xAddresses[] = { 0x4A, 0x4B };
const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses) / sizeof(bno08xAddresses[0]);
uint8_t bno08xAddress;
BNO080 bno08x;

//Dual
double headingcorr = 900;  //90deg heading correction (90deg*10)
// Heading correction 180 degrees, because normally the heading antenna is in front, but we have it at the back
//double headingcorr = 1800;  // 180deg heading correction (180deg*10)

double baseline = 0;
double rollDual = 0;
double relPosD = 0;
double heading = 0;

byte ackPacket[72] = {0xB5, 0x62, 0x01, 0x3C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

constexpr int serial_buffer_size = 512;
uint8_t GPSrxbuffer[serial_buffer_size];    //Extra serial rx buffer
uint8_t GPStxbuffer[serial_buffer_size];    //Extra serial tx buffer
uint8_t GPS2rxbuffer[serial_buffer_size];   //Extra serial rx buffer
uint8_t GPS2txbuffer[serial_buffer_size];   //Extra serial tx buffer
uint8_t RTKrxbuffer[serial_buffer_size];    //Extra serial rx buffer

/* A parser is declared with 3 handlers at most */
NMEAParser<2> parser;

bool isTriggered = false;
bool blink = false;

bool Autosteer_running = true; //Auto set off in autosteer setup
bool Ethernet_running = false; //Auto set on in ethernet setup
bool GGA_Available = false;    //Do we have GGA on correct port?
uint32_t PortSwapTime = 0;

float roll = 0;
float pitch = 0;
float yaw = 0;

//Fusing BNO with Dual
double rollDelta;
double rollDeltaSmooth;
double correctionHeading;
double gyroDelta;
double imuGPS_Offset;
double gpsHeading;
double imuCorrected;
#define twoPI 6.28318530717958647692
#define PIBy2 1.57079632679489661923

// Buffer to read chars from Serial, to check if "!AOG" is found
uint8_t aogSerialCmd[4] = { '!', 'A', 'O', 'G'};
uint8_t aogSerialCmdBuffer[6];
uint8_t aogSerialCmdCounter = 0;

// Booleans to indictate to passthrough GPS or GPS2
bool passThroughGPS = false;
bool passThroughGPS2 = false;

//-=-=-=-=- UBX binary specific variables
struct ubxPacket
{
	uint8_t cls;
	uint8_t id;
	uint16_t len; //Length of the payload. Does not include cls, id, or checksum bytes
	uint16_t counter; //Keeps track of number of overall bytes received. Some responses are larger than 255 bytes.
	uint16_t startingSpot; //The counter value needed to go past before we begin recording into payload array
	uint8_t *payload; // We will allocate RAM for the payload if/when needed.
	uint8_t checksumA; //Given to us from module. Checked against the rolling calculated A/B checksums.
	uint8_t checksumB;
    
	////sfe_ublox_packet_validity_e valid;			 //Goes from NOT_DEFINED to VALID or NOT_VALID when checksum is checked
	////sfe_ublox_packet_validity_e classAndIDmatch; // Goes from NOT_DEFINED to VALID or NOT_VALID when the Class and ID match the requestedClass and requestedID
};

// Setup procedure ------------------------
void setup()
{
    delay(500);                         //Small delay so serial can monitor start up
    //set_arm_clock(150000000);
    //delay(100);           //Set CPU speed to 150mhz
    Serial.print("CPU speed set to: ");
    Serial.println(F_CPU_ACTUAL);

  pinMode(GGAReceivedLED, OUTPUT);
  /*pinMode(Power_on_LED, OUTPUT);
  pinMode(Ethernet_Active_LED, OUTPUT);
  pinMode(GPSRED_LED, OUTPUT);
  pinMode(GPSGREEN_LED, OUTPUT);
  pinMode(AUTOSTEER_STANDBY_LED, OUTPUT);
  pinMode(AUTOSTEER_ACTIVE_LED, OUTPUT);*/

  // the dash means wildcard
  parser.setErrorHandler(errorHandler);
  parser.addHandler("G-GGA", GGA_Handler);
  parser.addHandler("G-VTG", VTG_Handler);

  delay(10);
  Serial.println("Start setup");

  SerialGPS->begin(baudGPS);
  SerialGPS->addMemoryForRead(GPSrxbuffer, serial_buffer_size);
  SerialGPS->addMemoryForWrite(GPStxbuffer, serial_buffer_size);

  delay(10);
  SerialRTK.begin(baudRTK);
  SerialRTK.addMemoryForRead(RTKrxbuffer, serial_buffer_size);

  delay(10);
  SerialGPS2->begin(baudGPS);
  SerialGPS2->addMemoryForRead(GPS2rxbuffer, serial_buffer_size);
  SerialGPS2->addMemoryForWrite(GPS2txbuffer, serial_buffer_size);

  Serial.println("SerialAOG, SerialRTK, SerialGPS, SerialGPS2, SerailIMU all initialized");

  Serial.println("\r\nStarting AutoSteer...");
  autosteerSetup();
  
  Serial.println("\r\nStarting Ethernet...");
  EthernetStart();

  Serial.println("\r\nStarting IMU...");
  //bool reboot = false;    // for testing, used to reboot until no BNO is detected, can be removed 

  //check for RVC/Serial BNO first
  pinMode(A4, INPUT); // set SDA to high impedance
  SerialIMU->begin(115200);
  rvc.begin(SerialIMU);
  static elapsedMillis rvcBnoTimer = 0;
  Serial.println("Checking for RVC/Serial BNO08x");
  while (rvcBnoTimer < 1000)
  {
      if (rvc.read(&bnoData)) //check if new bnoData
      {
          useBNO08xRVC = true;
          Serial.println(" - RVC/Serial BNO08x Good To Go :-)");
          imuHandler();
          //reboot = true;
          break;
      }
  }

  if(!useBNO08xRVC) // if no RVC BNO, then look for I2C BNO
  {
      Serial.println(" - no RVC/Serial BNO08x Connected or Found");
      Serial.println("Checking for I2C BNO08x");
      
      ImuWire.begin();
      uint8_t bnoAttempts = 10;  // 7 seems to be enough, tested all night, 10 might be safer but causes more delay if not detected
      
      for (int16_t i = 0; i < nrBNO08xAdresses; i++)
      {
          uint8_t tries = 0;
          while (!bno08x.begin(bno08xAddresses[i], ImuWire) && tries < bnoAttempts) //??? Passing NULL to non pointer argument, remove maybe ???
          {
            Serial.print(" - BNO08x not detected at 0x"); Serial.println(bno08xAddresses[i], HEX);
            tries++;
            //delay(50);
          }
          //if (tries == 0) reboot = true;
          if (tries < bnoAttempts){
            ImuWire.setClock(400000); 
            delay(300);
            bno08x.enableGameRotationVector(REPORT_INTERVAL);
            useBNO08x = true;
            Serial.println(" - I2C BNO08x Good To Go!");
            //reboot = true;
            break;
          }
      }
  }

  //delay(100);
  Serial.print("\r\nuseBNO08xRVC = ");
  Serial.println(useBNO08xRVC);
  Serial.print("useBNO08x = ");
  Serial.println(useBNO08x);

  Serial.println(inoVersion);
  Serial.println("\r\nwaiting for GPS...\r\n");
/*
  if (useBNO08x && reboot){
    delay(100);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
  }

  if (useBNO08xRVC && reboot){
    delay(100);
    SCB_AIRCR = 0x05FA0004; //Teensy Reset
  }*/
}

void loop()
{
/*    if (teensyReset.update()){  // true return means reset settings to defaults
      // set to firmware defaults code goes here

      // set defaults
      //networkAddress = defaultNetworkAddress;
      //steerConfig = defaultSteerConfig; // doesn't work because loop()doesn't have access to these structs, don't know how to fix it
      

      // save all defaults to EEPROM, needs restructuring into a function call so that these lines are only in one place
      // copied from autosteerSetup()
      // EEPROM doesn't work here either
      //EEPROM.put(10, steerSettings);
      //EEPROM.put(40, steerConfig);
      //EEPROM.put(60, networkAddress);    

      Serial.println("\r\n\n****** Factory/firmware defaults set ******");
      Serial.println("\r\n**************** Rebooting ****************");
      teensyReset.reboot(true);      
    }*/

    if (GGA_Available == false && !passThroughGPS && !passThroughGPS2)
    {
        if (systick_millis_count - PortSwapTime >= 10000)
        {
            Serial.println("GPS Data Missing... Check F9P Config\r\n");
            SerialGPSTmp = SerialGPS;
            SerialGPS = SerialGPS2;
            SerialGPS2 = SerialGPSTmp;
            PortSwapTime = systick_millis_count;
        }
    }

    if (!useCMPS && !useBNO08x)
    {
        //RVC BNO08x
        if (rvc.read(&bnoData)) useBNO08xRVC = true;
    }

    /*if (useBNO08x && bnoTimer > 99){
      imuHandler();
      bnoTimer = 0;
    }*/

    if (useBNO08xRVC && bnoTimer > 40 && bnoTrigger)
    {
        bnoTrigger = false;
        imuHandler();   //Get IMU data ready
    }

    // Pass NTRIP etc to GPS
    if (SerialAOG.available())
    {
        uint8_t incoming_char = SerialAOG.read();

        // Check incoming char against the aogSerialCmd array
        // The configuration utility will send !AOGR1, !AOGR2 or !AOGED (close/end)
        if (aogSerialCmdCounter < 4 && aogSerialCmd[aogSerialCmdCounter] == incoming_char)
        {
            aogSerialCmdBuffer[aogSerialCmdCounter] = incoming_char;
            aogSerialCmdCounter++;
        }
        // Whole command prefix is in, handle it
        else if (aogSerialCmdCounter == 4)
        {
            aogSerialCmdBuffer[aogSerialCmdCounter] = incoming_char;
            aogSerialCmdBuffer[aogSerialCmdCounter + 1] = SerialAOG.read();

            if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'R')
            {
                HardwareSerial* autoBaudSerial = NULL;

                // Reset SerialGPS and SerialGPS2
                SerialGPS = &Serial7;
                SerialGPS2 = &Serial2;

                if (aogSerialCmdBuffer[aogSerialCmdCounter + 1] == '1')
                {
                    passThroughGPS = true;
                    passThroughGPS2 = false;
                    autoBaudSerial = SerialGPS;
                }
                else if (aogSerialCmdBuffer[aogSerialCmdCounter + 1] == '2')
                {
                    passThroughGPS = false;
                    passThroughGPS2 = true;
                    autoBaudSerial = SerialGPS2;
                }
				
				const uint8_t UBX_SYNCH_1 = 0xB5;
                const uint8_t UBX_SYNCH_2 = 0x62;
                const uint8_t UBX_CLASS_ACK = 0x05;
                const uint8_t UBX_CLASS_CFG = 0x06;
                const uint8_t UBX_CFG_RATE = 0x08;

                ubxPacket packetCfg{};

                packetCfg.cls = UBX_CLASS_CFG;
                packetCfg.id = UBX_CFG_RATE;
                packetCfg.len = 0;
                packetCfg.startingSpot = 0;

                calcChecksum(&packetCfg);

                byte mon_rate[] = {0xB5, 0x62, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
                mon_rate[2] = packetCfg.cls; 
                mon_rate[3] = packetCfg.id; 
                mon_rate[4] = packetCfg.len & 0xFF; 
                mon_rate[5] = packetCfg.len >> 8;
                mon_rate[6] = packetCfg.checksumA; 
                mon_rate[7] = packetCfg.checksumB; 

                // Check baudrate
                bool communicationSuccessfull = false;
				uint32_t baudrate = 0;                

				for (uint32_t i = 0; i < nrBaudrates; i++)
				{
					baudrate = baudrates[i];

					Serial.print(F("Checking baudrate: "));
					Serial.println(baudrate);

					autoBaudSerial->begin(baudrate);
					delay(100);

					// first send dumb data to make sure its on
					autoBaudSerial->write(0xFF);

					// Clear
					while (autoBaudSerial->available() > 0)
					{
						autoBaudSerial->read();
					}

					// Send request
					autoBaudSerial->write(mon_rate, 8);

					uint32_t millis_read = systick_millis_count;
					constexpr uint32_t UART_TIMEOUT = 1000;
					int ubxFrameCounter = 0;
					bool isUbx = false;
					uint8_t incoming = 0;

					uint8_t requestedClass = packetCfg.cls;
					uint8_t requestedID = packetCfg.id;

					uint8_t packetBufCls = 0;
					uint8_t packetBufId = 0;

					do
					{
						while (autoBaudSerial->available() > 0)
						{
							incoming = autoBaudSerial->read();

							if (!isUbx && incoming == UBX_SYNCH_1) // UBX binary frames start with 0xB5, aka μ
							{
								ubxFrameCounter = 0;
								isUbx = true;
							}

							if (isUbx)
							{
								// Decide what type of response this is
								if ((ubxFrameCounter == 0) && (incoming != UBX_SYNCH_1))      // ISO 'μ'
								{
									isUbx = false;                                            // Something went wrong. Reset.
								}
								else if ((ubxFrameCounter == 1) && (incoming != UBX_SYNCH_2)) // ASCII 'b'
								{
									isUbx = false;                                            // Something went wrong. Reset.
								}
								else if (ubxFrameCounter == 1 && incoming == UBX_SYNCH_2)
								{
									// Serial.println("UBX_SYNCH_2");
									// isUbx should be still true
								}
								else if (ubxFrameCounter == 2) // Class
								{
									// Record the class in packetBuf until we know what to do with it
									packetBufCls = incoming; // (Duplication)
								}
								else if (ubxFrameCounter == 3) // ID
								{
									// Record the ID in packetBuf until we know what to do with it
									packetBufId = incoming; // (Duplication)

									// We can now identify the type of response
									// If the packet we are receiving is not an ACK then check for a class and ID match
									if (packetBufCls != UBX_CLASS_ACK)
									{
										// This is not an ACK so check for a class and ID match
										if ((packetBufCls == requestedClass) && (packetBufId == requestedID))
										{
											// This is not an ACK and we have a class and ID match
											communicationSuccessfull = true;
										}
										else
										{
											// This is not an ACK and we do not have a class and ID match
											// so we should keep diverting data into packetBuf and ignore the payload
											isUbx = false;
										}
									}
								}
							}

							// Finally, increment the frame counter
							ubxFrameCounter++;
						}
					} while (systick_millis_count - millis_read < UART_TIMEOUT);

					if (communicationSuccessfull)
					{
						break;
					}
				}

				if (communicationSuccessfull)
				{
					SerialAOG.write(aogSerialCmdBuffer, 6);
					SerialAOG.print(F("Found reciever at baudrate: "));
					SerialAOG.println(baudrate);

					// Let the configuring program know it can proceed
					SerialAOG.println("!AOGOK");
				}
				else
				{
					SerialAOG.println(F("u-blox GNSS not detected. Please check wiring."));
				}

				aogSerialCmdCounter = 0;
			}
            // END command. maybe think of a different abbreviation
            else if (aogSerialCmdBuffer[aogSerialCmdCounter] == 'E' && aogSerialCmdBuffer[aogSerialCmdCounter + 1] == 'D')
            {
                passThroughGPS = false;
                passThroughGPS2 = false;
                aogSerialCmdCounter = 0;
            }
        }
        else
        {
            aogSerialCmdCounter = 0;
		}

        if (passThroughGPS)
        {
            SerialGPS->write(incoming_char);
        }
        else if (passThroughGPS2)
        {
            SerialGPS2->write(incoming_char);
        }
        else
        {
            SerialGPS->write(incoming_char);
        }
    }

    // Read incoming nmea from GPS
    if (SerialGPS->available())
    {
        if (passThroughGPS)
        {
            SerialAOG.write(SerialGPS->read());
        }
        else
        {
            parser << SerialGPS->read();
        }
    }

    // Read any NMEA sent via UDP
    int packetLength = Eth_udpNEMA.parsePacket();

    if (packetLength > 0)
    {
        Eth_udpNEMA.read(Eth_NEMA_packetBuffer, packetLength);
        for (int i = 0; i < packetLength; i++)
        {
            parser << Eth_NEMA_packetBuffer[i];
        }
    }

    udpNtrip();

    // Check for RTK Radio
    if (SerialRTK.available())
    {
        SerialGPS->write(SerialRTK.read());
    }

    // If anything comes in SerialGPS2 RelPos data
    if (SerialGPS2->available())
    {
        uint8_t incoming_char = SerialGPS2->read();  //Read RELPOSNED from F9P

        if (passThroughGPS2)
        {
            SerialAOG.write(incoming_char);
        }
        else
        {
            // Just increase the byte counter for the first 3 bytes
            if (relposnedByteCount < 4 && incoming_char == ackPacket[relposnedByteCount])
            {
                relposnedByteCount++;
            }
            else if (relposnedByteCount > 3)
            {
                // Real data, put the received bytes in the buffer
                ackPacket[relposnedByteCount] = incoming_char;
                relposnedByteCount++;
            }
            else
            {
                // Reset the counter, becaues the start sequence was broken
                relposnedByteCount = 0;
            }
        }
    }

    // Check the message when the buffer is full
    if (relposnedByteCount > 71)
    {
        if (calcChecksum())
        {
            //if(deBug) Serial.println("RelPos Message Recived");
            //digitalWrite(GPSRED_LED, LOW);   //Turn red GPS LED OFF (we are now in dual mode so green LED)
            useDual = true;
            dualDataFail = false;
            relPosDecode();
        }

        else 
        {
            dualDataFail = true;
        }
        
        relposnedByteCount = 0;
    }

    //GGA timeout, turn off GPS LED's etc
    if((systick_millis_count - gpsReadyTime) > 10000) //GGA age over 10sec
    {
      //digitalWrite(GPSRED_LED, LOW);
      //digitalWrite(GPSGREEN_LED, LOW);
      useDual = false;
    }

    //Read BNO
    if((systick_millis_count - READ_BNO_TIME) > REPORT_INTERVAL && useBNO08x)
    {
      READ_BNO_TIME = systick_millis_count;
      readBNO();
    }
    
    if (Autosteer_running) autosteerLoop();
    else ReceiveUdp();
    
  if (Ethernet.linkStatus() == LinkOFF) 
  {
    //digitalWrite(Power_on_LED, 1);
    //digitalWrite(Ethernet_Active_LED, 0);
  }
  if (Ethernet.linkStatus() == LinkON) 
  {
    //digitalWrite(Power_on_LED, 0);
    //digitalWrite(Ethernet_Active_LED, 1);
  }

}//End Loop
//**************************************************************************

bool calcChecksum()
{
  CK_A = 0;
  CK_B = 0;

  for (int i = 2; i < 70; i++)
  {
    CK_A = CK_A + ackPacket[i];
    CK_B = CK_B + CK_A;
  }

  return (CK_A == ackPacket[70] && CK_B == ackPacket[71]);
}

//Given a message, calc and store the two byte "8-Bit Fletcher" checksum over the entirety of the message
//This is called before we send a command message
void calcChecksum(ubxPacket *msg)
{
  msg->checksumA = 0;
  msg->checksumB = 0;

  msg->checksumA += msg->cls;
  msg->checksumB += msg->checksumA;

  msg->checksumA += msg->id;
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len & 0xFF);
  msg->checksumB += msg->checksumA;

  msg->checksumA += (msg->len >> 8);
  msg->checksumB += msg->checksumA;

  for (uint16_t i = 0; i < msg->len; i++)
  {
    msg->checksumA += msg->payload[i];
    msg->checksumB += msg->checksumA;
  }
}
