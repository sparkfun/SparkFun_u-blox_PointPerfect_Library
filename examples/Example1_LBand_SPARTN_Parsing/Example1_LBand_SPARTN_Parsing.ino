/*
  u-blox PointPerfect Library (PPL) example
  =========================================

  This example shows how to:
  * Configure a u-blox NEO-D9S to receive L-Band correction data
  * The PMP messages output by the NEO-D9S are parsed and assembled into SPARTN messages
  * The SPARTN data is CRC-checked and then sent into the PPL
  * The PPL will output RTCM data
  * The RTCM data is pushed to a u-blox ZED-F9P allowing it to achieve a FIXED RTK solution
  * NMEA GPGGA messages are sent from the ZED into the PPL. GPZDA is also beneficial
  * The current encryption key is read from secrets.h and sent into the PPL
  * The PPL also needs satellite ephemeris data
    * This can be RXM-SFRBX, or RTCM (1019, 1020, 1042, 1046). This example uses SFRBX
  
  Note: this example needs v3.1.0 of the u-blox_GNSS_V3 library - which is able to buffer
        14 consecutive SFRBX messages for callback

  Note: you need a valid u-blox Thingstream PointPerfect L-Band or L-Band + IP account to
        access the SPARTN decryption key. Copy and paste the current key into secrets.h
  
  Hardware:
  This example was written for:
  * SparkFun Thing Plus - ESP32 WROOM (USB-C) (https://www.sparkfun.com/products/20168)
  * The ZED-F9P / NEO-D9S Combo Board (https://www.sparkfun.com/products/22560)
  * Although you can use separate boards if desired
    * ZED-F9P (https://www.sparkfun.com/products/16481)
    * NEO-D9S (https://www.sparkfun.com/products/19390)
    * Optional antenna splitter (https://www.sparkfun.com/products/21223)

  Connect the Thing Plus to the Combo Board using a Qwiic cable
  Connect a good quality L1/L2/L-Band antenna to the NEO-D9S and ZED-F9P

  Output:
  The PPL works well, even with a marginal L-Band PMP signal. Here the ZED-F9P has a fixed solution
  even though the NEO-D9S ebno is around 5dB and the error rate (fecBits) is high.
  Ideally, ebno should be >= 9dB and fecBits close to zero.

  Lat: 54.8000000  Long: -1.4000000  Height: 103.367  Fix: 3 (3D)  Carrier Solution: 2 (Fixed)  Horizontal Accuracy Estimate: 32 (mm)
  New RXM-PMP data received. userData: 504 Bytes. fecBits: 313. ebno (dB): 5.37.
  Received RTCM 1005 1033 1230 1074 1084 1094 from PPL. Pushing to the GNSS...
  UBX-RXM-COR:  ebno: 0.000  protocol: RTCM3  errStatus: Error-free  msgUsed: Used
  Lat: 54.8000000  Long: -1.4000000  Height: 103.365  Fix: 3 (3D)  Carrier Solution: 2 (Fixed)  Horizontal Accuracy Estimate: 32 (mm)
  New RXM-PMP data received. userData: 504 Bytes. fecBits: 311. ebno (dB): 5.50.
  Received RTCM 1005 1033 1230 1074 1084 1094 from PPL. Pushing to the GNSS...
  Lat: 54.8000000  Long: -1.4000000  Height: 103.329  Fix: 3 (3D)  Carrier Solution: 2 (Fixed)  Horizontal Accuracy Estimate: 31 (mm)
  UBX-RXM-COR:  ebno: 0.000  protocol: RTCM3  errStatus: Error-free  msgUsed: Used
  SPARTN: 1205	SFRBX: 21455	GPGGA: 1316	GPZDA: 1316

  Licence: please see LICENSE.md for more details. The PPL is reproduced by kind permission of u-blox

*/

#include "secrets.h" // <- Copy and paste the Current Key into secrets.h

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myGNSS; // ZED-F9x
SFE_UBLOX_GNSS myLBand; // NEO-D9S

#include "PPL_PublicInterface.h" // The PPL
#include "PPL_Version.h"

const uint32_t myLBandFreq = 1556290000; // Uncomment this line to use the US SPARTN 1.8 service
//const uint32_t myLBandFreq = 1545260000; // Uncomment this line to use the EU SPARTN 1.8 service

#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

// Keep count of the number of messages received. Print the count periodically
uint32_t sfrbxReceived = 0;
uint32_t spartnReceived = 0;
uint32_t nmeaGGAreceived = 0;
uint32_t nmeaZDAreceived = 0;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

const char *PPLReturnStatusToStr(ePPL_ReturnStatus status)
{
  switch (status)
  {
    case ePPL_Success:
      return("operation was successful");
    case ePPL_IncorrectLibUsage:
      return("incorrect usage of the library");
    case ePPL_LibInitFailed:
      return("library failed to initialize");
    case ePPL_LibExpired:
      return("library has expired");
    case ePPL_LBandChannelNotEnabled:
      return("L-Band channel has not been initialized");
    case ePPL_IPChannelNotEnabled:
      return("IP channel has not been initialized");
    case ePPL_AuxChannelNotEnabled:
      return("Aux channel has not been initialized");
    case ePPL_NoDynamicKey:
      return("stream is encrypted and no valid dynamic key found");
    case ePPL_FailedDynKeyLibPush:
      return("key push failed");
    case ePPL_InvalidDynKey:
      return("invalid key format");
    case ePPL_IncorrectDynKey:
      return("key does not match the key used for data encryption");
    case ePPL_RcvPosNotAvailable:
      return("GGA position not available");
    case ePPL_LeapSecsNotAvailable:
      return("leap seconds not available");
    case ePPL_AreaDefNotAvailableForPos:
      return("position is outside of Point Perfect coverage");
    case ePPL_TimeNotResolved:
      return("time is not resolved");
    case ePPL_PotentialBuffOverflow:
      return("potential buffer overflow");
    case ePPL_UnknownState:
    default:
      return("unknown state");
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Parse RTCM data
// This is a very simplistic parser. It only extracts the Message Numbers.
// It does not CRC-check the data.

String & parseRTCM(uint8_t *dataBytes, size_t numDataBytes)
{
  static String desc;

  static uint16_t dataLength = 0;
  static uint16_t messageNumber = 0;

  enum parseStates
  {
    waitingForD3,
    length1,
    length2,
    number1,
    number2,
    readingBytes
  };
  static parseStates parseState = waitingForD3;

  if (parseState == waitingForD3)
    desc = ""; // Clear any existing description

  for (size_t i = 0; i < numDataBytes; i++) // Step through each byte
  {
    switch (parseState)
    {
    case waitingForD3:
      if (*(dataBytes + i) == 0xD3)
        parseState = length1;
      break;
    case length1:
      dataLength = (((uint16_t)*(dataBytes + i)) & 0x03) << 8;
      parseState = length2;
      break;
    case length2:
      dataLength |= *(dataBytes + i);
      dataLength += 3; // Add CRC
      parseState = number1;
      break;
    case number1:
      messageNumber = ((uint16_t)*(dataBytes + i)) << 4;
      parseState = number2;
      dataLength--;
      break;
    case number2:
      messageNumber |= (*(dataBytes + i)) >> 4;
      parseState = readingBytes;
      dataLength--;
      desc += String(messageNumber);
      desc += String(" ");
      break;
    case readingBytes:
      dataLength--;
      if (dataLength == 0)
        parseState = waitingForD3;
      break;
    }
  }

  return (desc);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: processRXMPMP will be called when new PMP data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_PMP_message_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRXMPMPmessageCallbackPtr
//        /               _____  This _must_ be UBX_RXM_PMP_message_data_t
//        |              /                         _____ You can use any name you like for the struct
//        |              |                        /
//        |              |                        |
void processRXMPMP(UBX_RXM_PMP_message_data_t *pmpData)
{
  //Extract the raw message payload length
  uint16_t payloadLen = ((uint16_t)pmpData->lengthMSB << 8) | (uint16_t)pmpData->lengthLSB;

  uint16_t numBytesUserData = pmpData->payload[2] | ((uint16_t)pmpData->payload[3] << 8);
  uint16_t fecBits = pmpData->payload[20] | ((uint16_t)pmpData->payload[21] << 8);
  float ebno = (float)pmpData->payload[22] / 8;
  
  Serial.print(F("New RXM-PMP data received. userData: "));
  Serial.print(numBytesUserData);
  Serial.print(F(" Bytes. fecBits: "));
  Serial.print(fecBits);
  Serial.print(F(". ebno (dB): "));
  Serial.print(ebno);
  Serial.println(F("."));

  //Parse the SPARTN data stream contained in the userData
  for (uint16_t i = 0; i < numBytesUserData; i++)
  {
    bool valid = false;
    uint16_t len;
    uint8_t *spartn = myLBand.parseSPARTN(pmpData->payload[24 + i], valid, len);

    if (valid)
    {
      // Serial.print(F("Valid SPARTN data parsed. Pushing "));
      // Serial.print(len);
      // Serial.println(F(" bytes to the PPL..."));

      spartnReceived++;
      
      ePPL_ReturnStatus result = PPL_SendAuxSpartn(spartn, len); // Push the SPARTN data to the PPL
      if (result != ePPL_Success)
      {
        Serial.print(F("processRXMPMP PPL_SendAuxSpartn: "));
        Serial.println(PPLReturnStatusToStr(result));
      }
    }
  }

  (void)payloadLen; // Avoid pesky compiler warnings-as-errors
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: processSFRBX will be called when new RXM SFRBX data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_SFRBX_message_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMSFRBXmessageCallbackPtr
//        /               _____  This _must_ be UBX_RXM_SFRBX_message_data_t
//        |              /                         _____ You can use any name you like for the struct
//        |              |                        /
//        |              |                        |
void processSFRBX(UBX_RXM_SFRBX_message_data_t *sfrbxData)
{
  //Extract the raw message payload length
  uint16_t payloadLen = ((uint16_t)sfrbxData->lengthMSB << 8) | (uint16_t)sfrbxData->lengthLSB;

  // uint8_t gnssId = sfrbxData->payload[0];
  // uint8_t svId = sfrbxData->payload[1];

  // Serial.print(F("New RXM-SFRBX data received. gnssId: "));
  // Serial.print(gnssId);
  // Serial.print(F(" svId: "));
  // Serial.print(svId);

  // Serial.print(F(". Pushing "));
  // Serial.print(payloadLen);
  // Serial.println(F(" bytes to the PPL..."));

  sfrbxReceived++;
      
  ePPL_ReturnStatus result = PPL_SendRcvrData((const char *)&sfrbxData->sync1, (size_t)payloadLen + 6); // Push the SFRBX data to the PPL
  if (result != ePPL_Success)
  {
    Serial.print(F("processSFRBX PPL_SendRcvrData (1): "));
    Serial.println(PPLReturnStatusToStr(result));
  }
  result = PPL_SendRcvrData((const char *)&sfrbxData->checksumA, (size_t)2);
  if (result != ePPL_Success)
  {
    Serial.print(F("processSFRBX PPL_SendRcvrData (2): "));
    Serial.println(PPLReturnStatusToStr(result));
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printPVTdata will be called when new NAV PVT data arrives
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallbackPtr
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  double latitude = ubxDataStruct->lat; // Print the latitude
  Serial.print(F("Lat: "));
  Serial.print(latitude / 10000000.0, 7);

  double longitude = ubxDataStruct->lon; // Print the longitude
  Serial.print(F("  Long: "));
  Serial.print(longitude / 10000000.0, 7);

  double altitude = ubxDataStruct->hMSL; // Print the height above mean sea level
  Serial.print(F("  Height: "));
  Serial.print(altitude / 1000.0, 3);

  uint8_t fixType = ubxDataStruct->fixType; // Print the fix type
  Serial.print(F("  Fix: "));
  Serial.print(fixType);
  if (fixType == 0)
    Serial.print(F(" (None)"));
  else if (fixType == 1)
    Serial.print(F(" (Dead Reckoning)"));
  else if (fixType == 2)
    Serial.print(F(" (2D)"));
  else if (fixType == 3)
    Serial.print(F(" (3D)"));
  else if (fixType == 4)
    Serial.print(F(" (GNSS + Dead Reckoning)"));
  else if (fixType == 5)
    Serial.print(F(" (Time Only)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint8_t carrSoln = ubxDataStruct->flags.bits.carrSoln; // Print the carrier solution
  Serial.print(F("  Carrier Solution: "));
  Serial.print(carrSoln);
  if (carrSoln == 0)
    Serial.print(F(" (None)"));
  else if (carrSoln == 1)
    Serial.print(F(" (Floating)"));
  else if (carrSoln == 2)
    Serial.print(F(" (Fixed)"));
  else
    Serial.print(F(" (UNKNOWN)"));

  uint32_t hAcc = ubxDataStruct->hAcc; // Print the horizontal accuracy estimate
  Serial.print(F("  Horizontal Accuracy Estimate: "));
  Serial.print(hAcc);
  Serial.print(F(" (mm)"));

  Serial.println();    
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printRXMCOR will be called when new RXM COR data arrives
// See u-blox_structs.h for the full definition of UBX_RXM_COR_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setRXMCORcallbackPtr
//        /                  _____  This _must_ be UBX_RXM_COR_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void printRXMCOR(UBX_RXM_COR_data_t *ubxDataStruct)
{
  Serial.print(F("UBX-RXM-COR:  ebno: "));
  Serial.print((double)ubxDataStruct->ebno / 8, 3); //Convert ebno to dB

  Serial.print(F("  protocol: "));
  if (ubxDataStruct->statusInfo.bits.protocol == 1)
    Serial.print(F("RTCM3"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 2)
    Serial.print(F("SPARTN"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 29)
    Serial.print(F("PMP (SPARTN)"));
  else if (ubxDataStruct->statusInfo.bits.protocol == 30)
    Serial.print(F("QZSSL6"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  errStatus: "));
  if (ubxDataStruct->statusInfo.bits.errStatus == 1)
    Serial.print(F("Error-free"));
  else if (ubxDataStruct->statusInfo.bits.errStatus == 2)
    Serial.print(F("Erroneous"));
  else
    Serial.print(F("Unknown"));

  Serial.print(F("  msgUsed: "));
  if (ubxDataStruct->statusInfo.bits.msgUsed == 1)
    Serial.print(F("Not used"));
  else if (ubxDataStruct->statusInfo.bits.msgUsed == 2)
    Serial.print(F("Used"));
  else
    Serial.print(F("Unknown"));

  Serial.println();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printGPGGA will be called when new GPGGA NMEA data arrives
// See u-blox_structs.h for the full definition of NMEA_GGA_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setNMEAGPGGAcallback
//        /               _____  This _must_ be NMEA_GGA_data_t
//        |              /           _____ You can use any name you like for the struct
//        |              |          /
//        |              |          |
void printGPGGA(NMEA_GGA_data_t *nmeaData)
{
  // Serial.print(F("GPGGA received. Pushing "));
  // Serial.print(nmeaData->length);
  // Serial.println(F(" bytes to the PPL..."));

  nmeaGGAreceived++;
  
  ePPL_ReturnStatus result = PPL_SendRcvrData((const char *)nmeaData->nmea, nmeaData->length); // Push the data to the PPL
  if (result != ePPL_Success)
  {
    Serial.print(F("printGPGGA PPL_SendRcvrData: "));
    Serial.println(PPLReturnStatusToStr(result));
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Callback: printGPZDA will be called when new GPZDA NMEA data arrives
// See u-blox_structs.h for the full definition of NMEA_ZDA_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setNMEAGPZDAcallback
//        /               _____  This _must_ be NMEA_ZDA_data_t
//        |              /           _____ You can use any name you like for the struct
//        |              |          /
//        |              |          |
void printGPZDA(NMEA_ZDA_data_t *nmeaData)
{
  // Serial.print(F("GPZDA received. Pushing "));
  // Serial.print(nmeaData->length);
  // Serial.println(F(" bytes to the PPL..."));

  nmeaZDAreceived++;
  
  ePPL_ReturnStatus result = PPL_SendRcvrData((const char *)nmeaData->nmea, nmeaData->length); // Push the data to the PPL
  if (result != ePPL_Success)
  {
    Serial.print(F("printGPZDA PPL_SendRcvrData: "));
    Serial.println(PPLReturnStatusToStr(result));
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);

  Serial.begin(115200);
  Serial.println(F("PointPerfect Library Corrections"));
  Serial.println(PPL_SDK_VERSION);

  Wire.begin(); //Start I2C

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the ZED-F9x

  //myGNSS.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myGNSS.begin() == false) //Connect to the u-blox module using Wire port
  {
    Serial.println(F("u-blox GNSS module not detected at default I2C address. Please check wiring."));
    delay(1000);
  }
  Serial.println(F("u-blox GNSS module connected"));

  uint8_t ok = myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // We need NMEA and RTCM3
  if (ok) ok = myGNSS.setI2CInput(COM_TYPE_UBX | COM_TYPE_RTCM3); // We need RTCM3
  if (ok) ok = myGNSS.setUART1Output(0);
  if (ok) ok = myGNSS.setUART1Input(0);
  if (ok) ok = myGNSS.setUART2Output(0);
  if (ok) ok = myGNSS.setUART2Input(0); // No cheating! Disable SPARTN input on combo board
  if (ok) ok = myGNSS.setDGNSSConfiguration(SFE_UBLOX_DGNSS_MODE_FIXED); // Set the differential mode - ambiguities are fixed whenever possible
  if (ok) ok = myGNSS.setNavigationFrequency(1); //Set output in Hz.
  if (ok) ok = myGNSS.setVal8(UBLOX_CFG_MSGOUT_UBX_RXM_COR_I2C, 1); // Enable UBX-RXM-COR messages on I2C
  
  // Disable or enable various NMEA sentences over the I2C interface
  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR); // Use cfgValset to disable / enable individual NMEA messages
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GLL_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_VTG_I2C, 0);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_I2C, 1); // Enable GPGGA on I2C
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_ZDA_I2C, 1); // Enable GPZDA on I2C
  if (ok) ok = myGNSS.sendCfgValset(); // Send the configuration VALSET

  // Set the Main Talker ID to "GP". The NMEA GGA messages will be GPGGA instead of GNGGA
  if (ok) ok = myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_GP);
  //if (ok) ok = myGNSS.setMainTalkerID(SFE_UBLOX_MAIN_TALKER_ID_DEFAULT); // Uncomment this line to restore the default main talker ID

  Serial.print(F("GNSS: configuration "));
  Serial.println(OK(ok));

  myGNSS.setAutoPVTcallbackPtr(&printPVTdata); // Enable automatic NAV PVT messages with callback to printPVTdata so we can watch the carrier solution go to fixed

  myGNSS.setRXMCORcallbackPtr(&printRXMCOR); // Print the contents of UBX-RXM-COR messages so we can check if the PPL RTCM data is being generated and used

  myGNSS.setAutoRXMSFRBXmessageCallbackPtr(&processSFRBX); // Send incoming SFRBX data to the PointPerfect Library

  // Set up the callback for GPGGA
  myGNSS.setNMEAGPGGAcallbackPtr(&printGPGGA); // Send GPGGA to the PointPerfect Library

  // Set up the callback for GPZDA
  myGNSS.setNMEAGPZDAcallbackPtr(&printGPZDA); // Send GPZDA to the PointPerfect Library

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the NEO-D9S L-Band receiver

  //myLBand.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myLBand.begin(Wire, 0x43) == false) //Connect to the u-blox NEO-D9S using Wire port. The D9S default I2C address is 0x43 (not 0x42)
  {
    Serial.println(F("u-blox NEO-D9S not detected at default I2C address. Please check wiring."));
    delay(1000);
  }
  Serial.println(F("u-blox NEO-D9S connected"));

  myLBand.newCfgValset(); // Create a new Configuration Interface message - this defaults to VAL_LAYER_RAM_BBR (change in RAM and BBR)
  myLBand.addCfgValset(UBLOX_CFG_PMP_CENTER_FREQUENCY,     myLBandFreq); // Default 1539812500 Hz
  myLBand.addCfgValset(UBLOX_CFG_PMP_SEARCH_WINDOW,        2200);        // Default 2200 Hz
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_SERVICE_ID,       0);           // Default 1 
  myLBand.addCfgValset(UBLOX_CFG_PMP_SERVICE_ID,           21845);       // Default 50821
  myLBand.addCfgValset(UBLOX_CFG_PMP_DATA_RATE,            2400);        // Default 2400 bps
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_DESCRAMBLER,      1);           // Default 1
  myLBand.addCfgValset(UBLOX_CFG_PMP_DESCRAMBLER_INIT,     26969);       // Default 23560
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_PRESCRAMBLING,    0);           // Default 0
  myLBand.addCfgValset(UBLOX_CFG_PMP_UNIQUE_WORD,          16238547128276412563ull); 
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C,   1);           // Ensure UBX-RXM-PMP is enabled on the I2C port 
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1, 0);           // No cheating! Disable UBX-RXM-PMP output on UART1
  myLBand.addCfgValset(UBLOX_CFG_UART2OUTPROT_UBX,         1);           // Enable UBX output on UART2
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 0);           // No cheating! Disable UBX-RXM-PMP output on UART2
  myLBand.addCfgValset(UBLOX_CFG_UART1_BAUDRATE,           38400);       // match baudrate with ZED default
  myLBand.addCfgValset(UBLOX_CFG_UART2_BAUDRATE,           38400);       // match baudrate with ZED default
  ok = myLBand.sendCfgValset(); // Apply the settings
  
  Serial.print(F("L-Band: configuration "));
  Serial.println(OK(ok));

  myLBand.softwareResetGNSSOnly(); // Do a restart

  myLBand.setRXMPMPmessageCallbackPtr(&processRXMPMP); // Call processRXMPMP when new PMP data arrives. Push it to the PPL

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin the PointPerfect Library and give it the current key

  ePPL_ReturnStatus result = PPL_Initialize(PPL_CFG_ENABLE_AUX_CHANNEL);
  Serial.print(F("PPL_Initialize: "));
  Serial.println(PPLReturnStatusToStr(result));

  result = PPL_SendDynamicKey(currentDynamicKey, currentKeyLength);
  Serial.print(F("PPL_SendDynamicKey: "));
  Serial.println(PPLReturnStatusToStr(result));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myGNSS.checkUblox(); // Check for the arrival of new GNSS data and process it.
  myGNSS.checkCallbacks(); // Check if any GNSS callbacks are waiting to be processed.

  myLBand.checkUblox(); // Check for the arrival of new PMP data and process it.
  myLBand.checkCallbacks(); // Check if any LBand callbacks are waiting to be processed.

  // Check if the PPL has generated any RTCM. If it has, push it to the GNSS
  static uint8_t rtcmBuffer[PPL_MAX_RTCM_BUFFER];
  uint32_t rtcmLength;
  ePPL_ReturnStatus result = PPL_GetRTCMOutput( rtcmBuffer, PPL_MAX_RTCM_BUFFER, &rtcmLength);
  if (result == ePPL_Success)
  {
    if (rtcmLength > 0)
    {
      Serial.print(F("Received RTCM "));
      Serial.print(parseRTCM(rtcmBuffer, rtcmLength));
      Serial.println(F("from PPL. Pushing to the GNSS..."));
      myGNSS.pushRawData(rtcmBuffer, rtcmLength);
    }
  }
  else
  {
    Serial.print(F("PPL_GetRTCMOutput Result: "));
    Serial.println(PPLReturnStatusToStr(result));
  }

  // Print the message counts
  static unsigned long lastPrint = 0;
  if (millis() > (lastPrint + 5000))
  {
    lastPrint = millis();
    Serial.print(F("SPARTN: "));
    Serial.print(spartnReceived);
    Serial.print(F("\tSFRBX: "));
    Serial.print(sfrbxReceived);
    Serial.print(F("\tGPGGA: "));
    Serial.print(nmeaGGAreceived);
    Serial.print(F("\tGPZDA: "));
    Serial.println(nmeaZDAreceived);
  }
}