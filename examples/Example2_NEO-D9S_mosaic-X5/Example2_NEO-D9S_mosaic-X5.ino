/*
  u-blox PointPerfect Library (PPL) example
  =========================================

  This example shows how to:
  * Configure a NEO-D9S to receive L-Band correction data
  * The PMP messages output by the NEO-D9S are parsed and assembled into SPARTN messages
  * The SPARTN data is CRC-checked and then sent into the PPL
  * The PPL will output RTCM data
  * The RTCM data is pushed to a mosaic-X5 allowing it to achieve a FIXED RTK solution
  * NMEA GPGGA messages are sent from the X5 into the PPL. GPZDA is also beneficial
  * The current encryption key is read from secrets.h and sent into the PPL
  * The PPL also needs satellite ephemeris data
    * This can be RXM-SFRBX, or RTCM (1019, 1020, 1042, 1046). This example uses RTCM from the X5

  Note: you need a valid u-blox Thingstream PointPerfect L-Band or L-Band + IP account to
        access the SPARTN decryption key. Copy and paste the current key into secrets.h
  
  Hardware:
  This example was written for:
  * SparkFun Thing Plus - ESP32 WROOM (USB-C) (https://www.sparkfun.com/products/20168)
  * NEO-D9S Breakout (https://www.sparkfun.com/products/19390)
  * Septentrio mosaic-X5 Breakout (https://www.sparkfun.com/products/23088)
  * Optional antenna splitter (https://www.sparkfun.com/products/21223)

  Connect the Thing Plus C to the NEO-D9S using a Qwiic cable
  Use jumper wires to link:
    ESP32 TX (17) to mosaic-X5 COM2 RX
    ESP32 RX (16) to mosaic-X5 COM2 TX
    GND to GND
  X5 Power:
    The easiest way to power the X5 breakout is to connect USB-C.
    That will also allow you to view the X5's internal web page and monitor the RTK solution.
    Another option would be to connect the Thing Plus VUSB to the X5 breakout VIN.
  Connect a good quality L1/L2/L-Band antenna to the NEO-D9S and mosaic-X5

  Licence: please see LICENSE.md for more details. The PPL is reproduced by kind permission of u-blox

*/

#include "secrets.h" // <- Copy and paste the Current Key into secrets.h

#include <driver/uart.h>      //Required for uart_set_rx_full_threshold() on cores <v2.0.5
const int serialTxPin = 17;
const int serialRxPin = 16;
HardwareSerial serialGNSS(2); // UART2: TX on 17, RX on 16

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myLBand;              // NEO-D9S

#include "PPL_PublicInterface.h" // The PPL
#include "PPL_Version.h"

const uint32_t myLBandFreq = 1556290000; // Uncomment this line to use the US SPARTN 1.8 service
//const uint32_t myLBandFreq = 1545260000; // Uncomment this line to use the EU SPARTN 1.8 service

#define OK(ok) (ok ? F("  ->  OK") : F("  ->  ERROR!")) // Convert uint8_t into OK/ERROR

// Keep count of the number of messages received. Print the count periodically
uint32_t spartnReceived = 0;
uint32_t mosaicReceived = 0;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

const char *PPLReturnStatusToStr(ePPL_ReturnStatus status)
{
  switch (status)
  {
  case ePPL_Success:
    return ("operation was successful");
  case ePPL_IncorrectLibUsage:
    return ("incorrect usage of the library");
  case ePPL_LibInitFailed:
    return ("library failed to initialize");
  case ePPL_LibExpired:
    return ("library has expired");
  case ePPL_LBandChannelNotEnabled:
    return ("L-Band channel has not been initialized");
  case ePPL_IPChannelNotEnabled:
    return ("IP channel has not been initialized");
  case ePPL_AuxChannelNotEnabled:
    return ("Aux channel has not been initialized");
  case ePPL_NoDynamicKey:
    return ("stream is encrypted and no valid dynamic key found");
  case ePPL_FailedDynKeyLibPush:
    return ("key push failed");
  case ePPL_InvalidDynKey:
    return ("invalid key format");
  case ePPL_IncorrectDynKey:
    return ("key does not match the key used for data encryption");
  case ePPL_RcvPosNotAvailable:
    return ("GGA position not available");
  case ePPL_LeapSecsNotAvailable:
    return ("leap seconds not available");
  case ePPL_AreaDefNotAvailableForPos:
    return ("position is outside of Point Perfect coverage");
  case ePPL_TimeNotResolved:
    return ("time is not resolved");
  case ePPL_PotentialBuffOverflow:
    return ("potential buffer overflow");
  case ePPL_UnknownState:
  default:
    return ("unknown state");
  }
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Parse RTCM data
// This is a very simplistic parser. It only extracts the Message Numbers.
// It does not CRC-check the data.

String &parseRTCM(uint8_t *dataBytes, size_t numDataBytes)
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
      dataLength = (((uint16_t) * (dataBytes + i)) & 0x03) << 8;
      parseState = length2;
      break;
    case length2:
      dataLength |= *(dataBytes + i);
      dataLength += 3; // Add CRC
      parseState = number1;
      break;
    case number1:
      messageNumber = ((uint16_t) * (dataBytes + i)) << 4;
      parseState = number2;
      dataLength--;
      break;
    case number2:
      messageNumber |= (*(dataBytes + i)) >> 4;
      parseState = readingBytes;
      dataLength--;
      desc += String(messageNumber); // Add the Message Number to the description String
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

// Buffer RTCM and NMEA data
// This is a very simplistic message buffer. It performs no CRC-checking.
// It is simply a way of passing whole messages to the PPL instead of individual bytes.

const uint8_t *bufferRTCMNMEA(uint8_t dataByte, bool &valid, size_t &numDataBytes)
{
  const int maxRTCMNMEA = 1100;
  static uint8_t buffer[maxRTCMNMEA];

  static uint16_t dataLength = 0;
  static uint16_t remaining = 0;
  static uint8_t *ptr;

  valid = false;

  enum parseStates
  {
    waitingForD3Dollar,
    rtcmLength1,
    rtcmLength2,
    rtcmReadingBytes,
    nmeaReadingBytes
  };
  static parseStates parseState = waitingForD3Dollar;

  switch (parseState)
  {
  case waitingForD3Dollar:
  {
    ptr = buffer;
    *ptr++ = dataByte;
    if (dataByte == 0xD3)
    {
      parseState = rtcmLength1;
    }
    else if (dataByte == '$')
    {
      parseState = nmeaReadingBytes;
      dataLength = 1;
    }
  }
  break;
  case rtcmLength1:
  {
    *ptr++ = dataByte;
    dataLength = (((uint16_t)dataByte) & 0x03) << 8;
    parseState = rtcmLength2;
  }
  break;
  case rtcmLength2:
  {
    *ptr++ = dataByte;
    dataLength |= dataByte;
    dataLength += 3; // Add CRC
    remaining = dataLength;
    dataLength += 3; // Add D3 and length bytes
    parseState = rtcmReadingBytes;
  }
  break;
  case rtcmReadingBytes:
  {
    *ptr++ = dataByte;
    remaining--;
    if (remaining == 0)
    {
      parseState = waitingForD3Dollar;
      numDataBytes = dataLength;
      valid = true;
    }
  }
  break;
  case nmeaReadingBytes:
  {
    *ptr++ = dataByte;
    dataLength++;
    if (dataByte == '\n')
    {
      parseState = waitingForD3Dollar;
      numDataBytes = dataLength;
      valid = true;
    }
  }
  break;
  }

  if (ptr >= &buffer[maxRTCMNMEA - 1]) // Check if the buffer is full
    parseState = waitingForD3Dollar;   // Something has gone wrong...

  return (const uint8_t *)buffer;
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
  // Extract the raw message payload length
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

  // Parse the SPARTN data stream contained in the userData
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

void flushRX(unsigned long timeout = 0); // Header
void flushRX(unsigned long timeout)
{
  if (timeout > 0)
  {
    unsigned long startTime = millis();
    while (millis() < (startTime + timeout))
      if (serialGNSS.available())
        serialGNSS.read();
  }
  else
  {
    while (serialGNSS.available())
      serialGNSS.read();
  }
}

bool sendWithResponse(const char *message, const char *reply, unsigned long timeout = 1000, unsigned long wait = 100); // Header
bool sendWithResponse(const char *message, const char *reply, unsigned long timeout, unsigned long wait)
{
  if (strlen(reply) == 0) // Reply can't be zero-length
    return false;

  if (strlen(message) > 0)
    serialGNSS.write(message, strlen(message)); // Send the message

  unsigned long startTime = millis();
  size_t replySeen = 0;
  bool keepGoing = true;

  while ((keepGoing) && (replySeen < strlen(reply))) // While not timed out and reply not seen
  {
    if (serialGNSS.available()) // If a char is available
    {
      uint8_t chr = serialGNSS.read(); // Read it
      if (chr == *(reply + replySeen)) // Is it a char from reply?
        replySeen++;
      else
        replySeen = 0; // Reset replySeen on an unexpected char
    }

    // If the reply has started to arrive at the timeout, allow extra time
    if (millis() > (startTime + timeout)) // Have we timed out?
      if (replySeen == 0)                 // If replySeen is zero, don't keepGoing
        keepGoing = false;

    if (millis() > (startTime + timeout + wait)) // Have we really timed out?
      keepGoing = false;                         // Don't keepGoing
  }

  if (replySeen == strlen(reply)) // If the reply was seen
  {
    flushRX(wait); // wait and flush
    return true;
  }

  return false;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);

  Serial.begin(115200);
  Serial.println(F("PointPerfect Library Corrections"));
  Serial.println(PPL_SDK_VERSION);

  Wire.begin(); // Start I2C

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin and configure the NEO-D9S L-Band receiver

  // myLBand.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  while (myLBand.begin(Wire, 0x43) == false) // Connect to the u-blox NEO-D9S using Wire port. The D9S default I2C address is 0x43 (not 0x42)
  {
    Serial.println(F("u-blox NEO-D9S not detected at default I2C address. Please check wiring."));
    delay(1000);
  }
  Serial.println(F("u-blox NEO-D9S connected"));

  myLBand.newCfgValset();                                            // Create a new Configuration Interface message - this defaults to VAL_LAYER_RAM_BBR (change in RAM and BBR)
  myLBand.addCfgValset(UBLOX_CFG_PMP_CENTER_FREQUENCY, myLBandFreq); // Default 1539812500 Hz
  myLBand.addCfgValset(UBLOX_CFG_PMP_SEARCH_WINDOW, 2200);           // Default 2200 Hz
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_SERVICE_ID, 0);             // Default 1
  myLBand.addCfgValset(UBLOX_CFG_PMP_SERVICE_ID, 21845);             // Default 50821
  myLBand.addCfgValset(UBLOX_CFG_PMP_DATA_RATE, 2400);               // Default 2400 bps
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_DESCRAMBLER, 1);            // Default 1
  myLBand.addCfgValset(UBLOX_CFG_PMP_DESCRAMBLER_INIT, 26969);       // Default 23560
  myLBand.addCfgValset(UBLOX_CFG_PMP_USE_PRESCRAMBLING, 0);          // Default 0
  myLBand.addCfgValset(UBLOX_CFG_PMP_UNIQUE_WORD, 16238547128276412563ull);
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_I2C, 1);   // Ensure UBX-RXM-PMP is enabled on the I2C port
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART1, 0); // No cheating! Disable UBX-RXM-PMP output on UART1
  myLBand.addCfgValset(UBLOX_CFG_UART2OUTPROT_UBX, 1);         // Enable UBX output on UART2
  myLBand.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_PMP_UART2, 0); // No cheating! Disable UBX-RXM-PMP output on UART2
  myLBand.addCfgValset(UBLOX_CFG_UART1_BAUDRATE, 38400);       // match baudrate with ZED default
  myLBand.addCfgValset(UBLOX_CFG_UART2_BAUDRATE, 38400);       // match baudrate with ZED default
  bool ok = myLBand.sendCfgValset();                           // Apply the settings

  Serial.print(F("L-Band: configuration "));
  Serial.println(OK(ok));

  myLBand.softwareResetGNSSOnly(); // Do a restart

  myLBand.setRXMPMPmessageCallbackPtr(&processRXMPMP); // Call processRXMPMP when new PMP data arrives. Push it to the GNSS

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Begin the PointPerfect Library and give it the current key

  ePPL_ReturnStatus result = PPL_Initialize(PPL_CFG_ENABLE_AUX_CHANNEL);
  Serial.print(F("PPL_Initialize: "));
  Serial.println(PPLReturnStatusToStr(result));

  result = PPL_SendDynamicKey(currentDynamicKey, currentKeyLength);
  Serial.print(F("PPL_SendDynamicKey: "));
  Serial.println(PPLReturnStatusToStr(result));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Connect to the mosaic-X5
  // Tell it to output the following on COM2:
  //   GPGGA
  //   GPZDA
  //   RTCM1019
  //   RTCM1020
  //   RTCM1042
  //   RTCM1046

  serialGNSS.begin(115200, SERIAL_8N1, serialRxPin, serialTxPin); // UART2 on pins 16/17.

  flushRX(100);

  Serial.println(F("Configuring mosaic-X5 message output on COM2"));

  while (!sendWithResponse("sdio,COM2,,RTCMv3+NMEA\n\r", "DataInOut")) // Set Data In/Out - enable RTCMv3 output
  {
    Serial.println(F("No response from mosaic-X5. Retrying - with escape sequence..."));
    sendWithResponse("SSSSSSSSSSSSSSSSSSSS\n\r", "COM2>"); // Send escape sequence
  }

  sendWithResponse("snti,GP\n\r", "NMEATalkerID"); // Set Talker ID

  sendWithResponse("sno,Stream1,COM2,GGA+ZDA,sec1\n\r", "NMEAOutput"); // Set NMEA Output

  sendWithResponse("sr3i,RTCM1019+RTCM1020+RTCM1042+RTCM1046,1\n\r", "RTCMv3Interval"); // RTCMv3 message interval

  sendWithResponse("sr3o,COM2,RTCM1019+RTCM1020+RTCM1042+RTCM1046\n\r", "RTCMv3Output"); // RTCMv3 output

  // Mark L5 as healthy
  sendWithResponse("setHealthMask,Tracking,off\n\r", "HealthMask");
  sendWithResponse("setHealthMask,PVT,off\n\r", "HealthMask");
  sendWithResponse("setSignalTracking,+GPSL5\n\r", "SignalTracking");
  sendWithResponse("setSignalUsage,+GPSL5,+GPSL5\n\r", "SignalUsage");

  Serial.println(F("mosaic-X5 configured. Here we go..."));
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  myLBand.checkUblox();     // Check for the arrival of new PMP data and process it.
  myLBand.checkCallbacks(); // Check if any LBand callbacks are waiting to be processed.

  bool newRTCMNMEA = false;

  // Check serialGNSS for GGA/ZDA/RTCM. Send it to the PPL
  while (serialGNSS.available())
  {
    size_t numDataBytes;

    // Check for a whole RTCM / NMEA message
    const uint8_t *buffer = bufferRTCMNMEA(serialGNSS.read(), newRTCMNMEA, numDataBytes);
    
    if (newRTCMNMEA)
    {
      mosaicReceived++;

      ePPL_ReturnStatus result = PPL_SendRcvrData(buffer, numDataBytes); // Send it to the PPL
      if (result != ePPL_Success)
      {
        Serial.print(F("PPL_SendRcvrData Result: "));
        Serial.println(PPLReturnStatusToStr(result));
      }

      break; // Exit the while loop - we sent a valid NMEA/RTCM message
    }
  }

  if (newRTCMNMEA) // Uncomment this line to only call PPL_GetRTCMOutput when new RTCM / NMEA data is available
  {
    // Check if the PPL has generated any RTCM. If it has, push it to the GNSS
    static uint8_t rtcmBuffer[PPL_MAX_RTCM_BUFFER];
    uint32_t rtcmLength;
    ePPL_ReturnStatus result = PPL_GetRTCMOutput(rtcmBuffer, PPL_MAX_RTCM_BUFFER, &rtcmLength);
    if (result == ePPL_Success)
    {
      if (rtcmLength > 0)
      {
        Serial.print(F("Received RTCM "));
        Serial.print(parseRTCM(rtcmBuffer, rtcmLength));
        Serial.println(F("from PPL. Pushing to the GNSS..."));
        serialGNSS.write(rtcmBuffer, rtcmLength);
      }
    }
    else
    {
      Serial.print(F("PPL_GetRTCMOutput Result: "));
      Serial.println(PPLReturnStatusToStr(result));
    }
  }

  // Print the message counts
  static unsigned long lastPrint = 0;
  if (millis() > (lastPrint + 5000))
  {
    lastPrint = millis();
    Serial.print(F("Message counts:  SPARTN: "));
    Serial.print(spartnReceived);
    Serial.print(F("  mosaic-X5: "));
    Serial.println(mosaicReceived);
  }
}