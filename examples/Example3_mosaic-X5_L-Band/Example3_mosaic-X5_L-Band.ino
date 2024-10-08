/*
  u-blox PointPerfect Library (PPL) example
  =========================================

  This example shows how to:
  * Configure a mosaic-X5 to receive PointPerfect L-Band correction data
  * The raw L-Band data stream is bit-aligned and parsed for SPARTN messages
  * The SPARTN data is CRC-checked and then sent into the PPL
  * The PPL will output RTCM data
  * The RTCM data is pushed back to the mosaic-X5 allowing it to achieve a FIXED RTK solution
  * NMEA GPGGA messages are sent from the X5 into the PPL. GPZDA is also beneficial
  * The current encryption key is read from secrets.h and sent into the PPL
  * The PPL also needs satellite ephemeris data
    * This can be RXM-SFRBX, or RTCM (1019, 1020, 1042, 1046). This example uses RTCM from the X5

  Note: you need a valid u-blox Thingstream PointPerfect L-Band or L-Band + IP account to
        access the SPARTN decryption key. Copy and paste the current key into secrets.h
  
  Hardware:
  This example was written for:
  * SparkFun Thing Plus - ESP32 WROOM (USB-C) (https://www.sparkfun.com/products/20168)
  * Septentrio mosaic-X5 Breakout (https://www.sparkfun.com/products/23088)

  Connect the Thing Plus C to the mosaic-X5 Breakout using jumper wires:
    ESP32 serialTxPin (17) to mosaic-X5 COM2 RX (This carries the RTCM data from the PPL)
    ESP32 serialRxPin (16) to mosaic-X5 COM2 TX (This carries the GPGGA, GPZDA and RTCM1019/1020/1042/1046)
    ESP32 lbandRxPin (14) to mosaic-X5 COM3 TX (This carries the raw L-Band data stream)
    GND to GND
  Connect the mosaic-X5 USB-C to your computer to:
    Provide power
    Let you view the X5's web page at 192.168.3.1
  Connect the Thing Plus C USB-C to your computer to:
    Provide power
    Open the Serial Monitor at 115200 baud to view the SPARTN and RTCM information
  Connect a good quality L1/L2/L5/L-Band antenna to the mosaic-X5

  Licence: please see LICENSE.md for more details. The PPL is reproduced by kind permission of u-blox

*/

#include "secrets.h" // <- Copy and paste the Current Key into secrets.h

#include <driver/uart.h>      //Required for uart_set_rx_full_threshold() on cores <v2.0.5
const int serialTxPin = 17;
const int serialRxPin = 16;
HardwareSerial serialGNSS(2); // UART2: TX on 17, RX on 16
const int lbandRxPin = 14;
const int lbandTxPin = 15;
HardwareSerial lbandSerial(1);  // UART1: TX on 15, RX on 14

#include "PPL_PublicInterface.h" // The PPL
#include "PPL_Version.h"

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myLBand;              // Only needed for parseSPARTN

// US SPARTN 1.8 service is on 1556290000 Hz
// EU SPARTN 1.8 service is on 1545260000 Hz
const String lBandFreq = "1556290000";

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
bool sendWithResponse(String message, const char *reply, unsigned long timeout = 1000, unsigned long wait = 100); // Header
bool sendWithResponse(String message, const char *reply, unsigned long timeout, unsigned long wait)
{
  return sendWithResponse(message.c_str(), reply, timeout, wait);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool configureLBand(bool restart = false); // Header
bool configureLBand(bool restart)
{
  if (restart) // If no SPARTN data is received, the L-Band may need a 'kick'. Turn L-Band off and back on again!
  {
    while (!sendWithResponse(String("slsm,off\n\r"), "LBandSelectMode")) // Turn L-Band off
    {
      Serial.println(F("configureLBand: No response from mosaic-X5 (1). Retrying - with escape sequence..."));
      sendWithResponse("SSSSSSSSSSSSSSSSSSSS\n\r", "COM2>"); // Send escape sequence
    }

    delay(1000);
  }

  // US SPARTN 1.8 service is on 1556290000 Hz
  // EU SPARTN 1.8 service is on 1545260000 Hz
  while (!sendWithResponse(String("slbb,User1," + lBandFreq + ",baud2400,PPerfect,EU,Enabled\n\r"), "LBandBeams")) // Set Freq, baud rate
  {
    Serial.println(F("configureLBand: No response from mosaic-X5 (2). Retrying - with escape sequence..."));
    sendWithResponse("SSSSSSSSSSSSSSSSSSSS\n\r", "COM2>"); // Send escape sequence
  }

  bool result = true;

  result &= sendWithResponse("slcs,5555,6959\n\r", "LBandCustomServiceID"); // 21845 = 0x5555; 26969 = 0x6959

  result &= sendWithResponse("slsm,manual,Inmarsat,User1,\n\r", "LBandSelectMode"); // Set L-Band demodulator to manual

  result &= sendWithResponse("sdio,COM3,none,LBandBeam1\n\r", "DataInOut"); // Output LBandBeam1 on COM3

  return result;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  delay(1000);

  Serial.begin(115200);
  Serial.println(F("PointPerfect Library Corrections"));
  Serial.println(PPL_SDK_VERSION);

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
  // Also configure L-Band and direct the raw stream to COM3

  serialGNSS.begin(115200, SERIAL_8N1, serialRxPin, serialTxPin); // UART2 on pins 16/17.

  lbandSerial.begin(115200, SERIAL_8N1, lbandRxPin, lbandTxPin); // UART1 on pins 14/15.

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
  sendWithResponse("shm,Tracking,off\n\r", "HealthMask");
  sendWithResponse("shm,PVT,off\n\r", "HealthMask");
  sendWithResponse("snt,+GPSL5\n\r", "SignalTracking");
  sendWithResponse("snu,+GPSL5,+GPSL5\n\r", "SignalUsage");

  Serial.println(F("Configuring mosaic-X5 L-Band output on COM3"));

  if (configureLBand())
    Serial.println(F("mosaic-X5 configured. Here we go..."));
  else
    Serial.println(F("L-Band configuration failed. Please investigate..."));

  //myLBand.enableDebugging(); // Uncomment this line to enable GNSS Library debug messages - so we can see what parseSPARTN is doing
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  bool newRTCMNMEA = false;

  // Check serialGNSS for GGA/ZDA/RTCM. Send it to the PPL
  while (serialGNSS.available())
  {
    size_t numDataBytes;
    // Check for a complete RTCM / NMEA message
    const uint8_t *buffer = bufferRTCMNMEA(serialGNSS.read(), newRTCMNMEA, numDataBytes);

    if (newRTCMNMEA)
    {
      // Serial.print(F("Valid RTCM / NMEA data parsed. Pushing "));
      // Serial.print(numDataBytes);
      // Serial.println(F(" bytes to the PPL..."));

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

  bool newSPARTN = false;

  // Check for new L-Band data. Send it to the PPL
  while (lbandSerial.available())
  {
    uint16_t len;
    // Check for a complete SPARTN message
    uint8_t *spartn = myLBand.parseSPARTN(lbandSerial.read(), newSPARTN, len);

    if (newSPARTN)
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

      break; // Exit the while loop - we sent a valid SPARTN message
    }
  }

  if (newRTCMNMEA || newSPARTN) // Decide when to call PPL_GetRTCMOutput
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

  // Check if we should restart L-Band (turn it off and back on again!)
  static unsigned long lastSPARTNSeen = millis(); // Record when we last received valid SPARTN, so we can restart L-Band if needed
  if (newSPARTN)
    lastSPARTNSeen = millis();
  if (millis() > (lastSPARTNSeen + 60000)) // Restart L-Band if no data is received for 60s
  {
    if (configureLBand(true)) // Restart L-Band
      Serial.println(F("mosaic-X5 L-Band restarted. Here we go again..."));
    else
      Serial.println(F("L-Band restart failed. Please investigate..."));

    lastSPARTNSeen = millis();
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