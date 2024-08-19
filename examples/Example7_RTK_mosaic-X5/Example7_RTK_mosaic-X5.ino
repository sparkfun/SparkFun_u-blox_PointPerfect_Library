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

  * In this example, the L-Band, NMEA and RTCM output from the X5 is "Encapsulated" in SBF format
  *   - on a single COM port!

  Note: you need a valid u-blox Thingstream PointPerfect L-Band or L-Band + IP account to
        access the SPARTN decryption key. Copy and paste the current key into secrets.h
  
  Hardware:
  This example was written for:
  * SparkFun RTK mosaic-X5 (https://www.sparkfun.com/products/23748)

  This code is uploaded onto the ESP32 inside the RTK mosaic-X5, replacing its firmware.
  The original firmware can be restored using the binaries in:
  https://github.com/sparkfun/SparkFun_RTK_mosaic-X5/tree/main/Firmware/RTK_mosaic-X5_Firmware/build
  There are batch files to help erase the flash and restore the firmware in:
  https://github.com/sparkfun/SparkFun_RTK_mosaic-X5/tree/main/Firmware/RTK_mosaic-X5_Firmware

  Connect a good quality L1/L2/L5/L-Band antenna to the MOSAIC SMA connector.
  Connect the CONFIG ESP32 USB-C port to your computer.
  You should see a single CH340 COM ports appear in the Device Manager.
  In the Arduino IDE, select ESP32 Arduino \ ESP32 Wrover Module as the board.
  Set the Port to the CH340 COM port.
  Compile and upload this code.
  Open a Serial Monitor / Terminal Emulator at 115200 baud to see the output.
  If needed, install Septentrio RxTools - this includes the X5 USB driver.
    https://www.septentrio.com/en/support/software/rxtools
  Point a web browser at 192.168.3.1 to view the mosaic-X5's internal web page.

  Licence: please see LICENSE.md for more details. The PPL is reproduced by kind permission of u-blox

*/

#include "secrets.h" // <- Copy and paste the Current Key into secrets.h

#include "PPL_PublicInterface.h" // The PPL
#include "PPL_Version.h"

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS myLBand;              // Only needed for parseSPARTN

// US SPARTN 1.8 service is on 1556290000 Hz
// EU SPARTN 1.8 service is on 1545260000 Hz
const String lBandFreq = "1545260000";

// Keep count of the number of messages received. Print the count periodically
uint32_t spartnReceived = 0;
uint32_t nmeaReceived = 0;
uint32_t rtcmReceived = 0;

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Hardware specifics for the SparkFun RTK mosaic-X5

#include <driver/uart.h>      //Required for uart_set_rx_full_threshold() on cores <v2.0.5
const int serialTxPin = 2;
const int serialRxPin = 4;
HardwareSerial serialGNSS(1); // UART1: TX on 2, RX on 4. Connected to mosaic-X5 COM4

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

// Buffer SBF data
// This is a very simplistic message buffer. It performs no CRC-checking.

const uint8_t *bufferSBF(uint8_t dataByte, bool &valid, size_t &numDataBytes)
{
  const int maxLen = 2000;
  static uint8_t buffer[maxLen];

  static uint16_t dataLength = 0;
  static uint16_t remaining = 0;
  static uint8_t *ptr;

  static uint16_t expectedCRC = 0;
  static uint16_t sbfID = 0;

  valid = false;

  enum parseStates
  {
    Sync1,
    Sync2,
    CRC1,
    CRC2,
    ID1,
    ID2,
    LengthLSB,
    LengthMSB,
    readingBytes
  };
  static parseStates parseState = Sync1;

  switch (parseState)
  {
  case Sync1:
  {
    ptr = buffer;
    *ptr++ = dataByte;
    if (dataByte == '$')
      parseState = Sync2;
  }
  break;
  case Sync2:
  {
    *ptr++ = dataByte;
    if (dataByte == '@')
      parseState = CRC1;
    else
      parseState = Sync1;
  }
  break;
  case CRC1:
  {
    *ptr++ = dataByte;
    expectedCRC = dataByte;
    parseState = CRC2;
  }
  break;
  case CRC2:
  {
    *ptr++ = dataByte;
    expectedCRC |= ((uint16_t)dataByte) << 8;
    parseState = ID1;
  }
  break;
  case ID1:
  {
    *ptr++ = dataByte;
    sbfID = dataByte;
    parseState = ID2;
  }
  break;
  case ID2:
  {
    *ptr++ = dataByte;
    sbfID |= ((uint16_t)dataByte) << 8;
    parseState = LengthLSB;
  }
  break;
  case LengthLSB:
  {
    *ptr++ = dataByte;
    dataLength = dataByte;
    parseState = LengthMSB;
  }
  break;
  case LengthMSB:
  {
    *ptr++ = dataByte;
    dataLength |= ((uint16_t)dataByte) << 8;
    if (dataLength % 4 == 0) // Length is always a multiple of 4
    {
      remaining = dataLength - 8;
      parseState = readingBytes;
    }
    else
      parseState = Sync1;
  }
  break;
  case readingBytes:
  {
    *ptr++ = dataByte;
    remaining--;
    if (remaining == 0)
    {
      parseState = Sync1;
      numDataBytes = dataLength;
      valid = true;
    }
  }
  break;
  }

  if (ptr >= &buffer[maxLen - 1]) // Check if the buffer is full
    parseState = Sync1;   // Something has gone wrong...

  return (const uint8_t *)buffer;
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Analyze encapsulated SBF data

const uint8_t *checkEncapsulatedRTCMv3(const uint8_t *buffer, size_t &numDataBytes)
{
  return checkEncapsulatedOutput(2, buffer, numDataBytes);
}

const uint8_t *checkEncapsulatedNMEA(const uint8_t *buffer, size_t &numDataBytes)
{
  return checkEncapsulatedOutput(4, buffer, numDataBytes);
}

const uint8_t *checkEncapsulatedLBand(const uint8_t *buffer, size_t &numDataBytes)
{
  return checkEncapsulatedOutput(5, buffer, numDataBytes); // 5 is also ASCIIDisplay...
}

const uint8_t *checkEncapsulatedOutput(uint8_t mode, const uint8_t *buffer, size_t &numDataBytes)
{
  const uint8_t *ptr = buffer;
  ptr += 4; // Point to the ID
  uint16_t id = *ptr; // Extract the ID
  ptr++;
  id |= ((uint16_t)(*ptr)) << 8; // Extract the ID
  if ((id & 0x1FFF) != 4097) // Check if this is encapsulated output
    return nullptr;
  ptr += 9; // Point to the mode
  if (*ptr != mode) // Check the mode
    return nullptr;
  ptr += 2; // Point to the payload length
  uint16_t len = *ptr; // Extract the payload length
  ptr++;
  len |= ((uint16_t)(*ptr)) << 8; // Extract the payload length
  ptr += 3; // Point to the payload
  numDataBytes = (size_t)len; // Return the payload length
  return ptr; // Return a pointer to the payload
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

bool waitCR(unsigned long timeout = 100); // Header
bool waitCR(unsigned long timeout)
{
  unsigned long startTime = millis();
  while (millis() < (startTime + timeout))
  {
    if (serialGNSS.available())
    {
      uint8_t c = serialGNSS.read();
      if (c == '\r')
        return true;
    }
  }
  return false;
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
    return waitCR(wait); // wait for a carriage return
  }

  return false;
}
bool sendWithResponse(String message, const char *reply, unsigned long timeout = 1000, unsigned long wait = 100); // Header
bool sendWithResponse(String message, const char *reply, unsigned long timeout, unsigned long wait)
{
  return sendWithResponse(message.c_str(), reply, timeout, wait);
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bool configureMosaic(void)
{
  int retries = 0;
  const int retryLimit = 20; // Mosaic could still be starting up, so allow many retries

  Serial.println(F("Disabling mosaic-X5 message output on COM4"));

  // Disable output on COM4 while we configure the messages and L-Band
  while (!sendWithResponse("sdio,COM4,CMD,none\n\r", "DataInOut"))
  {
    if (retries == retryLimit)
      break;
    retries++;
    Serial.println(F("No response from mosaic-X5. Retrying - with escape sequence..."));
    sendWithResponse("SSSSSSSSSSSSSSSSSSSS\n\r", "COM4>"); // Send escape sequence
  }

  if (retries == retryLimit)
  {
    Serial.println(F("Retry limit reached. Mosaic communication FAIL!"));
    return false;
  }

  Serial.println(F("Configuring mosaic-X5 message output on COM4"));

  bool result = true;

  result &= sendWithResponse("snti,GP\n\r", "NMEATalkerID"); // Set Talker ID

  result &= sendWithResponse("sno,Stream1,COM4,GGA+ZDA,sec1\n\r", "NMEAOutput"); // Set NMEA Output (GGA + ZDA)

  result &= sendWithResponse("sr3i,RTCM1019+RTCM1020+RTCM1042+RTCM1046,1\n\r", "RTCMv3Interval"); // RTCMv3 message interval

  result &= sendWithResponse("sr3o,COM4,RTCM1019+RTCM1020+RTCM1042+RTCM1046\n\r", "RTCMv3Output"); // RTCMv3 output (RTCM1019 + RTCM1020 + RTCM1042 + RTCM1046)

  // Mark L5 as healthy
  result &= sendWithResponse("setHealthMask,Tracking,off\n\r", "HealthMask");
  result &= sendWithResponse("setHealthMask,PVT,off\n\r", "HealthMask");
  result &= sendWithResponse("setSignalTracking,+GPSL5\n\r", "SignalTracking");
  result &= sendWithResponse("setSignalUsage,+GPSL5,+GPSL5\n\r", "SignalUsage");

  Serial.println(F("Configuring mosaic-X5 L-Band output on COM4"));

  // If no SPARTN data is received, the L-Band may need a 'kick'. Turn L-Band off and back on again!
  result &= sendWithResponse(String("slsm,off\n\r"), "LBandSelectMode"); // Turn L-Band off

  // US SPARTN 1.8 service is on 1556290000 Hz
  // EU SPARTN 1.8 service is on 1545260000 Hz
  result &= sendWithResponse(String("slbb,User1," + lBandFreq + ",baud2400,PPerfect,EU,Enabled\n\r"), "LBandBeams"); // Set Freq, baud rate
  result &= sendWithResponse("slcs,5555,6959\n\r", "LBandCustomServiceID"); // 21845 = 0x5555; 26969 = 0x6959
  result &= sendWithResponse("slsm,manual,Inmarsat,User1,\n\r", "LBandSelectMode"); // Set L-Band demodulator to manual

  // Finally, enable message output on COM4 using Encapsulation
  result &= sendWithResponse("sdio,COM4,CMD+RTCMv3,RTCMv3+NMEA+LBandBeam1+Encapsulate\n\r", "DataInOut");

  if (result)
    Serial.println(F("mosaic-X5 configured. Here we go..."));
  else
    Serial.println(F("mosaic-X5 configuration failed. Please investigate..."));

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
  // Tell it to output the following on COM4 :
  //   GPGGA
  //   GPZDA
  //   RTCM1019
  //   RTCM1020
  //   RTCM1042
  //   RTCM1046
  // Also configure L-Band and direct the SBF-encapsulated raw stream to COM4

  serialGNSS.begin(115200, SERIAL_8N1, serialRxPin, serialTxPin);

  flushRX(100);

  configureMosaic();

  //myLBand.enableDebugging(); // Uncomment this line to enable GNSS Library debug messages - so we can see what parseSPARTN is doing
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  bool newSBF = false;
  bool newNMEA = false;
  bool newRTCMv3 = false;
  bool newSPARTN = false;

  // Check serialGNSS for SBF-encapsulated data
  while (serialGNSS.available())
  {
    size_t numDataBytes;
    // Check for a complete SBF message
    const uint8_t *buffer = bufferSBF(serialGNSS.read(), newSBF, numDataBytes);

    if (newSBF)
    {
      size_t payloadLength;
      const uint8_t *payloadPtr = checkEncapsulatedNMEA(buffer, payloadLength);
      if (payloadPtr != nullptr)
      {
        nmeaReceived++;
        newNMEA = true;

        ePPL_ReturnStatus result = PPL_SendRcvrData(payloadPtr, payloadLength); // Send it to the PPL
        if (result != ePPL_Success)
        {
          Serial.print(F("PPL_SendRcvrData Result: "));
          Serial.println(PPLReturnStatusToStr(result));
        }

        break; // Exit the while loop - we sent a valid NMEA message
      }

      payloadPtr = checkEncapsulatedRTCMv3(buffer, payloadLength);
      if (payloadPtr != nullptr)
      {
        rtcmReceived++;
        newRTCMv3 = true;

        ePPL_ReturnStatus result = PPL_SendRcvrData(payloadPtr, payloadLength); // Send it to the PPL
        if (result != ePPL_Success)
        {
          Serial.print(F("PPL_SendRcvrData Result: "));
          Serial.println(PPLReturnStatusToStr(result));
        }

        break; // Exit the while loop - we sent a valid RTCM message
      }

      payloadPtr = checkEncapsulatedLBand(buffer, payloadLength);
      if (payloadPtr != nullptr)
      {
        // Check for a complete SPARTN message
        for (size_t i = 0; i < payloadLength; i++)
        {
          bool validSPARTN;
          uint16_t len;
          uint8_t *spartn = myLBand.parseSPARTN(*payloadPtr++, validSPARTN, len);

          if (validSPARTN)
          {
            // Serial.print(F("Valid SPARTN data parsed. Pushing "));
            // Serial.print(len);
            // Serial.println(F(" bytes to the PPL..."));

            spartnReceived++;
            newSPARTN = true;

            ePPL_ReturnStatus result = PPL_SendAuxSpartn(spartn, len); // Push the SPARTN data to the PPL
            if (result != ePPL_Success)
            {
              Serial.print(F("PPL_SendAuxSpartn Result: "));
              Serial.println(PPLReturnStatusToStr(result));
            }

            // Do not exit the while loop - we need to process all the L-Band data
          }
        }
      }
    }
  }

  if (newNMEA || newRTCMv3 || newSPARTN) // Decide when to call PPL_GetRTCMOutput
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
    if (configureMosaic()) // Restart mosaic
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
    Serial.print(F("  RTCMv3: "));
    Serial.print(rtcmReceived);
    Serial.print(F("  NMEA: "));
    Serial.println(nmeaReceived);
  }
}