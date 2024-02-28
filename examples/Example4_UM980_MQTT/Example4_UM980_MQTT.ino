/*
  u-blox PointPerfect Library (PPL) example using IP based corrections from PointPerfect
  ======================================================================================

  This example shows how to:
    Get UM980 connected to PointPerfect IP corrections via MQTT
    The current encryption key is read from secrets.h and sent into the PPL
    Configure the UM980 to output NMEA GPGGA messages fed into the PPL. GPZDA is also beneficial.
    Configure a UM980 to output RTCM (1019, 1020, 1042, 1046) fed into the PPL
    The PPL will output RTCM data
    The RTCM data is pushed back to the UM980 allowing it to achieve a FIXED RTK solution

  Hardware:
  This example was written for:
    SparkFun Torch - The ESP32 UART1 is connected to UM980 UART 3
    Note: The Torch is limited to 51200bps upload speed
    Note: PSRAM is required to support WiFi and PPL simultaneously

  Select ESP32 Dev Module as the board
  Set PSRAM to Enabled

  Press and hold power button for 4 seconds to enable code upload
  For code upload, select USB-Enhanced-SERIAL-B CH342 (not A)
  Use upload baud rate of 512000 (921600 fails)

  Licence: please see LICENSE.md for more details. The PPL is reproduced by kind permission of u-blox

*/

#include "secrets.h"

// UM980 Connections
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int pin_UART1_TX = 27;
int pin_UART1_RX = 26;
int pin_GNSS_DR_Reset = 22; // Push low to reset GNSS/DR.

#include <SparkFun_Unicore_GNSS_Arduino_Library.h> //http://librarymanager/All#SparkFun_Unicore_GNSS

UM980 myGNSS;

HardwareSerial serialGNSS(1); //Use UART1 on the ESP32
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// PointPerfect MQTT for IP based corrections
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <ArduinoMqttClient.h> // Click here to get the library: http://librarymanager/All#ArduinoMqttClient

WiFiClientSecure mqttSecureClient = WiFiClientSecure();
MqttClient mqttClient(mqttSecureClient);
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// PointPerfect Library (PPL)
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "PPL_PublicInterface.h" // The PPL
#include "PPL_Version.h"

// Keep count of the number of messages received. Print the count periodically
uint32_t spartnReceived = 0;
uint32_t um980Received = 0;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Globals
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastReceived_ms = 0; //Timestamp of last MQTT messsage received from broker
int maxTimeBeforeHangup_ms = 10000; //If we fail to get more MQTT message after 10s, then disconnect from broker

bool newRTCMNMEA = false;
bool newSPARTN = false;
//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void setup()
{
  Serial.begin(115200);
  delay(250);

  psramBegin();

  reportHeap();

  // Begin the PointPerfect Library and give it the current key
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  Serial.print("PointPerfect Library Corrections version: ");
  Serial.println(PPL_SDK_VERSION);

  ePPL_ReturnStatus result = PPL_Initialize(PPL_CFG_ENABLE_IP_CHANNEL);
  Serial.print("PPL_Initialize: ");
  Serial.println(PPLReturnStatusToStr(result));

  //Note: the current encryption key is received via MQTT and pushed to the PPL by 

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  reportHeap();

  // Connect to WiFi
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  wifiBegin();
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

  reportHeap();

  //Configure the GNSS to output RTCM and NMEA needed for PPL
  um980Begin();

  reportHeap();

  // Connect to the MQTT broker and IP correction topic
  pointPerfectClientBegin();

  reportHeap();
}

void loop()
{
  //Check MQTT channel for new SPARTN messages
  //Callback feeds SPARTN to PPL
  if (mqttSecureClient.connected() == true)
    mqttClient.poll();

  // Check serialGNSS for GGA/ZDA/RTCM. Send it to the PPL
  getGnssSerial();

  //Close MQTT client if we don't have new data for 10s
  if (mqttSecureClient.connected() == true)
  {
    if (millis() - lastReceived_ms > maxTimeBeforeHangup_ms)
    {
      Serial.println("MQTT client timeout. Disconnecting...");
      mqttStop();
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

        // Wait for the UM980 to update PVT data
        // This blocks for a second but allows us to leverage the Unicore library parser
        // The delay could be decreased by increasing the report rate of BESTNAVB.
        unsigned long startTime = millis();
        while (millis() - startTime < 1000)
        {
          myGNSS.update(); //Regularly call to parse any new data on the UART
          delay(10);
        }

        printUm980Stats();
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
    Serial.print("Message counts:  SPARTN: ");
    Serial.print(spartnReceived);
    Serial.print("  um980: ");
    Serial.println(um980Received);
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
