// Check serialGNSS for GGA/ZDA/RTCM. 
// Send GGA/ZDA/RTCM to the PPL.
// For BESTNAV messages, print UM980 
void getGnssSerial()
{
  while (serialGNSS.available())
  {
    size_t numDataBytes;

    // Check for a complete RTCM / NMEA message
    const uint8_t *buffer = bufferRTCMNMEA(serialGNSS.read(), newRTCMNMEA, numDataBytes);

    if (newRTCMNMEA)
    {
       Serial.print("Valid RTCM / NMEA data parsed. Pushing ");
       Serial.print(numDataBytes);
       Serial.println(" bytes to the PPL...");

      um980Received++;

      ePPL_ReturnStatus result = PPL_SendRcvrData(buffer, numDataBytes); // Send it to the PPL
      if (result != ePPL_Success)
      {
        Serial.print("PPL_SendRcvrData Result: ");
        Serial.println(PPLReturnStatusToStr(result));
      }

      break; // Exit the while loop - we sent a valid NMEA/RTCM message
    }
  }  
}
