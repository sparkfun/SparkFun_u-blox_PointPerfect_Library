void um980Begin()
{
  pinMode(pin_GNSS_DR_Reset, OUTPUT);
  digitalWrite(pin_GNSS_DR_Reset, HIGH); // Tell UM980 and DR to boot

  //We must start the serial port before using it in the library
  serialGNSS.begin(115200, SERIAL_8N1, pin_UART1_RX, pin_UART1_TX);

  Serial.println("Configuring UM980 to output on its UART3 to ESP32 UART1");

  myGNSS.enableDebugging(); // Print all debug to Serial

  if (myGNSS.begin(serialGNSS) == false) //Give the serial port over to the library
  {
    Serial.println("UM980 failed to respond. Check ports and baud rates. Freezing...");
    while (true);
  }
  Serial.println("UM980 detected!");

  // Clear saved configurations, satellite ephemerides, position information, and reset baud rate to 115200bps.
  // resetUM980();

  bool response = true;

  float outputRate = 1; //0.5 = 2 reports per second.

  char comPort[] = "COM3"; //UM980 UART3 is connected to ESP32 UART1 on the Torch

  // Configure UM980 to output:
  //   GPGGA
  //   GPZDA
  //   RTCM1019
  //   RTCM1020
  //   RTCM1042
  //   RTCM1046
  response &= myGNSS.setRTCMPortMessage("RTCM1019", comPort, outputRate);
  response &= myGNSS.setRTCMPortMessage("RTCM1020", comPort, outputRate);
  response &= myGNSS.setRTCMPortMessage("RTCM1042", comPort, outputRate);
  response &= myGNSS.setRTCMPortMessage("RTCM1046", comPort, outputRate);

  response &= myGNSS.setNMEAPortMessage("GPGGA", comPort, outputRate);
  response &= myGNSS.setNMEAPortMessage("GPZDA", comPort, outputRate);

  response &= myGNSS.saveConfiguration(); //Save the current configuration into non-volatile memory (NVM)

  //If any one command fails, it will force response to false
  if (response == false)
  {
    Serial.println("UM980 failed to configure. Freezing...");
    while (true);
  }

  Serial.println("UM980 configured. Here we go...");
}

void printUm980Stats()
{
  Serial.print("Lat/Long/Alt: ");
  Serial.print(myGNSS.getLatitude(), 11);
  Serial.print("/");
  Serial.print(myGNSS.getLongitude(), 11);
  Serial.print("/");
  Serial.println(myGNSS.getAltitude(), 4);

  Serial.print("Deviation of Lat/Long/Alt (m): ");
  Serial.print(myGNSS.getLatitudeDeviation(), 4);
  Serial.print("/");
  Serial.print(myGNSS.getLongitudeDeviation(), 4);
  Serial.print("/");
  Serial.println(myGNSS.getAltitudeDeviation(), 4);

  Serial.print("ECEF X/Y/Z (m): ");
  Serial.print(myGNSS.getEcefX(), 4);
  Serial.print("/");
  Serial.print(myGNSS.getEcefY(), 4);
  Serial.print("/");
  Serial.println(myGNSS.getEcefZ(), 4);

  Serial.print("Deviation of ECEF X/Y/Z (m): ");
  Serial.print(myGNSS.getEcefXDeviation(), 4);
  Serial.print("/");
  Serial.print(myGNSS.getEcefYDeviation(), 4);
  Serial.print("/");
  Serial.println(myGNSS.getEcefZDeviation(), 4);

  Serial.print("Date (yyyy/mm/dd): ");
  Serial.print(myGNSS.getYear());
  Serial.print("/");
  if (myGNSS.getMonth() < 10)
    Serial.print("0");
  Serial.print(myGNSS.getMonth());
  Serial.print("/");
  if (myGNSS.getDay() < 10)
    Serial.print("0");
  Serial.print(myGNSS.getDay());

  int timeStatus = myGNSS.getTimeStatus();
  Serial.print(" Time status: ");
  Serial.print(timeStatus);
  Serial.print(" - ");
  if (timeStatus == 0) Serial.print("Valid");
  else if (timeStatus == 3) Serial.print("Invalid!");
  else Serial.print("Unknown");
  Serial.println();

  Serial.print("Satellites tracked: ");
  Serial.println(myGNSS.getSatellitesTracked());
  Serial.print("Satellites used: ");
  Serial.println(myGNSS.getSatellitesUsed());

  int positionType = myGNSS.getPositionType();
  Serial.print("Position Type: ");
  Serial.print(positionType);
  Serial.print(" - ");
  if (positionType == 0) Serial.print("No solution");
  else if (positionType == 8) Serial.print("Velocity computed using instantaneous Doppler");
  else if (positionType == 16) Serial.print("Single point positioning");
  else if (positionType == 17) Serial.print("Pseudorange differential solution");
  else if (positionType == 18) Serial.print("SBAS positioning");
  else if (positionType == 32) Serial.print("L1 float solution");
  else if (positionType == 33) Serial.print("Ionosphere-free float solution");
  else if (positionType == 34) Serial.print("Narrow-lane float solution");
  else if (positionType == 48) Serial.print("L1 fixed solution");
  else if (positionType == 49) Serial.print("Wide-lane fixed solution");
  else if (positionType == 50) Serial.print("Narrow-lane fixed solution");
  else Serial.print("Unknown");
  Serial.println();

  int solutionStatus = myGNSS.getSolutionStatus();
  Serial.print("Solution Status: ");
  Serial.print(solutionStatus);
  Serial.print(" - ");
  if (solutionStatus == 0) Serial.print("Solution computed");
  else if (solutionStatus == 1) Serial.print("Insufficient observation");
  else if (solutionStatus == 2) Serial.print("No convergence, invalid solution");
  else if (solutionStatus == 4) Serial.print("Covariance matrix trace exceeds maximum");
  else Serial.print("Unknown");
  Serial.println();

  int rtkSolution = myGNSS.getRTKSolution();
  Serial.print("RTK Solution: ");
  Serial.print(rtkSolution);
  Serial.print(" - ");
  if (rtkSolution == 0) Serial.print("Unchecked");
  else if (rtkSolution == 1) Serial.print("Checked");
  else Serial.print("Unknown");
  Serial.println();

  int pseudorangeCorrection = myGNSS.getPseudorangeCorrection();
  Serial.print("Pseudorange Correction: ");
  Serial.print(pseudorangeCorrection);
  Serial.print(" - ");
  if (pseudorangeCorrection == 0) Serial.print("Unknown");
  else if (pseudorangeCorrection == 0x001) Serial.print("Klobuchar broadcast ephemeris correction");
  else if (pseudorangeCorrection == 0x010) Serial.print("SBAS ionospheric grid correction");
  else if (pseudorangeCorrection == 0x011) Serial.print("Multi-frequency correction");
  else if (pseudorangeCorrection == 0x100) Serial.print("Pseudorange differential correction");
  else Serial.print("Unknown");
  Serial.println();

  Serial.println();
}
