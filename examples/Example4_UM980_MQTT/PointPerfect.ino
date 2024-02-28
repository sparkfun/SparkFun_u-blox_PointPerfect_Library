//Connect to STARTN MQTT broker, receive RTCM, and push to ZED module over I2C
void pointPerfectClientBegin()
{
  Serial.println("Subscribing to Broker. Press 's' to stop broker.");

  // Connect to AWS IoT
  mqttSecureClient.setCACert(AWS_CERT_CA);
  mqttSecureClient.setCertificate(AWS_CERT_CRT);
  mqttSecureClient.setPrivateKey(AWS_CERT_PRIVATE);

  mqttClient.setId(MQTT_CLIENT_ID);
  mqttClient.setKeepAliveInterval(60 * 1000);
  mqttClient.setConnectionTimeout( 5 * 1000);

  if (!mqttClient.connect(AWS_IOT_ENDPOINT, AWS_IOT_PORT)) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());
    return;
  } else {
    Serial.print("You're connected to the PointPerfect MQTT broker: ");
    Serial.println(AWS_IOT_ENDPOINT);

    // Subscribe to MQTT and register a callback
    mqttClient.onMessage(mqttMessageHandler);

    mqttClient.subscribe(MQTT_TOPIC_KEY);
    mqttClient.subscribe(MQTT_TOPIC_SPARTN);

    lastReceived_ms = millis();
  }
}

void mqttStop()
{
  Serial.println("Stopping mqtt client");
  if (mqttClient.connected() == true)
    mqttClient.stop();
}

//Call back for when a new MQTT arrives
//Feed any SPARTN message to PPL
//Record any key messages
void mqttMessageHandler(int messageSize)
{
  const uint16_t mqttLimit = 10 * 1024;
  static uint8_t mqttData[mqttLimit]; // Allocate memory to hold the MQTT data
  if (mqttData == NULL)
  {
    Serial.println("Memory allocation for mqttData failed!");
    return;
  }

  Serial.print("Received MQTT data from the ");
  Serial.print(mqttClient.messageTopic());
  Serial.println(" topic");

  bool topicKey = false;
  bool topicSPARTN = false;
  if (strcmp(mqttClient.messageTopic().c_str(), MQTT_TOPIC_KEY) == 0)
    topicKey = true;
  else if (strcmp(mqttClient.messageTopic().c_str(), MQTT_TOPIC_SPARTN) == 0)
    topicSPARTN = true;

  while (mqttClient.available())
  {
    uint16_t mqttCount = 0;

    while (mqttClient.available())
    {
      char ch = mqttClient.read();
      mqttData[mqttCount++] = ch;

      if (mqttCount == mqttLimit)
        break;
    }

    if (mqttCount > 0)
    {
      if (topicKey)
      {
        // This is a cheat... We should be using a JSON library to read the keys.
        // But life is short... Here we check the nodes using sscanf.
        char *keyPtr = strstr((const char *)mqttData, "\"dynamickeys\":{");
        if (keyPtr != nullptr)
        {
          keyPtr += strlen("\"dynamickeys\":{");
          keyPtr = strstr(keyPtr, "\"current\":{");
          if (keyPtr != nullptr)
          {
            keyPtr += strlen("\"current\":{");
            keyPtr = strstr(keyPtr, "\"value\":\"");
            if (keyPtr != nullptr)
            {
              keyPtr += strlen("\"value\":\"");
              char theKey[33];
              if (sscanf(keyPtr, "%32s\"", theKey) == 1)
              {
                ePPL_ReturnStatus result = PPL_SendDynamicKey(theKey, strlen(theKey));
                Serial.print("PPL_SendDynamicKey: ");
                Serial.println(PPLReturnStatusToStr(result));
              }
            }
          }
        }
        else
        {
          Serial.print("Unexpected JSON: ");
          Serial.println((const char *)mqttData);
        }
      }
      else if (topicSPARTN)
      {
        //Push SPARTN data to PPL
        ePPL_ReturnStatus result = PPL_SendSpartn(mqttData, mqttCount); // Push the SPARTN data to the PPL
        if (result != ePPL_Success)
        {
          Serial.print("ERROR processRXMPMP PPL_SendAuxSpartn: ");
          Serial.println(PPLReturnStatusToStr(result));
        }

        spartnReceived += mqttCount; //Global print
        
        lastReceived_ms = millis();
        newSPARTN = true;
      }
      else
      {
        Serial.println("Unexpected topic!");
      }
    }
  }
}
