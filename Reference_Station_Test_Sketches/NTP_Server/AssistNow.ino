// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Assist Now

#include "secrets.h" // Update secrets.h with your AssistNow token string

const uint8_t numAssistNowServers = 2;
const char assistNowServers[numAssistNowServers][40] = { "online-live1.services.u-blox.com", "online-live2.services.u-blox.com" };
//const char assistNowServer[] = "https://online-live1.services.u-blox.com";
//const char assistNowServer[] = "https://online-live2.services.u-blox.com"; // Alternate server

const char assistNowGetQuery[] = "GetOnlineData.ashx?";
const char assistNowTokenPrefix[] = "token=";
const char assistNowTokenSuffix[] = ";";
const char assistNowGetGNSS[] = "gnss="; // GNSS can be: gps,qzss,glo,bds,gal
const uint8_t numAssistNowGnsss = 4;
const char assistNowGnsss[numAssistNowGnsss][5] = { "gps;", "glo;", "bds;", "gal;" };
const char assistNowGetDataType[] = "datatype="; // Data type can be: eph,alm,aux,pos
const uint8_t numAssistNowDataTypes = 3;
const char assistNowDataTypes[numAssistNowDataTypes][5] = { "eph;", "alm;", "aux;" };

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Connect to the Assist Now server. Push data to the GNSS
// To keep the packets small enough for the W5500, request: the eph and alm separately; each gnss separately

bool pushAssistNow()
{
  bool success = false;
  for (uint8_t s = 0; s < numAssistNowServers; s++) // If required, try both servers
  {
    for (uint8_t t = 0; t < numAssistNowDataTypes; t++) // For each data type
    {
      for (uint8_t g = 0; g < numAssistNowGnsss; g++) // For each GNSS
      {
        success |= connectAssistNow((const char *)assistNowServers[s], (const char *)assistNowDataTypes[t], (const char *)assistNowGnsss[g]);
      }
    }
    if (success)
      return true; // Exit now if the first server responded
  }
  return success;
}

bool connectAssistNow(const char *server, const char *dataType, const char *gnss)
{
  bool success = false;

  const int REQUEST_BUFFER_SIZE  = 256;
  char theRequest[REQUEST_BUFFER_SIZE]; // This will contain the HTTPS GET

  // Assemble the request
  // Note the slash before the getQuery
  snprintf(theRequest, REQUEST_BUFFER_SIZE, "GET /%s%s%s%s%s%s%s%s HTTP/1.1",
    assistNowGetQuery,
    assistNowTokenPrefix,
    myAssistNowToken,
    assistNowTokenSuffix,
    assistNowGetGNSS,
    gnss,
    assistNowGetDataType,
    dataType);

  Serial.print(F("Connecting to: "));
  Serial.println(server);
  Serial.print(F("Request: "));
  Serial.println(theRequest);
  
  if (assistNowClient.connect(server, 443))
  {
    Serial.println(F("Connected!"));

    assistNowClient.println(theRequest);
    assistNowClient.print("Host: ");
    assistNowClient.println(server);
    assistNowClient.println("Connection: close");
    assistNowClient.println();

    success = true;
    
    unsigned long startConnection = millis();
    bool keepGoing = true;

    const int maxMGAdata = 2048; // Allocate 2KB of storage for the UBX data (and HTTP header / footer (~230 bytes))
    uint8_t *buffer = new uint8_t[maxMGAdata];

    if (buffer == nullptr) // Check if memory was allocated successfully
    {
      Serial.println(F("Could not allocate memory to hold the MGA data!"));
      return false;
    }

    uint8_t *bufferPtr = buffer;

    while (keepGoing)
    {
      int len = assistNowClient.available(); // Check how much data is waiting

      if (len > 0)
      {
        if (maxMGAdata > ((bufferPtr - buffer) + len)) // Check there is enough room to hold the data
        {
          assistNowClient.read(bufferPtr, len); // Read the data
          bufferPtr += len;

          Serial.print(F("Adding "));
          Serial.print(len);
          Serial.println(F(" bytes to the buffer"));
        }
        else
        {
          Serial.println(F("Too much data! Discarding..."));
        }
      }
      
      if ((millis() > (startConnection + 5000)) // Wait for up to 5 seconds for all data to arrive
          || ((!assistNowClient.connected()) && (assistNowClient.available() == 0))) // Check we are still connected
      {
        assistNowClient.stop();
        keepGoing = false;
      }
    }

    if (bufferPtr > buffer)
    {
      // Push the data to the GNSS. pushAssistNowData will discard any non-UBX data so we don't need to strip the HTTP header first.
      // Wait for up to 100ms for each ACK to arrive. 100ms is a bit excessive... 7ms is nearer the mark.
      Serial.print(F("Pushing "));
      Serial.print(theGNSS.pushAssistNowData(buffer, (size_t)(bufferPtr - buffer), SFE_UBLOX_MGA_ASSIST_ACK_YES, 100));
      Serial.println(F(" bytes to the GNSS"));
    }

    delete[] buffer; // Free the memory
  }
  else
  {
    Serial.println(F("Could not connect to the server!"));
  }

  return success;
}
