/*
  SparkFun GNSS Reference Station Test Sketch

  License: MIT. Please see LICENSE.md for more details

  This example shows how to obtain AssistNow Online data from u-blox Thingstream over WiFi
  and push it to the u-blox module.

  You will need to have a token to be able to access Thingstream.

  Update secrets.h with your:
  - WiFi credentials
  - AssistNow token string

  Uncomment the "#define USE_MGA_ACKs" below to test the more robust method of using the
  UBX_MGA_ACK_DATA0 acknowledgements to confirm that each MGA message has been accepted.

  Pin Allocations:
  D0  : Boot + Boot Button
  D1  : Serial TX (CH340 RX)
  D2  : SDIO DAT0 - via 74HC4066 switch
  D3  : Serial RX (CH340 TX)
  D4  : SDIO DAT1
  D5  : GNSS Chip Select
  D12 : SDIO DAT2 - via 74HC4066 switch
  D13 : SDIO DAT3
  D14 : SDIO CLK
  D15 : SDIO CMD - via 74HC4066 switch
  D16 : Serial1 RXD
  D17 : Serial1 TXD
  D18 : SPI SCK
  D19 : SPI POCI
  D21 : I2C SDA
  D22 : I2C SCL
  D23 : SPI PICO
  D25 : GNSS Time Pulse
  D26 : STAT LED
  D27 : Ethernet Chip Select
  D32 : PWREN
  D33 : Ethernet Interrupt
  A34 : GNSS TX RDY
  A35 : Board Detect (1.1V)
*/

#define USE_MGA_ACKs // Uncomment this line to use the UBX_MGA_ACK_DATA0 acknowledgements

#define USE_SERVER_ASSISTANCE // Uncomment this line to include the position in the AssistNow data request

#include <SPI.h> // Needed for SPI to GNSS
#include <HTTPClient.h>
#include "secrets.h"

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SUPER theGNSS;

#include "time.h"
const char* ntpServer = "pool.ntp.org"; // The Network Time Protocol Server

const int GNSS_CS = 5; // Chip select for the GNSS SPI interface
const int ETHERNET_CS = 27; // Chip select for the WizNet 5500
const int PWREN = 32; // 3V3_SW and SDIO Enable

const char assistNowServer[] = "https://online-live1.services.u-blox.com";
//const char assistNowServer[] = "https://online-live2.services.u-blox.com"; // Alternate server

const char getQuery[] = "GetOnlineData.ashx?";
const char tokenPrefix[] = "token=";
const char tokenSuffix[] = ";";
const char getGNSS[] = "gnss=gps,glo,bds,gal;"; // GNSS can be: gps,qzss,glo,bds,gal
const char getDataType[] = "datatype=eph,alm,aux;"; // Data type can be: eph,alm,aux,pos

#ifdef USE_SERVER_ASSISTANCE
// Use 55 degrees (*10^7) north, 1 degree (*10^7) west, 100m (10000cm) altitude, 100km (10000000cm) accuracy. Replace these with your position.
// The units for lat and lon are degrees * 1e-7 (WGS84)
// The units for alt (WGS84) and posAcc (stddev) are cm.

const char useLatitude[] = "lat=55.0;"; // Use an approximate latitude of 55 degrees north. Replace this with your latitude.
const char useLongitude[] = "lon=-1.0;"; // Use an approximate longitude of 1 degree west. Replace this with your longitude.
const char useAlt[] = "alt=100;"; // Use an approximate latitude of 100m above WGS84. Replace this with your altitude.
const char usePosAcc[] = "pacc=100000;"; // Use a position accuracy of 100000m (100km)

const int32_t myLat = 550000000; // Replace this with your latitude.
const int32_t myLon = -10000000; // Replace this with your longitude.
const int32_t myAlt = 10000; // Replace this with your altitude.
const uint32_t posAcc = 10000000;
#endif

void setup()
{
  pinMode(GNSS_CS, OUTPUT);
  digitalWrite(GNSS_CS, HIGH);
  pinMode(ETHERNET_CS, OUTPUT);
  digitalWrite(ETHERNET_CS, HIGH);
  pinMode(PWREN, OUTPUT);
  digitalWrite(PWREN, HIGH);

  delay(1000);

  Serial.begin(115200);
  Serial.println("SparkFun Reference Station - Test Sketch");

  while (Serial.available()) Serial.read(); // Empty the serial buffer
  Serial.println(F("Press any key to begin..."));
  while (!Serial.available()); // Wait for a keypress

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Connect to the GNSS.

  // Uncomment the next line to enable the 'major' debug messages on Serial so you can see what AssistNow data is being sent
  //theGNSS.enableDebugging(Serial, true);

  while (!theGNSS.begin(SPI, GNSS_CS)) // Start the GNSS on SPI. Default to 4MHz
  {
    Serial.println("GNSS not detected. Retrying...");
  }

  theGNSS.setSPIOutput(COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)

  // Query module info

  if (theGNSS.getModuleInfo())
  {
      Serial.print(F("The GNSS module is: "));
      Serial.println(theGNSS.getModuleName());    

      Serial.print(F("The firmware type is: "));
      Serial.println(theGNSS.getFirmwareType());    

      Serial.print(F("The firmware version is: "));
      Serial.print(theGNSS.getFirmwareVersionHigh());
      Serial.print(F("."));
      Serial.println(theGNSS.getFirmwareVersionLow());
      
      Serial.print(F("The protocol version is: "));
      Serial.print(theGNSS.getProtocolVersionHigh());
      Serial.print(F("."));
      Serial.println(theGNSS.getProtocolVersionLow());
  }  

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Connect to WiFi.

  Serial.print(F("Connecting to local WiFi"));

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();

  Serial.println(F("WiFi connected!"));

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Set the RTC using network time. (Code taken from the SimpleTime example.)

  // Request the time from the NTP server and use it to set the ESP32's RTC.
  configTime(0, 0, ntpServer); // Set the GMT and daylight offsets to zero. We need UTC, not local time.

  struct tm timeinfo;
  if(!getLocalTime(&timeinfo))
  {
    Serial.println("Failed to obtain time");
  }
  else
  {
    Serial.println(&timeinfo, "Time is: %A, %B %d %Y %H:%M:%S");
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Use HTTP GET to receive the AssistNow_Online data

  const int URL_BUFFER_SIZE  = 256;
  char theURL[URL_BUFFER_SIZE]; // This will contain the HTTP URL
  int payloadSize = 0; // This will be updated with the length of the data we get from the server
  String payload; // This will store the data we get from the server

  // Assemble the URL
  // Note the slash after the first %s (assistNowServer)
#ifdef USE_SERVER_ASSISTANCE
    snprintf(theURL, URL_BUFFER_SIZE, "%s/%s%s%s%s%s%s%s%s%s%s",
      assistNowServer,
      getQuery,
      tokenPrefix,
      myAssistNowToken,
      tokenSuffix,
      getGNSS,
      getDataType,
      useLatitude,
      useLongitude,
      useAlt,
      usePosAcc);
#else
    snprintf(theURL, URL_BUFFER_SIZE, "%s/%s%s%s%s%s%s",
      assistNowServer,
      getQuery,
      tokenPrefix,
      myAssistNowToken,
      tokenSuffix,
      getGNSS,
      getDataType);
#endif

  Serial.print(F("HTTP URL is: "));
  Serial.println(theURL);

  HTTPClient http;

  http.begin(theURL);

  int httpCode = http.GET(); // HTTP GET

  // httpCode will be negative on error
  if(httpCode > 0)
  {
    // HTTP header has been sent and Server response header has been handled
    Serial.printf("[HTTP] GET... code: %d\r\n", httpCode);
  
    // If the GET was successful, read the data
    if(httpCode == HTTP_CODE_OK) // Check for code 200
    {
      payloadSize = http.getSize();
      Serial.printf("Server returned %d bytes\r\n", payloadSize);
      
      payload = http.getString(); // Get the payload

      // Pretty-print the payload as HEX
      /*
      int i;
      for(i = 0; i < payloadSize; i++)
      {
        if (payload[i] < 0x10) // Print leading zero
          Serial.print("0");
        Serial.print(payload[i], HEX);
        Serial.print(" ");
        if ((i % 16) == 15)
          Serial.println();
      }
      if ((i % 16) != 15)
        Serial.println();
      */
    }
  }
  else
  {
    Serial.printf("[HTTP] GET... failed, error: %s\r\n", http.errorToString(httpCode).c_str());
  }
  
  http.end();  
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Push the RTC time to the module

  // Uncomment the next line to enable the 'major' debug messages on Serial so you can see what AssistNow data is being sent
  //theGNSS.enableDebugging(Serial, true);

  if(getLocalTime(&timeinfo))
  {
    // setUTCTimeAssistance uses a default time accuracy of 2 seconds which should be OK here.
    // Have a look at the library source code for more details.
    theGNSS.setUTCTimeAssistance(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
  }
  else
  {
    Serial.println("Failed to obtain time. This will not work well. The GNSS needs accurate time to start up quickly.");
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // If desired - push initial position assistance to the module

#ifndef USE_SERVER_ASSISTANCE

  theGNSS.setPositionAssistanceLLH(myLat, myLon, myAlt, posAcc, SFE_UBLOX_MGA_ASSIST_ACK_YES, 100);

  // We could use setPositionAssistanceXYZ instead if needed.

#endif
  
  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Push the AssistNow data to the module

  if (payloadSize > 0)
  {  

#ifndef USE_MGA_ACKs

    // ***** Don't use the UBX_MGA_ACK_DATA0 messages *****

    // Tell the module not to return UBX_MGA_ACK_DATA0 messages when we push the AssistNow data
    theGNSS.setAckAiding(0);

    // Push all the AssistNow data. Don't use UBX_MGA_ACK_DATA0's. Use the default delay of 7ms between messages.
    // The 'true' parameter tells pushAssistNowData not to push any time data from the payload.
    theGNSS.pushAssistNowData(true, payload, (size_t)payloadSize);

#else

    // ***** Use the UBX_MGA_ACK_DATA0 messages *****

    // Tell the module to return UBX_MGA_ACK_DATA0 messages when we push the AssistNow data
    theGNSS.setAckAiding(1);

    // Push all the AssistNow data.
    // We have called setAckAiding(1) to instruct the module to return MGA-ACK messages.
    // So, we could set the pushAssistNowData mgaAck parameter to SFE_UBLOX_MGA_ASSIST_ACK_YES.
    // But, just for giggles, let's use SFE_UBLOX_MGA_ASSIST_ACK_ENQUIRE just to confirm that the
    // MGA-ACK messages are actually enabled.
    // Wait for up to 100ms for each ACK to arrive! 100ms is a bit excessive... 7ms is nearer the mark.
    // The 'true' parameter tells pushAssistNowData not to push any time data from the payload.
    theGNSS.pushAssistNowData(true, payload, (size_t)payloadSize, SFE_UBLOX_MGA_ASSIST_ACK_YES, 100);

#endif

  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Disconnect the WiFi as it's no longer needed

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  Serial.println(F("WiFi disconnected"));

  
  Serial.println(F("Here we go!"));
  Serial.println();
}

//=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void loop()
{
  // Print the UBX-NAV-PVT data so we can see how quickly the fixType goes to 3D
  
  long latitude = theGNSS.getLatitude();
  Serial.print(F("Lat: "));
  Serial.print(latitude);

  long longitude = theGNSS.getLongitude();
  Serial.print(F(" Long: "));
  Serial.print(longitude);
  Serial.print(F(" (degrees * 10^-7)"));

  long altitude = theGNSS.getAltitudeMSL();
  Serial.print(F(" Alt: "));
  Serial.print(altitude);
  Serial.print(F(" (mm)"));

  byte SIV = theGNSS.getSIV();
  Serial.print(F(" SIV: "));
  Serial.print(SIV);

  byte fixType = theGNSS.getFixType();
  Serial.print(F(" Fix: "));
  if(fixType == 0) Serial.print(F("No fix"));
  else if(fixType == 1) Serial.print(F("Dead reckoning"));
  else if(fixType == 2) Serial.print(F("2D"));
  else if(fixType == 3) Serial.print(F("3D"));
  else if(fixType == 4) Serial.print(F("GNSS + Dead reckoning"));
  else if(fixType == 5) Serial.print(F("Time only"));

  Serial.println();
}
