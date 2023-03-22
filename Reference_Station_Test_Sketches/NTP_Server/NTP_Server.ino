/*
  SparkFun GNSS Reference Station Test Sketch

  License: MIT. Please see LICENSE.md for more details

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
  A36 : microSD card detect
  A39 : Unused analog pin - used to generate random values for SSL
*/

#include <SPI.h> // Needed for SPI to GNSS

#include <Ethernet.h> // http://librarymanager/All#Arduino_Ethernet
//#include <EthernetLarge.h> // https://github.com/OPEnSLab-OSU/EthernetLarge
//#include <EthernetUdp.h>
//#include "utility/w5100.h"

#include <SSLClient.h> //http://librarymanager/All#SSLClient
#include "sslCerts.h"

#include <Esp.h>
#include <ESP32Time.h> //http://librarymanager/All#ESP32Time by FBiego v2.0.0
#include "time.h"
#include <sys/time.h>
ESP32Time rtc;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Pin definitions

const int GNSS_CS = 5; // Chip select for the GNSS SPI interface
const int ETHERNET_CS = 27; // Chip select for the WizNet 5500
const int PWREN = 32; // 3V3_SW and SDIO Enable
const int ETHERNET_INT = 33;
const int STAT_LED = 26;
const int GNSS_INT = 25;
const int ANALOG_RANDOM = 39;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// GNSS object

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SUPER theGNSS;

// Uncomment the following line to include the position in the AssistNow data request
// This also helps keep the Assist Now data requests under 2KB
#define USE_SERVER_ASSISTANCE

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// NTP port server (UDP)

#define NTP_PORT 123

class derivedEthernetUDP : public EthernetUDP
{
  public:
    uint8_t getSockIndex() { return sockindex; } // sockindex is protected in EthernetUDP. A derived class can access it.
};
derivedEthernetUDP timeServer;

#define fixedIPAddress
#ifdef fixedIPAddress
IPAddress myIPAddress = { 192, 168, 0, 123 }; // Use this IP Address - not DHCP

#define useDNS
#ifdef useDNS
IPAddress myDNS = { 194, 168, 4, 100 };

#define useGateway
#ifdef useGateway
IPAddress myGateway = { 192, 168, 0, 1 };

#define useSubnetMask
#ifdef useSubnetMask
IPAddress mySubnetMask = { 255, 255, 255, 0 };

#endif

#endif

#endif

#endif

// TODO: Add gateway and subnet options

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Assist Now (HTTPS)

EthernetClient baseAssistNowClient;
SSLClient assistNowClient(baseAssistNowClient, TAs, (size_t)TAs_NUM, ANALOG_RANDOM);

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Globals

volatile bool syncRTC = true;                   // Flag to indicate if the RTC should be sync'd
volatile struct timeval gnssTv;                 // This will hold the GNSS time
volatile struct timeval gnssSyncTv;             // This will hold the time the GNSS was last sync'd
volatile struct timeval ethernetTv;             // This will hold the time the Ethernet packet arrived
volatile bool timeFullyResolved = false;        // Record if GNSS time is fully resolved
volatile unsigned long lastRTCsync = 0;         // Record the millis when the RTC is sync'd
volatile unsigned long gnssUTCreceived = 0;     // Record the millis when GNSS time is received
const unsigned long resyncAfterMillis = 60000;  // Re-sync every minute - for testing only. Once per hour should be more than adequate
const unsigned long gnssStaleAfter = 999;       // Treat GNSS time as stale after this many millis
volatile uint32_t tAcc;                         // Record the GNSS time accuracy
const uint32_t tAccLimit = 5000;                // Only sync if the time accuracy estimate is better than 15us (5000 nanoseconds)
volatile uint8_t sockIndex;                     // The W5500 socket index for the timeServer - so we can enable and read the correct interrupt

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Time Pulse ISR
// Triggered by the rising edge of the time pulse signal, indicates the top-of-second.
// Set the ESP32 RTC to UTC

void tpISR()
{
  unsigned long millisNow = millis();
  if (syncRTC) // Only sync if syncRTC is true
  {
    if ((lastRTCsync + resyncAfterMillis) < millisNow) // Only sync if it is more than resyncAfterMillis since the last sync
    {
      if (millisNow < (gnssUTCreceived + gnssStaleAfter)) // Only sync if the GNSS time is not stale
      {
        if (timeFullyResolved) // Only sync if GNSS time is fully resolved
        {
          if (tAcc < tAccLimit) // Only sync if the tAcc is better than tAccLimit
          {
            rtc.setTime(gnssTv.tv_sec, gnssTv.tv_usec); // Sync the RTC
            lastRTCsync = millis(); // Update lastSync
            gnssSyncTv.tv_sec = gnssTv.tv_sec; // Store the timeval of the sync
            gnssSyncTv.tv_usec = gnssTv.tv_usec;
          }
        }
      }
    }
  }

  digitalWrite(STAT_LED, !digitalRead(STAT_LED)); // Flash the LED to indicate TP interrupts are being received
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup()
{
  pinMode(GNSS_CS, OUTPUT);
  digitalWrite(GNSS_CS, HIGH);
  pinMode(ETHERNET_CS, OUTPUT);
  digitalWrite(ETHERNET_CS, HIGH);
  pinMode(PWREN, OUTPUT);
  digitalWrite(PWREN, HIGH);

  pinMode(STAT_LED, OUTPUT);
  pinMode(GNSS_INT, INPUT);
  pinMode(ETHERNET_INT, INPUT_PULLUP);

  delay(1000);

  Serial.begin(115200);
  Serial.println("SparkFun Reference Station - Test Sketch");

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  Ethernet.init(ETHERNET_CS); // Configure the W5500 CS

  // Read the ESP32 MAC address. Define the Ethernet MAC address
  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  mac[5] += 3; // ESP32 MAC is base + 0, 1, 2, 3 for WiFi, WiFi AP, BT, Ethernet

  // Start the Ethernet connection:
#ifdef fixedIPAddress
  Serial.println("Initialize Ethernet with fixed IP Address:");
#if defined (useSubnetMask)
  Ethernet.begin(mac, myIPAddress, myDNS, myGateway, mySubnetMask);
#elif defined (useGateway)
  Ethernet.begin(mac, myIPAddress, myDNS, myGateway);
#elif defined (useDNS)
  Ethernet.begin(mac, myIPAddress, myDNS);
#else
  Ethernet.begin(mac, myIPAddress);
#endif
#else
  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0)
  {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware)
    {
      Serial.println("Ethernet not found.  Sorry, can't run without hardware. :(");
    }
    else if (Ethernet.linkStatus() == LinkOFF)
    {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  }
#endif
  // Print your local IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  // Print the MAC address
  Serial.print("My MAC address: ");
  for (byte thisByte = 0; thisByte < 6; thisByte++)
  {
    // print the value of each byte of the IP address:
    if (mac[thisByte] < 0x10)
      Serial.print("0");
    Serial.print(mac[thisByte], HEX);
    if (thisByte < 5) {
      Serial.print(":");
    }
  }
  Serial.println();

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Begin communication with the GNSS

  if (!configureGNSS())
  {
    Serial.println("Failed to configure the GNSS! Freezing...");
    while (1)
      ;
  }

  //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // If desired - push initial position assistance to the module

#ifndef USE_SERVER_ASSISTANCE

  theGNSS.setPositionAssistanceLLH(myLat, myLon, myAlt, posAcc, SFE_UBLOX_MGA_ASSIST_ACK_YES, 100);

  // We could use setPositionAssistanceXYZ instead if needed.

#endif

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Attach the TP interrupt to the ISR
  
  attachInterrupt(GNSS_INT, tpISR, RISING);
  
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Connect to the Assist Now server

  delay(2000); // Allow time for the Ethernet connection to initialize

  // Uncomment the next line to enable the 'major' debug messages on Serial so you can see what AssistNow data is being sent
  //theGNSS.enableDebugging(Serial, true);

  pushAssistNow();

  theGNSS.disableDebugging();

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Start the timeServer

  timeServer.begin(NTP_PORT); // Begin listening
  sockIndex = timeServer.getSockIndex(); // Get the socket index
  Serial.print(F("timeServer socket index: "));
  Serial.println(sockIndex);

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Configure Ethernet interrupt

  w5500ClearSocketInterrupts(); // Clear all interrupts

  w5500EnableSocketInterrupt(sockIndex); // Enable the RECV interrupt for the desired socket index
  
  // Attach the W5500 interrupt to the ISR
  attachInterrupt(ETHERNET_INT, ethernetISR, FALLING);

}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void loop()
{
  // Check for the arrival of fresh PVT data
  theGNSS.checkUblox();
  theGNSS.checkCallbacks();

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Check for new NTP requests - if the time has been sync'd
  
  static char ntpDiag[512]; // Char array to hold diagnostic messages
  bool processed = processOneRequest((lastRTCsync > 0), (const timeval *)&ethernetTv, (const timeval *)&gnssSyncTv, ntpDiag, sizeof(ntpDiag));

  if (processed)
  {
    //w5500ClearSocketInterrupt(sockIndex); // Not sure if it is best to clear the interrupt(s) here - or in the ISR?
    Serial.print("NTP request processed: ");
    Serial.println(ntpDiag);
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Maintain the ethernet connection
  
  switch (Ethernet.maintain()) {
    case 1:
      //renewed fail
      Serial.println("Ethernet.maintain: Error: renewed fail");
      break;

    case 2:
      //renewed success
      Serial.println("Ethernet.maintain: Renewed success");
      //print your local IP address:
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;

    case 3:
      //rebind fail
      Serial.println("Ethernet.maintain: Error: rebind fail");
      break;

    case 4:
      //rebind success
      Serial.println("Ethernet.maintain: Rebind success");
      //print your local IP address:
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;

    default:
      //nothing happened
      break;
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Diagnostics

  // Print helpful messages
  if (lastRTCsync == 0)
  {
    static unsigned long lastPrint = 0;
    if ((lastPrint + 5000) < millis())
    {
      Serial.println("Waiting for GNSS time sync...\r\n");
      lastPrint = millis();
    }
  }

  // Check if the RTC has been sync'd
  static unsigned long previousRTCsync = 0;
  if (previousRTCsync != lastRTCsync)
  {
    struct tm timeinfo = rtc.getTimeStruct();
    Serial.println(&timeinfo, "RTC re-sync'd at: %A, %B %d %Y %H:%M:%S");
    Serial.print("tAcc is ");
    Serial.print(tAcc);
    Serial.println("ns\r\n");
    previousRTCsync = lastRTCsync;
  }

  // Print the time of each W5500 interrupt
  static struct timeval lastW5500interrupt;
  if ((lastW5500interrupt.tv_sec != ethernetTv.tv_sec) || (lastW5500interrupt.tv_usec != ethernetTv.tv_usec))
  {
    lastW5500interrupt.tv_sec = ethernetTv.tv_sec;
    lastW5500interrupt.tv_usec = ethernetTv.tv_usec;
    time_t nowtime;
    struct tm *nowtm;
    char tmbuf[64], buf[96];
    nowtime = lastW5500interrupt.tv_sec;
    nowtm = localtime(&nowtime);
    strftime(tmbuf, sizeof (tmbuf), "W5500 interrupt at: %A, %B %d %Y %H:%M:%S", nowtm);
    snprintf(buf, sizeof (buf), "%s.%06ld\r\n", tmbuf, lastW5500interrupt.tv_usec);
    Serial.println(buf);
  }
}
