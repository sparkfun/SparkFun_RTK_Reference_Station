/*
  SparkFun GNSS Reference Station Test Sketch

  NTP Server - based on Mooneer Salem's arduino-ntpd

  https://github.com/tmiw/arduino-ntpd

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
*/

#include <SPI.h> // Needed for SPI to GNSS

#include <Ethernet.h>
#include <EthernetUdp.h>
#include "utility/w5100.h"
#include "NTPServer.h"
#include "NTPPacket.h"

#include <Esp.h>
#include <time.h>

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SUPER theGNSS;

const int GNSS_CS = 5; // Chip select for the GNSS SPI interface
const int ETHERNET_CS = 27; // Chip select for the WizNet 5500
const int PWREN = 32; // 3V3_SW and SDIO Enable
const int ETHERNET_INT = 33;
const int STAT_LED = 26;
const int GNSS_INT = 25;

// Enter a MAC address for your controller below.
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};

#define NTP_PORT 123

NtpServer timeServer;

volatile bool syncRTC = true;                   // Flag to indicate if the RTC should be sync'd
volatile struct timeval gnssTv;                 // This will hold the GNSS time
volatile struct timeval ethernetTv;             // This will hold the time the Ethernet packet arrived
const timezone tz_utc = { 0, 0 };               // Set the timezone to UTC
volatile bool timeFullyResolved = false;        // Record if GNSS time is fully resolved
volatile unsigned long lastRTCsync = 0;         // Record the millis when the RTC is sync'd
volatile unsigned long gnssUTCreceived = 0;     // Record the millis when GNSS time is received
const unsigned long resyncAfterMillis = 60000;  // Re-sync every minute
const unsigned long gnssStaleAfter = 999;       // Treat GNSS time as stale after this many millis
volatile uint32_t tAcc;                         // Record the GNSS time accuracy
const uint32_t tAccLimit = 10000;               // Only sync if the time accuracy estimate is better than 10000 nanoseconds

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
            settimeofday((const timeval *)&gnssTv, &tz_utc); // Sync the RTC
            lastRTCsync = millis(); // Update lastSync
          }
        }
      }
    }
  }
}

// Triggered by the falling edge of the W5500 interrupt signal - indicates the arrival of a packet
// Record the time the packet arrived
void ethernetISR()
{
  gettimeofday((timeval *)&ethernetTv, (timezone *)&tz_utc);
}

// Callback: newPVTdata will be called when new NAV PVT data arrives.
// It is sent ahead of the top-of-second and contains the UTC time for the next top-of-second
// as indicated by the TP pulse.
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void newPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  uint32_t t = SFE_UBLOX_DAYS_FROM_1970_TO_2020;                                                             // Jan 1st 2020 as days from Jan 1st 1970
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_2020[ubxDataStruct->year - 2020];                                      // Add on the number of days since 2020
  t += (uint32_t)SFE_UBLOX_DAYS_SINCE_MONTH[ubxDataStruct->year % 4 == 0 ? 0 : 1][ubxDataStruct->month - 1]; // Add on the number of days since Jan 1st
  t += (uint32_t)ubxDataStruct->day - 1;                                                                     // Add on the number of days since the 1st of the month
  t *= 24;                                                                                                   // Convert to hours
  t += (uint32_t)ubxDataStruct->hour;                                                                        // Add on the hour
  t *= 60;                                                                                                   // Convert to minutes
  t += (uint32_t)ubxDataStruct->min;                                                                         // Add on the minute
  t *= 60;                                                                                                   // Convert to seconds
  t += (uint32_t)ubxDataStruct->sec;                                                                         // Add on the second

  int32_t us = ubxDataStruct->nano / 1000;                                                                   // Convert nanos to micros
  uint32_t micro;
  // Adjust t if nano is negative
  if (us < 0)
  {
    micro = (uint32_t)(us + 1000000); // Make nano +ve
    t--;                              // Decrement t by 1 second
  }
  else
  {
    micro = us;
  }

  gnssTv.tv_sec = time_t(t);
  gnssTv.tv_usec = micro;
  gnssUTCreceived = millis();
}

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
  pinMode(ETHERNET_INT, INPUT);

  delay(1000);

  Serial.begin(115200);
  Serial.println("SparkFun Reference Station - Test Sketch");

  Ethernet.init(ETHERNET_CS); // Configure the W5500 CS

  // Start the Ethernet connection:
  Serial.println("Initialize Ethernet with DHCP:");
  if (Ethernet.begin(mac) == 0) {
    Serial.println("Failed to configure Ethernet using DHCP");
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.println("Ethernet not found.  Sorry, can't run without hardware. :(");
    } else if (Ethernet.linkStatus() == LinkOFF) {
      Serial.println("Ethernet cable is not connected.");
    }
    // no point in carrying on, so do nothing forevermore:
    while (true) {
      delay(1);
    }
  }
  // Print your local IP address:
  Serial.print("My IP address: ");
  Serial.println(Ethernet.localIP());
  // Print the MAC address
  Serial.print("My MAC address: ");
  for (byte thisByte = 0; thisByte < 6; thisByte++) {
    // print the value of each byte of the IP address:
    Serial.print(mac[thisByte], HEX);
    if (thisByte < 5) {
      Serial.print(":");
    }
  }
  Serial.println();

  // Begin communication with the GNSS
  while (!theGNSS.begin(SPI, GNSS_CS)) // Start the GNSS on SPI. Default to 4MHz
    Serial.println("Failed to begin the GNSS. Retrying...");
    
  theGNSS.setSPIOutput(COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)

  theGNSS.newCfgValset(VAL_LAYER_RAM_BBR); // Create a new Configuration Interface VALSET message

  // While the module is _locking_ to GNSS time, make it generate 1Hz
  theGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_TP1, 1); // Set the frequency to 1Hz
  theGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_TP1, 10.0); // Set the pulse ratio / duty to 10%

  // When the module is _locked_ to GNSS time, make it generate 1Hz
  theGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_LOCK_TP1, 1); // Set the frequency to 1Hz
  theGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_LOCK_TP1, 10.0); // Set the pulse ratio / duty to 10%

  theGNSS.addCfgValset(UBLOX_CFG_TP_TP1_ENA, 1); // Make sure the enable flag is set to enable the time pulse. (Set to 0 to disable.)
  theGNSS.addCfgValset(UBLOX_CFG_TP_USE_LOCKED_TP1, 1); // Tell the module to use FREQ while locking and FREQ_LOCK when locked to GNSS time
  theGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_DEF, 1); // Tell the module that we want to set the frequency (not the period). PERIOD = 0. FREQ = 1.
  theGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_LENGTH_DEF, 0); // Tell the module to set the pulse ratio / duty (not the pulse length). RATIO = 0. LENGTH = 1.
  theGNSS.addCfgValset(UBLOX_CFG_TP_POL_TP1, 1); // Tell the module that we want the rising edge at the top of second. Falling Edge = 0. Rising Edge = 1.

  // Now set the time pulse parameters
  if (theGNSS.sendCfgValset() == false)
  {
    Serial.println(F("VALSET failed!"));
  }

  theGNSS.setAutoPVTcallbackPtr(&newPVTdata); // Enable automatic NAV PVT messages with callback to newPVTdata

  // Attach the TP interrupt to the ISR
  attachInterrupt(GNSS_INT, tpISR, RISING);

  // Attach the W5500 interrupt to the ISR
  attachInterrupt(ETHERNET_INT, ethernetISR, FALLING);

  // NOTE: NTP server must _always_ be initialized first to ensure that it occupies socket 0
  // and thus allow input capture to work properly for grabbing RX time.
  timeServer.beginListening(NTP_PORT);
}

void loop()
{
  theGNSS.checkUblox();
  theGNSS.checkCallbacks();
  
  switch (Ethernet.maintain()) {
    case 1:
      //renewed fail
      Serial.println("Error: renewed fail");
      break;

    case 2:
      //renewed success
      Serial.println("Renewed success");
      //print your local IP address:
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;

    case 3:
      //rebind fail
      Serial.println("Error: rebind fail");
      break;

    case 4:
      //rebind success
      Serial.println("Rebind success");
      //print your local IP address:
      Serial.print("My IP address: ");
      Serial.println(Ethernet.localIP());
      break;

    default:
      //nothing happened
      break;
  }

  bool processed = timeServer.processOneRequest((const timeval *)&ethernetTv);

  if (processed)
  {
    // Clear all interrupts.
    W5100.writeIR(0xE0);
    W5100.writeSnIR(3, 0xff);
    W5100.writeSnIR(2, 0xff);
    W5100.writeSnIR(1, 0xff);
    W5100.writeSnIR(0, 0xff);
  }
}
