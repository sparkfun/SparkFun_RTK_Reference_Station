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
*/

#include <SPI.h> // Needed for SPI to GNSS

#include <Ethernet.h>
#include <EthernetUdp.h>
#include "utility/w5100.h"

#include <Esp.h>
#include <time.h>

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// GNSS object

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SUPER theGNSS;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Pin definitions

const int GNSS_CS = 5; // Chip select for the GNSS SPI interface
const int ETHERNET_CS = 27; // Chip select for the WizNet 5500
const int PWREN = 32; // 3V3_SW and SDIO Enable
const int ETHERNET_INT = 33;
const int STAT_LED = 26;
const int GNSS_INT = 25;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// NTP port server (UDP)

#define NTP_PORT 123

EthernetUDP timeServer;

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Globals

volatile bool syncRTC = true;                   // Flag to indicate if the RTC should be sync'd
volatile struct timeval gnssTv;                 // This will hold the GNSS time
volatile struct timeval gnssSyncTv;             // This will hold the time the GNSS was last sync'd
volatile struct timeval ethernetTv;             // This will hold the time the Ethernet packet arrived
const timezone tz_utc = { 0, 0 };               // Set the timezone to UTC
volatile bool timeFullyResolved = false;        // Record if GNSS time is fully resolved
volatile unsigned long lastRTCsync = 0;         // Record the millis when the RTC is sync'd
volatile unsigned long gnssUTCreceived = 0;     // Record the millis when GNSS time is received
const unsigned long resyncAfterMillis = 60000;  // Re-sync every minute - for testing only. Once per hour should be more than adequate
const unsigned long gnssStaleAfter = 999;       // Treat GNSS time as stale after this many millis
volatile uint32_t tAcc;                         // Record the GNSS time accuracy
const uint32_t tAccLimit = 5000;               // Only sync if the time accuracy estimate is better than 15us (5000 nanoseconds)

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
            settimeofday((const timeval *)&gnssTv, &tz_utc); // Sync the RTC
            lastRTCsync = millis(); // Update lastSync
            gnssSyncTv.tv_sec = gnssTv.tv_sec; // Store the timeval of the sync
            gnssSyncTv.tv_usec = gnssTv.tv_usec;
          }
        }
      }
    }
  }
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Ethernet (W5500) ISR
// Triggered by the falling edge of the W5500 interrupt signal - indicates the arrival of a packet
// Record the time the packet arrived

void ethernetISR()
{
  gettimeofday((timeval *)&ethernetTv, (timezone *)&tz_utc);
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// GNSS PVT Callback: newPVTdata will be called when new NAV PVT data arrives.
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
  // Convert date and time into Unix epoch
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

  gnssTv.tv_sec = time_t(t); // Store the time in timeval format
  gnssTv.tv_usec = micro;
  timeFullyResolved = ubxDataStruct->valid.bits.fullyResolved;
  tAcc = ubxDataStruct->tAcc;
  gnssUTCreceived = millis();
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// NTP Packet storage and utilities

struct NTPpacket
{
  static const uint8_t NTPpacketSize = 48;
  
  uint8_t packet[NTPpacketSize]; // Copy of the NTP packet
  void setPacket(uint8_t *ptr) { memcpy(packet, ptr, NTPpacketSize); }
  void getPacket(uint8_t *ptr) { memcpy(ptr, packet, NTPpacketSize); }

  const uint32_t NTPtoUnixOffset = 2208988800; // NTP starts at Jan 1st 1900. Unix starts at Jan 1st 1970.
  
  uint8_t LiVnMode; // Leap Indicator, Version Number, Mode

  // Leap Indicator is 2 bits :
  // 00 : No warning
  // 01 : Last minute of the day has 61s
  // 10 : Last minute of the day has 59 s
  // 11 : Alarm condition (clock not synchronized)
  uint8_t LI() { return LiVnMode >> 6; }
  void LI(uint8_t val) { LiVnMode = (LiVnMode & 0x3F) | ((val & 0x03) << 6); }

  // Version Number is 3 bits. NTP version is currently four (4)
  uint8_t VN() { return (LiVnMode >> 3) & 0x07; }
  void VN(uint8_t val) { LiVnMode = (LiVnMode & 0xC7) | ((val & 0x07) << 3); }

  // Mode is 3 bits:
  // 0 : Reserved
  // 1 : Symmetric active
  // 2 : Symmetric passive
  // 3 : Client
  // 4 : Server
  // 5 : Broadcast
  // 6 : NTP control message
  // 7 : Reserved for private use
  uint8_t mode() { return (LiVnMode & 0x07); }
  void mode(uint8_t val) { LiVnMode = (LiVnMode & 0xF8) | (val & 0x07); }

  // Stratum is 8 bits:
  // 0 : Unspecified
  // 1 : Reference clock (e.g., radio clock)
  // 2-15 : Secondary server (via NTP)
  // 16-255 : Unreachable
  uint8_t stratum;

  // Poll exponent
  // This is an eight-bit unsigned integer indicating the maximum interval between successive messages,
  // in seconds to the nearest power of two.
  // In the reference implementation, the values can range from 3 (8 s) through 17 (36 h).
  uint8_t pollExponent;

  // Precision
  // This is an eight-bit signed integer indicating the precision of the system clock,
  // in seconds to the nearest power of two. For instance, a value of -18 corresponds to a precision of about 4us.
  int8_t precision;

  // Root delay
  // This is a 32-bit, unsigned, fixed-point number indicating the total round-trip delay to the reference clock,
  // in seconds with fraction point between bits 15 and 16. In contrast to the calculated peer round-trip delay,
  // which can take both positive and negative values, this value is always positive.
  uint32_t rootDelay;

  // Root dispersion
  // This is a 32-bit, unsigned, fixed-point number indicating the maximum error relative to the reference clock,
  // in seconds with fraction point between bits 15 and 16.
  uint32_t rootDispersion;

  // Reference identifier
  // This is a 32-bit code identifying the particular reference clock. The interpretation depends on the value in
  // the stratum field. For stratum 0 (unsynchronized), this is a four-character ASCII (American Standard Code for
  // Information Interchange) string called the kiss code, which is used for debugging and monitoring purposes.
  // GPS : Global Positioning System
  char referenceId[4];

  // Reference timestamp
  // This is the local time at which the system clock was last set or corrected, in 64-bit NTP timestamp format.
  uint32_t referenceTimestampSeconds;
  uint32_t referenceTimestampFraction;

  // Originate timestamp
  // This is the local time at which the request departed the client for the server, in 64-bit NTP timestamp format.
  uint32_t originateTimestampSeconds;
  uint32_t originateTimestampFraction;

  // Receive timestamp
  // This is the local time at which the request arrived at the server, in 64-bit NTP timestamp format.
  uint32_t receiveTimestampSeconds;
  uint32_t receiveTimestampFraction;

  // Transmit timestamp
  // This is the local time at which the reply departed the server for the client, in 64-bit NTP timestamp format.
  uint32_t transmitTimestampSeconds;
  uint32_t transmitTimestampFraction;

  typedef union
  {
    int8_t signed8;
    uint8_t unsigned8;
  } unsignedSigned8;

  uint32_t extractUnsigned32(uint8_t *ptr)
  {
    uint32_t val = 0;
    val |= *ptr++ << 24; // NTP data is Big-Endian
    val |= *ptr++ << 16;
    val |= *ptr++ << 8;
    val |= *ptr++;
    return val;
  }

  void insertUnsigned32(uint8_t *ptr, uint32_t val)
  {
    *ptr++ = val >> 24; // NTP data is Big-Endian
    *ptr++ = (val >> 16) & 0xFF;
    *ptr++ = (val >> 8) & 0xFF;
    *ptr++ = val & 0xFF;
  }

  // Extract the data from an NTP packet into the correct fields
  void extract()
  {
    uint8_t *ptr = packet;
    
    LiVnMode = *ptr++;
    stratum = *ptr++;
    pollExponent = *ptr++;
    
    unsignedSigned8 converter8;
    converter8.unsigned8 = *ptr++; // Convert to int8_t without ambiguity
    precision = converter8.signed8;

    rootDelay = extractUnsigned32(ptr);
    ptr += 4;
    rootDispersion = extractUnsigned32(ptr);
    ptr += 4;

    for (uint8_t i = 0; i < 4; i++)
      referenceId[i] = *ptr++;

    referenceTimestampSeconds = extractUnsigned32(ptr);
    ptr += 4;
    referenceTimestampFraction = extractUnsigned32(ptr); // Note: the fraction is in increments of (1 / 2^32) secs, not microseconds
    ptr += 4;
    originateTimestampSeconds = extractUnsigned32(ptr);
    ptr += 4;
    originateTimestampFraction = extractUnsigned32(ptr);
    ptr += 4;
    transmitTimestampSeconds = extractUnsigned32(ptr);
    ptr += 4;
    transmitTimestampFraction = extractUnsigned32(ptr);
    ptr += 4;
    receiveTimestampSeconds = extractUnsigned32(ptr);
    ptr += 4;
    receiveTimestampFraction = extractUnsigned32(ptr);
    ptr += 4;
  }

  // Insert the data from the fields into an NTP packet
  void insert()
  {
    uint8_t *ptr = packet;
    
    *ptr++ = LiVnMode;
    *ptr++ = stratum;
    *ptr++ = pollExponent;
    
    unsignedSigned8 converter8;
    converter8.signed8 = precision;
    *ptr++ = converter8.unsigned8; // Convert to uint8_t without ambiguity

    insertUnsigned32(ptr, rootDelay);
    ptr += 4;
    insertUnsigned32(ptr, rootDispersion);
    ptr += 4;

    for (uint8_t i = 0; i < 4; i++)
      *ptr++ = referenceId[i];

    insertUnsigned32(ptr, referenceTimestampSeconds);
    ptr += 4;
    insertUnsigned32(ptr, referenceTimestampFraction); // Note: the fraction is in increments of (1 / 2^32) secs, not microseconds
    ptr += 4;
    insertUnsigned32(ptr, originateTimestampSeconds);
    ptr += 4;
    insertUnsigned32(ptr, originateTimestampFraction);
    ptr += 4;
    insertUnsigned32(ptr, transmitTimestampSeconds);
    ptr += 4;
    insertUnsigned32(ptr, transmitTimestampFraction);
    ptr += 4;
    insertUnsigned32(ptr, receiveTimestampSeconds);
    ptr += 4;
    insertUnsigned32(ptr, receiveTimestampFraction);
    ptr += 4;
  }

  uint32_t convertMicrosToFraction(uint32_t val)
  {
    val %= 1000000; // Just in case
    double v = val; // Convert micros to double
    v /= 1000000.0; // Convert micros to seconds
    v *= pow(2.0, 32.0); // Convert to fraction
    return (uint32_t)v;
  }

  uint32_t convertFractionToMicros(uint32_t val)
  {
    double v = val; // Convert fraction to double
    v /= pow(2.0, 32.0); // Convert fraction to seconds
    v *= 1000000.0; // Convert to micros
    uint32_t ret = (uint32_t)v;
    ret %= 1000000; // Just in case
    return ret;
  }

  uint32_t convertNTPsecondsToUnix(uint32_t val)
  {
    return (val - NTPtoUnixOffset);
  }
  
  uint32_t convertUnixSecondsToNTP(uint32_t val)
  {
    return (val + NTPtoUnixOffset);
  }
};

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Extra code for the W5500 (vs. w5100.h)

const uint16_t w5500RTR = 0x0019; // Retry Count Register - Common Register Block
const uint16_t w5500SIR = 0x0017; // Socket Interrupt Register - Common Register Block
const uint16_t w5500SIMR = 0x0018; // Socket Interrupt Mask Register - Common Register Block
const uint16_t w5500SnIR = 0x0002; // Socket n Interrupt Register - Socket Register Block
const uint16_t w5500SnIMR = 0x002C; // Socket n Interrupt Mask Register - Socket Register Block

const uint8_t w5500CommonRegister = 0x00 << 3; // Block Select
const uint8_t w5500Socket0Register = 0x01 << 3; // Block Select
const uint8_t w5500Socket1Register = 0x05 << 3; // Block Select
const uint8_t w5500Socket2Register = 0x09 << 3; // Block Select
const uint8_t w5500Socket3Register = 0x0D << 3; // Block Select
const uint8_t w5500Socket4Register = 0x11 << 3; // Block Select
const uint8_t w5500Socket5Register = 0x15 << 3; // Block Select
const uint8_t w5500Socket6Register = 0x19 << 3; // Block Select
const uint8_t w5500Socket7Register = 0x1D << 3; // Block Select
const uint8_t w5500RegisterWrite = 0x01 << 2; // Read/Write bit
const uint8_t w5500VDM = 0x00 << 0; // Variable Data Length Mode
const uint8_t w5500FDM1 = 0x01 << 0; // Fixed Data Length Mode 1 Byte
const uint8_t w5500FDM2 = 0x02 << 0; // Fixed Data Length Mode 2 Byte
const uint8_t w5500FDM4 = 0x03 << 0; // Fixed Data Length Mode 4 Byte

const uint8_t w5500SocketRegisters[] = { w5500Socket0Register, w5500Socket1Register,
                                         w5500Socket2Register, w5500Socket3Register,
                                         w5500Socket4Register, w5500Socket5Register,
                                         w5500Socket6Register, w5500Socket7Register };

const uint8_t w5500SIR_ClearAll = 0xFF;
const uint8_t w5500SIMR_EnableAll = 0xFF;

const uint8_t w5500SnIR_ClearAll = 0xFF;

const uint8_t w5500SnIMR_CON = 0x01 << 0;
const uint8_t w5500SnIMR_DISCON = 0x01 << 1;
const uint8_t w5500SnIMR_RECV = 0x01 << 2;
const uint8_t w5500SnIMR_TIMEOUT = 0x01 << 3;
const uint8_t w5500SnIMR_SENDOK = 0x01 << 4;

void w5500write(SPIClass &spiPort, const int cs, uint16_t address, uint8_t control, uint8_t *data, uint8_t len)
{
  // Apply settings
  spiPort.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

  // Signal communication start
  digitalWrite(cs, LOW);

  spiPort.transfer(address >> 8); // Address Phase
  spiPort.transfer(address & 0xFF);
  
  spiPort.transfer(control | w5500RegisterWrite | w5500VDM); // Control Phase

  for (uint8_t i = 0; i < len; i++)
  {
    spiPort.transfer(*data++); // Data Phase
  }

  // End communication
  digitalWrite(cs, HIGH);
  spiPort.endTransaction();
}

void w5500ClearInterrupts()
{
  // Clear all W5500 interrupts.
  for (uint8_t i = 0; i < (sizeof(w5500SocketRegisters) / sizeof(uint8_t)); i++)
  {
    w5500write(SPI, GNSS_CS, w5500SnIR, w5500SocketRegisters[i], (uint8_t *)&w5500SnIR_ClearAll, 1);
  }
  w5500write(SPI, GNSS_CS, w5500SIR, w5500CommonRegister, (uint8_t *)&w5500SIR_ClearAll, 1);
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

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Attach the TP interrupt to the ISR
  attachInterrupt(GNSS_INT, tpISR, RISING);

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Start the server

  // NOTE: NTP server must _always_ be initialized first to ensure that it occupies socket 0
  // and thus allow input capture to work properly for grabbing RX time.
  timeServer.begin(NTP_PORT); // Begin listening

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Configure Ethernet interrupts
  
  // Enable the RECV interrupt on all eight sockets
  for (uint8_t i = 0; i < (sizeof(w5500SocketRegisters) / sizeof(uint8_t)); i++)
  {
    w5500write(SPI, GNSS_CS, w5500SnIMR, w5500SocketRegisters[i], (uint8_t *)&w5500SnIMR_RECV, 1);
  }

  w5500write(SPI, GNSS_CS, w5500SIMR, w5500CommonRegister, (uint8_t *)&w5500SIMR_EnableAll, 1); // Enable the socket interrupt on all eight sockets
 
  // Attach the W5500 interrupt to the ISR
  attachInterrupt(ETHERNET_INT, ethernetISR, FALLING);

  w5500ClearInterrupts();
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void loop()
{
  // Check for the arrival of fresh PVT data
  theGNSS.checkUblox();
  theGNSS.checkCallbacks();

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Check for new NTP requests
  static char ntpDiag[256]; // Char array to hold diagnostic messages
  bool processed = processOneRequest((const timeval *)&ethernetTv, (const timeval *)&gnssSyncTv, ntpDiag, sizeof(ntpDiag));

  if (processed)
  {
    w5500ClearInterrupts();
    Serial.print("NTP request processed: ");
    Serial.println(ntpDiag);
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Maintain the ethernet connection
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

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Diagnostics

  // Print helpful messages
  if (lastRTCsync == 0)
  {
    static unsigned long lastPrint = 0;
    if ((lastPrint + 5000) < millis())
    {
      Serial.println("Waiting for GNSS time sync...");
      lastPrint = millis();
    }
  }

  // Check if the RTC has been sync'd
  static unsigned long previousRTCsync = 0;
  if (previousRTCsync != lastRTCsync)
  {
    struct tm timeinfo;
    getLocalTime(&timeinfo);
    Serial.println(&timeinfo, "RTC re-sync'd at: %A, %B %d %Y %H:%M:%S");
    Serial.print("tAcc is ");
    Serial.print(tAcc);
    Serial.println("ns");
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
    snprintf(buf, sizeof (buf), "%s.%06ld", tmbuf, lastW5500interrupt.tv_usec);
    Serial.println(buf);
  }
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// Process one NTP request
// recTv contains the timeval the NTP packet was received - from the W5500 interrupt
// syncTv contains the timeval when the RTC was last sync'd
// ntpDiag will contain useful diagnostics
bool processOneRequest(const timeval * recTv, const timeval * syncTv, char *ntpDiag = nullptr, size_t ntpDiagSize = 0); // Header
bool processOneRequest(const timeval * recTv, const timeval * syncTv, char *ntpDiag, size_t ntpDiagSize)
{
  bool processed = false;

  if (ntpDiag != nullptr)
    *ntpDiag = 0; // Clear any existing results
  
  int packetDataSize = timeServer.parsePacket();

  if (ntpDiag != nullptr) // Add the packet size and remote IP/Port to the diagnostics
  {
    IPAddress remote = timeServer.remoteIP();
    snprintf(ntpDiag, ntpDiagSize, "Packet Size: %d  Remote IP: %d.%d.%d.%d  Remote Port: %d\r\n",
      packetDataSize, remote[0], remote[1], remote[2], remote[3], timeServer.remotePort());
  }
    
  if (packetDataSize && (packetDataSize >= NTPpacket::NTPpacketSize))
  {
    // Read the NTP packet
    NTPpacket packet;
    timeServer.read((char*)&packet.packet, NTPpacket::NTPpacketSize); // Copy the NTP data into our packet
    packet.extract(); // Extract the raw data into fields
    
    packet.LI(0); // Clear the leap second adjustment. TODO: set this correctly using getLeapSecondEvent from the GNSS
    packet.VN(4); // Set the version number to 4
    packet.mode(4); // Set the mode to 4 : Server
    packet.stratum = 1; // Set the stratum to 1 : primary server
    packet.pollExponent = 6; // Set the poll interval: 6-10 per RFC 5905. 2^6 = 64 seconds
    packet.precision = -20; // Set the precision: 2^-20 = ~1 microsecond precision.
    packet.rootDelay = 60 * (0xFFFF / 1000); // Set the Root Delay to ~60 milliseconds. Format is Seconds:16.Fraction:16
    packet.rootDispersion = 10 * (0xFFFF / 1000); // Set the Root Dispersion to ~10 milliseconds. Format is Seconds:16.Fraction:16
    packet.referenceId[0] = 'G'; // Set the reference Id to GPS
    packet.referenceId[1] = 'P';
    packet.referenceId[2] = 'S';
    packet.referenceId[3] = 0;

    // REF: http://support.ntp.org/bin/view/Support/DraftRfc2030
    // '.. the client sets the Transmit Timestamp field in the request
    // to the time of day according to the client clock in NTP timestamp format.'
    // '.. The server copies this field to the originate timestamp in the reply and 
    // sets the Receive Timestamp and Transmit Timestamp fields to the time of day 
    // according to the server clock in NTP timestamp format.'

    // Important note: the NTP Era started January 1st 1900.
    // tv will contain the time based on the Unix epoch (January 1st 1970)
    // We need to adjust...

    // First, add the client transmit timestamp to our diagnostics
    if (ntpDiag != nullptr)
    {
      char tmpbuf[128];
      snprintf(tmpbuf, sizeof(tmpbuf), "Client timestamp:\t%u.%06u\r\n",
        packet.transmitTimestampSeconds, packet.convertFractionToMicros(packet.transmitTimestampFraction));
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
    }

    // Copy the client transmit timestamp into the originate timestamp
    packet.originateTimestampSeconds = packet.transmitTimestampSeconds;
    packet.originateTimestampFraction = packet.transmitTimestampFraction;

    // Set the receive timestamp to the time we received the packet (logged by the W5500 interrupt)
    packet.receiveTimestampSeconds = packet.convertUnixSecondsToNTP(recTv->tv_sec); // Unix -> NTP
    packet.receiveTimestampFraction = packet.convertMicrosToFraction(recTv->tv_usec); // Micros to 1/2^32
    
    // Add the receive timestamp to the diagnostics
    if (ntpDiag != nullptr)
    {
      char tmpbuf[128];
      snprintf(tmpbuf, sizeof(tmpbuf), "Received timestamp:\t%u.%06u\r\n",
        packet.receiveTimestampSeconds, packet.convertFractionToMicros(packet.receiveTimestampFraction));
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
    }

    // Add when our clock was last sync'd
    packet.referenceTimestampSeconds = packet.convertUnixSecondsToNTP(syncTv->tv_sec); // Unix -> NTP
    packet.referenceTimestampFraction = packet.convertMicrosToFraction(syncTv->tv_usec); // Micros to 1/2^32

    // Add that to the diagnostics
    if (ntpDiag != nullptr)
    {
      char tmpbuf[128];
      snprintf(tmpbuf, sizeof(tmpbuf), "Last sync:\t\t%u.%06u\r\n",
        packet.referenceTimestampSeconds, packet.convertFractionToMicros(packet.referenceTimestampFraction));
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
    }

    // Add the transmit time - i.e. now!
    timeval txTime;
    gettimeofday(&txTime, (timezone *)&tz_utc);
    packet.transmitTimestampSeconds = packet.convertUnixSecondsToNTP(txTime.tv_sec); // Unix -> NTP
    packet.transmitTimestampFraction = packet.convertMicrosToFraction(txTime.tv_usec); // Micros to 1/2^32

    // Now transmit the response to the client.
    packet.insert(); // Copy the data fields back into the buffer
    timeServer.beginPacket(timeServer.remoteIP(), timeServer.remotePort());
    for (int count = 0; count < NTPpacket::NTPpacketSize; count++)
    {
      timeServer.write(packet.packet[count]);
    }
    timeServer.endPacket();
    processed = true;

    // Add our server transmit time to the diagnostics
    if (ntpDiag != nullptr)
    {
      char tmpbuf[128];
      snprintf(tmpbuf, sizeof(tmpbuf), "Server timestamp:\t%u.%06u\r\n",
        packet.transmitTimestampSeconds, packet.convertFractionToMicros(packet.transmitTimestampFraction));
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
    }
  }
  
  return processed;
}
