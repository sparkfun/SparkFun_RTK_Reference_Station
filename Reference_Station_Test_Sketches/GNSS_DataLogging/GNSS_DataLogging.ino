/*
  SparkFun GNSS Reference Station Test Sketch

  Configuring the GNSS to automatically send RXM SFRBX and RAWX reports at 4Hz over SPI
  and log them to file on SD card using full 4-bit SDIO.

  Also (optionally) logs NMEA and RTCM.

  Memory use:
  GNSS file buffer: 4096
  GNSS RTCM buffer: 4096
  Intermediate myBuffer: 512

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

#define DISABLE_SD   0 // Change this to 1 to disable SD logging
#define DISABLE_NMEA 0 // Change this to 1 to disable NMEA
#define DISABLE_RTCM 0 // Change this to 1 to disable RTCM

const int GNSS_CS = 5; // Chip select for the GNSS SPI interface
const int ETHERNET_CS = 27; // Chip select for the WizNet 5500
const int PWREN = 32; // 3V3_SW and SDIO Enable
const int STAT_LED = 26;

#if !DISABLE_SD
#include "FS.h"
#include "SD_MMC.h"
File myFile;
#endif

#define sdWriteSize 512     // Write data to the SD card in blocks of n*512 bytes
uint8_t *myBuffer;          // Use myBuffer to hold the data while we write it to SD card

#include <SPI.h>
#define spiPort SPI

#include <SparkFun_u-blox_GNSS_v3.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS_v3
SFE_UBLOX_GNSS_SPI myGNSS;

#define fileBufferSize 4096 // Allocate RAM for UBX message storage
#define navRate 4           // Set the Nav Rate (Frequency) to 20Hz

unsigned long lastPrint; // Record when the last Serial print took place

// Note: we'll keep a count of how many SFRBX and RAWX messages arrive - but the count will not be completely accurate.
// If two or more SFRBX messages arrive together as a group and are processed by one call to checkUblox, the count will
// only increase by one.

int numSFRBX = 0; // Keep count of how many SFRBX message groups have been received (see note above)
int numRAWX = 0; // Keep count of how many RAWX message groups have been received (see note above)

// Callback: newSFRBX will be called when new RXM SFRBX data arrives
// See u-blox_structs.h for the full definition of UBX_RXMSFRBX_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMSFRBXcallback
//        /                  _____  This _must_ be UBX_RXM_SFRBX_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void newSFRBX(UBX_RXM_SFRBX_data_t *ubxDataStruct)
{
  numSFRBX++; // Increment the count
}

// Callback: newRAWX will be called when new RXM RAWX data arrives
// See u-blox_structs.h for the full definition of UBX_RXMRAWX_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoRXMRAWXcallback
//        /             _____  This _must_ be UBX_RXM_RAWX_data_t
//        |            /                _____ You can use any name you like for the struct
//        |            |               /
//        |            |               |
void newRAWX(UBX_RXM_RAWX_data_t *ubxDataStruct)
{
  numRAWX++; // Increment the count
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
  digitalWrite(STAT_LED, LOW);

  delay(1000);

  Serial.begin(115200);
  Serial.println("SparkFun Reference Station - Test Sketch");

  Serial.printf("1) FreeHeap: %d / HeapLowestPoint: %d / LargestBlock: %d\r\n", ESP.getFreeHeap(), xPortGetMinimumEverFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Initialize the GNSS

  spiPort.begin();

  Serial.println(F("Initializing the GNSS..."));

  //myGNSS.enableDebugging(); // Uncomment this line to see helpful debug messages on Serial

  myGNSS.setFileBufferSize(fileBufferSize); // setFileBufferSize must be called _before_ .begin
#if !DISABLE_RTCM
  myGNSS.setRTCMBufferSize(fileBufferSize); // setRTCMBufferSize must be called _before_ .begin
#endif

  // Connect to the u-blox module using SPI port, csPin and speed setting
  // ublox devices generally work up to 5MHz. We'll use 4MHz for this example:
  bool begun = false;
  do
  {
    begun = myGNSS.begin(spiPort, GNSS_CS, 4000000);
    if (!begun)
    {
      Serial.println(F("u-blox GNSS not detected on SPI bus."));
      delay(1000);
    }
  }
  while (!begun);
  
  Serial.printf("2) FreeHeap: %d / HeapLowestPoint: %d / LargestBlock: %d\r\n", ESP.getFreeHeap(), xPortGetMinimumEverFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Wait for a 3D fix. Get the date and time for the log file

  //myGNSS.factoryDefault(); delay(5000); // Uncomment this line to reset the module back to its factory defaults

#if DISABLE_NMEA && DISABLE_RTCM
  myGNSS.setSPIOutput(COM_TYPE_UBX); //Set the SPI port to output UBX only
#elif DISABLE_RTCM
  myGNSS.setSPIOutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the SPI port to output UBX and NMEA messages
#elif DISABLE_NMEA
  myGNSS.setSPIOutput(COM_TYPE_UBX | COM_TYPE_RTCM3); //Set the SPI port to output UBX and RTCM messages
#else
  myGNSS.setSPIOutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); //Set the SPI port to output UBX, NMEA and RTCM messages
#endif

  //myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Optional: save (only) the communications port settings to flash and BBR

  Serial.print(F("Waiting for a 3D fix"));
  delay(100);

  uint8_t fix = 0;

  do
  {
    fix = myGNSS.getFixType();
    delay(1000);
    Serial.print(F("."));
  }
  while ( fix != 3 ); // Wait for a 3D fix

  Serial.println();

  uint16_t y = myGNSS.getYear();
  uint8_t M = myGNSS.getMonth();
  uint8_t d = myGNSS.getDay();
  uint8_t h = myGNSS.getHour();
  uint8_t m = myGNSS.getMinute();
  uint8_t s = myGNSS.getSecond();

  char szBuffer[40] = {'\0'};
  snprintf(szBuffer, sizeof(szBuffer), "/%04d%02d%02d%02d%02d%02d.ubx", y, M, d, h, m, s);

  Serial.println(F("GNSS initialized."));

  Serial.printf("3) FreeHeap: %d / HeapLowestPoint: %d / LargestBlock: %d\r\n", ESP.getFreeHeap(), xPortGetMinimumEverFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Initialize the SD card. Open the log file

#if !DISABLE_SD
  Serial.println(F("Initializing SD card..."));

  // Begin the SD card
  if(!SD_MMC.begin())
  {
    Serial.println(F("Card mount failed. Freezing..."));
    while(1);
  }

  // Open the log file for writing
  Serial.printf("Log file is: %s\r\n", szBuffer);
  myFile = SD_MMC.open((const char *)szBuffer, FILE_WRITE);
  if(!myFile)
  {
    Serial.println(F("Failed to open log file for writing. Freezing..."));
    while(1);
  }

  Serial.println(F("SD card initialized."));
#endif

  Serial.printf("4) FreeHeap: %d / HeapLowestPoint: %d / LargestBlock: %d\r\n", ESP.getFreeHeap(), xPortGetMinimumEverFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Wait for a key press

  while (Serial.available()) // Make sure the Serial buffer is empty
  {
    Serial.read();
  }

  Serial.println(F("Press any key to start logging."));

  while (!Serial.available()) // Wait for the user to press a key
  {
    ; // Do nothing
  }

  delay(100); // Wait, just in case multiple characters were sent

  while (Serial.available()) // Empty the Serial buffer
  {
    Serial.read();
  }

  // -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
  // Enable RAWX and SFRBX

  myGNSS.setAutoRXMSFRBXcallbackPtr(&newSFRBX); // Enable automatic RXM SFRBX messages with callback to newSFRBX

  myGNSS.logRXMSFRBX(); // Enable RXM SFRBX data logging

  myGNSS.setAutoRXMRAWXcallbackPtr(&newRAWX); // Enable automatic RXM RAWX messages with callback to newRAWX

  myGNSS.logRXMRAWX(); // Enable RXM RAWX data logging

  myBuffer = new uint8_t[sdWriteSize]; // Create our own buffer to hold the data while we write it to SD card  

#if !DISABLE_NMEA
  myGNSS.newCfgValset(VAL_LAYER_RAM);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GGA_SPI, navRate); // Ensure the GxGGA (Global positioning system fix data) message is enabled. Send every second.
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSA_SPI, navRate); // Ensure the GxGSA (GNSS DOP and Active satellites) message is enabled. Send every second.
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GSV_SPI, navRate); // Ensure the GxGSV (GNSS satellites in view) message is enabled. Send every second.
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_GST_SPI, navRate); // Ensure the GxGST (Position error statistics) message is enabled. Send every second.
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_NMEA_ID_RMC_SPI, navRate); // Ensure the GxRMC (Recommended minimum: position, velocity and time) message is enabled. Send every second.
  myGNSS.sendCfgValset();

  myGNSS.setNMEALoggingMask(SFE_UBLOX_FILTER_NMEA_GGA | SFE_UBLOX_FILTER_NMEA_GSA | SFE_UBLOX_FILTER_NMEA_GSV | SFE_UBLOX_FILTER_NMEA_GST | SFE_UBLOX_FILTER_NMEA_RMC); // Log only these NMEA messages
#endif

#if !DISABLE_RTCM
  myGNSS.newCfgValset(VAL_LAYER_RAM);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_SPI, navRate); // Enable RTCM3 4072_0 at the navigation rate. This is output without needing a survey-in.
  myGNSS.sendCfgValset();

  myGNSS.setRTCMLoggingMask(SFE_UBLOX_FILTER_RTCM_TYPE4072_0); // Log only this RTCM3 message
#endif

  myGNSS.setNavigationFrequency(navRate); // Set navigation rate

  Serial.printf("5) FreeHeap: %d / HeapLowestPoint: %d / LargestBlock: %d\r\n", ESP.getFreeHeap(), xPortGetMinimumEverFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

  Serial.println(F("Press any key to stop logging."));

  lastPrint = millis(); // Initialize lastPrint
}

void loop()
{
  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  myGNSS.checkUblox(); // Check for the arrival of new data and process it.
  myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  while (myGNSS.fileBufferAvailable() >= sdWriteSize) // Check to see if we have at least sdWriteSize waiting in the buffer
  {
    digitalWrite(STAT_LED, HIGH); // Flash the STAT LED each time we write to the SD card

    myGNSS.extractFileBufferData(myBuffer, sdWriteSize); // Extract exactly sdWriteSize bytes from the UBX file buffer and put them into myBuffer

#if !DISABLE_SD
    myFile.write(myBuffer, sdWriteSize); // Write exactly sdWriteSize bytes from myBuffer to the ubxDataFile on the SD card
#endif

    // In case the SD writing is slow or there is a lot of data to write, keep checking for the arrival of new data
    myGNSS.checkUblox(); // Check for the arrival of new data and process it.
    myGNSS.checkCallbacks(); // Check if any callbacks are waiting to be processed.

    digitalWrite(STAT_LED, LOW); // Turn the STAT LED off again
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (millis() > (lastPrint + 1000)) // Print the message count once per second
  {
    uint16_t maxBufferBytes = myGNSS.getMaxFileBufferAvail(); // Get how full the file buffer has been (not how full it is now)
    float bufferHigh = 100.0 * (float)maxBufferBytes / (float)fileBufferSize;

    Serial.print(F("Message groups received: SFRBX: ")); // Print how many message groups have been received (see note above)
    Serial.print(numSFRBX);
    Serial.print(F(" RAWX: "));
    Serial.print(numRAWX);
    Serial.print(F("  \tBuffer high tide: "));
    Serial.print(bufferHigh, 1); // It is a fun thing to watch how full the buffer gets
    if (bufferHigh > 90.)
      Serial.println(F("%!!"));
    else if (bufferHigh > 80.)
      Serial.println(F("%!"));
    else
      Serial.println(F("%"));

    Serial.printf("L) FreeHeap: %d / HeapLowestPoint: %d / LargestBlock: %d\r\n", ESP.getFreeHeap(), xPortGetMinimumEverFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    lastPrint = millis(); // Update lastPrint
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  if (Serial.available()) // Check if the user wants to stop logging
  {
    uint16_t remainingBytes = myGNSS.fileBufferAvailable(); // Check if there are any bytes remaining in the file buffer

    while (remainingBytes > 0) // While there is still data in the file buffer
    {
      digitalWrite(STAT_LED, HIGH); // Flash the STAT LED while we write to the SD card

      uint16_t bytesToWrite = remainingBytes; // Write the remaining bytes to SD card sdWriteSize bytes at a time
      if (bytesToWrite > sdWriteSize)
      {
        bytesToWrite = sdWriteSize;
      }

      myGNSS.extractFileBufferData(myBuffer, bytesToWrite); // Extract bytesToWrite bytes from the UBX file buffer and put them into myBuffer

#if !DISABLE_SD
      myFile.write(myBuffer, bytesToWrite); // Write bytesToWrite bytes from myBuffer to the ubxDataFile on the SD card
#endif

      remainingBytes -= bytesToWrite; // Decrement remainingBytes
    }

    digitalWrite(STAT_LED, LOW); // Turn the STAT LED off

#if !DISABLE_SD
    myFile.close(); // Close the data file
#endif

    myGNSS.setNavigationFrequency(1); // Set navigation rate to 1Hz

    myGNSS.newCfgValset(VAL_LAYER_RAM);
    myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_RAWX_SPI, 0);
    myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_RXM_SFRBX_SPI, 0);
    myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE4072_0_SPI, 0);
    myGNSS.sendCfgValset();

    myGNSS.setSPIOutput(COM_TYPE_UBX | COM_TYPE_NMEA | COM_TYPE_RTCM3); // Re-enable NMEA and RTCM

    Serial.printf("6) FreeHeap: %d / HeapLowestPoint: %d / LargestBlock: %d\r\n", ESP.getFreeHeap(), xPortGetMinimumEverFreeHeapSize(), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));

    Serial.println(F("Logging stopped. Freezing..."));
    while(1); // Do nothing more
  }

  // =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
}
