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

#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

SFE_UBLOX_GNSS_SUPER theGNSS;

const int GNSS_CS = 5; // Chip select for the GNSS SPI interface
const int ETHERNET_CS = 27; // Chip select for the WizNet 5500
const int PWREN = 32; // 3V3_SW and SDIO Enable
const int STAT_LED = 26;
const int GNSS_INT = 25;

void tpISR()
{
  digitalWrite(STAT_LED, digitalRead(GNSS_INT));
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

  pinMode(GNSS_INT, INPUT); //Needed on ESP32 v2.0.2

  delay(1000);

  Serial.begin(115200);
  Serial.println("SparkFun Reference Station - Test Sketch");

  //theGNSS.enableDebugging(Serial); // Uncomment this line to enable debug messages on Serial

  while (!theGNSS.begin(SPI, GNSS_CS)) // Start the GNSS on SPI. Default to 4MHz
  {
    Serial.println("GNSS not detected. Retrying...");
  }

  theGNSS.setSPIOutput(COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)

  while (Serial.available())
    Serial.read(); // Make sure the serial RX buffer is empty

  Serial.println("Press any key to change the frequency");
}

void loop()
{
  const uint32_t pulseFreqs[] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000 }; // 1Hz, 10Hz, 100Hz, 1kHz, 10kHz, 100kHz, 1MHz, 10MHz
  static uint8_t pulseFreq = 0;

  if (Serial.available()) // Wait for a key press
  {
  
    theGNSS.newCfgValset(VAL_LAYER_RAM); // Create a new Configuration Interface VALSET message. Apply the changes in RAM only (not BBR).
  
    // While the module is _locking_ to GNSS time, make it generate:
    theGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_TP1, pulseFreqs[pulseFreq]); // Set the frequency
    theGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_TP1, 50.0); // Set the pulse ratio / duty to 50%
  
    // When the module is _locked_ to GNSS time, make it generate:
    theGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_LOCK_TP1, pulseFreqs[pulseFreq]); // Set the frequency
    theGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_LOCK_TP1, 50.0); // Set the pulse ratio / duty to 50%
  
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
    else
    {
      Serial.print(F("Frequency is now "));
      Serial.print(pulseFreqs[pulseFreq]);
      Serial.println(F("Hz"));

      pulseFreq++; // Change the frequency for next time

      if (pulseFreq == (sizeof(pulseFreqs) / sizeof(uint32_t))) // Have we reached the end of pulseFreqs
        pulseFreq = 0; // Reset pulseFreq
    }

    while (Serial.available())
      Serial.read(); // Make sure the serial RX buffer is empty
  }
}
