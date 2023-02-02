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
// Begin communication with the GNSS

bool configureGNSS()
{
  int beginCount = 0;
  
  while ((!theGNSS.begin(SPI, GNSS_CS)) && (beginCount < 5)) // Start the GNSS on SPI. Default to 4MHz
  {
    beginCount++;
  }

  if (beginCount == 5)
    return false;

  bool success = true;
  
  success &= theGNSS.setSPIOutput(COM_TYPE_UBX); //Set the SPI port to output UBX only (turn off NMEA noise)

  success &= theGNSS.newCfgValset(VAL_LAYER_RAM_BBR); // Create a new Configuration Interface VALSET message

  // While the module is _locking_ to GNSS time, stop the TP pulses
  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_TP1, 0); // Set the frequency to 0Hz
  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_TP1, 0.0); // Set the pulse ratio / duty to 0%

  // When the module is _locked_ to GNSS time, make it generate 1Hz
  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_FREQ_LOCK_TP1, 1); // Set the frequency to 1Hz
  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_DUTY_LOCK_TP1, 10.0); // Set the pulse ratio / duty to 10%

  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_TP1_ENA, 1); // Make sure the enable flag is set to enable the time pulse. (Set to 0 to disable.)
  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_USE_LOCKED_TP1, 1); // Tell the module to use FREQ while locking and FREQ_LOCK when locked to GNSS time
  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_DEF, 1); // Tell the module that we want to set the frequency (not the period). PERIOD = 0. FREQ = 1.
  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_PULSE_LENGTH_DEF, 0); // Tell the module to set the pulse ratio / duty (not the pulse length). RATIO = 0. LENGTH = 1.
  success &= theGNSS.addCfgValset(UBLOX_CFG_TP_POL_TP1, 1); // Tell the module that we want the rising edge at the top of second. Falling Edge = 0. Rising Edge = 1.

  // Now set the time pulse parameters
  success &= theGNSS.sendCfgValset();

  success &= theGNSS.setAutoPVTcallbackPtr(&newPVTdata); // Enable automatic NAV PVT messages with callback to newPVTdata

  // Tell the module to return UBX_MGA_ACK_DATA0 messages when we push the AssistNow data
  success &= theGNSS.setAckAiding(1);
    
  return success;
}
