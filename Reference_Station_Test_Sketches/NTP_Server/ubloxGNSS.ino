// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// GNSS PVT Callback: newPVTdata will be called when new NAV PVT data arrives.
// See u-blox_structs.h for the full definition of UBX_NAV_PVT_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoPVTcallback
//        /                  _____  This _must_ be UBX_NAV_PVT_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void newPVTdata(UBX_NAV_PVT_data_t *ubxDataStruct)
{
  timeFullyResolved = ubxDataStruct->valid.bits.fullyResolved;
  tAcc = ubxDataStruct->tAcc;
}

// =-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
// GNSS TIM TP Callback: newTIMTPdata will be called when new TIM TP data arrives.
// It is sent ahead of the top-of-second and contains the UTC time for the next top-of-second
// as indicated by the TP pulse.
// See u-blox_structs.h for the full definition of UBX_TIM_TP_data_t
//         _____  You can use any name you like for the callback. Use the same name when you call setAutoTIMTPcallback
//        /                  _____  This _must_ be UBX_TIM_TP_data_t
//        |                 /               _____ You can use any name you like for the struct
//        |                 |              /
//        |                 |              |
void newTIMTPdata(UBX_TIM_TP_data_t *ubxDataStruct)
{
  // Convert the time pulse of week to Unix Epoch
  uint32_t tow = ubxDataStruct->week - SFE_UBLOX_JAN_1ST_2020_WEEK; //Calculate the number of weeks since Jan 1st 2020
  tow *= SFE_UBLOX_SECS_PER_WEEK; //Convert weeks to seconds
  tow += SFE_UBLOX_EPOCH_WEEK_2086; //Add the TOW for Jan 1st 2020
  tow += ubxDataStruct->towMS / 1000; //Add the TOW for the next TP

  uint32_t us = ubxDataStruct->towMS % 1000; //Extract the milliseconds
  us *= 1000; // Convert to microseconds

  double subMS = ubxDataStruct->towSubMS; //Get towSubMS (ms * 2^-32)
  subMS *= pow(2.0, -32.0); //Convert to milliseconds
  subMS *= 1000; //Convert to microseconds

  us += (uint32_t)subMS; // Add subMS
  
  gnssTv.tv_sec = tow; // Store the time in timeval format
  gnssTv.tv_usec = us;  
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
  success &= theGNSS.setAutoTIMTPcallbackPtr(&newTIMTPdata); // Enable automatic TIM TP messages with callback to newTIMTPdata

  // Tell the module to return UBX_MGA_ACK_DATA0 messages when we push the AssistNow data
  success &= theGNSS.setAckAiding(1);
    
  return success;
}
