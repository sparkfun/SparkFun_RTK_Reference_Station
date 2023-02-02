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
  const uint8_t defaultLeapInd = 0;
  uint8_t LI() { return LiVnMode >> 6; }
  void LI(uint8_t val) { LiVnMode = (LiVnMode & 0x3F) | ((val & 0x03) << 6); }

  // Version Number is 3 bits. NTP version is currently four (4)
  const uint8_t defaultVersion = 4;
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
  const uint8_t defaultMode = 4;
  uint8_t mode() { return (LiVnMode & 0x07); }
  void mode(uint8_t val) { LiVnMode = (LiVnMode & 0xF8) | (val & 0x07); }

  // Stratum is 8 bits:
  // 0 : Unspecified
  // 1 : Reference clock (e.g., radio clock)
  // 2-15 : Secondary server (via NTP)
  // 16-255 : Unreachable
  //
  // We'll use 1 = Reference Clock
  const uint8_t defaultStratum = 1;
  uint8_t stratum;

  // Poll exponent
  // This is an eight-bit unsigned integer indicating the maximum interval between successive messages,
  // in seconds to the nearest power of two.
  // In the reference implementation, the values can range from 3 (8 s) through 17 (36 h).
  //
  // RFC 5905 suggests 6-10. We'll use 6. 2^6 = 64 seconds
  const uint8_t defaultPollExponent = 6;
  uint8_t pollExponent;

  // Precision
  // This is an eight-bit signed integer indicating the precision of the system clock,
  // in seconds to the nearest power of two. For instance, a value of -18 corresponds to a precision of about 4us.
  //
  // tAcc is usually around 1us. So we'll use -20 (0xEC). 2^-20 = 0.95us
  const int8_t defaultPrecision = -20; // 0xEC
  int8_t precision;

  // Root delay
  // This is a 32-bit, unsigned, fixed-point number indicating the total round-trip delay to the reference clock,
  // in seconds with fraction point between bits 15 and 16. In contrast to the calculated peer round-trip delay,
  // which can take both positive and negative values, this value is always positive.
  //
  // We are the reference clock, so we'll use zero (0x00000000).
  const uint32_t defaultRootDelay = 0x00000000;
  uint32_t rootDelay;

  // Root dispersion
  // This is a 32-bit, unsigned, fixed-point number indicating the maximum error relative to the reference clock,
  // in seconds with fraction point between bits 15 and 16.
  //
  // Tricky... Could depend on interrupt service time? Maybe go with ~1ms?
  const uint32_t defaultRootDispersion = 0x00000042; // 1007us
  uint32_t rootDispersion;

  // Reference identifier
  // This is a 32-bit code identifying the particular reference clock. The interpretation depends on the value in
  // the stratum field. For stratum 0 (unsynchronized), this is a four-character ASCII (American Standard Code for
  // Information Interchange) string called the kiss code, which is used for debugging and monitoring purposes.
  // GPS : Global Positioning System
  const uint8_t referenceIdLen = 4;
  const char defaultReferenceId[4] = { 'G', 'P', 'S', 0 };
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

    for (uint8_t i = 0; i < referenceIdLen; i++)
      referenceId[i] = *ptr++;

    referenceTimestampSeconds = extractUnsigned32(ptr);
    ptr += 4;
    referenceTimestampFraction = extractUnsigned32(ptr); // Note: the fraction is in increments of (1 / 2^32) secs, not microseconds
    ptr += 4;
    originateTimestampSeconds = extractUnsigned32(ptr);
    ptr += 4;
    originateTimestampFraction = extractUnsigned32(ptr);
    ptr += 4;
    receiveTimestampSeconds = extractUnsigned32(ptr);
    ptr += 4;
    receiveTimestampFraction = extractUnsigned32(ptr);
    ptr += 4;
    transmitTimestampSeconds = extractUnsigned32(ptr);
    ptr += 4;
    transmitTimestampFraction = extractUnsigned32(ptr);
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
    insertUnsigned32(ptr, receiveTimestampSeconds);
    ptr += 4;
    insertUnsigned32(ptr, receiveTimestampFraction);
    ptr += 4;
    insertUnsigned32(ptr, transmitTimestampSeconds);
    ptr += 4;
    insertUnsigned32(ptr, transmitTimestampFraction);
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
// Process one NTP request
// recTv contains the timeval the NTP packet was received - from the W5500 interrupt
// syncTv contains the timeval when the RTC was last sync'd
// ntpDiag will contain useful diagnostics
bool processOneRequest(bool process, const timeval * recTv, const timeval * syncTv, char *ntpDiag = nullptr, size_t ntpDiagSize = 0); // Header
bool processOneRequest(bool process, const timeval * recTv, const timeval * syncTv, char *ntpDiag, size_t ntpDiagSize)
{
  bool processed = false;

  if (ntpDiag != nullptr)
    *ntpDiag = 0; // Clear any existing results
  
  int packetDataSize = timeServer.parsePacket();

  IPAddress remoteIP = timeServer.remoteIP();
  uint16_t remotePort = timeServer.remotePort();
  
  if (ntpDiag != nullptr) // Add the packet size and remote IP/Port to the diagnostics
  {
    snprintf(ntpDiag, ntpDiagSize, "Packet Size: %d  Remote IP: %d.%d.%d.%d  Remote Port: %d\r\n",
      packetDataSize, remoteIP[0], remoteIP[1], remoteIP[2], remoteIP[3], remotePort);
  }
    
  if (packetDataSize && (packetDataSize >= NTPpacket::NTPpacketSize))
  {
    // Read the NTP packet
    NTPpacket packet;
    
    timeServer.read((char*)&packet.packet, NTPpacket::NTPpacketSize); // Copy the NTP data into our packet

    // If process is false, return now
    if (!process)
    {
      char tmpbuf[128];
      snprintf(tmpbuf, sizeof(tmpbuf), "Packet ignored. Time has not been synchronized.\r\n");
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
      return false;
    }
    
    packet.extract(); // Extract the raw data into fields
    
    packet.LI(packet.defaultLeapInd); // Clear the leap second adjustment. TODO: set this correctly using getLeapSecondEvent from the GNSS
    packet.VN(packet.defaultVersion); // Set the version number
    packet.mode(packet.defaultMode); // Set the mode
    packet.stratum = packet.defaultStratum; // Set the stratum
    packet.pollExponent = packet.defaultPollExponent; // Set the poll interval
    packet.precision = packet.defaultPrecision; // Set the precision
    packet.rootDelay = packet.defaultRootDelay; // Set the Root Delay
    packet.rootDispersion = packet.defaultRootDispersion; // Set the Root Dispersion
    for (uint8_t i = 0; i < packet.referenceIdLen; i++)
      packet.referenceId[i] = packet.defaultReferenceId[i]; // Set the reference Id

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
      snprintf(tmpbuf, sizeof(tmpbuf), "Originate Timestamp (Client Transmit):\t%u.%06u\r\n",
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
      snprintf(tmpbuf, sizeof(tmpbuf), "Received Timestamp:\t\t\t%u.%06u\r\n",
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
      snprintf(tmpbuf, sizeof(tmpbuf), "Reference Timestamp (Last Sync):\t%u.%06u\r\n",
        packet.referenceTimestampSeconds, packet.convertFractionToMicros(packet.referenceTimestampFraction));
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
    }

    // Add the transmit time - i.e. now!
    timeval txTime;
    gettimeofday(&txTime, (timezone *)&tz_utc);
    packet.transmitTimestampSeconds = packet.convertUnixSecondsToNTP(txTime.tv_sec); // Unix -> NTP
    packet.transmitTimestampFraction = packet.convertMicrosToFraction(txTime.tv_usec); // Micros to 1/2^32

    packet.insert(); // Copy the data fields back into the buffer

    // Now transmit the response to the client.
    timeServer.beginPacket(remoteIP, remotePort);
    timeServer.write(packet.packet, NTPpacket::NTPpacketSize);
    int result = timeServer.endPacket();
    processed = true;

    // Add our server transmit time to the diagnostics
    if (ntpDiag != nullptr)
    {
      char tmpbuf[128];
      snprintf(tmpbuf, sizeof(tmpbuf), "Transmit Timestamp:\t\t\t%u.%06u\r\n",
        packet.transmitTimestampSeconds, packet.convertFractionToMicros(packet.transmitTimestampFraction));
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
    }

    // Add the socketSendUDP result to the diagnostics
    if (ntpDiag != nullptr)
    {
      char tmpbuf[128];
      snprintf(tmpbuf, sizeof(tmpbuf), "socketSendUDP result: %d\r\n", result);
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
    }

    /*
    // Add the packet to the diagnostics
    if (ntpDiag != nullptr)
    {
      char tmpbuf[128];
      snprintf(tmpbuf, sizeof(tmpbuf), "Packet: ");
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
      for (int i = 0; i < NTPpacket::NTPpacketSize; i++)
      {
        snprintf(tmpbuf, sizeof(tmpbuf), "%02X ", packet.packet[i]);
        strlcat(ntpDiag, tmpbuf, ntpDiagSize);
      }
      snprintf(tmpbuf, sizeof(tmpbuf), "\r\n");
      strlcat(ntpDiag, tmpbuf, ntpDiagSize);
    }
    */
    
  }
  
  return processed;
}
