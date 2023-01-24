/*
 * File: NTPServer.cpp
 * Description:
 *   NTP server implementation.
 * Author: Mooneer Salem <mooneer@gmail.com>
 * License: New BSD License
 */

#if defined(ARDUINO)

#include <time.h>
#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

#include "NTPPacket.h"
#include "NTPServer.h"

void NtpServer::beginListening(unsigned long port)
{
    timeServerPort_.begin(port);
}

bool NtpServer::processOneRequest(const timeval * tv)
{
    bool processed = false;
    
    int packetDataSize = timeServerPort_.parsePacket();
    if (packetDataSize && packetDataSize >= NTP_PACKET_SIZE)
    {
        // Received what is probably an NTP packet. Read it in and verify
        // that it's legit.
        NtpPacket packet;
        timeServerPort_.read((char*)&packet, NTP_PACKET_SIZE);
        // TODO: verify packet.
        
        // Populate response.
        packet.swapEndian();

        packet.leapIndicator(0);
        packet.versionNumber(3);
        packet.mode(4);
        packet.stratum = 1;
        packet.poll = 6;//10; // 6-10 per RFC 5905.
        packet.precision = -21; // ~0.5 microsecond precision.
        packet.rootDelay = 60 * (0xFFFF / 1000); // ~60 milliseconds, TBD
        packet.rootDispersion = 1 * (0xFFFF / 1000); // ~10 millisecond dispersion, TBD
        packet.referenceId[0] = 'G';
        packet.referenceId[1] = 'P';
        packet.referenceId[2] = 'S';
        packet.referenceId[3] = 0;

//        timeSource_.now(&packet.referenceTimestampSeconds, &packet.referenceTimestampFraction);
        // REF: http://support.ntp.org/bin/view/Support/DraftRfc2030
        // '.. the client sets the Transmit Timestamp field in the request
        // to the time of day according to the client clock in NTP timestamp format.'
        // '.. The server copies this field to the originate timestamp in the reply and 
        // sets the Receive Timestamp and Transmit Timestamp fields to the time of day 
        // according to the server clock in NTP timestamp format.'
        packet.originTimestampSeconds = packet.transmitTimestampSeconds;
        packet.originTimestampFraction = packet.transmitTimestampFraction;

        packet.receiveTimestampSeconds = tv->tv_sec;
        packet.receiveTimestampFraction = tv->tv_usec;
        
        // ...and the transmit time.
        //timeSource_.now(&packet.referenceTimestampSeconds, &packet.referenceTimestampFraction);
        packet.transmitTimestampSeconds = packet.referenceTimestampSeconds;
        packet.transmitTimestampFraction = packet.referenceTimestampFraction;
//        timeSource_.now(&packet.transmitTimestampSeconds, &packet.transmitTimestampFraction);

        Serial.print("do=");
        Serial.print(packet.receiveTimestampSeconds - packet.originTimestampSeconds );
        Serial.print(":");
        Serial.println(packet.receiveTimestampSeconds - packet.originTimestampFraction);

        Serial.println("--------------");
        Serial.print("rf=");
        Serial.print(packet.referenceTimestampSeconds);
        Serial.print(":");
        Serial.println(packet.referenceTimestampFraction);
        
        Serial.print("or=");
        Serial.print(packet.originTimestampSeconds);
        Serial.print(":");
        Serial.println(packet.originTimestampFraction);
        
        Serial.print("rx=");
        Serial.print(packet.receiveTimestampSeconds);
        Serial.print(":");
        Serial.println(packet.receiveTimestampFraction);
        
        Serial.print("tx=");
        Serial.print(packet.transmitTimestampSeconds);
        Serial.print(":");
        Serial.println(packet.transmitTimestampFraction);

        // Now transmit the response to the client.
        packet.swapEndian();
        timeServerPort_.beginPacket(timeServerPort_.remoteIP(), timeServerPort_.remotePort());
        for (int count = 0; count < NTP_PACKET_SIZE; count++)
        {
            timeServerPort_.write(packet.packet()[count]);
        }
        timeServerPort_.endPacket();
        processed = true;
    } 
    
    return processed;
}

#endif // defined(ARDUINO)
