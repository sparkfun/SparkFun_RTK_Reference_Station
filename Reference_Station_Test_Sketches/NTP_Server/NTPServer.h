/*
 * File: NTPServer.h
 * Description:
 *   NTP server implementation.
 * Author: Mooneer Salem <mooneer@gmail.com>
 * License: New BSD License
 */
 
#ifndef NTP_SERVER_H
#define NTP_SERVER_H

class NtpServer
{
public:
    NtpServer()
    {
        // empty
    }
    
    /*
     * Begins listening for NTP requests.
     */
    void beginListening(unsigned long port);
    
    /*
     * Processes a single NTP request.
     */
    bool processOneRequest(const timeval * tv);
    
private:
    EthernetUDP timeServerPort_;
};

#endif // NTP_SERVER_H
