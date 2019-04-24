//
// Created by jin on 19.04.19.
//

#ifndef WEBSOCKET_TEST_UDPCONTROLCONNECTIONHOST_H
#define WEBSOCKET_TEST_UDPCONTROLCONNECTIONHOST_H

#include <set>
#include "AbstractUdpControlConnection.h"

class UdpControlConnectionHost: public AbstractUdpControlConnection {
public:
    UdpControlConnectionHost(boost::asio::io_service &io);
    explicit UdpControlConnectionHost(boost::asio::io_service &io, int port, const std::string &magic_string);

    /// at this moment i don't have any idea to use copy or assigment of server object
    UdpControlConnectionHost(const UdpControlConnectionHost &rhs) = delete;
    UdpControlConnectionHost(const UdpControlConnectionHost &&rhs) = delete;

    UdpControlConnectionHost &operator=(const UdpControlConnectionHost &rhs) = delete;
    UdpControlConnectionHost &operator=(const UdpControlConnectionHost &&rhs) = delete;

    void init(int port, const std::string &magic_string);
    void run();
    void open_connection() override;

private:
    int port_;
    std::set<std::string> ip_table_;
};


#endif //WEBSOCKET_TEST_UDPCONTROLCONNECTIONHOST_H
