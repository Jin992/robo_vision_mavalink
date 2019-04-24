//
// Created by jin on 19.04.19.
//

#ifndef WEBSOCKET_TEST_UDPCONTROLCONNECTIONCLI_H
#define WEBSOCKET_TEST_UDPCONTROLCONNECTIONCLI_H

#include <mutex>
#include "AbstractUdpControlConnection.h"
#include "../bitrate.h"
#define CUR_TIME_M_SEC (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() & 0xffffffff)

class UdpControlConnectionCli: public AbstractUdpControlConnection {
public:
    explicit    UdpControlConnectionCli(boost::asio::io_service &io);
    explicit    UdpControlConnectionCli(boost::asio::io_service &io, std::string ip, std::string port, const std::string &magic_string);
    void        init_stream(std::string ip, std::string port, const std::string &magic_string);

    /// at this moment i don't have any idea to use copy or assigment of cli object
    UdpControlConnectionCli(const UdpControlConnectionCli &rhs) = delete;
    UdpControlConnectionCli(const UdpControlConnectionCli &&rhs) = delete;
    UdpControlConnectionCli &operator=(const UdpControlConnectionCli &rhs) = delete;
    UdpControlConnectionCli &operator=(const UdpControlConnectionCli &&rhs) = delete;

    void        run( std::mutex &mutex, bool &reconnect, int32_t  latency_threshold, bitrate_t &);
    void        open_connection() override;

private:
    void        init_message();

private:
    uint8_t         sequence_;
    struct pollfd   sock_in_[1];
    int             poll_res_;
    std::string     ip_;
    std::string     port_;
    int32_t         last_sended_;
};
#endif //WEBSOCKET_TEST_UDPCONTROLCONNECTIONCLI_H
