//
// Created by jin on 19.04.19.
//

#include "UdpControlConnectionHost.h"
#include <chrono>
#include <iostream>

UdpControlConnectionHost::UdpControlConnectionHost(boost::asio::io_service &io)
: AbstractUdpControlConnection(io), port_(0)
{}

/**
 *
 * @param io - ref to boost::asio::io_service object
 * @param port  - port
 * @param magic_string  - string to generate magic string
 */
UdpControlConnectionHost::UdpControlConnectionHost(boost::asio::io_service &io, int port, const std::string &magic_string)
: AbstractUdpControlConnection(io), port_(port) {
    /// generate magic string
    set_magic_string(magic_string);
    /// set connection initalization flag to true
    set_connection_init(true);
}

/**
 *
 * @param port - port
 * @param magic_string - string to generate magic string
 */
void UdpControlConnectionHost::init(int port, const std::string &magic_string) {
    if(!get_connection_init()) {
        port_ = port;
        set_magic_string(magic_string);
        set_connection_init(true);
    }
}

/**
 * Start to listen incoming connections
 */
void UdpControlConnectionHost::open_connection() {
    get_socket() = udp::socket(get_io_service(), udp::endpoint(udp::v4(), port_));
    if (!get_socket().is_open()) {
        throw "UdpControlConnectionHost: failed to open socket"; /// temporary exception stub
    }
//    std::cout << "Udp control stream: " << get_socket().local_endpoint().address().to_string() << ":";
//    std::cout << get_socket().local_endpoint().port() << std::endl;
}

/**
 *  event loop
 */
void UdpControlConnectionHost::run() {
    while (true)
    {
        boost::system::error_code err;
        get_error_code();
        /// prepare buffer
        get_request_buff().fill(0);
        get_socket().receive_from(boost::asio::buffer(get_request_buff()),
                            get_remote_endpoint());
        /// check errors
        if (err && err != boost::asio::error::message_size) {
            throw boost::system::system_error(err);
        }
        /// check packets magic string if it's equal calculate
        if (std::string(get_request_buff().data(), get_magic_string().size()) == get_magic_string()) {


            std::cerr << get_remote_endpoint().address().to_string() << std::endl;
            ctl_pkt res;
            /// read ctl struct from pkt
            ctl_pkt *r_pkt = (ctl_pkt *)(get_request_buff().data() + get_magic_string().size());
            res.last_received_id = r_pkt->last_received_id;
            res.last_received_time = (std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() & 0xffffffff);

            get_res_buff().fill(0);
            /// fill magic string and pkt to buffer
            memcpy(get_res_buff().data(), get_magic_string().c_str(), get_magic_string().size());
            memcpy(get_res_buff().data() + get_magic_string().size(), &res, sizeof(ctl_pkt));
            /// send pkt
            get_socket().send_to(boost::asio::buffer(get_res_buff()), get_remote_endpoint(), 0, err);
            if (err) {
                throw "UdpControlConnectionHost: failed to send msg";
            }
        }
    }
}

