//
// Created by jin on 19.04.19.
//

#include "AbstractUdpControlConnection.h"
#include <openssl/md5.h>

/**
 * AbstractUdpControlConnection::AbstractUdpControlConnection(boost::asio::io_service &io)
 * @brief Constructor for UdpControlConnection interface
 * @param io - boost::asio::io_service object
 */
AbstractUdpControlConnection::AbstractUdpControlConnection(boost::asio::io_service &io)
: io_(io), socket_(io), request_buf_{0}, result_buf_{0}, connection_inited_(false)
{}

/**
 * std::string AbstractUdpControlConnection::make_magic_key(std::string const &key)
 * @brief Generate magic string
 * @param key - key word to generate magic string
 * @return magic string
 */
std::string AbstractUdpControlConnection::make_magic_key(std::string const &key) {
    char mdString[33];
    std::string to_transform(key + " this is magic");
    unsigned char digest[MD5_DIGEST_LENGTH];

    MD5((unsigned char*)to_transform.c_str(), to_transform.size(), (unsigned char*)&digest);
    for(int i = 0; i < 16; i++)
        sprintf(&mdString[i*2], "%02x", (unsigned int)digest[i]);
    return std::string(mdString);
}

/**
 * boost::system::error_code& AbstractUdpControlConnection::get_error_code()
 * @brief getter for boost::system::error_code object
 * @return  reference to boost::system::error_code object
 */
boost::system::error_code& AbstractUdpControlConnection::get_error_code() {
    return ec_;
}

/**
 * udp::socket& AbstractUdpControlConnection::get_socket()
 * @brief getter for boost::asio::ip::udp::socket object
 * @return reference to boost::asio::ip::udp::socket object
 */
udp::socket& AbstractUdpControlConnection::get_socket() {
    return socket_;
}

/**
 * boost::asio::io_service& AbstractUdpControlConnection::get_io_service()
 * @brief getter for boost::asio::io_service object
 * @return reference to boost::asio::io_service object
 */
boost::asio::io_service& AbstractUdpControlConnection::get_io_service() {
    return io_;
}

/**
 * std::string AbstractUdpControlConnection::get_magic_string()
 * @brief getter for magic string value
 * @return std::string with magic string as value
 */
std::string AbstractUdpControlConnection::get_magic_string() {
    return magic_string_;
}

/**
 * void AbstractUdpControlConnection::set_magic_string(std::string const &key)
 * @brief generate and assign magic string
 * @param key - word that transform to magic string
 */
void AbstractUdpControlConnection::set_magic_string(std::string const &key) {
    magic_string_ = make_magic_key(key);
}

/**
 * void AbstractUdpControlConnection::close_connection()
 * @brief close udp control stream
 */
void AbstractUdpControlConnection::close_connection() {
    socket_.close(get_error_code());
    if (get_error_code()) {
        throw "UdpControlStream close";
    }
}

/**
 * void AbstractUdpControlConnection::set_connection_init(bool status)
 * @brief set initialization status for control connection
 * @param status - initialization status
 */
void AbstractUdpControlConnection::set_connection_init(bool status) {
    connection_inited_ = status;
}

/**
 * bool AbstractUdpControlConnection::get_connection_init()
 * @brief getter for connection initialization status
 * @return initialization status
 */
bool AbstractUdpControlConnection::get_connection_init() {
    return connection_inited_;
}

/**
 * udp::endpoint& AbstractUdpControlConnection::get_remote_endpoint()
 * @brief getter for boost::asio::ip::udp::endpoint object (remote endpoint)
 * @return boost::asio::ip::udp::endpoint object
 */
udp::endpoint& AbstractUdpControlConnection::get_remote_endpoint() {
    return  receiver_endpoint_;
}

/**
 * ctl_buff_t & AbstractUdpControlConnection::get_res_buff()
 * @brief getter for result buffer
 * @return ctl_buff_t object
 */
ctl_buff_t & AbstractUdpControlConnection::get_res_buff() {
    return result_buf_;
}

/**
 * ctl_buff_t& AbstractUdpControlConnection::get_request_buff()
 * @brief getter for request buffer
 * @return ctl_buff_t object
 */
ctl_buff_t& AbstractUdpControlConnection::get_request_buff() {
    return request_buf_;
}