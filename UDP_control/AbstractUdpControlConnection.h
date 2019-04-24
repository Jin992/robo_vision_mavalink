//
// Created by jin on 19.04.19.
//

#ifndef WEBSOCKET_TEST_ABSTRACTUDPCONTROLCONNECTION_H
#define WEBSOCKET_TEST_ABSTRACTUDPCONTROLCONNECTION_H
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <string>

using   boost::asio::ip::udp;
typedef boost::asio::io_service io_object;


constexpr uint8_t magic_len = 32;

/** Struct to store data for transfer */
typedef struct  control_packet{
    uint8_t  last_received_id;
    int32_t last_received_time;
}               ctl_pkt;

typedef boost::array<char, magic_len + sizeof(ctl_pkt)> ctl_buff_t;
/**
 *  AbstractUdpControlConnection is interface for UDP Control Stream Client/Server
 */
class AbstractUdpControlConnection {
public:
    explicit AbstractUdpControlConnection(boost::asio::io_service &io);
    /** ~AbstractUdpControlConnection() is pure virtual destructor*/
    virtual ~AbstractUdpControlConnection() = 0;

    /** virtual methods may be overridden if needed */
    virtual void                close_connection();

    /** pure virtual methods need to be overridden */
    virtual void                open_connection() = 0;

protected:
    /** class getters */
    boost::asio::io_service     &get_io_service();      /// return ref to io_service object
    udp::socket                 &get_socket();          /// return ref to socket object
    boost::system::error_code   &get_error_code();      /// return ref to error object method
    ctl_buff_t                  &get_request_buff();    /// return ref to request buffer
    ctl_buff_t                  &get_res_buff();        /// return ref to result buffer
    udp::endpoint               &get_remote_endpoint(); /// return ref to endpoint object
    std::string                 get_magic_string();     /// return identification string
    bool                        get_connection_init();  /// return state of object initialization

    /** class setters */
    void                        set_magic_string(std::string const &key); /// create identification string
    void                        set_connection_init(bool status);         /// set initialization state of object

private:
    /** make_magic_key(std::string const &key) is same method class objects */
    static std::string          make_magic_key(std::string const &key); // generate magic string for identification

private:
    io_object                   &io_;                /// boost::asio::io_service object
    udp::socket                 socket_;             /// boost::asio::io_service::ip::udp::socket object
    udp::endpoint               receiver_endpoint_;  /// remote endpoint
    boost::system::error_code   ec_;                 /// boost::system::error_code object
    std::string                 magic_string_;       /// string that contains MD5 key
    ctl_buff_t                  request_buf_;        /// request buffer
    ctl_buff_t                  result_buf_;         /// result buffer
    bool                        connection_inited_ ; /// tell us if connection was inited
};

/** Assign empty destructor implementation*/
inline AbstractUdpControlConnection::~AbstractUdpControlConnection()
{}


#endif //WEBSOCKET_TEST_ABSTRACTUDPCONTROLCONNECTION_H
