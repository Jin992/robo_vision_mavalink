//
// Created by roda on 27.03.19.
//

#ifndef WEBSOCKET_TEST_BASE_SERVER_H
#define WEBSOCKET_TEST_BASE_SERVER_H

#include <websocketpp/config/asio_no_tls.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/endpoint.hpp>
#include <set>

/* websocketpp custom connections structs */
struct  connection_data {
    int id;
    int sessionid;
    std::string name;
    std::queue<std::pair<uint8_t *, size_t>> video_stream;
    size_t last_packet_id;
};

struct custom_config : public websocketpp::config::asio {
    // pull default settings from our core config
    typedef websocketpp::config::asio core;

    typedef core::concurrency_type concurrency_type;
    typedef core::request_type request_type;
    typedef core::response_type response_type;
    typedef core::message_type message_type;
    typedef core::con_msg_manager_type con_msg_manager_type;
    typedef core::endpoint_msg_manager_type endpoint_msg_manager_type;
    typedef core::alog_type alog_type;
    typedef core::elog_type elog_type;
    typedef core::rng_type rng_type;
    typedef core::transport_type transport_type;
    typedef core::endpoint_base endpoint_base;

    // Set a custom connection_base class
    typedef connection_data connection_base;
};
/* websocketpp custom connections structs */


typedef websocketpp::server<custom_config> server;
typedef server::connection_ptr connection_ptr;
typedef std::function<void(server::message_ptr &msg)> msg_handler;

using websocketpp::connection_hdl;
using websocketpp::lib::placeholders::_1;
using websocketpp::lib::placeholders::_2;
using websocketpp::lib::bind;

class base_server {
public:
    base_server();
    base_server(const base_server &rhs) = delete;
    base_server(const base_server &&rhs) = delete;
    ~base_server() = default;

    base_server &operator=(const base_server &rhs) = delete;
    base_server &operator=(const base_server &&rhs) = delete;

    void run(uint16_t); //(uint16_t port)
    // Handlers
    void on_open(connection_hdl);
    void on_close(connection_hdl);
    void on_message(connection_hdl, server::message_ptr);
    bool on_validate(connection_hdl);
    void on_http(connection_hdl);
    void set_msg_hdl(std::function<void(server::message_ptr &msg)> &hdl);
    bool action;

private:
    typedef std::vector<connection_ptr> con_list;
    server          m_server;
    con_list        m_connections;
    bool            is_connected;
    int             counter;
    int             id_counter;
    msg_handler     m_handler;

};

#endif //WEBSOCKET_TEST_BASE_SERVER_H
