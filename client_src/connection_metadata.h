//
// Created by roda on 27.03.19.
//

#ifndef WEBSOCKET_TEST_CONNECTION_METADATA_H
#define WEBSOCKET_TEST_CONNECTION_METADATA_H

#include <websocketpp/config/asio_no_tls_client.hpp>
#include <websocketpp/client.hpp>

typedef websocketpp::client<websocketpp::config::asio_client> client;

class connection_metadata {
public:
    typedef websocketpp::lib::shared_ptr<connection_metadata> ptr;

    connection_metadata( websocketpp::connection_hdl, std::string); //(int id, websocketpp::connection_hdl hdl, std::string uri)

    void on_open(client *, websocketpp::connection_hdl);
    void on_fail(client *, websocketpp::connection_hdl);
    void on_close(client *, websocketpp::connection_hdl);
    void on_message(websocketpp::connection_hdl, client::message_ptr);
    websocketpp::connection_hdl get_hdl() const;
    std::string get_status() const;
    void record_sent_message(std::string message);

private:
    websocketpp::connection_hdl m_hdl;
    std::string                 m_status;
    std::string                 m_uri;
    std::string                 m_server;
    std::string                 m_error_reason;
    std::vector<std::string>    m_messages;
};


#endif //WEBSOCKET_TEST_CONNECTION_METADATA_H
